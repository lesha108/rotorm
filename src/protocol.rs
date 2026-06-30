// протоеол обмена с исполнительным модулем певоротки
use super::*;

use embassy_stm32::usart::BufferedUart;
use embassy_time::{Duration, with_timeout};
use embedded_io_async::{Read, Write};

use crc::{CRC_8_LTE, Crc};
use heapless::Vec;

use crate::e220::*;
use crate::easycom::*;
use crate::errors::*;

const CMD_PACKET_SIZE: usize = 11;

pub struct Protocol {
    seq: u32, // последовательный номер пакета для контроля эфира
}

impl Protocol {
    pub fn new(rnd: u32) -> Self {
        Protocol { seq: rnd }
    }

    pub fn next_seq(&mut self) {
        self.seq = self.seq + 1;
    }

    pub fn get_seq(&self) -> u32 {
        self.seq
    }

    // расчёт контрольной суммы
    fn crc8(&mut self, block: &[u8]) -> u8 {
        let crc_provider = Crc::<u8>::new(&CRC_8_LTE);
        let mut digest = crc_provider.digest();
        digest.update(block);
        digest.finalize()
    }

    // Пакет команды повортке
    // 1 - байт 0xFF
    // 2,3,4,5 - последовательный номер
    // 6,7 - u16 AngleAz
    // 8,9 - u16 AngleEl
    // 10 - питание PTZ, LNA
    // 11 - CRC
    pub fn make_command(
        &mut self,
        az: &AzAngle,
        el: &ElAngle,
        rly: &Relays,
    ) -> Result<Vec<u8, CMD_PACKET_SIZE>, Error> {
        let mut payload: Vec<u8, CMD_PACKET_SIZE> = Vec::new();
        let seq_bytes_be: [u8; 4] = self.seq.to_be_bytes();

        let azf: f32 = az.into();
        let azu: u16 = (azf * 100.0) as u16;
        let az_bytes_be: [u8; 2] = azu.to_be_bytes();

        // конвертируем в формат поворотки с инверсией угла
        let elf: f32 = el.into();
        let elu: u16 = ((90.0 - elf) * 100.0) as u16;
        let el_bytes_be: [u8; 2] = elu.to_be_bytes();

        // формируем пакет
        payload.push(0xFF).unwrap();
        payload.extend_from_slice(&seq_bytes_be).unwrap();
        payload.extend_from_slice(&az_bytes_be).unwrap();
        payload.extend_from_slice(&el_bytes_be).unwrap();
        payload.push(rly.into()).unwrap();

        // считаем его CRC
        let crc8 = self.crc8(&payload);
        // добавляем CRC в коне пакета
        payload.push(crc8).unwrap();
        Ok(payload)
    }
}

// таск обработки команд e220 приёма и передачи в эфир
#[embassy_executor::task]
pub async fn process_e220(
    mut usart: BufferedUart<'static>,
    m0: Output<'static>,
    m1: Output<'static>,
    aux: Input<'static>,
    seq: u32,
    relay_ptz: Input<'static>,
    relay_lna: Input<'static>,
) {
    let mut e220 = E220Module::new(usart, m0, m1, aux);

    // инициализация радио модуля E220
    let mut init_count = 1;
    // после старта он должен поднять уровень AUX
    info!("Init e220...");
    e220.aux_wait().await;
    loop {
        info!("Init attempt {}", init_count);
        let r = e220.module_init(MODULE_ADDRESS_H, MODULE_ADDRESS_L).await;
        match r {
            Err(_) => {
                info!("Error init");
                init_count += 1;
                if init_count > 10 {
                    loop {}
                }
                continue;
            }
            Ok(_) => {
                info!("Init e220 OK");
                break;
            }
        }
    }
    e220.set_mode(E220Mode::default()).await;

    let mut protocol = Protocol::new(seq);

    let mut az_cmd = AzAngle(0.0);
    let mut el_cmd = ElAngle(0.0);
    let mut naz = AzAngle(0.0);
    let mut nel = ElAngle(0.0);
    let mut pwr = Relays::new();

    loop {
        // получаем уровень местного шума
        let res = e220.try_get_noise_dbm(3).await;
        match res {
            Err(_) => {
                info!("Err read noise dbm");
            }
            Ok(rssi) => {
                let dbm = rssi_to_dbm(rssi);
                info!("Noise dbm: {}", dbm);
                let ctx = APP_CONTEXT.lock().await;
                let mut inner = ctx.borrow_mut();
                inner.noise_m = rssi;
                LCD_REDRAW.signal(());
            }
        }

        // читаем целевые значения углов и питания
        {
            let ctx = APP_CONTEXT.lock().await;
            let inner = ctx.borrow();
            if inner.manual {
                az_cmd = inner.az_pot;
                el_cmd = inner.el_pot;
            } else {
                az_cmd = inner.target_az;
                el_cmd = inner.target_el;
            }
            //naz = inner.current_az;
            //nel = inner.current_el;
            pwr = inner.power_control;
        }

        // если ничего менять не надо - команду не посылаем
        /*if !(az_cmd == naz && el_cmd == nel) {
            Timer::after_millis(3000).await;
            continue;
        }*/

        // обработка сигналов включения питания
        if relay_ptz.is_high() {
            //info!("pwr high");
            pwr.ptz_off();
        } else {
            //info!("pwr low");
            pwr.ptz_on();
        }
        if relay_lna.is_high() {
            pwr.lna_off();
        } else {
            pwr.lna_on();
        }

        Timer::after_millis(2000).await;
        protocol.next_seq();
        {
            let ctx = APP_CONTEXT.lock().await;
            let mut inner = ctx.borrow_mut();
            inner.seq = protocol.get_seq();
        }

        // формируем команду для поворотки
        let cmd = protocol.make_command(&az_cmd, &el_cmd, &pwr).unwrap();
        // отправляем в эфир
        let res = e220
            .send_packet(SLAVE_ADDRESS_H, SLAVE_ADDRESS_L, &cmd)
            .await;
        match res {
            Err(_) => {
                info!("Command send failed");
            }
            Ok(_) => {
                info!("Command sent successfully");
            }
        }

        // ждем ответ от PTZ
        const RX_WAIT: Duration = Duration::from_millis(7000);
        let rx_timeout = RX_WAIT;

        // слушаем эфир
        info!("Waiting for packet for 7 sec...");
        //let mut rcv_try = 1;
        loop {
            let res = with_timeout(rx_timeout, e220.get_packet()).await;
            match res {
                Err(_) => {
                    info!("Rcv response timeout");
                    break;
                }
                Ok(rxres) => match rxres {
                    Err(_) => {
                        info!("Response rcv failed");
                        break;
                    }
                    Ok(_) => {
                        info!("Got packet {:?}", e220.rcv_buf);
                        if e220.rcv_buf.len() < 3 {
                            // ???? нужно пропустить первый пустой пакет
                            continue;
                        } else {
                            break;
                        }
                    }
                },
            }
        }

        // валидация полученноо пакета и парсинг пакета - группа операций
        if e220.rcv_buf.len() != 14 {
            info!("ERR packet len");
            continue;
        }
        if e220.rcv_buf[0] != 0xFF {
            info!("ERR 0xFF");
            continue;
        }
        // считаем и проверяем его CRC
        let crc8 = protocol.crc8(&e220.rcv_buf[0..12]);
        if e220.rcv_buf[12] != crc8 {
            info!("ERR CRC");
            continue;
        }
        // проверяем, что номер пакета всегда растет
        let seq_bytes = &e220.rcv_buf[1..=4];
        let ptz_seq = u32::from_be_bytes(seq_bytes.try_into().unwrap());
        // если в ответе меньший номер, чем мы передали
        if protocol.get_seq() < ptz_seq {
            info!("ERR Seq");
            continue;
        }
        let az_bytes = &e220.rcv_buf[5..=6];
        let new_azu = u16::from_be_bytes(az_bytes.try_into().unwrap());
        let new_azf = (new_azu / 100) as f32;
        let new_az = if let Ok(a) = AzAngle::try_from(new_azf) {
            a
        } else {
            info!("ERR Az");
            continue;
        };
        let el_bytes = &e220.rcv_buf[7..=8];
        let new_elu = u16::from_be_bytes(el_bytes.try_into().unwrap());
        let new_elf = ((9000 - new_elu) / 100) as f32;
        let new_el = if let Ok(a) = ElAngle::try_from(new_elf) {
            a
        } else {
            info!("ERR El");
            continue;
        };

        let rlys = if let Ok(a) = Relays::try_from(e220.rcv_buf[11]) {
            a
        } else {
            info!("ERR Rly");
            continue;
        };

        let ptz_rssi = e220.rcv_buf[9];
        let ptz_rssi_n = e220.rcv_buf[10];
        let rssi = e220.rcv_buf[13];
        info!(
            "RSSI {} {} {}",
            rssi_to_dbm(ptz_rssi),
            rssi_to_dbm(ptz_rssi_n),
            rssi_to_dbm(rssi)
        );

        // обновляем значения углов
        {
            let ctx = APP_CONTEXT.lock().await;
            let mut inner = ctx.borrow_mut();
            inner.current_az = new_az;
            inner.current_el = new_el;
            inner.signal_s = ptz_rssi;
            inner.noise_s = ptz_rssi_n;
            inner.signal_m = rssi;
            inner.power_control = rlys;
        }
        LCD_REDRAW.signal(());
    }
}
