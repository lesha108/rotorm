use super::*;
use core::sync::atomic::Ordering;
//use embassy_futures::select::{Either, select};
use errors::*;
//use embassy_stm32::peripherals::*;

// Написано для работы с модулем EBYTE E220-900T22D

// для РФ канал 868 Мгц = 850,125 МГц + RADIO_CHANNEL * 1 МГц
const RADIO_CHANNEL: u8 = 19;

#[allow(dead_code)]
const BROADCAST_ADDR: u8 = 0xFF;

// режимы работы модуля
const MODENORMALM0M1: (bool, bool) = (false, false);
const MODEWORTRANSMITTINGM0M1: (bool, bool) = (false, true);
const MODEWORRECEIVINGM0M1: (bool, bool) = (true, false);
const MODEDEEPSLEEPM0M1: (bool, bool) = (true, true);

#[derive(PartialEq, Eq, Copy, Clone)]
pub enum E220Mode {
    ModeNormal,
    ModeWORTransmitting,
    ModeWORReceiving,
    ModeDeepSleep,
}

impl Default for E220Mode {
    fn default() -> Self {
        E220Mode::ModeNormal
    }
}

// возвращает значения пинов M0, M1
impl From<E220Mode> for (bool, bool) {
    fn from(val: E220Mode) -> (bool, bool) {
        match val {
            E220Mode::ModeNormal => MODENORMALM0M1,
            E220Mode::ModeWORTransmitting => MODEWORTRANSMITTINGM0M1,
            E220Mode::ModeWORReceiving => MODEWORRECEIVINGM0M1,
            E220Mode::ModeDeepSleep => MODEDEEPSLEEPM0M1,
        }
    }
}

// команды работы с регистрами
#[allow(dead_code)]
#[derive(PartialEq, Eq, Copy, Clone)]
pub enum E220Command {
    SetRegister = 0xC0,
    ReadRegister = 0xC1,
    SetTempRegister = 0xC2,
}

impl Default for E220Command {
    fn default() -> Self {
        E220Command::SetRegister
    }
}

// регистры модуля
#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(PartialEq, Eq, Copy, Clone)]
pub enum E220Registers {
    ADDH = 0x00,
    ADDL = 0x01,
    REG0 = 0x02,
    REG1 = 0x03,
    REG2 = 0x04,
    REG3 = 0x05,
    CRYPT_H = 0x06,
    CRYPT_L = 0x07,
}

// REG0 биты
#[allow(dead_code)]
pub enum E220SerialRate {
    Rate1200 = 0b00000000,
    Rate2400 = 0b00100000,
    Rate4800 = 0b01000000,
    Rate9600 = 0b01100000,
    Rate19200 = 0b10000000,
    Rate38400 = 0b10100000,
    Rate57600 = 0b11000000,
    Rate115200 = 0b11100000,
}

impl Default for E220SerialRate {
    fn default() -> Self {
        E220SerialRate::Rate9600
    }
}

#[allow(dead_code)]
pub enum E220SerialParity {
    P8N1 = 0b00000000,
    P8O1 = 0b00001000,
    P8E1 = 0b00010000,
    P8N12 = 0b00011000, // как P8N1
}

impl Default for E220SerialParity {
    fn default() -> Self {
        E220SerialParity::P8N1
    }
}

#[allow(dead_code)]
pub enum E220AirRate {
    Rate24001 = 0b00000000,
    Rate24002 = 0b00000001,
    Rate24003 = 0b00000010,
    Rate4800 = 0b00000011,
    Rate9600 = 0b00000100,
    Rate19200 = 0b00000101,
    Rate38400 = 0b00000110,
    Rate62500 = 0b00000111,
}

impl Default for E220AirRate {
    fn default() -> Self {
        E220AirRate::Rate24003
    }
}

// REG1 биты
#[allow(dead_code)]
pub enum E220SubPacket {
    Bytes200 = 0b00000000,
    Bytes128 = 0b01000000,
    Bytes64 = 0b10000000,
    Bytes32 = 0b11000000,
}

impl Default for E220SubPacket {
    fn default() -> Self {
        E220SubPacket::Bytes200
    }
}

pub enum E220Noise {
    Disable = 0b00000000,
    Enable = 0b00100000,
}

impl Default for E220Noise {
    fn default() -> Self {
        E220Noise::Disable
    }
}

#[allow(dead_code)]
pub enum E220Power {
    Dbm22 = 0b00000000,
    Dbm17 = 0b00000001,
    Dbm13 = 0b00000010,
    Dbm10 = 0b00000011,
}

impl Default for E220Power {
    fn default() -> Self {
        E220Power::Dbm22
    }
}

// REG3 биты
pub enum E220Rssi {
    Disable = 0b00000000,
    Enable = 0b10000000,
}

impl Default for E220Rssi {
    fn default() -> Self {
        E220Rssi::Disable
    }
}

pub enum E220Transmission {
    Transparent = 0b00000000,
    Fixed = 0b01000000,
}

impl Default for E220Transmission {
    fn default() -> Self {
        E220Transmission::Transparent
    }
}

#[allow(dead_code)]
pub enum E220LBT {
    Disable = 0b00000000,
    Enable = 0b00010000,
}

impl Default for E220LBT {
    fn default() -> Self {
        E220LBT::Disable
    }
}

#[allow(dead_code)]
pub enum E220WOR {
    Ms500 = 0b00000000,
    Ms1000 = 0b00000001,
    Ms1500 = 0b00000010,
    Ms2000 = 0b00000011,
    Ms2500 = 0b00000100,
    Ms3000 = 0b00000101,
    Ms3500 = 0b00000110,
    Ms4000 = 0b00000111,
}

// структура для работы с модулем E220

pub struct E220Module {
    snd_buf: Vec<u8, 10>,
    pub rcv_buf: Vec<u8, 210>,
    e220_port: BufferedUart<'static>,
    m0: Output<'static>,
    m1: Output<'static>,
    aux: Input<'static>,
}

impl E220Module {
    pub fn new(
        e220_port: BufferedUart<'static>,
        m0: Output<'static>,
        m1: Output<'static>,
        aux: Input<'static>,
    ) -> Self {
        E220Module {
            snd_buf: Vec::new(),
            rcv_buf: Vec::new(),
            e220_port: e220_port,
            m0: m0,
            m1: m1,
            aux: aux,
        }
    }

    pub fn aux_ready(&mut self) -> bool {
        self.aux.is_high()
    }

    // ожидание готовности модуля по сигналу AUX - должен стать высокий уровень
    pub async fn aux_wait(&mut self) {
        loop {
            if self.aux_ready() {
                break;
            }
            embassy_futures::yield_now().await;
        }
    }

    // установка режима работы модуля
    pub async fn set_mode(&mut self, mode: E220Mode) {
        let (m0, m1) = &mode.into();
        if *m0 {
            let _ = self.m0.set_high();
        } else {
            let _ = self.m0.set_low();
        }
        if *m1 {
            let _ = self.m1.set_high();
        } else {
            let _ = self.m1.set_low();
        }
        if mode == E220Mode::ModeDeepSleep {
            Timer::after_millis(2).await;
            return;
        }
        self.aux_wait();
        Timer::after_millis(2).await;
    }

    // чтение единичного байта из порта модуля
    pub async fn read_byte(&mut self, timeout: Duration) -> Result<u8, Error> {
        let mut read_buf: [u8; 1] = [0; 1]; // Читаем по одному символу
        let res = with_timeout(timeout, self.e220_port.read(&mut read_buf)).await;
        match res {
            Err(_) => {
                // закончили приём команды по таймауту
                return Err(Error::Timeout);
            }
            Ok(rxres) => {
                match rxres {
                    Err(_) => {
                        //WTF?
                        return Err(Error::E220ReadErr);
                    }
                    Ok(x) => {
                        if x > 0 {
                            return Ok(read_buf[0]);
                        } else {
                            return Err(Error::E220ReadErr);
                        }
                    }
                }
            }
        }
    }

    // очиста мусора из входящего порта - по идее не должно его быть при норм работе
    pub async fn empty_rcv(&mut self) {
        let mut buf = [0u8; 1];
        let timeout = Duration::from_micros(100); // Shorter than inter-byte time

        loop {
            if !self.aux_ready() {
                // может начаться сейчас приём реального пакета
                return;
            }
            let res = with_timeout(timeout, self.e220_port.read(&mut buf)).await;
            match res {
                Err(_) => {
                    break;
                }
                Ok(rxres) => match rxres {
                    Err(_) => {
                        break;
                    }
                    Ok(_) => {}
                },
            }
        }
    }

    // чтение значения регистра модуля
    // должен быть режим E220Mode::ModeDeepSleep !!!
    pub async fn read_register(&mut self, reg: E220Registers) -> Result<u8, Error> {
        self.aux_wait().await;
        self.rcv_buf.clear();
        self.snd_buf.clear();
        self.empty_rcv().await;

        // отправка запроса на чтение регистра
        self.snd_buf.push(E220Command::ReadRegister as u8).unwrap();
        self.snd_buf.push(reg as u8).unwrap();
        self.snd_buf.push(0x01).unwrap();
        self.e220_port
            .write_all(self.snd_buf.as_ref())
            .await
            .map_err(|e| Error::E220WriteErr)?;

        // в ответ должны прийти 4 байта или ошибка FF FF FF - 3 байта
        let b1 = self.read_byte(Duration::from_millis(200)).await?;
        if b1 != (E220Command::ReadRegister as u8) {
            return Err(Error::WrongResp);
        }
        let b2 = self.read_byte(Duration::from_millis(200)).await?;
        if b2 != (reg as u8) {
            return Err(Error::WrongResp);
        }
        let b3 = self.read_byte(Duration::from_millis(200)).await?;
        if b3 != 0x01 {
            return Err(Error::WrongResp);
        }
        let b4 = self.read_byte(Duration::from_millis(200)).await?;
        Ok(b4)
    }

    // запись регистра модуля
    // должен быть режим E220Mode::ModeDeepSleep !!!
    pub async fn set_register(
        &mut self,
        method: E220Command,
        reg: E220Registers,
        to: u8,
    ) -> Result<(), Error> {
        if method == E220Command::ReadRegister {
            return Err(Error::WrongCommand);
        }
        // если значение верное, то не пишем
        let old = self.read_register(reg).await?;
        // старые значения для регистров шифрования всегда 0, для них форсим перезапись
        if (old == to) && !((reg == E220Registers::CRYPT_H) || (reg == E220Registers::CRYPT_L)) {
            return Ok(());
        }

        self.aux_wait().await;
        self.rcv_buf.clear();
        self.snd_buf.clear();
        self.empty_rcv().await;

        // отправка запроса на запись регистра
        self.snd_buf.push(method as u8).unwrap();
        self.snd_buf.push(reg as u8).unwrap();
        self.snd_buf.push(0x01).unwrap();
        self.snd_buf.push(to).unwrap();
        self.e220_port
            .write_all(self.snd_buf.as_ref())
            .await
            .map_err(|e| Error::E220WriteErr)?;

        // в ответ должны прийти 4 байта или ошибка FF FF FF - 3 байта
        let b1 = self.read_byte(Duration::from_millis(200)).await?;
        if b1 != (E220Command::ReadRegister as u8) {
            return Err(Error::WrongResp);
        }
        let b2 = self.read_byte(Duration::from_millis(200)).await?;
        if b2 != (reg as u8) {
            return Err(Error::WrongResp);
        }
        let b3 = self.read_byte(Duration::from_millis(200)).await?;
        if b3 != 0x01 {
            return Err(Error::WrongResp);
        }
        let b4 = self.read_byte(Duration::from_millis(200)).await?;
        if b4 != to {
            if ((reg == E220Registers::CRYPT_H) || (reg == E220Registers::CRYPT_L)) && b4 == 0 {
                return Ok(());
            }
            return Err(Error::WrongResp);
        }
        Ok(())
    }

    // настройка параметров модуля после запуска
    // используем прямую адресацию, возврат RSSI, измерение уровня шкма в канале
    // мощность самая низкая
    pub async fn module_init(&mut self, addh: u8, addl: u8) -> Result<(), Error> {
        self.set_mode(E220Mode::ModeDeepSleep).await;
        // настройка адреса ADDH
        self.set_register(E220Command::SetRegister, E220Registers::ADDH, addh)
            .await?;
        // настройка адреса ADDL
        self.set_register(E220Command::SetRegister, E220Registers::ADDL, addl)
            .await?;
        // настройка REG0
        let reg0 = (E220SerialRate::default() as u8)
            | (E220SerialParity::default() as u8)
            | (E220AirRate::default() as u8);
        self.set_register(E220Command::SetRegister, E220Registers::REG0, reg0)
            .await?;
        // настройка REG1
        let reg1 =
            (E220SubPacket::Bytes128 as u8) | (E220Noise::Enable as u8) | (E220Power::Dbm17 as u8);
        self.set_register(E220Command::SetRegister, E220Registers::REG1, reg1)
            .await?;
        // настройка REG2
        self.set_register(E220Command::SetRegister, E220Registers::REG2, RADIO_CHANNEL)
            .await?;
        // настройка REG3
        let reg3 = (E220Transmission::Fixed as u8)
            | (E220Rssi::Enable as u8)
            | (E220WOR::Ms500 as u8)
            | (E220LBT::default() as u8);
        self.set_register(E220Command::SetRegister, E220Registers::REG3, reg3)
            .await?;
        // настройка ключей шифрования
        self.set_register( E220Command::SetRegister, E220Registers::CRYPT_H, CRYPT_H).await?;
        self.set_register( E220Command::SetRegister, E220Registers::CRYPT_L, CRYPT_L).await?;
        Ok(())
    }

    // получение уровня шума в канале связи в dBm
    // работает только в нормальном режиме при установке E220Noise::Enable
    pub async fn get_noise_dbm(&mut self) -> Result<u8, Error> {
        self.aux_wait().await;
        self.rcv_buf.clear();
        self.snd_buf.clear();
        self.empty_rcv().await;

        // отправка запроса на чтение Noise level RSSI
        self.snd_buf.push(0xC0).unwrap();
        self.snd_buf.push(0xC1).unwrap();
        self.snd_buf.push(0xC2).unwrap();
        self.snd_buf.push(0xC3).unwrap();
        self.snd_buf.push(0x00).unwrap();
        self.snd_buf.push(0x01).unwrap();
        self.e220_port
            .write_all(self.snd_buf.as_ref())
            .await
            .map_err(|e| Error::E220WriteErr)?;

        // в ответ должны прийти 4 байта
        //let b3 = self.read_byte(Duration::from_millis(200)).await?;
        let b1 = self.read_byte(Duration::from_millis(5000)).await?;
        if b1 != 0xC1 {
            return Err(Error::WrongResp);
        }
        let b2 = self.read_byte(Duration::from_millis(200)).await?;
        if b2 != 0x00 {
            return Err(Error::WrongResp);
        }
        let b3 = self.read_byte(Duration::from_millis(200)).await?;
        if b3 != 0x01 {
            return Err(Error::WrongResp);
        }
        let b4 = self.read_byte(Duration::from_millis(200)).await?;
        // let rssi = -((256_u16 - (b4 as u16)) as i16);
        Ok(b4)
    }

    // пробуем за несколько попыток получить уровень шума - за одну сбоит
    pub async fn try_get_noise_dbm(&mut self, attempts: u8) -> Result<u8, Error> {
        for _ in 0..attempts {
            match self.get_noise_dbm().await {
                Ok(x) => return Ok(x),
                Err(_) => {}
            };
            Timer::after_millis(1000).await; // delay between attempts
        }
        Err(Error::AttemptsOvf)
    }

    // отправка данных в канал из буфера
    pub async fn send_packet(&mut self, addh: u8, addl: u8, msg: &[u8]) -> Result<(), Error> {
        // ожидание готовности к передаче
        self.aux_wait().await;

        // отправка адреса получателя пакета
        self.snd_buf.clear();
        self.snd_buf.push(addh as u8).unwrap();
        self.snd_buf.push(addl as u8).unwrap();
        self.snd_buf.push(RADIO_CHANNEL).unwrap();

        self.e220_port
            .write_all(self.snd_buf.as_ref())
            .await
            .map_err(|e| Error::E220WriteErr)?;

        // отправка тела пакета
        self.e220_port
            .write_all(msg)
            .await
            .map_err(|e| Error::E220WriteErr)?;

        // ожидание окончания передачи в эфир
        self.aux_wait().await;
        Ok(())
    }

    // приём пакета
    pub async fn get_packet(&mut self) -> Result<(), Error> {
        // подготовка буферов - надеемся, что пока чистим входной буфер  не начнет поступать пакет
        self.rcv_buf.clear();
        self.aux_wait().await;
        self.empty_rcv().await;

        // ожидаем переход AUX в ноль - предвестник потока даных
        loop {
            if !self.aux_ready() {
                break;
            }
            embassy_futures::yield_now().await;
        }

        const RX_TO: Duration = Duration::from_millis(50);
        let rx_timeout = RX_TO;

        // к этому моменту через 2 мс после AUX LOW должны пойти данные
        loop {
            let mut read_buf: [u8; 1] = [0; 1]; // Читаем по одному символу
            let res = with_timeout(rx_timeout, self.e220_port.read(&mut read_buf)).await;
            match res {
                Err(_) => {
                    if self.aux_ready() {
                        // закончили приём команды по таймауту и если AUX перешёл вверх, до данных более не будет
                        //info!("timeout get byte");
                        break;
                    }
                }
                Ok(rxres) => {
                    match rxres {
                        Err(_) => {
                            //info!("WTF get byte");
                            //WTF?
                            break;
                        }
                        Ok(x) => {
                            // защита от переполнения буфера
                            if self.rcv_buf.len() < 205 && x == 1 {
                                self.rcv_buf.push(read_buf[0]).unwrap();
                            } else {
                                // закончили приём команды по переполнению буфера
                                break;
                            }
                        }
                    }
                }
            }
        }

        Ok(())
    }
}

pub const fn rssi_to_dbm(rssi: u8) -> i16 {
    -((256_u16 - (rssi as u16)) as i16)
}
