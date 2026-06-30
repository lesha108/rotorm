//use core::sync::atomic::{AtomicU16, Ordering};
use embassy_stm32::usart::BufferedUart;
use embassy_time::with_timeout;
use embedded_io_async::Read;

use heapless::Vec;
//use serialport::{ClearBuffer, SerialPort};
//use std::io::{Read, Write};

use super::*;
use crate::pelcod::*;

// точность установки 5 градус
pub const AZ_TOLERANCE: u16 = 500;
pub const EL_TOLERANCE: u16 = 400;

// фиксированные скорости
pub const MAX_AZ_SPEED: u8 = 0x20;
pub const MAX_EL_SPEED: u8 = 0x3F;

const SPEED_TRESHOLD_HIGH: u16 = 2500;
const SPEED_TRESHOLD_LOW: u16 = 1000;

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct PTZState {
    target_az: AzAngle,
    target_el: ElAngle,
    current_az: AzAngle,
    current_el: ElAngle,
    pan_speed: PelcoSpeed,
    tilt_speed: PelcoSpeed,
}

impl PTZState {
    pub async fn new() -> PTZState {
        let ctx = APP_CONTEXT.lock().await;
        let inner = ctx.borrow();

        let pan_speed = PelcoSpeed::try_from(MAX_AZ_SPEED).unwrap();
        let tilt_speed = PelcoSpeed::try_from(MAX_EL_SPEED).unwrap();
        PTZState {
            target_az: inner.target_az,
            target_el: inner.target_el,
            current_az: inner.current_az,
            current_el: inner.current_el,
            pan_speed: pan_speed,
            tilt_speed: tilt_speed,
        }
    }

    pub async fn load_angles(&mut self) {
        let ctx = APP_CONTEXT.lock().await;
        let inner = ctx.borrow();

        self.target_az = inner.target_az;
        self.target_el = inner.target_el;
    }

    pub async fn store_angles(&self) {
        let ctx = APP_CONTEXT.lock().await;
        let mut inner = ctx.borrow_mut();
        inner.current_az = self.current_az;
        inner.current_el = self.current_el;
    }

    pub fn should_pan_right(&self) -> bool {
        self.target_az > self.current_az
            && ((*self.current_az.as_ref()).abs_diff(*self.target_az.as_ref())) > AZ_TOLERANCE
    }

    pub fn should_pan_left(&self) -> bool {
        self.target_az < self.current_az
            && ((*self.current_az.as_ref()).abs_diff(*self.target_az.as_ref())) > AZ_TOLERANCE
    }

    pub fn should_tilt_up(&self) -> bool {
        self.target_el > self.current_el
            && ((*self.current_el.as_ref()).abs_diff(*self.target_el.as_ref())) > EL_TOLERANCE
    }

    pub fn should_tilt_down(&self) -> bool {
        self.target_el < self.current_el
            && ((*self.current_el.as_ref()).abs_diff(*self.target_el.as_ref())) > EL_TOLERANCE
    }

    pub fn amend_az_speed(&self) -> PelcoSpeed {
        let diff = (*self.current_az.as_ref()).abs_diff(*self.target_az.as_ref());
        if diff < SPEED_TRESHOLD_LOW {
            self.pan_speed.half().half()
        } else if diff < SPEED_TRESHOLD_HIGH {
            self.pan_speed.half()
        } else {
            self.pan_speed.clone()
        }
    }

    pub fn amend_el_speed(&self) -> PelcoSpeed {
        let diff = (*self.current_el.as_ref()).abs_diff(*self.target_el.as_ref());
        if diff < SPEED_TRESHOLD_HIGH {
            self.tilt_speed.half()
        } else {
            self.tilt_speed.clone()
        }
    }
}

//#[derive(Debug)]
pub struct PTZDriver {
    port: BufferedUart<'static>,
    pelcoid: u8,
    state: PTZState,
    response_buf: Vec<u8, COMMAND_SIZE>, // буфер ответа PELCO-D от поворотки
}

impl PTZDriver {
    pub async fn new(id: u8, port: BufferedUart<'static>) -> Self {
        let state = PTZState::new().await;
        PTZDriver {
            port: port,
            pelcoid: id,
            state: state,
            response_buf: Vec::new(),
        }
    }

    async fn drain_buffered_uart(&mut self) {
        let mut buf = [0u8; 1];
        let timeout = Duration::from_micros(100); // Shorter than inter-byte time

        loop {
            let res = with_timeout(timeout, self.port.read(&mut buf)).await;
            match res {
                Err(TimeoutError) => {
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

    // основной 1 цикл взаимодействия с повороткой
    pub async fn runner(&mut self) -> Result<(), Error> {
        self.state.load_angles().await;
        /*
        1. Запрос поворота
        2. Запрос элевации
        3. обновление состояния
        4. выбор действия
        5. отправка команды действия
         */
        // перед запросом очищаем мусор на входе
        self.drain_buffered_uart().await;

        self.query_pan_position().await?;
        self.read_ptz().await?;

        self.query_tilt_position().await?;
        self.read_ptz().await?;

        // выбор функции движения

        if self.state.should_pan_right() && self.state.should_tilt_up() {
            self.pan_right_tilt_up().await?;
            self.read_ptz().await?;
        } else if self.state.should_pan_right() && self.state.should_tilt_down() {
            self.pan_right_tilt_down().await?;
            self.read_ptz().await?;
        } else if self.state.should_pan_left() && self.state.should_tilt_up() {
            self.pan_left_tilt_up().await?;
            self.read_ptz().await?;
        } else if self.state.should_pan_left() && self.state.should_tilt_down() {
            self.pan_left_tilt_down().await?;
            self.read_ptz().await?;
        } else if self.state.should_pan_right() {
            self.pan_right().await?;
            self.read_ptz().await?;
        } else if self.state.should_pan_left() {
            self.pan_left().await?;
            self.read_ptz().await?;
        } else if self.state.should_tilt_up() {
            self.tilt_up().await?;
            self.read_ptz().await?;
        } else if self.state.should_tilt_down() {
            self.tilt_down().await?;
            self.read_ptz().await?;
        } else {
            self.stop_pan().await?;
            self.read_ptz().await?;

            self.set_pan().await?;
            self.read_ptz().await?;

            self.set_tilt().await?;
            self.read_ptz().await?;
        }

        Ok(())
    }

    pub async fn send_cmd(&mut self, cmd: &Command) -> Result<(), Error> {
        // Write the bytes
        self.port
            .write_all(cmd.as_ref())
            .await
            .map_err(|e| Error::PTZWriteErr)?;
        //self.port.flush().map_err(|e| Error::PTZWriteErr)?;
        Ok(())
    }

    pub async fn read_ptz(&mut self) -> Result<(), Error> {
        /*
        1. начать ждать 2500 мс - errtimeout????
        2. принимать по символу до 7 символов или таймаут 50 мс
        3. 4 символа - просто ок
        4. 7 символов. Проверки команды - errptz reply
        5. если проверки прошли, то извлекаем углы по коду ответа и заносим в статики
        */
        self.response_buf.clear();

        const RX_TO: Duration = Duration::from_millis(50);
        const RX_TO_MAX: Duration = Duration::from_secs(3);

        let mut rx_timeout = RX_TO_MAX;
        loop {
            let mut read_buf: [u8; 1] = [0; 1]; // Читаем по одному символу
            let res = with_timeout(rx_timeout, self.port.read(&mut read_buf)).await;
            match res {
                Err(TimeoutError) => {
                    // закончили приём команды по таймауту
                    break;
                }
                Ok(rxres) => {
                    match rxres {
                        Err(_) => {
                            //WTF?
                            break;
                        }
                        Ok(x) => {
                            // начали принимать символы команды, окончание приёма по быстрому таймауту после последнего символа
                            rx_timeout = RX_TO;
                            // защита от переполнения буфера
                            if x > 0 && self.response_buf.len() < COMMAND_SIZE {
                                self.response_buf.push(read_buf[0]).unwrap();
                            } else {
                                // закончили приём команды по переполнению буфера
                                break;
                            }
                        }
                    }
                }
            }
        }

        // wait only for extended response. 4 byte standrd response - just ignore
        if self.response_buf.len() != COMMAND_SIZE {
            return Ok(());
        }

        // проверка корректности ответа
        if !(self.response_buf[0] == SYNC_BYTE
            && self.response_buf[1] == self.pelcoid
            && self.response_buf[2] == 0
            && (self.response_buf[3] == QResponse::QUERY_PAN_RESPONSE.bits()
                || self.response_buf[3] == QResponse::QUERY_TILT_RESPONSE.bits())
            && self.response_buf[6] == checksum(&self.response_buf[1..COMMAND_SIZE - 1]))
        {
            //let cs = checksum(&self.response_buf[1..COMMAND_SIZE-1]);
            //println!("Answer data: {:?}, cksm: {cs}", self.response_buf);
            return Err(Error::PTZDataErr);
        }

        let angle: u16 = u16::from_be_bytes([self.response_buf[4], self.response_buf[5]]);
        // тут после проверки могут быть только 2 варианта
        if self.response_buf[3] == QResponse::QUERY_PAN_RESPONSE.bits() {
            self.state.current_az = AzAngle::try_from(angle).unwrap_or_default();
        } else {
            self.state.current_el = ElAngle::try_from(angle).unwrap_or_default();
        }
        self.state.store_angles().await;
        // запрос перерисовки экрана - новые координаты пришли
        LCD_REDRAW.signal(());

        Ok(())
    }

    pub async fn query_pan_position(&mut self) -> Result<(), Error> {
        let cmd = Command::query_pan_position(self.pelcoid);
        self.send_cmd(&cmd).await?;
        Ok(())
    }

    pub async fn query_tilt_position(&mut self) -> Result<(), Error> {
        let cmd = Command::query_tilt_position(self.pelcoid);
        self.send_cmd(&cmd).await?;
        Ok(())
    }

    pub async fn pan_right(&mut self) -> Result<(), Error> {
        let ps = self.state.amend_az_speed();
        let cmd = Command::pan_right(self.pelcoid, &ps);
        //println!("speed {:?}", ps);
        self.send_cmd(&cmd).await?;
        Ok(())
    }

    pub async fn pan_left(&mut self) -> Result<(), Error> {
        let ps = self.state.amend_az_speed();
        let cmd = Command::pan_left(self.pelcoid, &ps);
        //println!("speed {:?}", ps);
        self.send_cmd(&cmd).await?;
        Ok(())
    }

    pub async fn tilt_up(&mut self) -> Result<(), Error> {
        let ts = self.state.amend_el_speed();
        let cmd = Command::tilt_up(self.pelcoid, &ts);
        self.send_cmd(&cmd).await?;
        Ok(())
    }

    pub async fn tilt_down(&mut self) -> Result<(), Error> {
        let ts = self.state.amend_el_speed();
        let cmd = Command::tilt_down(self.pelcoid, &ts);
        self.send_cmd(&cmd).await?;
        Ok(())
    }

    pub async fn pan_right_tilt_up(&mut self) -> Result<(), Error> {
        let ps = self.state.amend_az_speed();
        let ts = self.state.amend_el_speed();
        let cmd = Command::pan_right_tilt_up(self.pelcoid, &ps, &ts);
        self.send_cmd(&cmd).await?;
        Ok(())
    }

    pub async fn pan_right_tilt_down(&mut self) -> Result<(), Error> {
        let ps = self.state.amend_az_speed();
        let ts = self.state.amend_el_speed();
        let cmd = Command::pan_right_tilt_down(self.pelcoid, &ps, &ts);
        self.send_cmd(&cmd).await?;
        Ok(())
    }

    pub async fn pan_left_tilt_up(&mut self) -> Result<(), Error> {
        let ps = self.state.amend_az_speed();
        let ts = self.state.amend_el_speed();
        let cmd = Command::pan_left_tilt_up(self.pelcoid, &ps, &ts);
        self.send_cmd(&cmd).await?;
        Ok(())
    }

    pub async fn pan_left_tilt_down(&mut self) -> Result<(), Error> {
        let ps = self.state.amend_az_speed();
        let ts = self.state.amend_el_speed();
        let cmd = Command::pan_left_tilt_down(self.pelcoid, &ps, &ts);
        self.send_cmd(&cmd).await?;
        Ok(())
    }

    pub async fn stop_pan(&mut self) -> Result<(), Error> {
        let cmd = Command::pan_stop(self.pelcoid);
        self.send_cmd(&cmd).await?;
        Ok(())
    }

    pub async fn set_pan(&mut self) -> Result<(), Error> {
        let cmd = Command::set_pan(self.pelcoid, &self.state.target_az);
        self.send_cmd(&cmd).await?;
        Ok(())
    }

    pub async fn set_tilt(&mut self) -> Result<(), Error> {
        let cmd = Command::set_tilt(self.pelcoid, &self.state.target_el);
        self.send_cmd(&cmd).await?;
        Ok(())
    }

    /*pub fn stop_tilt(&mut self) -> Result<(), Error> {
        let cmd =
            Command::tilt_stop(self.pelcoid);
        self.send_cmd( &cmd)?;
        Ok(())
    }*/
}
