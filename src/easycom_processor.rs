// обработка команд EASYCOM

use core::fmt::Write as WriteCore;
use defmt::*;

use embassy_stm32::usart::BufferedUart;
use embassy_time::{Duration, with_timeout};
use embedded_io_async::{Read, Write};

use heapless::Vec;

use crate::easycom::*;
use crate::APP_CONTEXT;

// таск обработки команд EASYCOM
#[embassy_executor::task]
pub async fn process_easycom(mut uart: BufferedUart<'static>) {
    // rcv buffer
    const RCV_BUF_LEN: usize = 128;
    let mut rcv_buf: Vec<u8, RCV_BUF_LEN> = Vec::new();
    let mut rcv_count_raw: usize = 0;

    const ERR_ANS: &[u8] = b"?\n";
    // ждем первую команду сколь угодно долго
    let mut rx_timeout = Duration::MAX;
    // окончание приема по 20 мс таймауту
    const RX_TO: Duration = Duration::from_millis(20);
    const RX_TO_MAX: Duration = Duration::from_secs(60 * 60 * 24 * 7);
    loop {
        rcv_buf.clear();
        rcv_count_raw = 0;
        rx_timeout = RX_TO_MAX;
        loop {
            let mut read_buf: [u8; 1] = [0; 1]; // Читаем по одному символу
            let res = with_timeout(rx_timeout, uart.read(&mut read_buf)).await;
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
                            // начали принимать символы команды, окончание приёма по таймауту 20 мс после последнего символа
                            rx_timeout = RX_TO;
                            // защита от переполнения буфера
                            if x > 0 && rcv_buf.len() < RCV_BUF_LEN {
                                rcv_buf.push(read_buf[0]).unwrap();
                            } else {
                                // закончили приём команды по переполнению буфера
                                break;
                            }
                        }
                    }
                }
            }
        }
        rcv_count_raw = rcv_buf.len();

        // Convert command to upper case
        if rcv_count_raw != 0 {
            for c in rcv_buf[0..rcv_count_raw].iter_mut() {
                if 0x61 <= *c && *c <= 0x7a {
                    *c &= !0x20;
                }
            }
        }
        // strange short input?
        if rcv_count_raw == 1 {
            uart.write_all(ERR_ANS).await.ok();
        }
        info!("RX {:?}", rcv_buf.as_slice());

        // at least 1 command supposed to be in buffer
        if rcv_count_raw > 1 {
            let fullslice = &rcv_buf[0..rcv_count_raw]; // make slice from actual number of chars in buffer - expect one or more commands there
            // make iterator of commands - may produce empty subslices
            // chars splitters are ASCII CR, LF, SPACE
            let slice_iter = fullslice.split(|num| num == &10 || num == &13 || num == &32);
            for supposed_command in slice_iter {
                // command string must be at least 2 chars
                if supposed_command.len() < 2 {
                    continue;
                }
                let rcv_count = supposed_command.len();

                // protocol has 2 letter commands
                let possible_command = EasycomCommands::try_from(&supposed_command[0..2]);
                match possible_command {
                    Ok(cmd) => {
                        match cmd {
                            EasycomCommands::VE => {
                                uart.write_all(EASYCOM_PROTOCOL_VERSION).await.ok();
                                info!("VE");
                            }
                            EasycomCommands::AZ => {
                                match rcv_count {
                                    // simple query of angle
                                    2 => {
                                        let azf: f32 = {
                                            let ctx = APP_CONTEXT.lock().await;
                                            let inner = ctx.borrow();
                                            (&inner.current_az).into()
                                        };
                                        let mut line: Vec<u8, 50> = Vec::new();
                                        core::write!(line, "+{0:.1}\n", azf).unwrap();
                                        //write!(line, "+181.1 56.7\n").unwrap();
                                        uart.write_all(line.as_slice()).await.ok();
                                        info!("AZ?");
                                    } // angle format is X.X, XX.X, XXX.X
                                    5..=7 => {
                                        let azp: AzAngle = Default::default();
                                        match azp.from_degrees(&supposed_command[2..rcv_count]) {
                                            Ok(ang) => {
                                                let ctx = APP_CONTEXT.lock().await;
                                                let mut inner = ctx.borrow_mut();
                                                inner.target_az = ang;
                                                info!("AZ set");
                                            }
                                            Err(_) => {
                                                uart.write_all(ERR_ANS).await.ok();
                                                info!("AZ error");
                                            }
                                        }
                                    }
                                    _ => {
                                        // requested angle not identified
                                        uart.write_all(ERR_ANS).await.ok();
                                    }
                                }
                            }
                            EasycomCommands::EL => {
                                match rcv_count {
                                    // simple query of angle
                                    2 => {
                                        //let elf: f32 = el.into();
                                        let elf: f32 = {
                                            let ctx = APP_CONTEXT.lock().await;
                                            let inner = ctx.borrow();
                                            (&inner.current_el).into()
                                        };
                                        let mut line: Vec<u8, 10> = Vec::new();
                                        core::write!(line, "+{0:.1}\n", elf).unwrap();
                                        uart.write_all(line.as_slice()).await.ok();
                                        info!("EL?");
                                    } // angle format is X.X, XX.X, XXX.X
                                    5..=7 => {
                                        let elp: ElAngle = Default::default();
                                        match elp.from_degrees(&supposed_command[2..rcv_count]) {
                                            Ok(ang) => {
                                                let ctx = APP_CONTEXT.lock().await;
                                                let mut inner = ctx.borrow_mut();
                                                inner.target_el = ang;
                                                info!("EL set");
                                            }
                                            Err(_) => {
                                                uart.write_all(ERR_ANS).await.ok();
                                                info!("EL error");
                                            }
                                        }
                                    }
                                    _ => {
                                        // requested angle not identified
                                        uart.write_all(ERR_ANS).await.ok();
                                    }
                                }
                            }
                        }
                    }
                    // 2 letter command not identified or supported
                    Err(_) => {
                        uart.write_all(ERR_ANS).await.ok();
                        info!("CMD error");
                    }
                }
            }
        }
    }
}
