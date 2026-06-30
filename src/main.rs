#![no_std]
#![no_main]

mod errors;
//mod pelcod;
//mod ptzdriver;
mod e220;
mod easycom;
mod easycom_processor;
mod manual;
mod protocol;
mod relays;
mod renderer;

use core::cell::RefCell;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::Peri;
use embassy_stm32::adc::{
    Adc, AdcChannel, CONTINUOUS, Exten, RingBufferedAdc, SampleTime, Temperature, VrefInt,
};
use embassy_stm32::gpio::{Input, Level, Output, Speed};
use embassy_stm32::peripherals::{PA0, PA1, PC13};
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllRDiv,
    PllSource, Sysclk,
};
use embassy_stm32::spi;
use embassy_stm32::time::mhz;
use embassy_stm32::usart::BufferedUart;
use embassy_stm32::{Config, bind_interrupts, dma, peripherals, usart};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::{Mutex, MutexGuard};
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_io_async::{Read, Write};

use {defmt_rtt as _, panic_probe as _};

use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X12},
    pixelcolor::{Rgb565, raw::ToBytes},
    prelude::*,
    primitives::{Circle, PrimitiveStyle, Rectangle, Triangle},
    text::Text,
};
//use embedded_graphics_core::prelude::*;
//use embedded_graphics_core::pixelcolor::Rgb565;

//use ssd1331_async::{BitDepth, Framebuffer, Ssd1331, WritePixels};
use core::fmt::Write as WriteCore;
use heapless::{Vec, format};
use oorandom;
use sky_ili9341::{
    AsyncBuilder, AsyncDisplay, AsyncInterface, AsyncSpiInterface, Orientation, presets,
};
use static_cell::{ConstStaticCell, StaticCell};

use errors::Error;
//use pelcod::*;
//use ptzdriver::*;
use e220::*;
use easycom::*;
use easycom_processor::*;
use manual::*;
use protocol::*;
use relays::*;
use renderer::*;

// адрес модуля - для компиляции сервера =1
// для исполнительных модулей 2, 3 ...
const MODULE_ADDRESS_H: u8 = 78;
const MODULE_ADDRESS_L: u8 = 6;
const SLAVE_ADDRESS_H: u8 = 44;
const SLAVE_ADDRESS_L: u8 = 79;
const CRYPT_H: u8 = 50;
const CRYPT_L: u8 = 19;

// состояние приложения для обмена между потоками
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct AppContext {
    target_az: AzAngle,
    target_el: ElAngle,
    current_az: AzAngle,
    current_el: ElAngle,
    seq: u32,
    noise_m: u8,
    signal_m: u8,
    noise_s: u8,
    signal_s: u8,
    power_control: Relays,
    az_pot: AzAngle,
    el_pot: ElAngle,
    manual: bool,
}

static APP_CONTEXT: Mutex<CriticalSectionRawMutex, RefCell<AppContext>> =
    Mutex::new(RefCell::new(AppContext {
        target_az: AzAngle(0.0),
        target_el: ElAngle(0.0),
        current_az: AzAngle(0.0),
        current_el: ElAngle(0.0),
        seq: 0,
        noise_m: 0,
        signal_m: 0,
        noise_s: 0,
        signal_s: 0,
        power_control: Relays::new(),
        az_pot: AzAngle(0.0),
        el_pot: ElAngle(0.0),
        manual: false,
    }));

static LCD_REDRAW: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// таск для того, чтоб чип не засыпал и номально прошивался без ресет
#[embassy_executor::task]
async fn idle() {
    loop {
        embassy_futures::yield_now().await;
    }
}

// таск мигания светодиодом
#[embassy_executor::task]
async fn blinky(led: Peri<'static, PC13>) {
    let mut led = Output::new(led, Level::High, Speed::Low);
    loop {
        led.toggle();
        Timer::after_millis(300).await;
    }
}

bind_interrupts!(struct Irqs {
    DMA2_STREAM2 => dma::InterruptHandler<peripherals::DMA2_CH2>;
//    DMA2_STREAM3 => dma::InterruptHandler<peripherals::DMA2_CH3>;
});

bind_interrupts!(struct IrqsAdc {
    DMA2_STREAM0 => dma::InterruptHandler<peripherals::DMA2_CH0>;
});

bind_interrupts!(struct IrqsUART2 {
    USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
});

bind_interrupts!(struct IrqsUART1 {
    USART1 => usart::BufferedInterruptHandler<peripherals::USART1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // настраиваем тактирование всего
    let mut config = embassy_stm32::Config::default();
    // Configure HSE (25 MHz Crystal)
    config.rcc.hse = Some(Hse {
        freq: mhz(25),
        mode: HseMode::Oscillator,
    });

    // Configure PLL for 96 MHz System Clock
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV25,  // PLLM 25MHz / 25 = 1 MHz
        mul: PllMul::MUL384,       // PLLN 1MHz * 384 = 384 MHz (VCO)
        divp: Some(PllPDiv::DIV4), // 384MHz / 4 = 96 MHz (SYSCLK)
        divq: Some(PllQDiv::DIV8), // 384MHz / 8 = 48 MHz (USB CLK)
        divr: None,
    });

    // Set System Clock Source
    config.rcc.sys = Sysclk::PLL1_P;

    // Bus Prescalers (Critical for Stability)
    // AHB  = 96 MHz (Max 100)
    // APB1 = 48 MHz  (Max 50) -> Must be DIV2 or higher
    // APB2 = 96 MHz (Max 100)
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV1;

    let p = embassy_stm32::init(config);

    info!("Starting...");

    // ili9341 дисплей
    let mut rst = Output::new(p.PB0, Level::Low, Speed::VeryHigh);
    let mut display = {
        // настройка SPI
        let mut spi_config = spi::Config::default();
        spi_config.frequency = embassy_stm32::time::mhz(10);
        let spi_bus = spi::Spi::new_txonly(p.SPI1, p.PA5, p.PA7, p.DMA2_CH2, Irqs, spi_config);
        let cs = Output::new(p.PA4, Level::Low, Speed::VeryHigh);
        let spi_dev = ExclusiveDevice::new_no_delay(spi_bus, cs).unwrap();

        let dc = Output::new(p.PB1, Level::Low, Speed::VeryHigh);

        let di = AsyncSpiInterface::new(spi_dev, dc);
        AsyncBuilder::new(di)
            //            .orientation(Orientation::Portrait)
            .orientation(Orientation::Landscape)
            .init(&mut rst, &mut Delay {})
            .await
            .unwrap()
    };

    // черный фон
    display.clear_screen(0x0000).await.unwrap();

    // USART2 - порт для приёма команд easycom
    let usart2 = {
        let mut config_usart2 = usart::Config::default();
        config_usart2.baudrate = 9600;
        static TX_BUF2: StaticCell<[u8; 32]> = StaticCell::new();
        let tx_buf2 = &mut TX_BUF2.init([0; 32])[..];
        static RX_BUF2: StaticCell<[u8; 32]> = StaticCell::new();
        let rx_buf2 = &mut RX_BUF2.init([0; 32])[..];
        BufferedUart::new(
            p.USART2,
            p.PA3,
            p.PA2,
            tx_buf2,
            rx_buf2,
            IrqsUART2,
            config_usart2,
        )
        .unwrap()
    };

    // USART1 - порт для передачи команд e220
    let usart1 = {
        let mut config_usart1 = usart::Config::default();
        config_usart1.baudrate = 9600;
        static TX_BUF1: StaticCell<[u8; 32]> = StaticCell::new();
        let tx_buf1 = &mut TX_BUF1.init([0; 32])[..];
        static RX_BUF1: StaticCell<[u8; 32]> = StaticCell::new();
        let rx_buf1 = &mut RX_BUF1.init([0; 32])[..];
        BufferedUart::new(
            p.USART1,
            p.PB7,
            p.PB6,
            tx_buf1,
            rx_buf1,
            IrqsUART1,
            config_usart1,
        )
        .unwrap()
    };
    /*
    PB4 - M0
    PB3 - M1
    PA15 - AUX
     */
    let m0 = Output::new(p.PB4, Level::Low, Speed::VeryHigh);
    let m1 = Output::new(p.PB3, Level::Low, Speed::VeryHigh);
    let aux = Input::new(p.PA15, embassy_stm32::gpio::Pull::None);

    // входы для кнопок включение питания
    let ptz_pwr = Input::new(p.PB12, embassy_stm32::gpio::Pull::None);
    let lna_pwr = Input::new(p.PB13, embassy_stm32::gpio::Pull::None);

    // генерация случайного числа с помощью шума АЦП
    let mut adc = Adc::new_with_config(p.ADC1, Default::default());
    let mut temp = adc.enable_temperature();
    let mut initial_seed: u64 = 0;
    for _ in 0..64 {
        let adc_val = adc.blocking_read(&mut temp, SampleTime::CYCLES112);
        let first_bit = (adc_val & 0x01) as u64; // берем младший "шумный" бит
        initial_seed = (initial_seed << 1) | first_bit;
    }
    let mut rng = oorandom::Rand32::new(initial_seed);
    let random_start = rng.rand_u32() >> 1;
    info!("New seed is: {} New seq is {}", initial_seed, random_start);

    let manual = Input::new(p.PB14, embassy_stm32::gpio::Pull::None);

    spawner.spawn(idle().unwrap()); // бесконечный цикл для предотвращения сна
    spawner.spawn(blinky(p.PC13).unwrap()); // мигание светодиодом
    spawner.spawn(process_easycom(usart2).unwrap()); // управление повороткой
    spawner.spawn(process_e220(usart1, m0, m1, aux, random_start, ptz_pwr, lna_pwr).unwrap()); // работа в эфире
    spawner.spawn(process_adc(adc, p.DMA2_CH0, p.PA0, p.PA1, manual).unwrap()); // потенциометры

    // Use the first bytes of the static buffer to render text
    let pixel_data = PIXEL_DATA.take();
    let fh = 24;
    let fw = 12;
    //let font = TextRenderer::new(include_bytes!("./font_6x12.bin"), Size::new(6, 12));
    let font = TextRenderer::new(include_bytes!("./spleen12x24.bin"), Size::new(12, 24));
    //let start = Instant::now();

    font.render_text(
        "R2AJP PTZ Controller",
        Point::new(36, 0),
        Rgb565::CYAN,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;
    font.render_text(
        "PTZ AZ:",
        Point::new(0, fh),
        Rgb565::WHITE,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;
    font.render_text(
        "PTZ EL:",
        Point::new(0, fh * 2),
        Rgb565::WHITE,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;
    font.render_text(
        "REQ AZ:",
        Point::new(0, fh * 3),
        Rgb565::WHITE,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;
    font.render_text(
        "REQ EL:",
        Point::new(0, fh * 4),
        Rgb565::WHITE,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;
    font.render_text(
        "Signal M dBm:",
        Point::new(0, fh * 5),
        Rgb565::WHITE,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;
    font.render_text(
        "Noise M dBm:",
        Point::new(0, fh * 6),
        Rgb565::WHITE,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;
    font.render_text(
        "Signal S dBm:",
        Point::new(0, fh * 7),
        Rgb565::WHITE,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;
    font.render_text(
        "Noise S dBm:",
        Point::new(0, fh * 8),
        Rgb565::WHITE,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;
    font.render_text(
        "SEQ:",
        Point::new(0, fh * 9),
        Rgb565::WHITE,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;
    font.render_text(
        "PTZ Pwr:",
        Point::new(fw * 14, fh),
        Rgb565::WHITE,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;
    font.render_text(
        "LNA Pwr:",
        Point::new(fw * 14, fh * 2),
        Rgb565::WHITE,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;

    {
        let mut ctx = APP_CONTEXT.lock().await;
        let mut inner = ctx.borrow_mut();
        inner.target_az = AzAngle::try_from(100.0).unwrap();
        inner.target_el = ElAngle::try_from(45.0).unwrap();
    }

    let mut az2print = AzAngle(0.0);
    let mut el2print = ElAngle(0.0);
    let mut azr2print = AzAngle(0.0);
    let mut elr2print = ElAngle(0.0);
    let mut seq2print = 0;
    let mut noise_m2print = 0;
    let mut signal_m2print = 0;
    let mut noise_s2print = 0;
    let mut signal_s2print = 0;
    let mut relay2print = Relays::new();
    let mut adc_az2print = AzAngle(0.0);
    let mut adc_el2print = ElAngle(0.0);
    let mut manual2print = false;

    loop {
        //Timer::after_millis(500).await;

        //const ERR_ANS: &[u8] = b"?\n";
        //usart2.write_all(ERR_ANS).await.ok();
        //embassy_futures::yield_now().await;
        /*info!("high");
         led.set_high();
        Timer::after_millis(300).await;

        info!("low");
        led.set_low();
        Timer::after_millis(300).await;*/
        {
            let ctx = APP_CONTEXT.lock().await;
            let inner = ctx.borrow();
            az2print = inner.current_az;
            el2print = inner.current_el;
            azr2print = inner.target_az;
            elr2print = inner.target_el;
            seq2print = inner.seq;
            noise_m2print = rssi_to_dbm(inner.noise_m);
            signal_m2print = rssi_to_dbm(inner.signal_m);
            noise_s2print = rssi_to_dbm(inner.noise_s);
            signal_s2print = rssi_to_dbm(inner.signal_s);
            relay2print = inner.power_control;
            adc_az2print = inner.az_pot;
            adc_el2print = inner.el_pot;
            manual2print = inner.manual;
        }
        let az_str = format!(6;"{:03}", az2print).unwrap();
        font.render_text(
            &az_str,
            Point::new(fw * 8, fh),
            Rgb565::GREEN,
            Rgb565::BLACK,
            pixel_data,
            &mut display,
        )
        .await;
        let el_str = format!(6;"{:02}", el2print).unwrap();
        font.render_text(
            &el_str,
            Point::new(fw * 8, fh * 2),
            Rgb565::GREEN,
            Rgb565::BLACK,
            pixel_data,
            &mut display,
        )
        .await;
        let azr_str = format!(6;"{:03}", azr2print).unwrap();
        font.render_text(
            &azr_str,
            Point::new(fw * 8, fh * 3),
            Rgb565::GREEN,
            Rgb565::BLACK,
            pixel_data,
            &mut display,
        )
        .await;
        let elr_str = format!(6;"{:02}", elr2print).unwrap();
        font.render_text(
            &elr_str,
            Point::new(fw * 8, fh * 4),
            Rgb565::GREEN,
            Rgb565::BLACK,
            pixel_data,
            &mut display,
        )
        .await;
        let signal_m_str = format!(6;"{:04}", signal_m2print).unwrap();
        font.render_text(
            &signal_m_str,
            Point::new(fw * 14, fh * 5),
            Rgb565::GREEN,
            Rgb565::BLACK,
            pixel_data,
            &mut display,
        )
        .await;
        let noise_m_str = format!(6;"{:04}", noise_m2print).unwrap();
        font.render_text(
            &noise_m_str,
            Point::new(fw * 14, fh * 6),
            Rgb565::BLUE,
            Rgb565::BLACK,
            pixel_data,
            &mut display,
        )
        .await;
        let signal_s_str = format!(6;"{:04}", signal_s2print).unwrap();
        font.render_text(
            &signal_s_str,
            Point::new(fw * 14, fh * 7),
            Rgb565::GREEN,
            Rgb565::BLACK,
            pixel_data,
            &mut display,
        )
        .await;
        let noise_s_str = format!(6;"{:04}", noise_s2print).unwrap();
        font.render_text(
            &noise_s_str,
            Point::new(fw * 14, fh * 8),
            Rgb565::BLUE,
            Rgb565::BLACK,
            pixel_data,
            &mut display,
        )
        .await;
        let seq_str = format!(16;"{}", seq2print).unwrap();
        font.render_text(
            &seq_str,
            Point::new(fw * 5, fh * 9),
            Rgb565::GREEN,
            Rgb565::BLACK,
            pixel_data,
            &mut display,
        )
        .await;

        let adc1_str = format!(16;"{:03} ", adc_az2print).unwrap();
        font.render_text(
            &adc1_str,
            Point::new(fw * 14, fh * 3),
            Rgb565::GREEN,
            Rgb565::BLACK,
            pixel_data,
            &mut display,
        )
        .await;
        let adc2_str = format!(16;"{:02} ", adc_el2print).unwrap();
        font.render_text(
            &adc2_str,
            Point::new(fw * 14, fh * 4),
            Rgb565::GREEN,
            Rgb565::BLACK,
            pixel_data,
            &mut display,
        )
        .await;

        if relay2print.is_ptz_on() {
            font.render_text(
                "ON ",
                Point::new(fw * 14 + 108, fh),
                Rgb565::BLUE,
                Rgb565::BLACK,
                pixel_data,
                &mut display,
            )
            .await;
        } else {
            font.render_text(
                "OFF",
                Point::new(fw * 14 + 108, fh),
                Rgb565::RED,
                Rgb565::BLACK,
                pixel_data,
                &mut display,
            )
            .await;
        }
        if relay2print.is_lna_on() {
            font.render_text(
                "ON ",
                Point::new(fw * 14 + 108, fh * 2),
                Rgb565::BLUE,
                Rgb565::BLACK,
                pixel_data,
                &mut display,
            )
            .await;
        } else {
            font.render_text(
                "OFF",
                Point::new(fw * 14 + 108, fh * 2),
                Rgb565::RED,
                Rgb565::BLACK,
                pixel_data,
                &mut display,
            )
            .await;
        }

        if manual2print {
            font.render_text(
                "MANUAL",
                Point::new(fw * 14 + 72, fh * 3),
                Rgb565::WHITE,
                Rgb565::BLACK,
                pixel_data,
                &mut display,
            )
            .await;
        } else {
            font.render_text(
                "AUTO  ",
                Point::new(fw * 14 + 72, fh * 3),
                Rgb565::WHITE,
                Rgb565::BLACK,
                pixel_data,
                &mut display,
            )
            .await;
        }

        LCD_REDRAW.wait().await; // Wait for signal
        LCD_REDRAW.reset();
    }
}
