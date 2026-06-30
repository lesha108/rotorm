// ручное управление углами поворотки

use cortex_m::singleton;
use embassy_stm32::peripherals::{ADC1, DMA2_CH0};

use super::*;

// таск обработки команд e220 приёма и передачи в эфир
#[embassy_executor::task]
pub async fn process_adc(
    mut adcp: Adc<'static, ADC1>,
    mut dma: Peri<'static, DMA2_CH0>,
    mut az_pot_pin: Peri<'static, PA0>,
    mut el_pot_pin: Peri<'static, PA1>,
    manual: Input<'static>,
) {
    const ADC_BUF_SIZE: usize = 1024;
    let adc_data: &mut [u16; ADC_BUF_SIZE] =
        singleton!(ADCDAT : [u16; ADC_BUF_SIZE] = [0u16; ADC_BUF_SIZE]).unwrap();
    let mut adc: RingBufferedAdc<embassy_stm32::peripherals::ADC1> = adcp.into_ring_buffered(
        dma,
        adc_data,
        IrqsAdc,
        [
            (az_pot_pin.degrade_adc(), SampleTime::CYCLES112),
            (el_pot_pin.degrade_adc(), SampleTime::CYCLES112),
        ]
        .into_iter(),
        CONTINUOUS,
        Exten::DISABLED,
    );

    let mut az_adc = 0u32;
    let mut el_adc = 0u32;
    let mut last_az_adc = 0u32;
    let mut last_el_adc = 0u32;
    let mut last_manual = false;

    let mut buffer = [0u16; 512];
    adc.start();
    loop {
        if manual.is_high() {
            if !last_manual {
                // переводим в ручное управление
                let ctx = APP_CONTEXT.lock().await;
                let mut inner = ctx.borrow_mut();
                inner.manual = true;
                last_manual = true;
                LCD_REDRAW.signal(());
            }
        } else {
            if last_manual {
                // переводим в авто управление
                let ctx = APP_CONTEXT.lock().await;
                let mut inner = ctx.borrow_mut();
                inner.manual = false;
                last_manual = false;
                LCD_REDRAW.signal(());
            }
        }

        match adc.read(&mut buffer).await {
            Ok(_data) => {
                /*info!(
                    "\n adc1: {} n = {}",
                    buffer[0..16],
                    _data
                );*/
            }
            Err(e) => {
                warn!("ADC Error: {:?}", e);
                buffer = [0u16; 512];
                adc.start();
                continue;
            }
        }

        let mut sample_count = 1u32;
        for sample in &buffer {
            if sample_count % 2 == 0 {
                az_adc += *sample as u32;
            } else {
                el_adc += *sample as u32;
            }
            sample_count += 1;
        }
        let divider = sample_count / 2 + 1;
        az_adc /= divider;
        el_adc /= divider;

        // обновляем значения
        if az_adc != last_az_adc || el_adc != last_el_adc {
            let azf = map_float_range(az_adc as f32, 0.0, 4096.0, 1.0, 359.0);
            let elf = map_float_range(el_adc as f32, 0.0, 4096.0, 1.0, 89.0);
            let ctx = APP_CONTEXT.lock().await;
            let mut inner = ctx.borrow_mut();
            let a1 = &(inner.az_pot);
            let e1 = &(inner.el_pot);
            let azfiltered = <f32>::from(a1) * 0.995 + azf * 0.005;
            let elfiltered = <f32>::from(e1) * 0.995 + elf * 0.005;
            inner.az_pot = AzAngle::try_from(azfiltered).unwrap();
            inner.el_pot = ElAngle::try_from(elfiltered).unwrap();
            LCD_REDRAW.signal(());
        }
    }
}

fn map_float_range(value: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)
}
