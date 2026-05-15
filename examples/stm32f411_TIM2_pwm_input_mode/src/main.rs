#![no_std]
#![no_main]

use panic_halt as _;

#[rtic::app(
    device = stm32f4xx_hal::pac,
    peripherals = true
)]
mod app {

    use defmt_rtt as _;

    use stm32f4xx_hal::{
        adc::{
            config::{
                AdcConfig, Continuous, Dma, ExternalTrigger, SampleTime, Scan, Sequence,
                TriggerMode,
            },
            Adc,
        },
        dma::{config::DmaConfig, PeripheralToMemory, Stream0, StreamsTuple, Transfer},
        gpio::{Output, PushPull},
        pac::{ADC1, DMA2},
        prelude::*,
        rcc::Config,
        timer::DelayUs,
    };

    // =====================================================
    // GLOBAL DMA BUFFER
    // =====================================================

    static mut ADC_BUFFER: [u16; 64] = [0; 64];

    #[shared]
    struct Shared {
        generated_duty: u8,
        increasing: bool,

        // kept alive so DMA remains active
        transfer: Transfer<Stream0<DMA2>, 0, Adc<ADC1>, PeripheralToMemory, &'static mut [u16; 64]>,
    }

    #[local]
    struct Local {
        led: stm32f4xx_hal::gpio::gpioc::PC13<Output<PushPull>>,
        delay: DelayUs<stm32f4xx_hal::pac::TIM3>,
        pwm_ch2: stm32f4xx_hal::timer::PwmChannel<stm32f4xx_hal::pac::TIM1, 1>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let dp = ctx.device;
        dp.RCC.apb1enr().modify(|_, w| w.tim2en().set_bit());

        // =====================================================
        // CLOCKS
        // =====================================================

        let rcc = dp.RCC.constrain();

        let mut rcc = rcc.freeze(Config::hse(25.MHz()).sysclk(84.MHz()));

        // =====================================================
        // GPIO
        // =====================================================

        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpioc = dp.GPIOC.split(&mut rcc);

        let led = gpioc.pc13.into_push_pull_output();

        // =====================================================
        // PWM OUTPUT
        // TIM1 CH2 -> PA9
        // =====================================================

        let pwm_pin = gpioa.pa9.into_alternate();

        let (_pwm_manager, channels) = dp.TIM1.pwm_us(1000.micros(), &mut rcc);

        let mut pwm_ch2 = channels.1.with(pwm_pin);

        pwm_ch2.enable();

        let max = pwm_ch2.get_max_duty();

        // 50%
        pwm_ch2.set_duty(max / 2);

        defmt::info!("PWM READY");

        // =====================================================
        // PWM INPUT
        // TIM2 CH1 -> PA0
        //
        // IMPORTANT:
        // PHYSICALLY CONNECT:
        //
        // PA9 ---> PA0
        // =====================================================

        let _capture_pin = gpioa.pa0.into_alternate::<1>();

        // =====================================================
        // TIM2 BASIC PERIODIC INTERRUPT TEST
        // =====================================================

        let tim2 = dp.TIM2;

        unsafe {
            // stop timer during config
            tim2.cr1().reset();

            // ==========================================
            // TIMER CLOCK = 1 MHz
            // ==========================================

            tim2.psc().write(|w| w.psc().bits(84 - 1));

            tim2.arr().write(|w| w.arr().bits(u32::MAX));

            // ==========================================
            // PWM INPUT MODE
            // ==========================================

            // CC1 = direct TI1
            // CC2 = indirect TI1

            tim2.ccmr1_input().write(|w| {
                w.cc1s().bits(0b01);
                w.cc2s().bits(0b10)
            });

            // CC1 = rising edge
            // CC2 = falling edge

            tim2.ccer().write(|w| {
                w.cc1e().set_bit();
                w.cc1p().clear_bit();

                w.cc2e().set_bit();
                w.cc2p().set_bit()
            });

            // reset counter on rising edge

            tim2.smcr().write(|w| {
                w.ts().bits(0b101);
                w.sms().bits(0b100)
            });

            // interrupt enable

            tim2.dier().write(|w| w.cc1ie().set_bit());

            // clear flags

            tim2.sr().reset();

            // start timer

            tim2.cr1().modify(|_, w| w.cen().set_bit());

            // enable NVIC interrupt

            cortex_m::peripheral::NVIC::unmask(stm32f4xx_hal::pac::Interrupt::TIM2);
        }

        defmt::info!("TIM2 READY");

        // =====================================================
        // ADC
        // ADC1 IN2 -> PA2
        // =====================================================

        let adc_pin = gpioa.pa2.into_analog();

        // free-running ADC for now
        // I did not manage to make the external trigger mode work with the timer.
        // If possible, we would like to configure the timer to trigger the ADC at
        // the end of each PWM period.
        let adc_cfg = AdcConfig::default()
            .dma(Dma::Continuous)
            .continuous(Continuous::Continuous);
        //.external_trigger(
        //    TriggerMode::Disabled,
        //    ExternalTrigger::Tim_2_trgo,
        //);

        let mut adc = Adc::new(dp.ADC1, true, adc_cfg, &mut rcc);

        adc.configure_channel(&adc_pin, Sequence::One, SampleTime::Cycles_480);

        defmt::info!("ADC READY");

        // =====================================================
        // DMA
        // =====================================================

        let mut streams = StreamsTuple::new(dp.DMA2, &mut rcc);

        streams
            .0
            .set_channel(stm32f4xx_hal::dma::DmaChannel::Channel0);
        streams.0.set_circular_mode(true); // VERY IMPORTANT FOR CONTINUOUS MODE otherwise NDTR will never decrease

        let dma_config = DmaConfig::default()
            .memory_increment(true)
            .transfer_complete_interrupt(false);

        let buffer = unsafe { &mut ADC_BUFFER };

        let mut transfer =
            Transfer::init_peripheral_to_memory(streams.0, adc, &mut *buffer, None, dma_config);

        transfer.start(|adc| {
            adc.start_conversion();
        });

        defmt::info!("ADC DMA READY");

        // =====================================================
        // DELAY TIMER
        // =====================================================

        let delay = dp.TIM3.delay_us(&mut rcc);

        (
            Shared {
                generated_duty: 50,
                increasing: true,
                transfer: transfer,
            },
            Local {
                led,
                delay,
                pwm_ch2,
            },
        )
    }

    #[task(binds = TIM2, shared = [transfer])]
    fn tim2_irq(_ctx: tim2_irq::Context) {
        // Uncomment this block to inspect the DMA transfer state
        // If the circular DMA is healthy, the NDTR should be decreasing and then jump back to 64 when it reaches 0, as the DMA wraps around and starts filling the buffer again.

        // ctx.shared.transfer.lock(|transfer| {
        //     let ndtr = transfer.number_of_transfers();
        //     defmt::info!("NDTR={}", ndtr);
        // });

        let tim2 = unsafe { &*stm32f4xx_hal::pac::TIM2::ptr() };

        if tim2.sr().read().cc1if().bit_is_set() {
            let period = tim2.ccr1().read().bits();

            let high = tim2.ccr2().read().bits();

            if period > 0 {
                let duty = (high * 100) / period;

                defmt::info!(
                    "PWM INPUT: duty={}%, period={}us high={}us",
                    duty,
                    period,
                    high
                );
            }

            // ==========================================
            // READ ADC DMA BUFFER
            // ==========================================

            unsafe {
                let mut sum: u32 = 0;

                for sample in ADC_BUFFER.iter() {
                    sum += *sample as u32;
                }

                defmt::info!("ADC(avg_dma)={}", sum / 64);
            }

            // clear interrupt flag

            tim2.sr().modify(|_, w| w.cc1if().clear_bit());
        }
    }

    // =====================================================
    // IDLE
    // =====================================================

    #[idle(
        local = [led, delay, pwm_ch2],
        shared = [generated_duty, increasing]
    )]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
            ctx.local.led.toggle();

            let duty =
                (&mut ctx.shared.generated_duty, &mut ctx.shared.increasing).lock(|duty, inc| {
                    if *inc {
                        if *duty >= 95 {
                            *inc = false;
                            *duty -= 5;
                        } else {
                            *duty += 5;
                        }
                    } else {
                        if *duty <= 5 {
                            *inc = true;
                            *duty += 5;
                        } else {
                            *duty -= 5;
                        }
                    }

                    *duty
                });

            let max = ctx.local.pwm_ch2.get_max_duty();

            let scaled = (max as u32 * duty as u32 / 100) as u16;

            ctx.local.pwm_ch2.set_duty(scaled);

            ctx.local.delay.delay_ms(1000_u32);
        }
    }
}
