#![deny(warnings)]
#![no_main]
#![no_std]

use panic_halt as _;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {

    // =========================================================
    // HAL IMPORTS
    // =========================================================

    use stm32f4xx_hal::{
        adc::{
            config::AdcConfig,
            Adc,
        },
        gpio::{
            gpioa,
            gpioc,
            Output,
            PushPull,
        },
        pac::{
            ADC1,
            TIM3,
        },
        prelude::*,
        timer::{
            self,
            Channel,
        },
    };

    use defmt_rtt as _;

    // =========================================================
    // SHARED RTIC RESOURCES
    // =========================================================

    #[shared]
    struct Shared {

        // Measured PWM duty cycle (%)
        measured_duty: u32,

        // Measured PWM period in microseconds
        period_ticks: u32,

        // Measured HIGH pulse width in microseconds
        high_ticks: u32,

        // Raw ADC sample
        adc_value: u16,
    }

    // =========================================================
    // LOCAL RTIC RESOURCES
    // =========================================================

    #[local]
    struct Local {

        // Heartbeat LED
        led: gpioc::PC13<Output<PushPull>>,

        // Delay timer
        delay: timer::DelayUs<TIM3>,

        // ADC peripheral
        adc: Adc<ADC1>,

        // ADC input pin
        adc_pin: gpioa::PA2<stm32f4xx_hal::gpio::Analog>,

        // Last rising edge timestamp
        previous_rising: u32,
    }

    // =========================================================
    // INIT
    // =========================================================

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {

        let dp = ctx.device;

        defmt::info!("BOOT: init");

        // =====================================================
        // CLOCK CONFIGURATION
        // =====================================================

        let rcc = dp.RCC.constrain();

        let clocks = rcc
            .cfgr
            .use_hse(25.MHz())
            .sysclk(84.MHz())
            .freeze();

        defmt::info!("CLOCKS: configured");

        // =====================================================
        // GPIO CONFIGURATION
        // =====================================================

        let gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();

        // -----------------------------------------------------
        // HEARTBEAT LED
        // -----------------------------------------------------

        let led =
            gpioc.pc13.into_push_pull_output();

        // -----------------------------------------------------
        // PWM OUTPUT
        // -----------------------------------------------------
        //
        // TIM1 CH2 -> PA9
        //

        let pwm_pin =
            gpioa.pa9.into_alternate::<1>();

        // -----------------------------------------------------
        // INPUT CAPTURE INPUT
        // -----------------------------------------------------
        //
        // TIM2 CH1 -> PA0
        //
        // Connect:
        //
        // PA9 ---> PA0
        //

        let _capture_pin =
            gpioa.pa0.into_alternate::<1>();

        // -----------------------------------------------------
        // ADC INPUT
        // -----------------------------------------------------

        let adc_pin =
            gpioa.pa2.into_analog();

        // =====================================================
        // PWM GENERATION
        // =====================================================

        let mut pwm = dp.TIM1.pwm_hz(
            timer::Channel2::new(pwm_pin),
            1.kHz(),
            &clocks,
        );

        let max =
            pwm.get_max_duty();

        pwm.set_duty(
            Channel::C2,
            max / 2
        );

        pwm.enable(Channel::C2);

        defmt::info!(
            "TIM1: PWM started"
        );

        // =====================================================
        // TIM3 DELAY TIMER
        // =====================================================

        let delay =
            dp.TIM3.delay_us(&clocks);

        // =====================================================
        // ADC CONFIGURATION
        // =====================================================

        let adc_config =
            AdcConfig::default();

        let adc = Adc::adc1(
            dp.ADC1,
            true,
            adc_config,
        );

        defmt::info!("ADC1: ready");

        // =====================================================
        // TIM2 INPUT CAPTURE CONFIGURATION
        // =====================================================

        let tim2 = dp.TIM2;

        // =====================================================
        // ENABLE TIM2 CLOCK
        // =====================================================

        unsafe {

            let rcc =
                &(*stm32f4xx_hal::pac::RCC::ptr());

            rcc.apb1enr.modify(|_, w| {
                w.tim2en().enabled()
            });
        }

        defmt::info!(
            "TIM2 RCC clock enabled"
        );

        // =====================================================
        // TIMER BASE CONFIGURATION
        // =====================================================
        //
        // TIM2 clock:
        // 84 MHz
        //
        // Prescaler:
        // 84 - 1
        //
        // Result:
        // 1 MHz timer
        //
        // Therefore:
        // 1 tick = 1 microsecond
        //

        tim2.psc.write(|w|  {
            w.psc().bits(84 - 1)
        });

        tim2.arr.write(|w|  {
            w.arr().bits(u32::MAX)
        });

        // =====================================================
        // INPUT CAPTURE CONFIGURATION
        // =====================================================
        //
        // CH1 = rising edge
        // CH2 = falling edge
        //
        // BOTH channels observe TI1
        //

        tim2.ccmr1_input().modify(|_, w| {

            // CC1 mapped to TI1
            w.cc1s().ti1();

            // CC2 mapped to TI1
            w.cc2s().ti1();

            w
        });

        // =====================================================
        // EDGE POLARITIES
        // =====================================================

        tim2.ccer.modify(|_, w| {

            // CH1 rising edge
            w.cc1p().clear_bit();

            // CH2 falling edge
            w.cc2p().set_bit();

            // Enable capture channels
            w.cc1e().set_bit();
            w.cc2e().set_bit();

            w
        });

        // =====================================================
        // INTERRUPTS
        // =====================================================

        tim2.dier.modify(|_, w| {

            // CH1 interrupt
            w.cc1ie().set_bit();

            // CH2 interrupt
            w.cc2ie().set_bit();

            w
        });

        // =====================================================
        // START TIMER
        // =====================================================

        tim2.cr1.modify(|_, w| {
            w.cen().set_bit()
        });

        // =====================================================
        // DEBUG COUNTER
        // =====================================================

        let cnt =
            tim2.cnt.read().cnt().bits();

        defmt::info!(
            "TIM2 counter initial={}",
            cnt
        );

        // =====================================================
        // ENABLE NVIC IRQ
        // =====================================================

        unsafe {

            cortex_m::peripheral::NVIC::unmask(
                stm32f4xx_hal::pac::Interrupt::TIM2
            );
        }

        defmt::info!(
            "TIM2: input capture ready"
        );

        (
            Shared {

                measured_duty: 0,

                period_ticks: 0,

                high_ticks: 0,

                adc_value: 0,
            },

            Local {

                led,

                delay,

                adc,

                adc_pin,

                previous_rising: 0,
            },
        )
    }

    // =========================================================
    // IDLE LOOP
    // =========================================================

    #[idle(local = [led, delay])]
    fn idle(
        ctx: idle::Context
    ) -> ! {

        defmt::info!(
            "IDLE: started"
        );

        loop {

            ctx.local.led.toggle();

            ctx.local
                .delay
                .delay_ms(500_u32);
        }
    }

    // =========================================================
    // TIM2 INPUT CAPTURE IRQ
    // =========================================================

    #[task(
        binds = TIM2,
        local = [
            adc,
            adc_pin,
            previous_rising
        ],
        shared = [
            measured_duty,
            period_ticks,
            high_ticks,
            adc_value
        ]
    )]
    fn tim2_capture_irq(
        mut ctx: tim2_capture_irq::Context
    ) {

        let tim2 = unsafe {
            &*stm32f4xx_hal::pac::TIM2::ptr()
        };

        // =====================================================
        // RISING EDGE
        // =====================================================

        if tim2
            .sr
            .read()
            .cc1if()
            .bit_is_set()
        {

            let rising =
                tim2
                    .ccr1()
                    .read()
                    .ccr()
                    .bits();

            let period =
                rising.wrapping_sub(
                    *ctx.local.previous_rising
                );

            *ctx.local.previous_rising =
                rising;

            ctx.shared
                .period_ticks
                .lock(|p| {
                    *p = period;
                });

            // Clear interrupt flag

            tim2.sr.modify(|_, w| {
                w.cc1if().clear_bit()
            });

            defmt::info!(
                "RISING: period={} us",
                period
            );
        }

        // =====================================================
        // FALLING EDGE
        // =====================================================

        if tim2
            .sr
            .read()
            .cc2if()
            .bit_is_set()
        {

            let falling =
                tim2
                    .ccr2()
                    .read()
                    .ccr()
                    .bits();

            let previous_rising =
                *ctx.local.previous_rising;

            let high =
                falling.wrapping_sub(
                    previous_rising
                );

            ctx.shared
                .high_ticks
                .lock(|h| {
                    *h = high;
                });

            let period =
                ctx.shared
                    .period_ticks
                    .lock(|p| *p);

            let duty =
                if period > 0 {

                    (high * 100) / period

                } else {

                    0
                };

            ctx.shared
                .measured_duty
                .lock(|d| {
                    *d = duty;
                });

            // =================================================
            // ADC SAMPLE
            // =================================================

            let adc_result: u16 =
                ctx.local
                    .adc
                    .read(ctx.local.adc_pin)
                    .unwrap_or(0);

            ctx.shared
                .adc_value
                .lock(|a| {
                    *a = adc_result;
                });

            // Clear interrupt flag

            tim2.sr.modify(|_, w| {
                w.cc2if().clear_bit()
            });

            defmt::info!(
                "PWM measured: duty={}%, high={} us ADC={}",
                duty,
                high,
                adc_result
            );
        }
    }
}