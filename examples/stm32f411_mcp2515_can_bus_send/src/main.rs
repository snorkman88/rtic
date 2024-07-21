#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]

use panic_halt as _;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true,  dispatchers = [SPI2])]
mod app {

    use defmt::panic;
    use defmt_rtt as _;

    use stm32f4xx_hal::pac::SPI1;
    use stm32f4xx_hal::spi::{Mode, Phase, Polarity, Spi};
    use stm32f4xx_hal::{
        gpio::{self, Edge, Input, Output, Pin, PushPull},
        pac::TIM1,
        prelude::*,
        timer,
    };

    use embedded_can::nb::Can;
    use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};

    use embedded_can::{Frame, StandardId};
    use mcp25xx::bitrates::clock_16mhz::CNF_500K_BPS;
    use mcp25xx::registers::{OperationMode, RXB0CTRL, RXM};
    use mcp25xx::{CanFrame, Config, MCP25xx};

    // Resources shared between tasks
    #[shared]
    struct Shared {
        delayval: u32,
    }

    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        button: gpio::PA0<Input>,
        led: gpio::PC13<Output<PushPull>>,
        delay: timer::DelayMs<TIM1>,
        mcp25xx: MCP25xx<ExclusiveDevice<Spi<SPI1>, Pin<'A', 4, Output>, NoDelay>>,
    }



    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut dp = ctx.device;

        // Configure and obtain handle for delay abstraction
        // 1) Promote RCC structure to HAL to be able to configure clocks
        let rcc = dp.RCC.constrain();

        // 2) Configure the system clocks
        // 25 MHz must be used for HSE on the Blackpill-STM32F411CE board according to manual
        let clocks = rcc.cfgr.use_hse(25.MHz()).freeze();

        // 3) Create delay handle
        let mut delay = dp.TIM1.delay_ms(&clocks);

        // Configure the LED pin as a push pull output and obtain handle
        // On the Blackpill STM32F411CEU6 there is an on-board LED connected to pin PC13
        // 4) Promote the GPIOC PAC struct
        let gpioc = dp.GPIOC.split();

        // 5) Configure PORTC OUTPUT Pins and Obtain Handle
        let led = gpioc.pc13.into_push_pull_output();

        // Configure the button pin as input and obtain handle
        // On the Blackpill STM32F411CEU6 there is a button connected to pin PA0
        // 6) Promote the GPIOA PAC struct
        let gpioa: gpio::gpioa::Parts = dp.GPIOA.split();
        // 7) Configure buttonPin and Obtain Handle
        let mut button = gpioa.pa0.into_pull_up_input();

        // Configure Pins for SPI1 communication
        let sclk = gpioa.pa5.into_alternate::<5>();
        let miso = gpioa.pa6.into_alternate::<5>();
        let mosi = gpioa.pa7.into_alternate::<5>();
        let mut cs = gpioa.pa4.into_push_pull_output();

        // Configure Button Pin for Interrupts
        // 10) Promote SYSCFG structure to HAL to be able to configure interrupts
        let mut syscfg = dp.SYSCFG.constrain();
        // 11) Make button an interrupt source
        button.make_interrupt_source(&mut syscfg);
        // 12) Configure the interruption to be triggered on a rising edge
        button.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
        // 13) Enable gpio interrupt for button
        button.enable_interrupt(&mut dp.EXTI);

        let mut spi = Spi::new(
            dp.SPI1,
            (sclk, miso, mosi),
            Mode {
                polarity: Polarity::IdleLow,
                phase: Phase::CaptureOnFirstTransition,
            },
            16.MHz(),
            &clocks,
        );

        let exclusive_spi_bus = ExclusiveDevice::new_no_delay(spi, cs);
        let bus = exclusive_spi_bus.unwrap();

        let mut mcp25xx = MCP25xx { spi: bus };
        let config = Config::default()
            .mode(OperationMode::NormalOperation)
            .bitrate(CNF_500K_BPS)
            .receive_buffer_0(RXB0CTRL::default().with_rxm(RXM::ReceiveAny));
        mcp25xx.apply_config(&config).unwrap();

        // Send a frame without RTR bit set
        let can_id = StandardId::new(123).unwrap();
        let data = [1, 2, 3, 4, 5, 6, 7, 8];
        let frame = CanFrame::new(can_id, &data).unwrap();
        mcp25xx.transmit(&frame).unwrap();

        // Send a frame with RTR bit set and a datalength of 8
        let can_id = StandardId::new(333).unwrap();
        let frame = CanFrame::new_remote(can_id, 0).unwrap();
        mcp25xx.transmit(&frame).unwrap();

        // Receive a frame
        defmt::info!("Reading ");
        loop {
            if let Ok(frame) = mcp25xx.receive() {
                //if frame.is_remote_frame(){
                //    defmt::debug!("Received an RTR frame {:?}");
                //}
                //else{
                //    defmt::debug!("Received an DATA frame",frame.clone());
                //}
                let _can_id = frame.id();
                
                defmt::info!("Received an DATA frame {:?}",frame.clone());
                let _data = frame.data();
                defmt::info!("Incoming Data: {=[u8]:X}", _data);
            }
        }

        (
            // Initialization of shared resources. In this case delay value and the ADC instance
            Shared { delayval: 2001_u32 },
            // Initialization of task local resources
            Local {
                button,
                led,
                delay,
                mcp25xx,
            },
        )
    }

    // Background task, runs whenever no other tasks are running
    #[idle(local = [led, delay, mcp25xx], shared = [delayval])]
    fn idle(mut ctx: idle::Context) -> ! {
        let led = ctx.local.led;
        let delay = ctx.local.delay;
        loop {
            // Turn On LED
            led.set_high();
            // Obtain shared delay variable and delay
            delay.delay_ms(ctx.shared.delayval.lock(|del| *del));
            // Turn off LED
            led.set_low();
            // Obtain shared delay variable and delay
            delay.delay_ms(ctx.shared.delayval.lock(|del| *del));
        }
    }

    // Handle the IRQ generated when the button is pressed and interact with local and shared resources.
    #[task(binds = EXTI0, local = [button], shared=[delayval])]
    fn gpio_interrupt_handler(mut ctx: gpio_interrupt_handler::Context) {
        ctx.shared.delayval.lock(|del| {
            *del = *del - 100_u32;
            if *del < 200_u32 {
                *del = 2000_u32;
            }
            *del
        });

        ctx.shared.delayval.lock(|del| {
            defmt::info!("Current delay value {:?}", del);
        });

        ctx.local.button.clear_interrupt_pending_bit();
    }
}
