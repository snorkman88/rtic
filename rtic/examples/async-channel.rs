//! examples/async-channel.rs

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use panic_semihosting as _;

#[rtic::app(device = lm3s6965, dispatchers = [SSI0])]
mod app {
    use cortex_m_semihosting::{debug, hprintln};
    use rtic_channel::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        let (s, r) = make_channel!(u32, 5);

        receiver::spawn(r).unwrap();
        sender1::spawn(s.clone()).unwrap();
        sender2::spawn(s.clone()).unwrap();
        sender3::spawn(s).unwrap();

        (Shared {}, Local {})
    }

    #[task]
    async fn receiver(_c: receiver::Context, mut receiver: Receiver<'static, u32, 5>) {
        while let Ok(val) = receiver.recv().await {
            hprintln!("Receiver got: {}", val);
            if val == 5 {
                debug::exit(debug::EXIT_SUCCESS); // Exit QEMU simulator
            }
        }
    }

    #[task]
    async fn sender1(_c: sender1::Context, mut sender: Sender<'static, u32, 5>) {
        hprintln!("Sender 1 sending: 1");
        sender.send(1).await.unwrap();
    }

    #[task]
    async fn sender2(_c: sender2::Context, mut sender: Sender<'static, u32, 5>) {
        hprintln!("Sender 2 sending: 2");
        sender.send(2).await.unwrap();
    }

    #[task]
    async fn sender3(_c: sender3::Context, mut sender: Sender<'static, u32, 5>) {
        hprintln!("Sender 3 sending: 3");
        sender.send(5).await.unwrap();
    }
}