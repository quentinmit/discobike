#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use cortex_m_rt::entry;
use defmt::*;
use embassy::executor::Executor;
use embassy::time::{Duration, Timer};
use embassy::util::Forever;
use embassy_nrf as _;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::peripherals::P0_13;
use embassy_nrf::Peripherals;

use nrf_softdevice::Softdevice;
use nrf_softdevice_defmt_rtt as _;
use panic_probe as _;

static EXECUTOR: Forever<Executor> = Forever::new();

#[embassy::task]
async fn softdevice_task(sd: &'static Softdevice) {
    sd.run().await;
}

#[embassy::task]
async fn blinker(mut led: Output<'static, P0_13>, interval: Duration) {
    loop {
        led.set_high();
        Timer::after(interval).await;
        led.set_low();
        Timer::after(interval).await;
    }
}

#[entry]
fn main() -> ! {
    info!("Hello World!");
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

    let sd = Softdevice::enable(&Default::default());

    let led = Output::new(p.P0_13, Level::Low, OutputDrive::Standard);

    let executor = EXECUTOR.put(Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(softdevice_task(sd)));
        unwrap!(spawner.spawn(blinker(led, Duration::from_millis(300))));
    });
}
