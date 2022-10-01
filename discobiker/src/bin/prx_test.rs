#![no_std]
#![no_main]

#![feature(type_alias_impl_trait)]

use discobiker::drivers::esb::{self, ESB};
use embassy_executor::Spawner;
use embassy_nrf as _;
use embassy_nrf::pac;

#[cfg(feature = "defmt")]
use defmt_rtt as _;
use panic_probe as _;

#[embassy_executor::main()]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    let esb_config = esb::Config{
        ..Default::default()
    };
    let mut radio = unsafe { pac::Peripherals::steal() }.RADIO;
    let mut esb: ESB<32> = esb::ESB::new(&mut radio, esb_config);
    esb.start_rx(&mut radio);
}