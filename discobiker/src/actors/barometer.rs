use bme280::{Configuration, SensorMode, Error, Oversampling::*, IIRFilter::*, Standby::*, i2c::AsyncBME280};
use crate::{STATE, CELSIUS_ZERO};
use core::fmt;
use defmt::{panic, *};
use dim::f32prefixes::HECTO;
use dim::si::{
    f32consts::{K, PA},
    Pascal, Kelvin,
};
use futures::select_biased;
use futures::FutureExt;
use dim::traits::Dimensionless;
use ector::{actor, Actor, Address, Inbox};
use embassy_executor::time::{Duration, Instant, Timer, Delay};
use embedded_hal_async::i2c;

pub struct Barometer<I2C> {
    bme280: AsyncBME280<I2C>,
}

pub enum BarometerMessage {
    On,
    Off,
}

const POLL_HIGH_EVERY: Duration = Duration::from_millis(1000 / 5);

impl<I2C, E> Barometer<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        let bme280 = AsyncBME280::new_secondary(i2c);
        Self { bme280 }
    }

    async fn run_high_power<M>(&mut self, inbox: &mut M) -> Result<(), Error<E>>
    where
        M: Inbox<BarometerMessage>,
    {
        self.bme280.init_with_config(
            &mut Delay{},
            Configuration::default()
            .with_temperature_oversampling(Oversampling2X)
            .with_pressure_oversampling(Oversampling16X)
            .with_iir_filter(Coefficient4)
            .with_standby_period(Standby62_5MS)
        ).await?;

        loop {
            let data = self.bme280.measure(&mut Delay{}).await?;
            let temperature = (data.temperature + CELSIUS_ZERO) * K;
            let pressure = data.pressure * PA;
            let state = STATE.lock(|c| {
                c.update(|s| {
                    let mut s = s;
                    s.temperature = Some(temperature);
                    s.pressure = Some(pressure);
                    s
                })
            });
            trace!(
                "BMP280 temperature = {} °C, pressure = {} hPa",
                (temperature / K - CELSIUS_ZERO).value(),
                (pressure / (HECTO * PA)).value(),
            );
            select_biased! {
                message = inbox.next().fuse() => {
                    match message {
                        BarometerMessage::On => warn!("barometer on while already on"),
                        BarometerMessage::Off => {
                            self.bme280.set_mode(SensorMode::Sleep).await;
                            return Ok(());
                        },
                    }
                },
                _ = Timer::after(POLL_HIGH_EVERY).fuse() => (),
            };
        }
    }
    async fn run_barometer<M>(&mut self, inbox: &mut M) -> Result<(), Error<E>>
    where
        M: Inbox<BarometerMessage>,
    {
        loop {
            match inbox.next().await {
                BarometerMessage::On => {
                    info!("barometer on");
                    self.run_high_power(inbox).await;
                },
                BarometerMessage::Off => warn!("barometer off while already off"),
            };
        }
    }
}

#[actor]
impl<I2C, E> Actor for Barometer<I2C>
where
    I2C: i2c::I2c<Error = E>,
    E: fmt::Debug,
{
    type Message<'m> = BarometerMessage;

    async fn on_mount<M>(&mut self, _: Address<BarometerMessage>, mut inbox: M)
    where
        M: Inbox<BarometerMessage>,
    {
        loop {
            info!("run_barometer");
            if let Err(e) = self.run_barometer(&mut inbox).await {
                error!("run_barometer failed: {:?}", Debug2Format(&e));
            }
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}
