use crate::{Debug2Format, CELSIUS_ZERO, ActualState, Global};
use bme280::{
    i2c::AsyncBME280, Configuration, Error, IIRFilter::*, Oversampling::*, SensorMode, Standby::*,
};
use core::fmt;
use dim::f32prefixes::HECTO;
use dim::si::f32consts::{K, PA};
use dim::traits::Dimensionless;
use ector::{Actor, DynamicAddress, Inbox};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_async::i2c;
use futures::select_biased;
use futures::FutureExt;

pub struct Barometer<'d, I2C> {
    bme280: AsyncBME280<I2C>,
    state: &'d Global<ActualState>,
}

pub enum BarometerMessage {
    On,
    Off,
}

impl From<bool> for BarometerMessage {
    fn from(b: bool) -> Self {
        if b {
            Self::On
        } else {
            Self::Off
        }
    }
}

const POLL_HIGH_EVERY: Duration = Duration::from_millis(1000 / 5);

impl<'d, I2C, E> Barometer<'d, I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    pub fn new(state: &'d Global<ActualState>, i2c: I2C) -> Self {
        let bme280 = AsyncBME280::new_secondary(i2c);
        Self { bme280, state }
    }

    async fn run_high_power<M>(&mut self, inbox: &mut M) -> Result<(), Error<E>>
    where
        M: Inbox<BarometerMessage>,
    {
        self.bme280
            .init_with_config(
                &mut Delay {},
                Configuration::default()
                    .with_temperature_oversampling(Oversampling2X)
                    .with_pressure_oversampling(Oversampling16X)
                    .with_iir_filter(Coefficient4)
                    .with_standby_period(Standby62_5MS),
            )
            .await?;

        loop {
            let data = self.bme280.measure(&mut Delay {}).await?;
            let temperature = (data.temperature + CELSIUS_ZERO) * K;
            let pressure = data.pressure * PA;
            self.state.lock(|c| {
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
                            self.bme280.set_mode(SensorMode::Sleep).await?;
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
                    self.run_high_power(inbox).await?;
                }
                BarometerMessage::Off => warn!("barometer off while already off"),
            };
        }
    }
}

impl<I2C, E> Actor for Barometer<'_, I2C>
where
    I2C: i2c::I2c<Error = E>,
    E: fmt::Debug,
{
    type Message = BarometerMessage;

    async fn on_mount<M>(&mut self, _: DynamicAddress<BarometerMessage>, mut inbox: M) -> !
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
