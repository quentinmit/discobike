use crate::{Debug2Format, STATE};
use apds9960::{Apds9960Async, Error, LightData};
use core::fmt::{self, Debug};
use dim::si::{f32consts::LX, Lux};
use dim::traits::Dimensionless;
use ector::{actor, Actor, Address, Inbox};
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c;
use futures::select_biased;
use futures::FutureExt;

trait CalculateIlluminance {
    fn calculate_lux(&self) -> Lux<f32>;
}
impl CalculateIlluminance for LightData {
    fn calculate_lux(&self) -> Lux<f32> {
        ((-0.32466 * self.red as f32)
            + (1.57837 * self.green as f32)
            + (-0.73191 * self.blue as f32))
            * LX
    }
}

#[inline]
pub fn max<T: PartialOrd>(a: T, b: T) -> T {
    if a > b {
        a
    } else if b == b {
        b
    } else {
        a
    }
}

pub struct Light<I2C> {
    apds9960: Apds9960Async<I2C>,
}

pub enum LightMessage {
    On,
    Off,
}

const POLL_HIGH_EVERY: Duration = Duration::from_millis(1000 / 5);

impl<I2C, E> Light<I2C>
where
    I2C: i2c::I2c<Error = E>,
    E: Debug,
{
    pub async fn new(i2c: I2C) -> Result<Self, Error<E>> {
        let mut apds9960 = Apds9960Async::new(i2c);
        info!("APDS9960 ID: {}", apds9960.read_device_id().await?);
        Ok(Self { apds9960 })
    }

    async fn run_high_power<M>(&mut self, inbox: &mut M) -> Result<(), Error<E>>
    where
        M: Inbox<LightMessage>,
    {
        info!("Enabling apds9960");
        self.apds9960.enable().await?;
        self.apds9960.enable_light().await?;
        info!("Enabled");

        loop {
            let color = self
                .apds9960
                .read_light()
                .await
                .inspect_err(|e| {
                    error!("failed to read light sensor: {:?}", Debug2Format(&e));
                })
                .ok();
            // 3.5 counts/lux in the c channel according to datasheet
            let lux_value =
                color.map(|color| max(color.calculate_lux(), color.clear as f32 / (3.5 / LX))); // If C is overloaded, use RGB

            STATE.lock(|c| {
                c.update(|s| {
                    let mut s = s;
                    s.lux = lux_value;
                    s
                })
            });
            select_biased! {
                message = inbox.next().fuse() => {
                    match message {
                        LightMessage::On => warn!("Light on while already on"),
                        LightMessage::Off => {
                            self.apds9960.disable().await?;
                            return Ok(());
                        },
                    }
                },
                _ = Timer::after(POLL_HIGH_EVERY).fuse() => (),
            };
        }
    }
    async fn run_light<M>(&mut self, inbox: &mut M) -> Result<(), Error<E>>
    where
        M: Inbox<LightMessage>,
    {
        loop {
            match inbox.next().await {
                LightMessage::On => {
                    info!("Light on");
                    self.run_high_power(inbox).await?;
                }
                LightMessage::Off => warn!("Light off while already off"),
            };
        }
    }
}

#[actor]
impl<I2C, E> Actor for Light<I2C>
where
    I2C: i2c::I2c<Error = E>,
    E: fmt::Debug,
{
    type Message<'m> = LightMessage;

    async fn on_mount<M>(&mut self, _: Address<LightMessage>, mut inbox: M)
    where
        M: Inbox<LightMessage>,
    {
        loop {
            info!("run_light");
            if let Err(e) = self.run_light(&mut inbox).await {
                error!("run_light failed: {:?}", Debug2Format(&e));
            }
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}
