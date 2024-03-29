use crate::{Debug2Format, Global, ActualState};
use apds9960::{Apds9960Async, Error, LightData};
use core::fmt::{self, Debug};
use dim::si::{f32consts::LX, Lux};
use ector::{Actor, DynamicAddress, Inbox};
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

pub struct Light<'d, I2C> {
    apds9960: Apds9960Async<I2C>,
    state: &'d Global<ActualState>,
}

pub enum LightMessage {
    On,
    Off,
}

impl From<bool> for LightMessage {
    fn from(b: bool) -> Self {
        if b {
            Self::On
        } else {
            Self::Off
        }
    }
}

const POLL_HIGH_EVERY: Duration = Duration::from_millis(1000 / 5);

impl<'d, I2C, E> Light<'d, I2C>
where
    I2C: i2c::I2c<Error = E>,
    E: Debug,
{
    pub async fn new(state: &'d Global<ActualState>, i2c: I2C) -> Result<Light<'d, I2C>, Error<E>> {
        let mut apds9960 = Apds9960Async::new(i2c);
        info!("APDS9960 ID: {}", apds9960.read_device_id().await?);
        Ok(Self { apds9960, state })
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

            self.state.lock(|c| {
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

impl<I2C, E> Actor for Light<'_, I2C>
where
    I2C: i2c::I2c<Error = E>,
    E: fmt::Debug,
{
    type Message = LightMessage;

    async fn on_mount<M>(&mut self, _: DynamicAddress<LightMessage>, mut inbox: M) -> !
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
