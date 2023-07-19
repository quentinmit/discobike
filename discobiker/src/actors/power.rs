use crate::drivers::ina219::{ADCMode, INA219Async, INA219_ADDR};
use crate::{Debug2Format, Global, ActualState};
use core::fmt;
use dim::si::f32consts::{A, OHM, V};
use dim::traits::Dimensionless;
use ector::{actor, Actor, DynamicAddress, Inbox};
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c;
use futures::{select_biased, FutureExt};

pub struct Power<'a, I2C> {
    ina219: INA219Async<I2C>,
    state: &'a Global<ActualState>,
}

pub enum PowerMessage {
    DisplayOn,
    DisplayOff,
}

impl From<bool> for PowerMessage {
    fn from(b: bool) -> Self {
        if b {
            Self::DisplayOn
        } else {
            Self::DisplayOff
        }
    }
}

const POLL_HIGH_EVERY: Duration = Duration::from_millis(1000 / 30);
const POLL_LOW_EVERY: Duration = Duration::from_millis(1000);

impl<'a, I2C, E> Power<'a, I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    pub fn new(state: &'a Global<ActualState>, i2c: I2C) -> Self {
        let ina219 = INA219Async::new(i2c, INA219_ADDR);
        Power { ina219, state }
    }

    async fn run_high_power<M>(&mut self, inbox: &mut M) -> Result<(), E>
    where
        M: Inbox<PowerMessage>,
    {
        self.ina219.continuous_sample(true, true).await?;
        loop {
            let vext = self.ina219.get_bus_voltage().await?;
            let current = self.ina219.get_current().await?;
            self.state.lock(|c| {
                c.update(|s| {
                    let mut s = s;
                    s.vext = Some(vext);
                    s.current = current;
                    s.vext_poll_timer.update();
                    s
                })
            });
            trace!(
                "Vext = {} V, Iext = {:?} A",
                (vext / V).value(),
                current.map(|v| *(v / A).value()),
            );
            select_biased! {
                message = inbox.next().fuse() => {
                    match message {
                        PowerMessage::DisplayOn => warn!("Power on while already on"),
                        PowerMessage::DisplayOff => return Ok(()),
                    }
                },
                _ = Timer::after(POLL_HIGH_EVERY).fuse() => (),
            };
        }
    }
    async fn run_low_power<M>(&mut self, inbox: &mut M) -> Result<(), E>
    where
        M: Inbox<PowerMessage>,
    {
        loop {
            self.ina219.trigger_sample(true, false).await?;
            let vext = self.ina219.get_bus_voltage().await?;
            self.ina219.power_down().await?;
            self.state.lock(|c| {
                c.update(|s| {
                    let mut s = s;
                    s.vext = Some(vext);
                    s.vext_poll_timer.update();
                    s.current = None;
                    s
                })
            });
            trace!("Vext = {:?}", Debug2Format(&vext));
            select_biased! {
                message = inbox.next().fuse() => {
                    match message {
                        PowerMessage::DisplayOn => return Ok(()),
                        PowerMessage::DisplayOff => warn!("Power off while already off"),
                    }
                },
                _ = Timer::after(POLL_LOW_EVERY).fuse() => (),
            };
        }
    }
    async fn run_power<M>(&mut self, inbox: &mut M) -> Result<(), E>
    where
        M: Inbox<PowerMessage>,
    {
        info!("initializing ina219");
        self.ina219.set_adc_mode(ADCMode::SampleMode128).await?;
        self.ina219.set_current_range(3.2 * A, 0.1 * OHM).await?;
        info!("ina219 initialized");
        loop {
            self.run_low_power(inbox).await?;
            self.run_high_power(inbox).await?;
        }
    }
}

impl<'a, I2C, E> Actor for Power<'a, I2C>
where
    I2C: i2c::I2c<Error = E>,
    E: fmt::Debug,
{
    type Message = PowerMessage;

    async fn on_mount<M>(&mut self, _: DynamicAddress<PowerMessage>, mut inbox: M) -> !
    where
        M: Inbox<PowerMessage>,
    {
        loop {
            info!("run_power");
            if let Err(e) = self.run_power(&mut inbox).await {
                error!("run_power failed: {:?}", Debug2Format(&e));
            }
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}
