use crate::STATE;
use core::fmt;
use defmt::{*, panic};
use crate::drivers::ina219::{INA219Async, INA219_ADDR, ADCMode};
use ector::{actor, Actor, Address, Inbox};
use embedded_hal_async::i2c;
use embassy_executor::time::{Duration, Timer, Instant};
use dim::si::{
    f32consts::{A, LX, OHM},
    Volt, Lux,
};

pub struct Power<I2C> {
    ina219: INA219Async<I2C>,
}

pub enum PowerMessage {
    DisplayOn,
    DisplayOff,
}

const POLL_HIGH_EVERY: Duration = Duration::from_millis(1000/30);
const POLL_LOW_EVERY: Duration = Duration::from_millis(1000);

impl<I2C, E> Power<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        let ina219 = INA219Async::new(i2c, INA219_ADDR);
        Power {
            ina219
        }
    }

    async fn run_high_power(&mut self) -> Result<(), E>
    {
        self.ina219.continuous_sample(true, true).await?;
        loop {
            let vext = self.ina219.get_bus_voltage().await?;
            let current = self.ina219.get_current().await?;
            let state = STATE.lock(|c| {
                c.update(|s| {
                    let mut s = s;
                    s.vext = Some(vext);
                    s.current = current;
                    s.vext_poll_timer.update();
                    s
                })
            });
            info!(
                "Vext = {:?}, Iext = {:?}",
                defmt::Debug2Format(&vext),
                defmt::Debug2Format(&current)
            );
            if !state.display_on {
                return Ok(());
            }
            Timer::after(POLL_HIGH_EVERY).await;
        }
    }
    async fn run_low_power(&mut self) -> Result<(), E>
    {
        loop {
            self.ina219.trigger_sample(true, false).await?;
            let vext = self.ina219.get_bus_voltage().await?;
            self.ina219.power_down().await?;
            let state = STATE.lock(|c| {
                c.update(|s| {
                    let mut s = s;
                    s.vext = Some(vext);
                    s.vext_poll_timer.update();
                    s.current = None;
                    s
                })
            });
            info!("Vext = {:?}", defmt::Debug2Format(&vext));
            if state.display_on {
                return Ok(());
            }
            Timer::after(POLL_LOW_EVERY).await;
        }
    }
    async fn run_power<M>(&mut self, _inbox: &mut M) -> Result<(), E>
    where
        M: Inbox<PowerMessage>,
    {
        info!("initializing ina219");
        self.ina219.set_adc_mode(ADCMode::SampleMode128).await?;
        self.ina219.set_current_range(3.2 * A, 0.1 * OHM).await?;
        loop {
            let state = STATE.lock(|c| c.get());
            match state.display_on {
                true => self.run_high_power().await?,
                false => self.run_low_power().await?,
            }
        }
    }
}

#[actor]
impl<I2C, E> Actor for Power<I2C>
where
    I2C: i2c::I2c<Error = E>,
    E: fmt::Debug,
{
    type Message<'m> = PowerMessage;

    async fn on_mount<M>(&mut self, _: Address<PowerMessage>, mut inbox: M)
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
