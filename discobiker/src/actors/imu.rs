use crate::{LSM6DS33_ADDR, STATE};
use core::fmt;
use defmt::*;
use dim::derived;
use dim::{typenum::Pow, Sqrt};
use dim::typenum::P2;
use dim::si::{f32consts::MPS2, MeterPerSecond2};
use dim::ucum::{
    self,
    f32consts::{DEG, RAD, S},
    Radian, Second, UCUM,
};
use ector::{actor, Actor, Address, Inbox};
use embassy::time::{Duration, Timer};
use embedded_hal_async::i2c;
use lsm6ds33::{self, *};

derived!(ucum, UCUM: RadianPerSecond = Radian / Second);
//const RADPS: RadianPerSecond<f32> = RAD / S;

pub struct Imu<I2C> {
    lsm6ds33: Lsm6ds33<I2C>,
}

pub enum ImuMessage {}

#[derive(Debug)]
pub enum Error<I2C>
where
    I2C: i2c::I2c,
{
    I2c(I2C::Error),
    Lsm6ds33(lsm6ds33::Error<I2C::Error>),
}

// impl<I2C, E> From<E> for Error<I2C>
// where
//     I2C: i2c::I2c<Error = E>,
// {
//     fn from(e: E) -> Self {
//         Self::I2c(e)
//     }
// }

impl<I2C, E> From<lsm6ds33::Error<E>> for Error<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    fn from(e: lsm6ds33::Error<E>) -> Self {
        Self::Lsm6ds33(e)
    }
}

impl<I2C, E> Imu<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    pub async fn new(i2c: I2C) -> Result<Self, Error<I2C>> {
        let lsm6ds33 = Lsm6ds33::new(i2c, LSM6DS33_ADDR).await?;
        Ok(Imu { lsm6ds33 })
    }
    async fn run_imu<M>(&mut self, _inbox: &mut M) -> Result<(), Error<I2C>>
    where
        M: Inbox<ImuMessage>,
    {
        info!("initializing imu");
        self.lsm6ds33
            .set_accelerometer_scale(AccelerometerScale::G02)
            .await?;
        self.lsm6ds33
            .set_gyroscope_scale(GyroscopeFullScale::Dps245)
            .await?;
        self.lsm6ds33
            .set_accelerometer_output(AccelerometerOutput::Rate104)
            .await?;
        self.lsm6ds33
            .set_gyroscope_output(GyroscopeOutput::Rate104)
            .await?;
        self.lsm6ds33
            .set_accelerometer_bandwidth(AccelerometerBandwidth::Freq400)
            .await?;
        self.lsm6ds33.set_low_power_mode(true).await?;
        loop {
            let accel_raw = self.lsm6ds33.read_accelerometer().await?; // m/s^2
            let accel: [MeterPerSecond2<f32>; 3] =
                [accel_raw.0 * MPS2, accel_raw.1 * MPS2, accel_raw.2 * MPS2];
            let gyro_raw = self.lsm6ds33.read_gyro().await?; // rads/s
            let gyro: [RadianPerSecond<f32>; 3] = [
                gyro_raw.0 * RAD / S,
                gyro_raw.1 * RAD / S,
                gyro_raw.2 * RAD / S,
            ];
            let accel_mag =
                (accel[0].powi(P2::new()) + accel[1].powi(P2::new()) + accel[2].powi(P2::new())).sqrt();

            STATE.lock(|c| {
                c.update(|s| {
                    let mut s = s;
                    s.accel_mag = Some(accel_mag);
                    if s.accel_mag.map_or(false, |v| v > 12.0*MPS2) {
                        s.move_timer.update();
                    }
                    s
                })
            });
            Timer::after(Duration::from_millis(1000 / 50)).await;
        }
    }
}

#[actor]
impl<I2C, E> Actor for Imu<I2C>
where
    I2C: i2c::I2c<Error = E>,
    E: fmt::Debug,
{
    type Message<'m> = ImuMessage;

    async fn on_mount<M>(&mut self, _: Address<ImuMessage>, mut inbox: M)
    where
        M: Inbox<ImuMessage>,
    {
        loop {
            info!("run_imu");
            let res = self.run_imu(&mut inbox).await;
            /*res.map_err(|e| {
                error!("run_imu failed: {:?}", Debug2Format(&e));
            });*/
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}
