use crate::{LSM6DS33_ADDR, STATE, CELSIUS_ZERO};
use core::fmt;
use defmt::*;
use dim::si::{f32consts::{K, MPS2}, Kelvin, MeterPerSecond2};
use dim::typenum::P2;
use dim::ucum::{
    self,
    f32consts::{DEG, RAD, S},
    Radian, Second, UCUM,
};
use dim::{derived, Dimensionless};
use dim::{typenum::Pow, Sqrt};
use ector::{actor, Actor, Address, Inbox};
use embassy_executor::time::{Duration, Timer};
use embedded_hal_async::i2c;
use lsm6ds33::{self, *};

derived!(ucum, UCUM: RadianPerSecond = Radian / Second);
//const RADPS: RadianPerSecond<f32> = RAD / S;

pub struct Imu<I2C> {
    i2c: Option<I2C>,
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
    pub fn new(i2c: I2C) -> Self {
        Self { i2c: Some(i2c) }
    }
    async fn run_imu<M>(
        &mut self,
        lsm6ds33: &mut Lsm6ds33Async<I2C>,
        _inbox: &mut M,
    ) -> Result<(), Error<I2C>>
    where
        M: Inbox<ImuMessage>,
    {
        lsm6ds33
            .set_accelerometer_scale(AccelerometerScale::G02)
            .await?;
        lsm6ds33
            .set_gyroscope_scale(GyroscopeFullScale::Dps245)
            .await?;
        lsm6ds33
            .set_accelerometer_output(AccelerometerOutput::Rate104)
            .await?;
        lsm6ds33
            .set_gyroscope_output(GyroscopeOutput::Rate104)
            .await?;
        lsm6ds33
            .set_accelerometer_bandwidth(AccelerometerBandwidth::Freq400)
            .await?;
        lsm6ds33.set_low_power_mode(true).await?;
        loop {
            let accel_raw = lsm6ds33.read_accelerometer().await?; // m/s^2
            let accel: [MeterPerSecond2<f32>; 3] =
                [accel_raw.0 * MPS2, accel_raw.1 * MPS2, accel_raw.2 * MPS2];
            let gyro_raw = lsm6ds33.read_gyro().await?; // rads/s
            let gyro: [RadianPerSecond<f32>; 3] = [
                gyro_raw.0 * RAD / S,
                gyro_raw.1 * RAD / S,
                gyro_raw.2 * RAD / S,
            ];
            let temp = (lsm6ds33.read_temperature().await? + CELSIUS_ZERO) * K;
            let accel_mag =
                (accel[0].powi(P2::new()) + accel[1].powi(P2::new()) + accel[2].powi(P2::new()))
                    .sqrt();

            trace!(
                "accel: {} m/s^2 gyro: {} rad/s accel_mag: {} m/s^2",
                accel.map(|v| *(v / MPS2).value()),
                gyro.map(|v| *(v / (RAD / S)).value()),
                (accel_mag / MPS2).value(),
            );

            STATE.lock(|c| {
                c.update(|s| {
                    let mut s = s;
                    s.accel_mag = Some(accel_mag);
                    if s.accel_mag.map_or(false, |v| v > 12.0 * MPS2) {
                        s.move_timer.update();
                    }
                    s.accel_temperature = Some(temp);
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
            let mut i2c = self.i2c.take().unwrap();
            info!("initializing imu");
            let mut lsm6ds33 = Lsm6ds33Async::new(i2c, LSM6DS33_ADDR).await;
            match lsm6ds33 {
                Ok(mut lsm6ds33) => {
                    let res = self.run_imu(&mut lsm6ds33, &mut inbox).await;
                    self.i2c.replace(lsm6ds33.release());
                }
                Err((i2c, e)) => {
                    error!("constructing lsm6ds33 failed: {:?}", Debug2Format(&e));
                    self.i2c.replace(i2c);
                }
            };
            /*res.map_err(|e| {
                error!("run_imu failed: {:?}", Debug2Format(&e));
            });*/
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}
