use crate::{DESIRED_STATE, STATE};
use core::fmt;
use core::fmt::Write;
use defmt::{panic, *};
use embassy_executor::time::{Duration, Instant, Timer};
use embassy_util::yield_now;
use embedded_graphics::prelude::*;
use embedded_graphics::{
    mono_font::{iso_8859_1::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    text::{Baseline, Text},
};
use futures::select_biased;
use futures::FutureExt;
use staticvec::{StaticString, StaticVec};

use embedded_hal_async::i2c;

use display_interface::DisplayError;
use ssd1306::{
    mode::BufferedGraphicsMode, prelude::*, size::DisplaySize, I2CDisplayInterface, Ssd1306,
};

use bincode;

extern crate dimensioned as dim;
use dim::si::f32consts::{A, LX, MPS2, V};
use dim::traits::Dimensioned;
use physical_constants::STANDARD_ACCELERATION_OF_GRAVITY;

use ector::{actor, Actor, Address, Inbox};

pub struct Display<I2C, SIZE: DisplaySize> {
    // interface, mode, size, addr_mode, rotation
    display: Ssd1306<I2CInterface<I2C>, SIZE, BufferedGraphicsMode<SIZE>>,
}

pub enum DisplayMessage {
    On,
    Off,
}

#[derive(Debug)]
pub enum Error {
    DisplayError(DisplayError),
    FormatError(isize),
}

impl From<DisplayError> for Error {
    fn from(e: DisplayError) -> Self {
        Self::DisplayError(e)
    }
}

impl From<fmt::Error> for Error {
    fn from(_e: fmt::Error) -> Self {
        Self::FormatError(-1)
    }
}

trait WriteDim<T, W: Write> {
    fn write_dim(self, w: &mut W, unit: T, width: usize, decimals: usize) -> fmt::Result;
}

impl<T, W> WriteDim<T, W> for Option<T>
where
    T: Dimensioned + core::ops::Div + core::ops::Mul,
    <T as Dimensioned>::Value: PartialOrd<f32>,
    <T as core::ops::Div>::Output: fmt::Display,
    W: Write,
{
    fn write_dim(self, w: &mut W, unit: T, width: usize, decimals: usize) -> fmt::Result {
        match self {
            None => {
                for _ in 0..width - decimals - 1 {
                    w.write_char('-')?;
                }
                if decimals > 0 {
                    w.write_char('.')?;
                    for _ in 0..decimals {
                        w.write_char('-')?;
                    }
                } else {
                    w.write_char('-')?;
                };
            }
            Some(v) => {
                let decimals = if decimals > 0 && v.value_unsafe() < &0.0 {
                    decimals - 1
                } else {
                    decimals
                };
                core::write!(
                    w,
                    "{:0width$.precision$}",
                    v / unit,
                    width = width,
                    precision = decimals
                )?;
            }
        };
        Ok(())
    }
}

// DisplaySize128x64
impl<I2C, E, SIZE> Display<I2C, SIZE>
where
    I2C: i2c::I2c<Error = E>,
    SIZE: DisplaySize,
{
    pub fn new(i2c: I2C, size: SIZE) -> Self {
        let interface = I2CDisplayInterface::new(i2c);

        let mut display =
            Ssd1306::new(interface, size, DisplayRotation::Rotate180).into_buffered_graphics_mode();
        Self { display }
    }

    async fn run_display<M>(&mut self, inbox: &mut M) -> Result<(), Error>
    where
        M: Inbox<DisplayMessage>,
    {
        use Error::FormatError;
        self.display.init().await?;
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();
        const COLS: usize = 128 / 6;
        let line_height = text_style.font.character_size.height;
        loop {
            let state = STATE.lock(|c| c.get());
            if state.display_on {
                self.display.clear();
                let mut buf = StaticString::<COLS>::new();
                let desired_state = DESIRED_STATE.lock(|c| c.get());

                let mut x = StaticVec::<u8, 128>::filled_with(|| 0);
                match bincode::serde::encode_into_slice(
                    &state,
                    x.as_mut_slice(),
                    bincode::config::standard(),
                ) {
                    Err(e) => error!("serializing state: {}", Debug2Format(&e)),
                    Ok(n) => {
                        x.truncate(n);
                        info!("current state: {=[u8]}", x.as_slice());
                    }
                }

                // Line 0: XX.XVext X.XXXA X.XXV
                state.vext.write_dim(&mut buf, V, 4, 1).map_err(|_| FormatError(0))?;
                buf.push_str_truncating("Vext ");
                state.current.write_dim(&mut buf, A, 5, 3).map_err(|_| FormatError(0))?;
                buf.push_str_truncating("A ");
                state.vbat.write_dim(&mut buf, V, 4, 2).map_err(|_| FormatError(0))?;
                buf.push_str_truncating(if state.vbus_detected { "V" } else { "v" });
                Text::with_baseline(&buf, Point::zero(), text_style, Baseline::Top)
                    .draw(&mut self.display)?;
                yield_now().await;
                buf.clear();
                // Line 1: XXX°XXX° XXX.X°
                buf.push_str_truncating("XXX°XXX° XXX.X°");
                Text::with_baseline(
                    &buf,
                    Point::new(0, 1 * line_height as i32),
                    text_style,
                    Baseline::Top,
                )
                .draw(&mut self.display)?;
                yield_now().await;
                buf.clear();
                // Line 2: XXXX.XXhPa XXX.XX'
                // Line 3: XXX°TXXX.XX°
                buf.push_str_truncating("UL: ");
                core::write!(
                    &mut buf,
                    "{:?} {:02X}",
                    desired_state.underlight_mode,
                    state.underlight_brightness
                ).map_err(|_| FormatError(3))?;
                Text::with_baseline(
                    &buf,
                    Point::new(0, 3 * line_height as i32),
                    text_style,
                    Baseline::Top,
                )
                .draw(&mut self.display)?;
                yield_now().await;
                buf.clear();
                // Line 4: XXX.XXG    XXXXX lux
                state.accel_mag.write_dim(
                    &mut buf,
                    STANDARD_ACCELERATION_OF_GRAVITY as f32 * MPS2,
                    6,
                    2,
                ).map_err(|_| FormatError(4))?;
                buf.push_str_truncating("G    ");
                state.lux.write_dim(&mut buf, LX, 5, 0).map_err(|_| FormatError(4))?;
                buf.push_str_truncating(" lux");
                Text::with_baseline(
                    &buf,
                    Point::new(0, 4 * line_height as i32),
                    text_style,
                    Baseline::Top,
                )
                .draw(&mut self.display)?;
                yield_now().await;
                buf.clear();
                // Line 5: Mode: Day XXX% XXs
                buf.push_str_truncating("Mode: ");
                core::write!(
                    &mut buf,
                    "{:?} {:3.0} {}",
                    state.headlight_mode,
                    state.headlight_brightness * 100.0,
                    state.move_timer,
                ).map_err(|_| FormatError(5))?;
                Text::with_baseline(
                    &buf,
                    Point::new(0, 5 * line_height as i32),
                    text_style,
                    Baseline::Top,
                )
                .draw(&mut self.display)?;
                yield_now().await;
                buf.clear();
                let start_flush = Instant::now();
                self.display.flush().await?;
                info!(
                    "display.flush took {} µs",
                    start_flush.elapsed().as_micros()
                );
            }

            select_biased! {
                message = inbox.next().fuse() => {
                    match message {
                        DisplayMessage::On => info!("display on"),
                        DisplayMessage::Off => info!("display off"),
                    }
                },
                _ = Timer::after(Duration::from_millis(1000/5)).fuse() => (),
            };
        }
        Ok(())
    }
}

#[actor]
impl<I2C, E, SIZE> Actor for Display<I2C, SIZE>
where
    I2C: i2c::I2c<Error = E>,
    SIZE: DisplaySize,
    E: fmt::Debug,
{
    type Message<'m> = DisplayMessage;

    async fn on_mount<M>(&mut self, _: Address<DisplayMessage>, mut inbox: M)
    where
        M: Inbox<DisplayMessage>,
    {
        loop {
            info!("run_display");
            if let Err(e) = self.run_display(&mut inbox).await {
                error!("run_display failed: {:?}", Debug2Format(&e));
            }
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}
