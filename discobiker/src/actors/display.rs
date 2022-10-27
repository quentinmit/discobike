use crate::{Debug2Format, CELSIUS_ZERO, Global, DesiredState, ActualState};
use core::fmt;
use core::fmt::Write;
use embassy_futures::yield_now;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_graphics::prelude::*;
use embedded_graphics::{
    mono_font::{iso_8859_1::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    text::{Baseline, Text},
};
use futures::select_biased;
use futures::{FutureExt, StreamExt};
use staticvec::{StaticString, StaticVec};

use embedded_hal_async::i2c;

use display_interface::DisplayError;
use ssd1306::{
    mode::BufferedGraphicsMode, prelude::*, size::DisplaySize, I2CDisplayInterface, Ssd1306,
};

use bincode;

extern crate dimensioned as dim;
use dim::f32prefixes::HECTO;
use dim::si::f32consts::{A, FT, K, LX, M, MPS2, PA, V};
use dim::si::Kelvin;
use dim::traits::{Dimensioned, Dimensionless, Map};
use num_traits::Float;
use physical_constants::STANDARD_ACCELERATION_OF_GRAVITY;

use ector::{actor, Actor, Address, Inbox};

const R: Kelvin<f32> = Kelvin::new(5.0 / 9.0);
const FAHRENHEIT_ZERO: f32 = CELSIUS_ZERO * 9.0 / 5.0 - 32.0;

pub struct Display<'d, I2C, SIZE: DisplaySize> {
    // interface, mode, size, addr_mode, rotation
    display: Ssd1306<I2CInterface<I2C>, SIZE, BufferedGraphicsMode<SIZE>>,
    state: &'d Global<ActualState>,
    desired_state: &'d Global<DesiredState>,
}

pub enum DisplayMessage {
    On,
    Off,
}

impl From<bool> for DisplayMessage {
    fn from(b: bool) -> Self {
        if b {
            Self::On
        } else {
            Self::Off
        }
    }
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
    T: Dimensioned<Value = f32> + core::ops::Div + core::ops::Mul,
    <T as Dimensioned>::Value: PartialOrd<f32> + fmt::Display,
    <T as core::ops::Div>::Output: Dimensionless,
    <<T as core::ops::Div>::Output as Dimensioned>::Value:
        core::fmt::Display + Copy + PartialOrd<f32>,
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
                let value = *(v / unit).value();
                let decimals = if decimals > 0 && value < 0.0 {
                    decimals - 1
                } else {
                    decimals
                };
                core::write!(
                    w,
                    "{:0width$.precision$}",
                    value,
                    width = width,
                    precision = decimals
                )?;
            }
        };
        Ok(())
    }
}

#[cfg(debug_assertions)]
const DISPLAY_PERIOD: Duration = Duration::from_millis(1000 / 2);
#[cfg(not(debug_assertions))]
const DISPLAY_PERIOD: Duration = Duration::from_millis(1000 / 10);

// DisplaySize128x64
impl<'d, I2C, E, SIZE> Display<'d, I2C, SIZE>
where
    I2C: i2c::I2c<Error = E>,
    SIZE: DisplaySize,
{
    pub fn new(state: &'d Global<ActualState>, desired_state: &'d Global<DesiredState>, i2c: I2C, size: SIZE) -> Self {
        let interface = I2CDisplayInterface::new(i2c);

        let display =
            Ssd1306::new(interface, size, DisplayRotation::Rotate180).into_buffered_graphics_mode();
        Self { display, state, desired_state }
    }

    async fn run_display<MBOX>(&mut self, inbox: &mut MBOX) -> Result<(), Error>
    where
        MBOX: Inbox<DisplayMessage>,
    {
        self.display.init().await?;
        self.display.set_display_on(false).await?;
        loop {
            match inbox.next().await {
                DisplayMessage::On => {
                    info!("Display on");
                    self.run_display_on(inbox).await?;
                }
                DisplayMessage::Off => warn!("Display off while already off"),
            }
        }
    }

    async fn run_display_on<MBOX>(&mut self, inbox: &mut MBOX) -> Result<(), Error>
    where
        MBOX: Inbox<DisplayMessage>,
    {
        use Error::FormatError;
        self.display.set_display_on(true).await?;
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();
        // Allow two bytes per character for utf-8
        const COLS: usize = 128 / 3;
        let line_height = text_style.font.character_size.height;
        let mut ticker = Ticker::every(DISPLAY_PERIOD);
        loop {
            let state = self.state.lock(|c| c.get());
            self.display.clear();
            let mut buf = StaticString::<COLS>::new();
            let desired_state = self.desired_state.lock(|c| c.get());

            if false {
                let mut x = StaticVec::<u8, 128>::filled_with(|| 0);
                match bincode::serde::encode_into_slice(
                    &state,
                    x.as_mut_slice(),
                    bincode::config::standard(),
                ) {
                    Err(e) => error!("serializing state: {}", Debug2Format(&e)),
                    Ok(n) => {
                        x.truncate(n);
                        info!("current state: {:?}", x.as_slice());
                    }
                }
            }

            let start_draw = Instant::now();
            // Line 0: XX.XVext X.XXXA X.XXV
            // vext, current, vbat
            state
                .vext
                .write_dim(&mut buf, V, 4, 1)
                .map_err(|_| FormatError(0))?;
            buf.push_str_truncating("Vext ");
            state
                .current
                .write_dim(&mut buf, A, 5, 3)
                .map_err(|_| FormatError(0))?;
            buf.push_str_truncating("A ");
            state
                .vbat
                .write_dim(&mut buf, V, 4, 2)
                .map_err(|_| FormatError(0))?;
            buf.push_str_truncating(if state.vbus_detected { "V" } else { "v" });
            Text::with_baseline(&buf, Point::zero(), text_style, Baseline::Top)
                .draw(&mut self.display)?;
            yield_now().await;
            buf.clear();
            // Line 1: XXX°XXX° XXX.X°
            // heading, roll, bmp280 temperature
            buf.push_str_truncating("XXX°XXX°");
            match state.accel_temperature {
                None => buf.push_str_truncating("---.-°"),
                Some(temp) => {
                    trace!(
                        "Temp = {}°K ({}°C)",
                        (temp / K).value(),
                        (temp / K).value() - CELSIUS_ZERO
                    );
                    if let Err(_) = core::write!(&mut buf, "{:5.1}°", (temp / R) - FAHRENHEIT_ZERO)
                    {
                        buf.push_str_truncating("ERR");
                    }
                }
            };
            match state.temperature {
                None => buf.push_str_truncating("---.-°"),
                Some(temp) => {
                    trace!(
                        "Temp = {}°K ({}°C)",
                        (temp / K).value(),
                        (temp / K).value() - CELSIUS_ZERO
                    );
                    if let Err(_) = core::write!(&mut buf, "{:5.1}°", (temp / R) - FAHRENHEIT_ZERO)
                    {
                        buf.push_str_truncating("ERR");
                    }
                }
            };
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
            // pressure, altitude
            state
                .pressure
                .write_dim(&mut buf, HECTO * PA, 7, 2)
                .map_err(|_| FormatError(2))?;
            buf.push_str_truncating("hPa ");
            let altitude = state
                .pressure
                .map(|p| 44330.0 * M * (1.0 - (p / (1013.0 * HECTO * PA)).map(|v| v.powf(0.1903))));
            altitude
                .write_dim(&mut buf, FT, 6, 2)
                .map_err(|_| FormatError(2))?;
            buf.push_str_truncating("'");
            Text::with_baseline(
                &buf,
                Point::new(0, 2 * line_height as i32),
                text_style,
                Baseline::Top,
            )
            .draw(&mut self.display)?;
            yield_now().await;
            buf.clear();
            // Line 3: XXX°TXXX.XX°
            // debug: raw heading, mag_ok, acelerometer temperature
            buf.push_str_truncating("UL: ");
            core::write!(
                &mut buf,
                "{:?} {:02X}",
                desired_state.underlight_mode,
                state.underlight_brightness
            )
            .map_err(|_| FormatError(3))?;
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
            // accel magnitude, lux
            state
                .accel_mag
                .write_dim(
                    &mut buf,
                    STANDARD_ACCELERATION_OF_GRAVITY as f32 * MPS2,
                    6,
                    2,
                )
                .map_err(|_| FormatError(4))?;
            buf.push_str_truncating("G    ");
            state
                .lux
                .write_dim(&mut buf, LX, 5, 0)
                .map_err(|_| FormatError(4))?;
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
            // headlight mode, headlight brightness, seconds until off
            buf.push_str_truncating("Headlight: ");
            core::write!(
                &mut buf,
                "{:3.0}% {}",
                state.headlight_brightness * 100.0,
                state.move_timer,
            )
            .map_err(|_| FormatError(5))?;
            Text::with_baseline(
                &buf,
                Point::new(0, 5 * line_height as i32),
                text_style,
                Baseline::Top,
            )
            .draw(&mut self.display)?;
            yield_now().await;
            buf.clear();
            info!("display.draw took {} µs", start_draw.elapsed().as_micros());
            let start_flush = Instant::now();
            self.display.flush().await?;
            info!(
                "display.flush took {} µs",
                start_flush.elapsed().as_micros()
            );

            select_biased! {
                message = inbox.next().fuse() => {
                    match message {
                        DisplayMessage::On => warn!("display on while already on"),
                        DisplayMessage::Off => {
                            info!("display off");
                            self.display.set_display_on(false).await?;
                            return Ok(());
                        }
                    }
                },
                _ = ticker.next().fuse() => (),
            };
        }
    }
}

#[actor]
impl<I2C, E, SIZE> Actor for Display<'_, I2C, SIZE>
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
