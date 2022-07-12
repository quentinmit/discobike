use staticvec::StaticString;
use crate::STATE;
use core::fmt;
use core::fmt::Write;
use defmt::*;
use embassy::time::{Duration, Timer};
use embedded_graphics::prelude::*;
use embedded_graphics::{
    mono_font::{iso_8859_1::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    text::{Baseline, Text},
};

use embedded_hal_async::i2c;

use display_interface::DisplayError;
use ssd1306::{
    mode::BufferedGraphicsMode, prelude::*, size::DisplaySize, I2CDisplayInterface, Ssd1306,
};

extern crate dimensioned as dim;
use dim::si::{f32consts::{V, A, MPS2, LX}};
use dim::traits::Dimensioned;

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
    FormatError(fmt::Error),
}

impl From<DisplayError> for Error {
    fn from(e: DisplayError) -> Self {
        Self::DisplayError(e)
    }
}

impl From<fmt::Error> for Error {
    fn from(e: fmt::Error) -> Self {
        Self::FormatError(e)
    }
}

trait WriteDim<T, W: Write> {
    fn write_dim(self, w: &mut W, unit: T, width: usize, decimals: usize) -> fmt::Result;
}

impl <T, W> WriteDim<T, W> for Option<T>
where
    T: Dimensioned + core::ops::Div + core::ops::Mul,
    <T as Dimensioned>::Value: PartialOrd<f32>,
    <T as core::ops::Div>::Output: fmt::Display,
    W: Write,
{
    fn write_dim(self, w: &mut W, unit: T, width: usize, decimals: usize) -> fmt::Result
    {
        match self {
            None => {
                for _ in 0..width-decimals-1 {
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
            },
            Some(v) => {
                let decimals = if decimals > 0 && v.value_unsafe() < &0.0 { decimals - 1 } else { decimals };
                core::write!(w, "{:0width$.precision$}", v/unit, width = width, precision = decimals)?;
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

    async fn run_display<M>(&mut self, mut inbox: &M) -> Result<(), Error>
    where
        M: Inbox<DisplayMessage>,
    {
        self.display.init().await?;
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();
        const COLS: usize = 128 / 6;
        let line_height = text_style.font.character_size.height;
        loop {
            self.display.clear();
            let mut buf = StaticString::<COLS>::new();
            let state = STATE.lock(|c| c.get());
            // Line 0: XX.XVext X.XXXA X.XXV
            state.vext.write_dim(&mut buf, V, 4, 1)?;
            buf.push_str_truncating("Vext ");
            state.current.write_dim(&mut buf, A, 5, 3)?;
            buf.push_str_truncating("A ");
            state.vbat.write_dim(&mut buf, V, 4, 2)?;
            buf.push_str_truncating(if state.vbus_detected { "V" } else { "v" });
            Text::with_baseline(&buf, Point::zero(), text_style, Baseline::Top)
                .draw(&mut self.display)?;
            buf.clear();
            // Line 1: XXX°XXX° XXX.X°
            buf.push_str_truncating("XXX°XXX° XXX.X°");
            Text::with_baseline(&buf, Point::new(0, 1*line_height as i32), text_style, Baseline::Top)
                .draw(&mut self.display)?;
            buf.clear();
            // Line 2: XXXX.XXhPa XXX.XX'
            // Line 3: XXX°TXXX.XX°
            // Line 4: XXX.XXG    XXXXX lux
            // Line 5: Mode: Day XXX% XXs
            buf.push_str_truncating("Mode: ");
            core::write!(&mut buf, "{:?} {:3} {}", state.headlight_mode, state.headlight_brightness * 100.0, state.move_timer)?;
            Text::with_baseline(&buf, Point::new(0, 5*line_height as i32), text_style, Baseline::Top)
                .draw(&mut self.display)?;
            buf.clear();
            info!("display flush start");
            self.display.flush().await?;
            info!("display flush end");
            Timer::after(Duration::from_millis(1000/5)).await;
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