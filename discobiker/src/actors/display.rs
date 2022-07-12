use arrayvec::ArrayString;
use core::fmt;
use core::fmt::Write;
use defmt::*;
use embassy::time::{Duration, Timer};
use embedded_graphics::prelude::*;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    text::{Baseline, Text},
};

use embedded_hal_async::i2c;

use display_interface::DisplayError;
use ssd1306::{
    mode::BufferedGraphicsMode, prelude::*, size::DisplaySize, I2CDisplayInterface, Ssd1306,
};

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
        loop {
            let mut buf = ArrayString::<COLS>::new();
            // Line 0: XX.XVext X.XXXA X.XXV
            core::write!(
                &mut buf,
                "{:04.1}Vext {:05.3}A {:04.2}{}",
                12.1,
                0.322,
                4.01,
                "V"
            )?;
            Text::with_baseline(&buf, Point::zero(), text_style, Baseline::Top)
                .draw(&mut self.display)?;
            // Line 1: XXX°XXX° XXX.X°
            // Line 2: XXXX.XXhPa XXX.XX'
            // Line 3: XXX°TXXX.XX°
            // Line 4: XXX.XXG    XXXXX lux
            // Line 5: Mode: Day XXX% XXs
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
