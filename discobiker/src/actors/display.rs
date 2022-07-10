use embedded_graphics::prelude::*;

use embedded_hal_async::i2c;

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306, size::DisplaySize, mode::BufferedGraphicsMode};

use ector::{actor, Actor, Address, Inbox};

pub struct Display<I2C, SIZE: DisplaySize> {
    // interface, mode, size, addr_mode, rotation
    display: Ssd1306<I2CInterface<I2C>, SIZE, BufferedGraphicsMode<SIZE>>
}

pub enum DisplayMessage {
    On,
    Off,
}

// DisplaySize128x64
impl<I2C, E, SIZE> Display<I2C, SIZE>
where
    I2C: i2c::I2c<Error = E>,
    SIZE: DisplaySize,
{
    pub fn new(i2c: I2C, size: SIZE) -> Self {
        let interface = I2CDisplayInterface::new(i2c);

        let mut display = Ssd1306::new(interface, size, DisplayRotation::Rotate180)
            .into_buffered_graphics_mode();
        Self { display }
    }
}

#[actor]
impl<I2C, E, SIZE> Actor for Display<I2C, SIZE>
where
    I2C: i2c::I2c<Error = E>,
    SIZE: DisplaySize,
{
    type Message<'m> = DisplayMessage;

    async fn on_mount<M>(&mut self, _: Address<DisplayMessage>, mut inbox: M)
    where M: Inbox<DisplayMessage>
    {
        self.display.init().await.unwrap();

        loop {}
    }
}
