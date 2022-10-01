use crate::{peripherals};
use paste::paste;

macro_rules! define_pin {
    ($name:ident, $pin:ident) => {
        paste! {
                pub type [<Pin $name>] = peripherals::$pin;
                #[macro_export]
            macro_rules! [<use_pin_ $name:snake>] {
                ($pstruct:ident) => {
                    $pstruct.$pin
                }
            }
        }
    };
}

// Pins
// * = exposed on header
// 2* / P0.10
// 3 / P1.11 - Gyro + Accel IRQ
// 4 / P1.10 - Blue LED ("Conn")
//define_pin!(ConnLed, P1_10);
#[cfg(not(feature = "mdbt50q"))]
define_pin!(Led, P1_10);
// 5* / P1.08 - NeoPixel Connector (Propmaker)
define_pin!(Underlight, P1_08);
// 6* / P0.07 - IRQ (Propmaker)
// 7 / P1.02 - Button
// 8 / P0.16 - NeoPixel
define_pin!(NeoPixel, P0_16);
// 9* / P0.26 - Button (Propmaker)
// 10* / P0.27 - Power Enable (Propmaker)
define_pin!(PowerEnable, P0_27);
// 11* / P0.06 - Red LED (Propmaker)
define_pin!(TailC, P0_06);
// 12* / P0.08 - Green LED (Propmaker)
define_pin!(TailL, P0_08);
// 13* / P1.09 - Blue LED (Propmaker)
// 13* / P1.09 - Red LED
define_pin!(TailR, P1_09);
// 34 / P0.00 - PDM microphone data
define_pin!(PdmData, P0_00);
// 35 / P0.01 - PDM microphone clock
define_pin!(PdmClock, P0_01);
// 36 / P1.00 - APDS9960 Light Gesture Proximity IRQ
// 14 / A0* / P0.04 - Audio Amp Input (Propmaker)
// 15 / A1* / P0.05
define_pin!(Rst, P0_05);
// 16 / A2* / P0.30
// 17 / A3* / P0.28
// 18 / A4* / P0.02
// 19 / A5* / P0.03 - Headlight dim
define_pin!(HeadlightDim, P0_03);
// 20 / A6 / P0.29 - VOLTAGE_MONITOR (100K+100K voltage divider)
define_pin!(Vbat, P0_29);
// 21 / A7 / ???
// 22 / SDA* / P0.12 - I2C data
define_pin!(Sda, P0_12);
// 23 / SCL* / P0.11 - I2C clock
define_pin!(Scl, P0_11);
// 24 / MI* / P0.15
// 25 / MO* / P0.13
#[cfg(feature = "mdbt50q")]
define_pin!(Led, P0_13);
// 26 / SCK* / P0.14
// 0 / TX* / P0.25
// 1 / RX* / P0.24