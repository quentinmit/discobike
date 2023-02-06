#![no_std]

#![feature(type_alias_impl_trait)]
#![feature(generic_associated_types)]
#![feature(cell_update)]
#![feature(result_option_inspect)]
#![feature(int_log)]
#![feature(trace_macros)]
#![feature(generic_arg_infer)]
#![feature(mixed_integer_ops)]

use embassy_nrf::pac;
use embassy_nrf::peripherals;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embassy_sync::blocking_mutex::{
    raw::{CriticalSectionRawMutex, ThreadModeRawMutex},
    Mutex as BlockingMutex,
};

use core::cell::{Cell, RefCell};
use core::fmt;
use core::convert::{TryFrom, TryInto};
use core::num::TryFromIntError;
use serde::{Deserialize, Serialize, Serializer};
use dim::si::{f32consts::V, Ampere, Kelvin, Lux, MeterPerSecond2, Pascal, Volt};
use num_enum::TryFromPrimitive;
use drogue_device::drivers::led::neopixel::rgbw::Rgbw8;

use nom;
use nom::combinator::map_res;
use nom::error::ParseError;
use nom::Parser;
use nom_protobuf;

#[cfg(feature = "defmt")]
pub use defmt::Debug2Format;
#[allow(non_snake_case)]
#[cfg(not(feature = "defmt"))]
pub fn Debug2Format<'a, T: fmt::Debug>(item: &'a T) -> &'a T {
    item
}
#[macro_use]
extern crate dimensioned as dim;

#[macro_export]
pub mod log;

pub mod actors;
pub mod atomic_counter;
pub mod drivers;
mod pins;
pub use pins::*;

const CELSIUS_ZERO: f32 = 273.15;

#[derive(Copy, Clone)]
pub struct EventTimer {
    last: Instant,
}
impl EventTimer {
    pub const fn new() -> Self {
        EventTimer { last: Instant::MIN }
    }
    pub fn update(&mut self) {
        self.last = Instant::now();
    }
    pub fn elapsed(&self) -> Duration {
        self.last.elapsed()
    }
}

impl Serialize for EventTimer {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_f32(self.last.elapsed().as_micros() as f32 / 1e6)
    }
}

impl fmt::Display for EventTimer {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        if self.last == Instant::MIN {
            core::write!(formatter, "NEVER")
        } else {
            core::write!(formatter, "{}s", (Instant::now() - self.last).as_secs())
        }
    }
}

#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize, TryFromPrimitive)]
pub enum HeadlightMode {
    Off = 0,
    Auto = 1,
    Day = 2,
    Night = 3,
    Blink = 4,
    AutoDim = 5,
}

impl TryFrom<i32> for HeadlightMode {
    type Error = <Self as TryFrom<u8>>::Error;

    fn try_from(value: i32) -> Result<Self, Self::Error> {
        (value as u8).try_into()
    }
}

#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize, TryFromPrimitive)]
pub enum UnderlightMode {
    Off = 0,
    Auto = 1,
    On = 2,
    ForceOn = 3,
}

impl TryFrom<i32> for UnderlightMode {
    type Error = <Self as TryFrom<u8>>::Error;

    fn try_from(value: i32) -> Result<Self, Self::Error> {
        (value as u8).try_into()
    }
}

#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize, TryFromPrimitive)]
pub enum Effect {
    Solid = 0,
    ColorWipe,
    TheaterChase,
    Rainbow,
    TheaterChaseRainbow,
    CylonBounce,
    Fire,
    VuMeter,
    RgbVuMeter,
    Pulse,
    Traffic,
    Halloween,
}

impl TryFrom<i32> for Effect {
    type Error = <Self as TryFrom<u8>>::Error;

    fn try_from(value: i32) -> Result<Self, Self::Error> {
        (value as u8).try_into()
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone)]
pub struct DesiredState {
    pub headlight_mode: HeadlightMode,
    pub underlight_mode: UnderlightMode,
    pub underlight_effect: Effect,
    pub underlight_speed: i16,
    pub underlight_color: Rgbw8,
}

impl DesiredState {
    pub fn parse_from<'a, E>(&mut self, input: &'a [u8]) -> Result<(), nom::Err<E>>
    where E: ParseError<&'a [u8]> + nom::error::FromExternalError<&'a [u8], num_enum::TryFromPrimitiveError<HeadlightMode>> + nom::error::FromExternalError<&'a [u8], num_enum::TryFromPrimitiveError<UnderlightMode>> + nom::error::FromExternalError<&'a [u8], num_enum::TryFromPrimitiveError<Effect>> + nom::error::FromExternalError<&'a [u8], TryFromIntError>
    {
        let mut i = input;
        while i.len() > 0 {
            let (remainder, tag) = nom_protobuf::take_tag(i)?;
            i = match tag.field_number {
                0 => {let (r, v) = nom_protobuf::scalar::take_enum(tag.wire_type).parse(i)?; self.headlight_mode = v; r},
                1 => {let (r, v) = nom_protobuf::scalar::take_enum(tag.wire_type).parse(i)?; self.underlight_mode = v; r},
                2 => {let (r, v) = nom_protobuf::scalar::take_enum(tag.wire_type).parse(i)?; self.underlight_effect = v; r},
                3 => {let (r, v) = map_res(nom_protobuf::scalar::take_sint32(tag.wire_type), |v| v.try_into()).parse(i)?; self.underlight_speed = v; r},
                4 => {let (r, v) = nom_protobuf::scalar::take_uint32(tag.wire_type).parse(i)?; let v = v.to_be_bytes(); self.underlight_color = Rgbw8::new(v[0], v[1], v[2], v[3]); r},
                _ => nom::combinator::fail::<_, (), _>(i)?.0,
            };
        }
        Ok(())
    }
}

#[derive(Copy, Clone, Serialize)]
pub struct ActualState {
    pub headlight_brightness: f32,
    pub taillight_brightness: f32,
    pub underlight_brightness: u8,
    pub display_on: bool,
    pub vbus_detected: bool,
    pub vext_low: bool,

    pub vbus_timer: EventTimer,
    pub move_timer: EventTimer,
    pub vext_present_timer: EventTimer,
    pub vext_poll_timer: EventTimer,

    pub vbat: Option<Volt<f32>>,
    pub vext: Option<Volt<f32>>,
    pub current: Option<Ampere<f32>>,
    pub accel_mag: Option<MeterPerSecond2<f32>>,
    pub accel_temperature: Option<Kelvin<f32>>,
    pub lux: Option<Lux<f32>>,
    pub pressure: Option<Pascal<f32>>,
    pub temperature: Option<Kelvin<f32>>,
}

pub type Global<T> = BlockingMutex<CriticalSectionRawMutex, Cell<T>>;
