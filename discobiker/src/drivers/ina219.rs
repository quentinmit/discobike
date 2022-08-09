use defmt::*;
use modular_bitfield::prelude::*;
extern crate dimensioned as dim;
use dim::f32prefixes::*;
use dim::si::f32consts::{A, OHM, V};
use dim::si::{Ampere, Ohm, Volt};
use maybe_async_cfg;

use embedded_hal::i2c::blocking as i2c_mod;
#[cfg(feature = "async")]
use embedded_hal_async::i2c as i2c_mod_async;

use byteorder::{BigEndian, ByteOrder, LittleEndian};

pub const INA219_ADDR: u8 = 0x40;

enum Register {
    Configuration = 0x00,
    ShuntVoltage = 0x01,
    BusVoltage = 0x02,
    Power = 0x03,
    Current = 0x04,
    Calibration = 0x05,
}

#[derive(Copy, Clone, BitfieldSpecifier)]
#[bits = 1]
enum BusVoltageRange {
    V16 = 0,
    V32 = 1,
}
#[derive(Copy, Clone, BitfieldSpecifier, Debug, Format)]
#[bits = 2]
enum Gain {
    Full = 0,
    Half = 1,
    Quarter = 2,
    Eighth = 3,
}
#[allow(non_upper_case_globals)]
impl Gain {
    pub const mV40: Gain = Gain::Full;
    pub const mV80: Gain = Gain::Half;
    pub const mV160: Gain = Gain::Quarter;
    pub const mV320: Gain = Gain::Eighth;

    pub fn for_max_voltage(max_voltage: Volt<f32>) -> Option<Self> {
        if max_voltage < 40. * MILLI * V {
            Some(Self::mV40)
        } else if max_voltage < 80. * MILLI * V {
            Some(Self::mV80)
        } else if max_voltage <= 160. * MILLI * V {
            Some(Self::mV160)
        } else if max_voltage <= 320. * MILLI * V {
            Some(Self::mV320)
        } else {
            None
        }
    }
}
#[derive(Copy, Clone, BitfieldSpecifier)]
#[bits = 4]
pub enum ADCMode {
    BitMode9 = 0b00000000,
    BitMode10 = 0b00000001,
    BitMode11 = 0b00000010,
    BitMode12 = 0b00000011,
    SampleMode2 = 0b00001001,
    SampleMode4 = 0b00001010,
    SampleMode8 = 0b00001011,
    SampleMode16 = 0b00001100,
    SampleMode32 = 0b00001101,
    SampleMode64 = 0b00001110,
    SampleMode128 = 0b00001111,
}
#[derive(Copy, Clone)]
#[bitfield(bits = 16)]
struct Configuration {
    sample_shunt: bool,
    sample_bus: bool,
    sample_continuous: bool,
    shunt_adc_mode: ADCMode,
    bus_adc_mode: ADCMode,
    shunt_gain: Gain,
    bus_voltage_range: BusVoltageRange,
    #[skip]
    unused1: bool,
    reset: bool,
}

macro_rules! bitfield_u16 {
    ($bitfield:ty) => {
        impl From<u16> for $bitfield {
            fn from(val: u16) -> Self {
                let mut buf = [0; 2];
                LittleEndian::write_u16(&mut buf, val);
                Self::from_bytes(buf)
            }
        }
        impl From<$bitfield> for u16 {
            fn from(val: $bitfield) -> u16 {
                let bytes = val.into_bytes();
                LittleEndian::read_u16(&bytes)
            }
        }
    };
}

bitfield_u16!(Configuration);

#[bitfield(bits = 16)]
struct BusVoltage {
    overflow: bool,
    conversion_ready: bool,
    #[skip]
    unused1: bool,
    bus_voltage: B13,
}

bitfield_u16!(BusVoltage);

#[maybe_async_cfg::maybe(sync(keep_self), async(feature = "async"))]
pub struct INA219<I2C> {
    i2c: I2C,
    address: u8,
    last_config: Option<Configuration>,
    last_calibration: Option<u16>,
    // Need to save this to convert back.
    shunt_resistance: Option<Ohm<f32>>,
}

#[maybe_async_cfg::maybe(sync(keep_self), async(feature = "async", idents(i2c_mod(fn))))]
impl<I2C, E> INA219<I2C>
where
    I2C: i2c_mod::I2c<Error = E>,
{
    pub fn new(i2c: I2C, address: u8) -> INA219<I2C> {
        INA219 {
            i2c,
            address,
            last_config: None,
            last_calibration: None,
            shunt_resistance: None,
        }
    }

    pub async fn power_down(&mut self) -> Result<(), E> {
        self.modify_config(|c| {
            c.with_sample_continuous(false)
                .with_sample_bus(false)
                .with_sample_shunt(false)
        })
        .await
    }
    pub async fn trigger_sample(&mut self, bus: bool, shunt: bool) -> Result<(), E> {
        self.modify_config(|c| {
            c.with_sample_continuous(false)
                .with_sample_bus(bus)
                .with_sample_shunt(shunt)
        })
        .await?;
        loop {
            let v: BusVoltage = self.read_register(Register::BusVoltage).await?.into();
            if v.conversion_ready() {
                return Ok(());
            }
        }
    }
    pub async fn continuous_sample(&mut self, bus: bool, shunt: bool) -> Result<(), E> {
        self.modify_config(|c| {
            c.with_sample_continuous(true)
                .with_sample_bus(bus)
                .with_sample_shunt(shunt)
        })
        .await
    }
    pub async fn get_bus_voltage(&mut self) -> Result<Volt<f32>, E> {
        let config = self.config().await?;
        let v: BusVoltage = self.read_register(Register::BusVoltage).await?.into();
        // LSB = 4 mV
        Ok(v.bus_voltage() as f32 * 4.0 * MILLI * V)
    }
    pub async fn get_shunt_voltage(&mut self) -> Result<Volt<f32>, E> {
        let v = self.read_register(Register::ShuntVoltage).await? as i16;
        // LSB = 10 µV
        Ok(v as f32 * 10.0 * MICRO * V)
    }

    pub async fn get_current(&mut self) -> Result<Option<Ampere<f32>>, E> {
        let v = self.read_register(Register::Current).await? as i16;
        let lsb = self.current_lsb().await?;
        Ok(lsb.map(|lsb| (v as f32) * lsb))
    }

    async fn config(&mut self) -> Result<Configuration, E> {
        Ok(match self.last_config {
            Some(c) => c,
            None => {
                let config: Configuration =
                    self.read_register(Register::Configuration).await?.into();
                self.last_config = Some(config);
                config
            }
        })
    }

    async fn gain(&mut self) -> Result<Gain, E> {
        Ok(self.config().await?.shunt_gain())
    }

    async fn current_lsb(&mut self) -> Result<Option<Ampere<f32>>, E> {
        let cal = match self.last_calibration {
            Some(c) => c,
            None => {
                let c = self.read_register(Register::Calibration).await?;
                self.last_calibration = Some(c);
                c
            }
        };
        Ok(self
            .shunt_resistance
            .zip(if cal != 0 { Some(cal) } else { None })
            .map(|(r, cal)| 0.04096 * V / ((cal as f32) * r)))
    }

    pub async fn set_current_range(
        &mut self,
        max_current: Ampere<f32>,
        shunt_resistance: Ohm<f32>,
    ) -> Result<(), E> {
        // 1. Select gain
        let max_vshunt: Volt<f32> = max_current * shunt_resistance;
        let desired_gain = Gain::for_max_voltage(max_vshunt).unwrap_or(Gain::mV320);

        let desired_current_lsb: Ampere<f32> = max_current / 32767.;
        let cal = *(0.04096 * V / (desired_current_lsb * shunt_resistance)) as u16;
        let actual_current_lsb: Ampere<f32> = 0.04096 * V / ((cal as f32) * shunt_resistance);
        info!("requested max current = {=f32} A", max_current / A);
        info!("optimal gain setting {:?}", Debug2Format(&desired_gain));
        info!("calculated desired current LSB {=f32} µA calibration {=u16} actual current LSB {=f32} µA", desired_current_lsb/(MICRO*A), cal, actual_current_lsb/(MICRO*A));
        self.modify_config(|c| c.with_shunt_gain(desired_gain))
            .await?;
        self.write_register(Register::Calibration, cal).await?;
        self.last_calibration = Some(cal);
        self.shunt_resistance = Some(shunt_resistance);
        Ok(())
    }

    pub async fn set_adc_mode(&mut self, mode: ADCMode) -> Result<(), E> {
        self.modify_config(|c| c.with_bus_adc_mode(mode).with_shunt_adc_mode(mode))
            .await
    }
    async fn read_register(&mut self, reg: Register) -> Result<u16, E> {
        let mut bytes = [0; 2];
        self.i2c
            .write_read(self.address, &[reg as u8], &mut bytes)
            .await?;
        return Ok(BigEndian::read_u16(&bytes));
    }
    async fn write_register(&mut self, reg: Register, val: u16) -> Result<(), E> {
        let mut new_config_bytes = [reg as u8, 0, 0];
        BigEndian::write_u16(&mut new_config_bytes[1..], val);
        return self.i2c.write(self.address, &new_config_bytes).await;
    }
    async fn modify_config<F>(&mut self, f: F) -> Result<(), E>
    where
        F: FnOnce(Configuration) -> Configuration,
    {
        let old_value = self.read_register(Register::Configuration).await?;
        let old_config: Configuration = old_value.into();
        let new_config = f(old_config);
        match self
            .write_register(Register::Configuration, new_config.into())
            .await
        {
            Ok(()) => {
                self.last_config = Some(new_config);
                Ok(())
            }
            Err(error) => {
                self.last_config = None;
                Err(error)
            }
        }
    }
}
