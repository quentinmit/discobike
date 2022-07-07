use embedded_hal::blocking::i2c;
use modular_bitfield::prelude::*;
use uom::si::f32::ElectricPotential;
use uom::si::electric_potential::{volt, millivolt, microvolt};

use byteorder::{BigEndian, ByteOrder, LittleEndian};

pub const INA219_ADDR: u8 = 0x41;

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
#[derive(Copy, Clone, BitfieldSpecifier)]
#[bits = 2]
enum Gain {
    Full = 0,
    Half = 1,
    Quarter = 2,
    Eighth = 3,
}
impl Gain {
    pub const mV40: Gain = Gain::Full;
    pub const mV80: Gain = Gain::Half;
    pub const mV160: Gain = Gain::Quarter;
    pub const mV320: Gain = Gain::Eighth;
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

pub struct INA219<I2C> {
    i2c: I2C,
    address: u8,
    last_config: Option<Configuration>,
}

impl<I2C, E> INA219<I2C>
where
    I2C: i2c::Write<Error = E> + i2c::Read<Error = E> + i2c::WriteRead<Error = E>,
{
    pub fn new(i2c: I2C, address: u8) -> INA219<I2C> {
        INA219 {
            i2c,
            address,
            last_config: None,
        }
    }
    pub fn get_bus_voltage(&mut self) -> Result<ElectricPotential, E> {
        let v: BusVoltage = self.read_register(Register::BusVoltage)?.into();
        // LSB = 4 mV
        Ok(ElectricPotential::new::<millivolt>(v.bus_voltage() as f32 * 4.0))
    }
    pub fn get_shunt_voltage(&mut self) -> Result<ElectricPotential, E> {
        let v = self.read_register(Register::ShuntVoltage)? as i16;
        // LSB = 10 µV
        Ok(ElectricPotential::new::<microvolt>(v as f32 * 10.0))
    }

    pub fn set_adc_mode(&mut self, mode: ADCMode) -> Result<(), E> {
        self.modify_config(|c| c.with_bus_adc_mode(mode).with_shunt_adc_mode(mode))
    }
    fn read_register(&mut self, reg: Register) -> Result<u16, E> {
        let mut bytes = [0; 2];
        self.i2c
            .write_read(self.address, &[reg as u8], &mut bytes)?;
        return Ok(BigEndian::read_u16(&bytes));
    }
    fn write_register(&mut self, reg: Register, val: u16) -> Result<(), E> {
        let mut new_config_bytes = [reg as u8, 0, 0];
        BigEndian::write_u16(&mut new_config_bytes[1..2], val);
        return self.i2c.write(self.address, &new_config_bytes);
    }
    fn modify_config<F>(&mut self, f: F) -> Result<(), E>
    where
        F: FnOnce(Configuration) -> Configuration,
    {
        let old_config: Configuration = self.read_register(Register::Configuration)?.into();
        let new_config = f(old_config);
        match self.write_register(Register::Configuration, new_config.into()) {
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
