#![no_std]

use core::ops::BitAnd;
use nom::bytes::complete::take;
use nom::error::{ErrorKind, ParseError};
//use nom::Err::*;
use nom::Needed::Unknown;
use nom_varint::take_varint;
use num_enum::{IntoPrimitive, TryFromPrimitive};
#[cfg(feature = "defmt")]
use defmt;

mod varint;

#[derive(Clone, Copy, Debug, PartialEq, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum WireType {
    VARINT = 0,
    I64 = 1,
    LEN = 2,
    SGROUP = 3,
    EGROUP = 4,
    I32 = 5,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Tag {
    wire_type: WireType,
    field_number: usize,
}

pub fn take_tag<'a, E>(i: &'a [u8]) -> nom::IResult<&'a [u8], Tag, E>
where
    E: ParseError<&'a [u8]>,
{
    let (remainder, tl) = take_varint(i)?;
    let wire_type: WireType = (tl.bitand(0x07) as u8).try_into().map_err(|_| nom::Err::Error(E::from_error_kind(i, ErrorKind::TooLarge)))?;
    let field_number = tl.wrapping_shr(3);
    Ok((remainder, Tag{wire_type, field_number}))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_take_tag() {
        let result = take_tag::<()>(&[0x12]);
        assert_eq!(result, Ok((b"" as &[u8], Tag{wire_type: WireType::LEN, field_number: 2})));
    }
}
