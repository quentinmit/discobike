#![no_std]

use core::ops::BitAnd;
use nom::bytes::complete::take;
use nom::error::{ErrorKind, ParseError};
//use nom::Err::*;
#[cfg(feature = "defmt")]
use defmt;
use nom::Needed::Unknown;
use num_enum::{IntoPrimitive, TryFromPrimitive};

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
    let (remainder, tl) = take_varint::<usize, E>(i)?;
    let wire_type: WireType = (tl.bitand(0x07) as u8)
        .try_into()
        .map_err(|_| nom::Err::Error(E::from_error_kind(i, ErrorKind::TooLarge)))?;
    let field_number = tl.wrapping_shr(3);
    Ok((
        remainder,
        Tag {
            wire_type,
            field_number,
        },
    ))
}

pub use varint::{take_signed_varint, take_varint};

pub fn take_i32<'a, E>(i: &'a [u8]) -> nom::IResult<&'a [u8], i32, E>
where
    E: ParseError<&'a [u8]>,
{
    nom::number::complete::le_i32(i)
}

pub mod scalar {
    use crate::WireType;
    use nom::error::{ErrorKind, ParseError};
    use paste::paste;

    macro_rules! impl_type {
        ($proto_type:ident, ($wire_type:ident, $i:ident) -> ($rust_type:ty) $body:block) => {
            paste! {
                pub fn [<take_ $proto_type>]<'a, E>($wire_type: WireType, $i: &'a [u8]) -> nom::IResult<&'a [u8], $rust_type, E>
                where
                    E: ParseError<&'a [u8]>
                {
                    $body
                }
            }
        };
    }

    macro_rules! impl_int {
        ($proto_type:ident, $rust_type:ty) => {
            impl_int!($proto_type, $rust_type, $rust_type);
        };
        ($proto_type:ident, $intermediate_rust_type:ty, $rust_type:ty) => {
            impl_int!($proto_type, $intermediate_rust_type, $rust_type, take_varint::<$intermediate_rust_type>);
        };
        ($proto_type:ident, $intermediate_rust_type:ty, $rust_type:ty, $take_varint_function:ident::<$( $take_generics:ty ),*>) => {
            impl_type!($proto_type, (wire_type, i) -> ($rust_type) {
                    match wire_type {
                        WireType::VARINT => crate::$take_varint_function::<$($take_generics),*, E>(i).map(|(remainder, x)| (remainder, x as $rust_type)),
                        WireType::I64 => nom::number::complete::le_u64(i)
                            .and_then(|(remainder, x)|
                                 Ok((remainder,
                                     x.try_into().map_err(
                                         |_| nom::Err::Error(E::from_error_kind(i, ErrorKind::TooLarge)))?
                                 ))
                            ),
                        WireType::I32 => nom::number::complete::le_u32(i)
                            .and_then(|(remainder, x)|
                                 Ok((remainder,
                                     x.try_into().map_err(
                                         |_| nom::Err::Error(E::from_error_kind(i, ErrorKind::TooLarge)))?
                                 ))
                            ),
                        _ => Err(nom::Err::Error(E::from_error_kind(i, ErrorKind::MapOpt))),
                    }
                });
        };
    }

    impl_type!(double, (wire_type, i) -> (f64) {
        match wire_type {
            WireType::I64 => nom::number::complete::le_f64(i),
            WireType::I32 => nom::number::complete::le_f32(i).map(|(remainder, x)| (remainder, x as f64)),
            _ => Err(nom::Err::Error(E::from_error_kind(i, ErrorKind::MapOpt))),
        }
    });
    impl_type!(float, (wire_type, i) -> (f32) {
        match wire_type {
                WireType::I64 => nom::number::complete::le_f64(i).map(|(remainder, x)| (remainder, x as f32)),
            WireType::I32 => nom::number::complete::le_f32(i),
            _ => Err(nom::Err::Error(E::from_error_kind(i, ErrorKind::MapOpt))),
        }
    });
    impl_int!(int32, u32, i32); // 2's complement
    impl_int!(int64, u64, i64); // 2's complement
    impl_int!(uint32, u32);
    impl_int!(uint64, u64);
    impl_int!(sint32, i32, i32, take_signed_varint::<i32, u32>); // ZigZag
    impl_int!(sint64, i64, i64, take_signed_varint::<i64, u64>); // ZigZag
    impl_int!(fixed32, u32);
    impl_int!(fixed64, u64);
    impl_int!(sfixed32, u32, i32); // 2's complement
    impl_int!(sfixed64, u64, i64); // 2's complement
    // bool
    impl_type!(bool, (wire_type, i) -> (bool) {
        take_uint32(wire_type, i).map(|(remainder, x)| (remainder, x != 0))
    });
    // string
    // bytes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_take_tag() {
        let result = take_tag::<()>(&[0x12]);
        assert_eq!(
            result,
            Ok((
                b"" as &[u8],
                Tag {
                    wire_type: WireType::LEN,
                    field_number: 2
                }
            ))
        );
    }
}
