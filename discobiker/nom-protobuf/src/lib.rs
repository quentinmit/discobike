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
    pub wire_type: WireType,
    pub field_number: usize,
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
    use crate::WireType::*;
    use nom::bytes::complete::take;
    use nom::combinator::{fail, map_opt, map_res, map_parser};
    use nom::multi::length_data;
    use nom::error::{ErrorKind, ParseError, FromExternalError};
    use nom::{Parser, InputLength, ParseTo};
    use paste::paste;

    macro_rules! impl_type {
        ($proto_type:ident $(< $generic_name:ty : $generic:ty >)?, ($wire_type:ident) -> ($rust_type:ty) $(< $ext_error:ty >)? $body:block) => {
            paste! {
                pub fn [<take_ $proto_type>]<'a, $( $generic_name : $generic , )? E>($wire_type: WireType) -> impl Parser<&'a [u8], $rust_type, E>
                where
                    E: ParseError<&'a [u8]> $( + FromExternalError<&'a [u8], $ext_error > )?
                {
                    $body
                }
            }
        };
    }

    fn take_u64<'a, O: TryFrom<u64>, E: ParseError<&'a [u8]>> (i: &'a [u8]) -> nom::IResult<&'a [u8], O, E> {
        map_opt(
            nom::number::complete::le_u64,
            |x| x.try_into().ok()
        )(i)
    }

    fn take_u32<'a, O: TryFrom<u32>, E: ParseError<&'a [u8]>> (i: &'a [u8]) -> nom::IResult<&'a [u8], O, E> {
        map_opt(
            nom::number::complete::le_u32,
            |x| x.try_into().ok()
        )(i)
    }

    macro_rules! impl_packed {
        ($proto_type:ident, $normal_wire_type:expr, $rust_type:ty) => {
            paste! {
                impl_type!($proto_type, (wire_type) -> ($rust_type) {
                    move |i| match wire_type {
                        // Packed repeated integers are represented by LEN followed by repeated 
                        WireType::LEN =>
                            map_parser(
                                take_bytes(wire_type),
                                |mut i| {
                                    loop {
                                        match [< take_one_ $proto_type >]($normal_wire_type).parse(i) {
                                            Err(e) => return Err(e),
                                            Ok((rem, o)) => {
                                                if rem.input_len() == 0 {
                                                    return Ok((rem, o))
                                                }
                                                i = rem;
                                            },
                                        }
                                    }
                                }
                            ).parse(i),
                        _ => [< take_one_ $proto_type >](wire_type).parse(i),
                    }
                });
            }
        };
    }


    macro_rules! impl_int {
        ($proto_type:ident, $normal_wire_type:expr, $rust_type:ty) => {
            impl_int!($proto_type, $normal_wire_type, $rust_type, $rust_type);
        };
        ($proto_type:ident, $normal_wire_type:expr, $intermediate_rust_type:ty, $rust_type:ty) => {
            impl_int!($proto_type, $normal_wire_type, $intermediate_rust_type, $rust_type, take_varint::<$intermediate_rust_type>);
        };
        ($proto_type:ident, $normal_wire_type:expr, $intermediate_rust_type:ty, $rust_type:ty, $take_varint_function:ident::<$( $take_generics:ty ),*>) => {
            impl_packed!($proto_type, $normal_wire_type, $rust_type);
            paste! {
                impl_type!([< one_ $proto_type >], (wire_type) -> ($rust_type) {
                    (move |i| match wire_type {
                        WireType::VARINT => crate::$take_varint_function::<$($take_generics),*, E>(i),
                        WireType::I64 => take_u64(i),
                        WireType::I32 => take_u32(i),
                        _ => fail(i)
                    }).map(|x| x as $rust_type)
                });
            }
        };
    }

    impl_type!(double, (wire_type) -> (f64) {
        move |i| match wire_type {
            WireType::I64 => nom::number::complete::le_f64(i),
            WireType::I32 => nom::number::complete::le_f32.map(|x| x as f64).parse(i),
            _ => fail(i),
        }
    });
    impl_type!(float, (wire_type) -> (f32) {
        move |i| match wire_type {
            WireType::I64 => nom::number::complete::le_f64.map(|x| x as f32).parse(i),
            WireType::I32 => nom::number::complete::le_f32(i),
            _ => fail(i),
        }
    });
    impl_int!(int32, VARINT, u32, i32); // 2's complement
    impl_int!(int64, VARINT, u64, i64); // 2's complement
    impl_int!(uint32, VARINT, u32);
    impl_int!(uint64, VARINT, u64);
    impl_int!(sint32, VARINT, i32, i32, take_signed_varint::<i32, u32>); // ZigZag
    impl_int!(sint64, VARINT, i64, i64, take_signed_varint::<i64, u64>); // ZigZag
    impl_int!(fixed32, I32, u32);
    impl_int!(fixed64, I64, u64);
    impl_int!(sfixed32, I32, u32, i32); // 2's complement
    impl_int!(sfixed64, I64, u64, i64); // 2's complement
    impl_type!(bool, (wire_type) -> (bool) {
        take_uint32(wire_type).map(|x| x != 0)
    });
    impl_type!(string, (wire_type) -> (&'a str) <core::str::Utf8Error> {
        map_res(
            take_bytes(wire_type),
            core::str::from_utf8
        )
    });
    impl_type!(bytes, (wire_type) -> (&'a [u8]) {
        move |i| match wire_type {
            WireType::LEN =>
                length_data(crate::varint::take_varint::<usize, E>)(i),
            _ => fail(i),
        }
    });

    impl_type!(enum<V: TryFrom<i32>>, (wire_type) -> (V) <V::Error> {
        move |i| match wire_type {
            WireType::VARINT =>
                map_res(
                    crate::varint::take_varint::<u32, E>,
                    |x| (x as i32).try_into()
                )(i),
            _ => fail(i),
        }
    });
    #[cfg(test)]
    mod tests {
        use super::*;

        use test_log::test;
        use log::{info,trace};
        use num_enum::TryFromPrimitive;

        macro_rules! test_fields {
            (@tag $i:ident, $field_number:expr, $wire_type:expr) => {
                {
                    let ($i, tag) = crate::take_tag::<()>($i).unwrap();
                    info!("Read tag: {:?}", tag);
                    assert_eq!(tag, crate::Tag{wire_type: $wire_type, field_number: $field_number});
                    ($i, tag)
                }
            };
            (@field $i:ident (
                $field_number:expr,
                message, //$( $value:tt )*
                (
                    $( ($($value:tt)*) ),* $(,)?
                )
            )) => {
                {
                    let ($i, _) = test_fields!(@tag $i, $field_number, crate::WireType::LEN);
                    let ($i, len) = crate::take_varint::<usize, ()>($i).unwrap();
                    let ($i, body) = take::<usize, &[u8], ()>(len)($i).unwrap();
                    test_fields!(body, {
                        $( ( $( $value )* ) ),*
                    });
                    $i
                }
            };
            (@field $i:ident (
                $field_number:expr,
                group, //$( $value:tt )*
                (
                    $( ($($value:tt)*) ),* $(,)?
                )
            )) => {
                {
                    let ($i, _) = test_fields!(@tag $i, $field_number, crate::WireType::SGROUP);
                    $(
                        let $i = test_fields!(@field $i ( $( $value )* ));
                    )*
                    let ($i, _) = test_fields!(@tag $i, $field_number, crate::WireType::EGROUP);
                    $i
                }
            };
            (@field $i:ident ($field_number:expr, $proto_type:ident $(< $generic_type:ty >)?, $value:expr)) => {
                paste! {
                    {
                        let ($i, tag) = crate::take_tag::<()>($i).unwrap();
                        info!("Read tag: {:?}", tag);
                        assert_eq!(tag.field_number, $field_number);
                        let ($i, x) = [< take_ $proto_type >]::<$($generic_type,)? ()>(tag.wire_type).parse($i).unwrap();
                        trace!("Read {} value: {:?}", stringify!($proto_type), x);
                        assert_eq!(x, $value);
                        $i
                    }
                }
            };
            ($i:expr, { $( ($field_number:expr, $proto_type:tt $(< $generic_type:ty >)?, $( $value:tt )*) ),* $(,)? }) => {
                {
                    let i = $i;
                    $(
                        let i = test_fields!(@field i ($field_number, $proto_type $(< $generic_type >)?, $( $value )*));
                    )*
                    assert_eq!(i.len(), 0);
                }
            }
        }

        #[test]
        fn test_negative_integers() {
            test_fields!(&[0, 1], {
                (0, int32, 1),
            });
            test_fields!(&[0, 1], {
                (0, sint32, -1),
            });
            test_fields!(&[0, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F], {
                (0, int32, -1),
            });
            test_fields!(&[0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01], {
                (0, int64, -1),
            });
            test_fields!(&[0x05, 0xFF, 0xFF, 0xFF, 0xFF], {
                (0, int32, -1),
            });
            test_fields!(&[0x05, 0xFF, 0xFF, 0xFF, 0xFF], {
                (0, sfixed32, -1),
            });
            test_fields!(&[0x05, 0xFF, 0xFF, 0xFF, 0xFF], {
                (0, fixed32, 0xFFFFFFFF),
            });
        }

        #[test]
        fn test_packed_integers() {
            test_fields!(&[2, 2, 3, 4, 8, 5], {
                (0, int32, 4),
                (1, int32, 5),
            });
        }

        #[derive(Clone, Copy, Debug, PartialEq, TryFromPrimitive)]
        #[allow(non_camel_case_types)]
        #[repr(i32)]
        enum NestedEnum {
            FOO = 1,
            BAR = 2,
            BAZ = 3,
            NEG = -1,  // Intentionally negative.
        }

        #[derive(Clone, Copy, Debug, PartialEq, TryFromPrimitive)]
        #[allow(non_camel_case_types)]
        #[repr(i32)]
        enum ForeignEnum {
            FOREIGN_FOO = 4,
            FOREIGN_BAR = 5,
            FOREIGN_BAZ = 6,
        }

        #[derive(Clone, Copy, Debug, PartialEq, TryFromPrimitive)]
        #[allow(non_camel_case_types)]
        #[repr(i32)]
        enum ImportEnum {
              IMPORT_FOO = 7,
              IMPORT_BAR = 8,
              IMPORT_BAZ = 9,
        }

        #[test]
        fn test_golden_message() {
            let message_pb = include_bytes!("../testdata/message.pb");
            test_fields!(message_pb, {
                (1, int32, 101),
                (2, int64, 102),
                (3, uint32, 103),
                (4, uint64, 104),
                (5, sint32, 105),
                (6, sint64, 106),
                (7, fixed32, 107u32),
                (8, fixed64, 108u64),
                (9, sfixed32, 109i32),
                (10, sfixed64, 110i64),
                (11, float, 111.0f32), // 0x42de0000i32
                (12, double, 112.0f64), // 0x405c000000000000i64
                (13, bool, true),
                (14, string, "115"),
                (15, bytes, b"116"),
                (16, group, (
                    (17, int32, 117), // a
                )),
                (18, message, (
                    (1, int32, 118), // bb
                )),
                (19, message, (
                    (1, int32, 119), // c
                )),
                (20, message, (
                    (1, int32, 120),
                )),
                (21, enum<NestedEnum>, NestedEnum::BAZ),
                (22, enum<ForeignEnum>, ForeignEnum::FOREIGN_BAZ),
                (23, enum<ImportEnum>, ImportEnum::IMPORT_BAZ),
                (24, string, "124"), // string_piece
                (25, string, "125"), // cord
                (26, message, (
                    (1, int32, 126),
                )),
                (27, message, (
                    (1, int32, 127), // bb
                )),
                (28, message, (
                    (1, int32, 128),
                )),
                (31, int32, 201),
                (31, int32, 301),
                (32, int64, 202),
                (32, int64, 302),
                (33, uint32, 203),
                (33, uint32, 303),
                (34, uint64, 204),
                (34, uint64, 304),
                (35, sint32, 205),
                (35, sint32, 305),
                (36, sint64, 206),
                (36, sint64, 306),
                (37, fixed32, 207u32),
                (37, fixed32, 307u32),
                (38, fixed64, 208u64),
                (38, fixed64, 308u64),
                (39, sfixed32, 209i32),
                (39, sfixed32, 309i32),
                (40, sfixed64, 210i64),
                (40, sfixed64, 310i64),
                (41, float, 211.0f32), // 0x43530000i32
                (41, float, 311.0f32), // 0x439b8000i32
                (42, double, 212.0f64), // 0x406a800000000000i64
                (42, double, 312.0f64), // 0x4073800000000000i64
                (43, bool, true),
                (43, bool, false),
                (44, string, "215"),
                (44, string, "315"),
                (45, bytes, b"216"),
                (45, bytes, b"316"),
                (46, group, (
                    (47, int32, 217), // a
                )),
                (46, group, (
                    (47, int32, 317), // a
                )),
                (48, message, (
                    (1, int32, 218), // bb
                )),
                (48, message, (
                    (1, int32, 318), // bb
                )),
                (49, message, (
                    (1, int32, 219), // c
                )),
                (49, message, (
                    (1, int32, 319), // c
                )),
                (50, message, (
                    (1, int32, 220),
                )),
                (50, message, (
                    (1, int32, 320),
                )),
                (51, enum<NestedEnum>, NestedEnum::BAR),
                (51, enum<NestedEnum>, NestedEnum::BAZ),
                (52, enum<ForeignEnum>, ForeignEnum::FOREIGN_BAR),
                (52, enum<ForeignEnum>, ForeignEnum::FOREIGN_BAZ),
                (53, enum<ImportEnum>, ImportEnum::IMPORT_BAR),
                (53, enum<ImportEnum>, ImportEnum::IMPORT_BAZ),
                (54, string, "224"), // string_piece
                (54, string, "324"), // string_piece
                (55, string, "225"), // cord
                (55, string, "325"), // cord
                (57, message, (
                    (1, int32, 227), // bb
                )),
                (57, message, (
                    (1, int32, 327), // bb
                )),
                (61, int32, 401),
                (62, int64, 402),
                (63, uint32, 403),
                (64, uint64, 404),
                (65, sint32, 405),
                (66, sint64, 406),
                (67, fixed32, 407u32),
                (68, fixed64, 408u64),
                (69, sfixed32, 409i32),
                (70, sfixed64, 410i64),
                (71, float, 411.0f32), // 0x43cd8000i32
                (72, double, 412.0f64), // 0x4079c00000000000i64
                (73, bool, false),
                (74, string, "415"),
                (75, bytes, b"416"),
                (81, enum<NestedEnum>, NestedEnum::FOO),
                (82, enum<ForeignEnum>, ForeignEnum::FOREIGN_FOO),
                (83, enum<ImportEnum>, ImportEnum::IMPORT_FOO),
                (84, string, "424"), // string_piece
                (85, string, "425"), // cord
                (111, uint32, 601),
                (112, message, (
                    (1, int32, 602), // bb
                )),
                (113, string, "603"),
                (114, bytes, b"604"),
            });
        }
    }
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
