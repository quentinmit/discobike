use core::convert::TryInto;
use core::u32;
use nom::bytes::complete::take;
use nom::error::{ErrorKind, ParseError};
use nom::Err::*;
use nom::Needed::Unknown;
use core::ops::AddAssign;
use num_traits::{Unsigned, Zero, CheckedAdd, CheckedShl, CheckedShr, FromPrimitive};
use zigzag::ZigZagDecode;

/// Parses a varint.
///
/// # Examples
///
/// ```
/// assert_eq!(nom_protobuf::take_varint::<usize, ()>(&[0x0b]), Ok((&[] as &[u8], 0x0b)));
/// ```
pub fn take_varint<'a, N, E>(i: &'a [u8]) -> nom::IResult<&'a [u8], N, E>
where
    N: Unsigned + Zero + Eq + CheckedAdd + CheckedShl + CheckedShr + FromPrimitive + AddAssign + Copy + core::fmt::Debug,
    E: ParseError<&'a [u8]>,
{
    let mut res: N = N::zero();
    let mut count: usize = 0;
    let mut remainder = i;
    loop {
        let (rest, byte) = match take::<usize, &[u8], ()>(1)(remainder) {
            Ok((rest, bytes)) => Ok((rest, bytes[0])),
            Err(_) => Err(Incomplete(Unknown)),
        }?;
        let shift = (count * 7).try_into().unwrap_or(u32::MAX);
        res = N::from_u8(byte & 127)
            .and_then(|x| {
                let shifted = x.checked_shl(shift);
                // N.B. checked_shr checks shift size, NOT overflow.
                shifted.filter(|shifted| shifted.checked_shr(shift).map_or(false, |new| new == x))
            })
            .and_then(|x| x.checked_add(&res))
            .ok_or_else(|| Error(E::from_error_kind(remainder, ErrorKind::MapOpt)))?;
        remainder = rest;
        count += 1;
        if (byte >> 7) == 0 {
            return Ok((remainder, res));
        }
    }
}

pub fn take_signed_varint<'a, S, U, E>(i: &'a [u8]) -> nom::IResult<&'a [u8], S, E>
where
    U: Unsigned + Zero + Eq + CheckedAdd + CheckedShl + CheckedShr + FromPrimitive + AddAssign + Copy + core::fmt::Debug + ZigZagDecode<S>,
    E: ParseError<&'a [u8]>,
{
    // Signed varints are encoded as ZigZag
    take_varint::<U, E>(i).map(|(remainder, x)| (remainder, x.zigzag_decode()))
}

#[cfg(test)]
mod test {
    use nom::Err::Error;
    use nom::error::{ErrorKind, ParseError};

    use test_log::test;

    #[test]
    fn parse_varint_simple() {
        assert_eq!(
            super::take_varint::<_, ()>(&[0x0b, 0x01, 0x02, 0x03]),
            Ok((b"\x01\x02\x03" as &[u8], 11u16))
        );
    }

    #[test]
    fn parse_varint_twobyte() {
        assert_eq!(
            super::take_varint::<_, ()>(&[0x84, 0x02, 0x04, 0x05, 0x06]),
            Ok((b"\x04\x05\x06" as &[u8], 260u16))
        );
    }

    #[test]
    fn parse_varint_into_u8() {
        assert_eq!(
            super::take_varint::<u8, _>(&[0x84, 0x02, 0x04, 0x05, 0x06]),
            Err(Error(nom::error::Error::new(b"\x02\x04\x05\x06" as &[u8], ErrorKind::MapOpt)))
        );
    }

    #[test]
    fn parse_signed_varint_into_i16() {
        assert_eq!(
            super::take_signed_varint::<i16, u16, ()>(&[0x84, 0x02, 0x04, 0x05, 0x06]),
            Ok((b"\x04\x05\x06" as &[u8], 130i16))
        );
        assert_eq!(
            super::take_signed_varint::<i16, u16, ()>(&[0x83, 0x02, 0x04, 0x05, 0x06]),
            Ok((b"\x04\x05\x06" as &[u8], -130i16))
        );
    }
}
