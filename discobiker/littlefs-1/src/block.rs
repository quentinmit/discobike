use crate::structs::MetadataBlock;
use crate::FsError;
use arrayvec::ArrayVec;
use byte::{ctx::Bytes, BytesExt, Result as ByteResult, TryRead, TryWrite};
#[cfg(defmt)]
use defmt::*;

#[derive(Clone, Debug, PartialEq)]
pub struct Block<const BLOCK_SIZE: usize>(ArrayVec<u8, BLOCK_SIZE>);

#[cfg(defmt)]
impl<const BLOCK_SIZE: usize> Format for Block<BLOCK_SIZE> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "{:?}", self.0.as_slice(),)
    }
}

impl<const BLOCK_SIZE: usize> From<[u8; BLOCK_SIZE]> for Block<BLOCK_SIZE> {
    fn from(array: [u8; BLOCK_SIZE]) -> Self {
        Block(ArrayVec::from(array))
    }
}

impl<const BLOCK_SIZE: usize> AsRef<[u8]> for Block<BLOCK_SIZE> {
    fn as_ref(&self) -> &[u8] {
        self.0.as_ref()
    }
}

impl<const BLOCK_SIZE: usize> Block<BLOCK_SIZE> {
    pub fn empty() -> Self {
        Self(ArrayVec::new())
    }
    pub fn zero(block_size: usize) -> Self {
        let mut b: Self = [0u8; BLOCK_SIZE].into();
        b.0.truncate(block_size);
        b
    }
    pub fn try_from<T: TryWrite>(value: T) -> ByteResult<Self> {
        let offset = &mut 0;
        let mut b: Self = [0u8; BLOCK_SIZE].into();
        b.as_mut_slice().write(offset, value)?;
        b.0.truncate(*offset);
        Ok(b)
    }
    pub fn as_slice(&self) -> &[u8] {
        self.0.as_slice()
    }
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        self.0.as_mut_slice()
    }
    pub fn as_metadata<'a, T>(&self) -> Result<MetadataBlock<&[u8]>, FsError<T>> {
        self.as_slice().read(&mut 0).map_err(|_| FsError::Corrupt)
    }
    pub fn len(&self) -> usize {
        self.0.len()
    }
    pub fn try_extend_from_slice(&mut self, other: &[u8]) -> Result<(), arrayvec::CapacityError> {
        self.0.try_extend_from_slice(other)
    }
}

impl<'a, const BLOCK_SIZE: usize> TryRead<'a, Bytes> for Block<BLOCK_SIZE> {
    fn try_read(bytes: &'a [u8], ctx: Bytes) -> ByteResult<(Self, usize)> {
        let offset = &mut 0;
        let s: &[u8] = bytes.read_with(offset, ctx)?;
        Ok((
            Block(ArrayVec::try_from(s).map_err(|_| byte::Error::BadInput {
                err: "data larger than block",
            })?),
            *offset,
        ))
    }
}

impl<const BLOCK_SIZE: usize> TryWrite for Block<BLOCK_SIZE> {
    fn try_write(self, bytes: &mut [u8], _ctx: ()) -> ByteResult<usize> {
        let offset = &mut 0;
        bytes.write(offset, self.as_slice())?;
        Ok(*offset)
    }
}

impl<const BLOCK_SIZE: usize, Ctx> BytesExt<Ctx> for Block<BLOCK_SIZE> {
    fn read_with<'a, T>(&'a self, offset: &mut usize, ctx: Ctx) -> byte::Result<T>
    where
        T: TryRead<'a, Ctx>,
    {
        self.as_slice().read_with(offset, ctx)
    }
    fn read_iter<'a, 'i, T>(&'a self, offset: &'i mut usize, ctx: Ctx) -> byte::Iter<'a, 'i, T, Ctx>
    where
        T: TryRead<'a, Ctx>,
        Ctx: Clone,
    {
        self.as_slice().read_iter(offset, ctx)
    }
    fn write_with<T>(&mut self, offset: &mut usize, t: T, ctx: Ctx) -> byte::Result<()>
    where
        T: TryWrite<Ctx>,
    {
        self.as_mut_slice().write_with(offset, t, ctx)
    }
}
