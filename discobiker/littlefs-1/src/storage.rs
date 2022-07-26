use core::ops::{Deref, DerefMut};

#[cfg(feature = "async")]
use core::future::Future;
use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind};
#[cfg(feature = "sync")]
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
#[cfg(feature = "async")]
use embedded_storage_async::nor_flash::{AsyncNorFlash, AsyncReadNorFlash};

pub struct SliceStorage<S: Deref<Target = [u8]>> {
    buf: S,
}

impl<S: Deref<Target = [u8]>> SliceStorage<S> {
    pub fn new(slice: S) -> Self {
        SliceStorage { buf: slice }
    }

    fn do_read<'a>(
        &'a mut self,
        address: u32,
        data: &'a mut [u8],
    ) -> Result<(), NorFlashErrorKind> {
        let start = address as usize;
        let end = start
            .checked_add(data.len())
            .ok_or(NorFlashErrorKind::OutOfBounds)?;

        if end > self.buf.len() {
            Err(NorFlashErrorKind::OutOfBounds)
        } else {
            data.copy_from_slice(&self.buf[start..end]);
            Ok(())
        }
    }
}

impl<S: Deref<Target = [u8]>> ErrorType for SliceStorage<S> {
    type Error = NorFlashErrorKind;
}

#[cfg(feature = "async")]
impl<S: Deref<Target = [u8]>> AsyncReadNorFlash for SliceStorage<S> {
    const READ_SIZE: usize = 1;

    type ReadFuture<'a> = impl Future<Output = Result<(), NorFlashErrorKind>> + 'a
            where S: 'a
            ;
    fn read<'a>(&'a mut self, address: u32, data: &'a mut [u8]) -> Self::ReadFuture<'a> {
        async move { self.do_read(address, data) }
    }

    fn capacity(&self) -> usize {
        self.buf.len()
    }
}

#[cfg(feature = "sync")]
impl<S: Deref<Target = [u8]>> ReadNorFlash for SliceStorage<S> {
    const READ_SIZE: usize = 1;

    fn read(&mut self, address: u32, data: &mut [u8]) -> Result<(), NorFlashErrorKind> {
        self.do_read(address, data)
    }

    fn capacity(&self) -> usize {
        self.buf.len()
    }
}
