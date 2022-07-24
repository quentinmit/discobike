use core::future::Future;
use core::ops::{Deref, DerefMut};

use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind};
use embedded_storage_async::nor_flash::{AsyncNorFlash, AsyncReadNorFlash};

pub struct SliceStorage<S: Deref<Target = [u8]>> {
    buf: S,
}

impl<S: Deref<Target = [u8]>> SliceStorage<S> {
    pub fn new(slice: S) -> Self {
        SliceStorage { buf: slice }
    }
}

impl<S: Deref<Target = [u8]>> ErrorType for SliceStorage<S> {
    type Error = NorFlashErrorKind;
}

impl<S: Deref<Target = [u8]>> AsyncReadNorFlash for SliceStorage<S> {
    const READ_SIZE: usize = 1;

    type ReadFuture<'a> = impl Future<Output = Result<(), NorFlashErrorKind>> + 'a
        where S: 'a
        ;
    fn read<'a>(&'a mut self, address: u32, data: &'a mut [u8]) -> Self::ReadFuture<'a> {
        async move {
            // Reading is simple since SoC flash is memory-mapped :)
            // TODO check addr/len is in bounds.

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

    fn capacity(&self) -> usize {
        self.buf.len()
    }
}
