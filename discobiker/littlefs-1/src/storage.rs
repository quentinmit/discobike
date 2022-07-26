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

impl<S: DerefMut<Target = [u8]>> SliceStorage<S> {
    fn do_write<'a>(
        &'a mut self,
        min_size: usize,
        address: u32,
        data: &'a [u8],
    ) -> Result<(), NorFlashErrorKind> {
        let start = address as usize;
        let end = start
            .checked_add(data.len())
            .ok_or(NorFlashErrorKind::OutOfBounds)?;

        if end > self.buf.len() {
            Err(NorFlashErrorKind::OutOfBounds)
        } else if (end - start) < min_size {
            Err(NorFlashErrorKind::NotAligned)
        } else if start % min_size != 0 {
            Err(NorFlashErrorKind::NotAligned)
        } else {
            self.buf[start..end].copy_from_slice(data);
            Ok(())
        }
    }

    fn do_erase<'a>(
        &'a mut self,
        min_size: usize,
        from: u32,
        to: u32,
    ) -> Result<(), NorFlashErrorKind> {
        let start = from as usize;
        let end = to as usize;

        if end > self.buf.len() {
            Err(NorFlashErrorKind::OutOfBounds)
        } else if (end - start) < min_size {
            Err(NorFlashErrorKind::NotAligned)
        } else if start % min_size != 0 || end % min_size != 0 {
            Err(NorFlashErrorKind::NotAligned)
        } else {
            self.buf[start..end].fill(0xFF);
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
            where S: 'a;
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

#[cfg(feature = "async")]
impl<S: DerefMut<Target = [u8]>> AsyncNorFlash for SliceStorage<S> {
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = 4096;

    type WriteFuture<'a> = impl Future<Output = Result<(), NorFlashErrorKind>> + 'a
        where S: 'a;
    fn write<'a>(&'a mut self, offset: u32, data: &'a [u8]) -> Self::WriteFuture<'a> {
        async move { self.do_write(<Self as AsyncNorFlash>::WRITE_SIZE, offset, data) }
    }

    type EraseFuture<'a> = impl Future<Output = Result<(), NorFlashErrorKind>> + 'a
    where S: 'a;
    fn erase<'a>(&'a mut self, from: u32, to: u32) -> Self::EraseFuture<'a> {
        async move { self.do_erase(<Self as AsyncNorFlash>::ERASE_SIZE, from, to) }
    }
}

#[cfg(feature = "sync")]
impl<S: DerefMut<Target = [u8]>> NorFlash for SliceStorage<S> {
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = 4096;

    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), NorFlashErrorKind> {
        self.do_write(<Self as NorFlash>::WRITE_SIZE, offset, data)
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), NorFlashErrorKind> {
        self.do_erase(<Self as NorFlash>::ERASE_SIZE, from, to)
    }
}
