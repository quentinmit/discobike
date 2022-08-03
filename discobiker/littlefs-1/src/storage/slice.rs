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
            warn!("attempt to write past end: {}-{}", start, end);
            Err(NorFlashErrorKind::OutOfBounds)
        } else if (end - start) < min_size {
            warn!("attempt to write <1 block: {}-{}", start, end);
            Err(NorFlashErrorKind::NotAligned)
        } else if start % min_size != 0 || end % min_size != 0 {
            warn!("attempt to write unaligned: {}-{}", start, end);
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
            warn!("attempt to erase past end: {}-{}", start, end);
            Err(NorFlashErrorKind::OutOfBounds)
        } else if (end - start) < min_size {
            warn!("attempt to erase <1 block: {}-{}", start, end);
            Err(NorFlashErrorKind::NotAligned)
        } else if start % min_size != 0 || end % min_size != 0 {
            warn!("attempt to erase unaligned: {}-{}", start, end);
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
    const WRITE_SIZE: usize = 1; // TODO: 4 or configurable
    const ERASE_SIZE: usize = 512; // TODO: 4096 or configurable

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
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = 512;

    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), NorFlashErrorKind> {
        self.do_write(<Self as NorFlash>::WRITE_SIZE, offset, data)
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), NorFlashErrorKind> {
        self.do_erase(<Self as NorFlash>::ERASE_SIZE, from, to)
    }
}

#[cfg(test)]
#[maybe_async_cfg::maybe(
    idents(
        do_test(async = "block_on", sync = "block_on_sync"),
        AsyncNorFlash(async, sync = "NorFlash"),
        AsyncReadNorFlash(async, sync = "ReadNorFlash"),
    ),
    sync(self = "tests_sync", feature = "sync"),
    async(keep_self, feature = "async")
)]
mod tests_async {
    use super::*;
    use tokio_test::block_on;

    fn block_on_sync(_: ()) {}

    #[test]
    fn read() {
        do_test(async {
            const BUF: &[u8] = &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9];
            let mut ss = SliceStorage::new(BUF);
            let mut out = [0u8; 3];
            AsyncReadNorFlash::read(&mut ss, 0, &mut out).await.unwrap();
            assert_eq!(&BUF[0..3], &out);

            AsyncReadNorFlash::read(&mut ss, 3, &mut out).await.unwrap();
            assert_eq!(&BUF[3..6], &out);

            assert_eq!(
                AsyncReadNorFlash::read(&mut ss, 9, &mut out).await,
                Err(NorFlashErrorKind::OutOfBounds)
            );
        })
    }

    fn get_write_size<T: AsyncNorFlash>(_: &T) -> usize {
        T::WRITE_SIZE
    }

    #[test]
    fn write() {
        do_test(async {
            let mut buf = [0u8; 8192];
            let mut ss = SliceStorage::new(&mut buf[..]);

            const TESTDATA: &[u8] = &[0, 1, 2, 3, 4, 5, 6, 7];

            AsyncNorFlash::write(&mut ss, 0, &TESTDATA).await.unwrap();
            let mut readback = [0u8; 8];
            AsyncReadNorFlash::read(&mut ss, 0, &mut readback)
                .await
                .unwrap();
            assert_eq!(&readback, TESTDATA);

            if get_write_size(&ss) > 1 {
                assert_eq!(
                    AsyncNorFlash::write(&mut ss, 0, &TESTDATA[0..5]).await,
                    Err(NorFlashErrorKind::NotAligned)
                );
            }
        })
    }

    #[test]
    fn erase() {
        do_test(async {
            let mut buf = [0u8; 8192];
            let mut ss = SliceStorage::new(&mut buf[..]);

            AsyncNorFlash::erase(&mut ss, 4096, 8192).await.unwrap();
            let mut readback = [0u8; 8];
            AsyncReadNorFlash::read(&mut ss, 4092, &mut readback)
                .await
                .unwrap();
            assert_eq!(&readback, &[0, 0, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF]);

            assert_eq!(
                AsyncNorFlash::erase(&mut ss, 4, 8).await,
                Err(NorFlashErrorKind::NotAligned)
            );
        })
    }
}
