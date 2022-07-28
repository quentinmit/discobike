use core::ops::*;

#[cfg(feature = "async")]
use core::future::Future;
use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind};
#[cfg(feature = "sync")]
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
#[cfg(feature = "async")]
use embedded_storage_async::nor_flash::{AsyncNorFlash, AsyncReadNorFlash};

struct AddressedBlock<const BLOCK_SIZE: usize> {
    start: u32,
    buf: crate::Block<BLOCK_SIZE>,
}
impl<const BLOCK_SIZE: usize> AddressedBlock<BLOCK_SIZE> {
    pub fn contains(&self, addr: u32) -> bool {
        addr >= self.start && addr <= (self.start + self.buf.len() as u32)
    }
}

#[cfg(feature = "async")]
pub struct CachingStorage<T>
    where T: AsyncNorFlash
{
    storage: T,
    cache: Option<AddressedBlock<T::ERASE_SIZE>>,
}

#[cfg(feature = "async")]
impl CachingStorage<T>
    where T: AsyncNorFlash
{
    pub fn new(storage: T) -> Self {
        Self {
            storage,
            cache: None,
        }
    }
}

trait Split<T>
    where Self: Sized
{
    fn split_partials(&self, block_size: T) -> (Option<Self>, Option<Self>, Option<Self>);
    fn sub(&self, offset: T) -> Self;
}

impl<T> Split<T> for Range<T>
    where T: Rem<Output = T> + Sub<Output = T> + AddAssign + SubAssign + PartialEq + PartialOrd + Default + Copy
{
    fn split_partials(&self, block_size: T) -> (Option<Self>, Option<Self>, Option<Self>) {
        let mut start = self.start;
        let prefix = if self.start % block_size != Default::default() {
            start += block_size - (self.start % block_size);
            Some(self.start .. start)
        } else {
            None
        };
        let mut end = self.end;
        let suffix = if self.end % block_size != Default::default() {
            end -= self.end % block_size;
            Some(end .. self.end )
        } else {
            None
        };
        let infix = if end > start {
            Some(start .. end)
        } else {
            None
        };
        (prefix, infix, suffix)
    }
    fn sub(&self, offset: T) -> Self {
        (self.start - offset) .. (self.end - offset)
    }
}

#[cfg(feature = "async")]
impl<T: AsyncNorFlash> AsyncNorFlash for CachingStorage<T> {
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = 1;

    type WriteFuture<'a> = impl Future<Output = Result<(), NorFlashErrorKind>> + 'a
    where S: 'a;
    fn write<'a>(&'a mut self, offset: u32, data: &'a [u8]) -> Self::WriteFuture<'a> {
        async move {
            // Split the write into a leading partial write (if any), a full range write (if any), and a trailing partial write (if any)
            let range = (offset..offset+(data.len() as u32));
            let (prefix, infix, suffix) = range.split_partials(T::ERASE_SIZE);
            if let Some(prefix) = prefix {
                self.write_partial(prefix, &data[0..prefix.len()]).await?;
            }
            if let Some(infix) = infix {
                if self.cache.map_or(false, |c| c.contains(infix.start)) {
                    // Writing the whole block, so we can drop the cache.
                    self.cache = None;
                }
                self.storage.write(offset, &data[infix.sub(offset)]).await?;
            }
            if let Some(suffix) = suffix {
                self.write_partial(prefix, &data[0..prefix.len()]).await?;
            }
        }
    }

    type EraseFuture<'a> = impl Future<Output = Result<(), NorFlashErrorKind>> + 'a
    where S: 'a;
    fn erase<'a>(&'a mut self, from: u32, to: u32) -> Self::EraseFuture<'a> {
        async move { self.do_erase(<Self as AsyncNorFlash>::ERASE_SIZE, from, to) }
    }
}
