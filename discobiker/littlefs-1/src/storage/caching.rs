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

trait Rounding {
    fn last_multiple_of(&self, rhs: Self) -> Self;
    fn next_multiple_of(&self, rhs: Self) -> Self;
}

impl<T> Rounding for T
where T: Rem<Output = T> + Sub<Output = T> + Add<Output = T> + Default + PartialEq + Copy
{
    fn last_multiple_of(&self, rhs: Self) -> Self {
        *self - (*self % rhs)
    }
    fn next_multiple_of(&self, rhs: Self) -> Self {
        if *self % rhs != Default::default() {
            *self + rhs - (*self % rhs)
        } else {
            *self
        }
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
    async fn commit(&mut self) -> Result<(), NorFlashErrorKind> {
        match self.cache.take() {
            Some(block) => {
                self.storage.erase(block.start, block.start+data.len() as u32).await?;
                self.storage.write(block.start, block.buf.as_slice()).await?;
            }
            None => ()
        }
        Ok(())
    }
    async fn fetch_block_containing(&mut self, offset: u32) -> Result<(), NorFlashErrorKind> {
        match self.cache {
            Some(block) => {
                if !block.contains(offset) {
                    self.commit().await?;
                }
            },
            None => (),
        }
        match self.cache {
            None => {
                let start = offset.last_multiple_of(T::ERASE_SIZE);
                let buf = Block::<T::ERASE_SIZE>::zero(T::ERASE_SIZE);
                self.storage.read(start, buf.as_mut_slice()).await?;
                self.cache = Some(AddressedBlock{
                    start,
                    buf
                })
            },
            Some(_) => (),
        }
    }
    async fn write_partial(&mut self, offset: u32, data: &[u8]) -> Result<(), NorFlashErrorKind> {
        self.fetch_block_containing(offset).await?;
        match self.cache.as_mut() {
            None => panic!("failed to fetch block"),
            Some(block) => {
                let start = offset - block.start;
                block.buf.as_mut_slice()[start..start+data.len()].copy_from(data);
            }
        }
        Ok(())
    }
}

trait Split<T>
    where Self: Sized
{
    fn split_partials(&self, block_size: T) -> (Option<Self>, Option<Self>, Option<Self>);
    fn sub(&self, offset: T) -> Self;
}

impl<T> Split<T> for Range<T>
    where T: Rem<Output = T> + Sub<Output = T> + Add<Output = T> + AddAssign + SubAssign + PartialEq + PartialOrd + Default + Copy
{
    fn split_partials(&self, block_size: T) -> (Option<Self>, Option<Self>, Option<Self>) {
        let mut start = self.start.next_multiple_of(block_size);
        let prefix = if self.start != start {
            Some(self.start .. start)
        } else {
            None
        };
        let mut end = self.end.last_multiple_of(block_size);
        let suffix = if self.end != end {
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
                self.write_partial(prefix.start, &data[0..prefix.len()]).await?;
            }
            if let Some(infix) = infix {
                if self.cache.map_or(false, |c| c.contains(infix.start)) {
                    // Writing the whole block, so we can drop the cache.
                    self.cache = None;
                }
                self.storage.write(offset, &data[infix.sub(offset)]).await?;
            }
            if let Some(suffix) = suffix {
                self.write_partial(prefix.start, &data[0..prefix.len()]).await?;
            }
        }
    }

    type EraseFuture<'a> = impl Future<Output = Result<(), NorFlashErrorKind>> + 'a
    where S: 'a;
    fn erase<'a>(&'a mut self, from: u32, to: u32) -> Self::EraseFuture<'a> {
        async move { self.do_erase(<Self as AsyncNorFlash>::ERASE_SIZE, from, to) }
    }
}
