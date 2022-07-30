use crate::Block;
use core::ops::*;
use core::cmp::{max, min};
use num::{Integer, NumCast, ToPrimitive};
use paste::paste;

#[cfg(feature = "async")]
use core::future::Future;
use embedded_storage::nor_flash::{ErrorType, NorFlashError, NorFlashErrorKind};
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
where
    T: AsyncNorFlash,
    [(); T::ERASE_SIZE]:,
{
    storage: T,
    cache: Option<AddressedBlock<{ T::ERASE_SIZE }>>,
}

#[cfg(feature = "async")]
impl<T: AsyncNorFlash> CachingStorage<T>
where
    T: AsyncNorFlash,
    [(); T::ERASE_SIZE]:,
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
                self.storage
                    .erase(block.start, block.start + block.buf.len() as u32)
                    .await
                    .map_err(|e| e.kind())?;
                self.storage
                    .write(block.start, block.buf.as_slice())
                    .await
                    .map_err(|e| e.kind())?;
            }
            None => (),
        }
        Ok(())
    }
    async fn fetch_block_containing(&mut self, offset: u32) -> Result<(), NorFlashErrorKind> {
        match &self.cache {
            Some(block) => {
                if !block.contains(offset) {
                    self.commit().await?;
                }
            }
            None => (),
        }
        match self.cache {
            None => {
                let start = offset.prev_multiple_of(&(T::ERASE_SIZE as u32));
                let mut buf = Block::zero(T::ERASE_SIZE);
                self.storage
                    .read(start, buf.as_mut_slice())
                    .await
                    .map_err(|e| e.kind())?;
                self.cache = Some(AddressedBlock { start, buf })
            }
            Some(_) => (),
        }
        Ok(())
    }
    async fn write_partial(&mut self, offset: u32, data: &[u8]) -> Result<(), NorFlashErrorKind> {
        self.fetch_block_containing(offset).await?;
        let block = self.cache.as_mut().unwrap();
        let start = (offset - block.start) as usize;
        block.buf.as_mut_slice()[start..start + data.len()].copy_from_slice(data);
        Ok(())
    }
    async fn erase_partial(&mut self, from: u32, to: u32) -> Result<(), NorFlashErrorKind> {
        trace!("erase partial {:?}-{:?}", from, to);
        self.fetch_block_containing(from).await?;
        let block = self.cache.as_mut().unwrap();
        let start = (from - block.start) as usize;
        let end = (to - block.start) as usize;
        block.buf.as_mut_slice()[start..end].fill(0xFF);
        Ok(())
    }
}

trait Split<T>
where
    Self: Sized,
{
    fn split_partials(&self, block_size: T) -> (Option<Self>, Option<Self>, Option<Self>);
    fn sub(&self, offset: T) -> Self;
    fn to_usize(&self) -> Range<usize>;
}

impl<T> Split<T> for Range<T>
where
    T: Integer + ToPrimitive + Clone + Copy,
{
    fn split_partials(&self, block_size: T) -> (Option<Self>, Option<Self>, Option<Self>) {
        let start = min(self.end, self.start.next_multiple_of(&block_size));
        let prefix = if self.start != start {
            Some(self.start..start)
        } else {
            None
        };
        let end = max(start, self.end.prev_multiple_of(&block_size));
        let suffix = if self.end != end {
            Some(end..self.end)
        } else {
            None
        };
        let infix = if end > start { Some(start..end) } else { None };
        (prefix, infix, suffix)
    }
    fn sub(&self, offset: T) -> Self {
        (self.start - offset)..(self.end - offset)
    }
    fn to_usize(&self) -> Range<usize> {
        NumCast::from(self.start).unwrap_or(0)..NumCast::from(self.end).unwrap_or(0)
    }
}

impl<S: AsyncNorFlash> ErrorType for CachingStorage<S>
where
    S: AsyncNorFlash,
    [(); S::ERASE_SIZE]:,
{
    type Error = NorFlashErrorKind;
}

#[cfg(feature = "async")]
impl<S: AsyncNorFlash> AsyncReadNorFlash for CachingStorage<S>
where
    S: AsyncNorFlash,
    [(); S::ERASE_SIZE]:,
{
    const READ_SIZE: usize = 1;

    type ReadFuture<'a> = impl Future<Output = Result<(), NorFlashErrorKind>> + 'a
    where S: 'a;
    fn read<'a>(&'a mut self, address: u32, data: &'a mut [u8]) -> Self::ReadFuture<'a> {
        async move {
            let range = address..address + (data.len() as u32);
            // TODO: Answer from cache if possible.
            self.commit().await?;
            self.storage
                .read(address, data)
                .await
                .map_err(|e| e.kind())?;
            Ok(())
        }
    }

    fn capacity(&self) -> usize {
        self.storage.capacity()
    }
}

macro_rules! split_op {
    (@inner $self:ident, $range:expr, $f:ident $args:tt ) => {
        paste! {
            {
                let range = $range;
                let (prefix, infix, suffix) = range.split_partials(S::ERASE_SIZE as u32);
                trace!("{:?} split into {:?}, {:?}, {:?}", range, prefix, infix, suffix);
                if let Some(prefix) = prefix {
                    let slice = prefix.sub(range.start).to_usize();
                    let (a, b) = $args(prefix, slice);
                    $self.[<$f _partial>](a, b).await?;
                }
                if let Some(infix) = infix {
                    if $self.cache.as_ref().map_or(false, |c| c.contains(infix.start)) {
                        // Writing the whole block, so we can drop the cache.
                        $self.cache = None;
                    }
                    let slice = infix.sub(range.start).to_usize();
                    let (a, b) = $args(infix, slice);
                    $self.storage.$f(a, b).await.map_err(|e| e.kind())?;
                }
                if let Some(suffix) = suffix {
                    let slice = suffix.sub(range.start).to_usize();
                    let (a, b) = $args(suffix, slice);
                    $self.[<$f _partial>](a, b).await?;
                }
                Ok(())
            }
        }
    };
    (write, $self:ident, $range:expr, $data:expr) => {
        split_op!(@inner $self, $range, write(|range: Range<u32>, slice| (range.start, &$data[slice])))
    };
    (erase, $self:ident, $range:expr) => {
        split_op!(@inner $self, $range, erase(|range: Range<u32>, _| (range.start, range.end)))
    };
}

#[cfg(feature = "async")]
impl<S: AsyncNorFlash> AsyncNorFlash for CachingStorage<S>
where
    S: AsyncNorFlash,
    [(); S::ERASE_SIZE]:,
{
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = 1;

    type WriteFuture<'a> = impl Future<Output = Result<(), NorFlashErrorKind>> + 'a
    where S: 'a;
    fn write<'a>(&'a mut self, offset: u32, data: &'a [u8]) -> Self::WriteFuture<'a> {
        async move {
            // Split the write into a leading partial write (if any), a full range write (if any), and a trailing partial write (if any)
            split_op!(write, self, offset..offset + (data.len() as u32), &data)
        }
    }

    type EraseFuture<'a> = impl Future<Output = Result<(), NorFlashErrorKind>> + 'a
    where S: 'a;
    fn erase<'a>(&'a mut self, from: u32, to: u32) -> Self::EraseFuture<'a> {
        async move { split_op!(erase, self, from..to) }
    }
}

#[cfg(all(test, feature = "async"))]
pub(crate) mod tests_async {
    use super::super::slice::SliceStorage;
    use super::*;

    extern crate alloc;
    extern crate std;
    use std::println;
    use tokio_test::block_on as do_test;

    fn block_on_sync(_: ()) {}

    #[test]
    fn partial_write() {
        do_test(async {
            let mut buf = [0u8; 8192];
            let mut ss = SliceStorage::new(&mut buf[..]);
            let mut cache = CachingStorage::new(ss);

            const TESTDATA: &[u8] = &[0, 1, 2, 3];

            AsyncNorFlash::write(&mut cache, 0, &TESTDATA)
                .await
                .unwrap();
            AsyncNorFlash::erase(&mut cache, 10, 20)
                .await
                .unwrap();
            cache.commit().await.unwrap();
            println!("result: {:?}", buf);
        })
    }
}
