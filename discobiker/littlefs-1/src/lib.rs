#![no_std]

#![feature(generic_associated_types)]
#![feature(type_alias_impl_trait)]

mod structs;
mod storage;

use itertools::Itertools;
use byte::BytesExt;

use embedded_storage_async::nor_flash::AsyncReadNorFlash;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum FsError {
    Io,
    Corrupt,
    Inval,
}

struct LittleFs<S, const BLOCK_SIZE: usize> {
    storage: S,
    buf1: [u8; BLOCK_SIZE],
    buf2: [u8; BLOCK_SIZE],
    root_directory_ptr: BlockPointerPair,
    block_size: u32,
    block_count: u32,
}

impl<S, const BLOCK_SIZE: usize> LittleFs<S, BLOCK_SIZE> {
    pub fn new(storage: S) -> Self {
        LittleFs {
            storage,
            buf1: [0; BLOCK_SIZE],
            buf2: [0; BLOCK_SIZE],
            block_size: 0,
            block_count: 0,
            root_directory_ptr: Default::default(),
        }
    }
}

use structs::{BlockPointerPair, AsOffset};

impl<S: AsyncReadNorFlash, const BLOCK_SIZE: usize> LittleFs<S, BLOCK_SIZE> {
    async fn read_newer_block(&mut self, ptr: BlockPointerPair) -> Result<structs::MetadataBlock, FsError> {
        self.storage.read(ptr.a.as_offset(BLOCK_SIZE), &mut self.buf1).await.map_err(|_| FsError::Io)?;
        let block1: Option<structs::MetadataBlock> = self.buf1.read(&mut 0).ok();
        self.storage.read(ptr.b.as_offset(BLOCK_SIZE), &mut self.buf2).await.map_err(|_| FsError::Io)?;
        let block2: Option<structs::MetadataBlock> = self.buf2.read(&mut 0).ok();

        [block1, block2].into_iter().filter_map(|x| x).reduce(|a, b| if a.revision_count > b.revision_count { a } else { b }).ok_or(FsError::Corrupt)
    }
    pub async fn mount(&mut self) -> Result<(), FsError> {
        let sbmeta = self.read_newer_block(BlockPointerPair{a: 0, b: 1}).await?;
        let entry = sbmeta.into_iter().exactly_one().map_err(|_| FsError::Corrupt)?.map_err(|_| FsError::Corrupt)?;
        if entry.name != "littlefs" { return Err(FsError::Corrupt) }
        match entry.data {
            structs::DirEntryData::Superblock { root_directory_ptr, block_size, block_count, .. } => {
                self.block_size = block_size;
                if block_size != BLOCK_SIZE as u32 {
                    return Err(FsError::Inval);
                }
                self.root_directory_ptr = root_directory_ptr;
                self.block_count = block_count;
                Ok(())
            }
            _ => Err(FsError::Corrupt)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::storage::SliceStorage;

    extern crate std;
    use std::println;
    extern crate alloc;
    use alloc::vec::Vec;

    use itertools::repeat_n;
    use tokio_test::block_on;

    use byte::{BytesExt, Result};

    const THREE_FILES: &[u8] = include_bytes!("../testdata/three-files.img");

    #[test]
    fn three_files() {
        block_on(async {
            let mut fs: LittleFs<_, 512> = LittleFs::new(SliceStorage::new(THREE_FILES));
            fs.mount().await.unwrap();
        });
    }

    #[test]
    fn three_files_corrupt() {
        block_on(async {
            let data: Vec<u8> = repeat_n(0u8, 512).chain(THREE_FILES[512..].iter().cloned()).collect();
            let mut fs: LittleFs<_, 512> = LittleFs::new(SliceStorage::new(&data[..]));
            fs.mount().await.unwrap();
        });
    }
}
