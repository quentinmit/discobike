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
        }
    }
}

type BlockPointer = u32;

impl<S: AsyncReadNorFlash, const BLOCK_SIZE: usize> LittleFs<S, BLOCK_SIZE> {
    async fn read_newer_block(&mut self, a: BlockPointer, b: BlockPointer) -> Result<structs::MetadataBlock, FsError> {
        self.storage.read(a * (BLOCK_SIZE as u32), &mut self.buf1).await.map_err(|_| FsError::Io)?;
        let block1: Option<structs::MetadataBlock> = self.buf1.read(&mut 0).ok();
        self.storage.read(b * (BLOCK_SIZE as u32), &mut self.buf2).await.map_err(|_| FsError::Io)?;
        let block2: Option<structs::MetadataBlock> = self.buf2.read(&mut 0).ok();

        [block1, block2].into_iter().filter_map(|x| x).reduce(|a, b| if a.revision_count > b.revision_count { a } else { b }).ok_or(FsError::Corrupt)
    }
    pub async fn mount(&mut self) -> Result<(), FsError> {
        let sbmeta = self.read_newer_block(0, 1).await?;
        let entry = sbmeta.into_iter().exactly_one().map_err(|_| FsError::Corrupt)?;
        Ok(())
    }
}

fn parse_superblock(bytes: &[u8]) -> Result<structs::DirEntry, FsError> {
    let block: structs::MetadataBlock = bytes.read(&mut 0).map_err(|_| FsError::Corrupt)?;
    let mut iter = block.into_iter();
    let entry = iter
        .next()
        .map(|e| e.ok())
        .flatten()
        .ok_or(FsError::Corrupt)?;
    if iter.next() != None {
        return Err(FsError::Corrupt);
    }
    if entry.name != "littlefs" {
        return Err(FsError::Corrupt);
    }
    Ok(entry)
}

#[cfg(test)]
mod tests {
    use super::*;

    extern crate std;
    use std::println;
    extern crate alloc;
    use alloc::vec::Vec;

    use byte::{BytesExt, Result};

    const THREE_FILES: &[u8] = include_bytes!("../testdata/three-files.img");

    #[test]
    fn three_files() {
        let offset = &mut 0;

        let entry = parse_superblock(THREE_FILES).unwrap();

        println!("superblock entry: {:?}", entry);
    }
}
