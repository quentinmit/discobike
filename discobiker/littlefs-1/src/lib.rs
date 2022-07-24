#![no_std]

mod structs;

use byte::{BytesExt};

use embedded_storage_async::nor_flash::AsyncNorFlash;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum FsError {
    Corrupt,
}

struct LittleFs<S, const MAX_BLOCK_SIZE: usize> {
    storage: S,
    buf: [u8; MAX_BLOCK_SIZE],
    block_size: u32,
    block_count: u32,
}

impl<S: AsyncNorFlash, const MAX_BLOCK_SIZE: usize> LittleFs<S, MAX_BLOCK_SIZE> {
    pub fn new(storage: S) -> Self {
        LittleFs {
            storage,
            buf: [0; MAX_BLOCK_SIZE],
            block_size: 0,
            block_count: 0,
        }
    }
}
    fn parse_superblock(bytes: &[u8]) -> Result<structs::DirEntry, FsError> {
        let block: structs::MetadataBlock = bytes.read(&mut 0).map_err(|_| FsError::Corrupt)?;
        let mut iter = block.into_iter();
        let entry = iter.next().map(|e| e.ok()).flatten().ok_or(FsError::Corrupt)?;
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
