#![no_std]

#![feature(generic_associated_types)]
#![feature(type_alias_impl_trait)]

mod structs;
mod storage;

use arrayvec::ArrayString;
use itertools::Itertools;
use byte::BytesExt;

#[cfg(not(test))]
use defmt::{info, warn, trace};
#[cfg(test)]
use log::{info, warn, trace};

use embedded_storage_async::nor_flash::AsyncReadNorFlash;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum FsError {
    Io,
    Corrupt,
    Noent,
    NotDir,
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

#[derive(Copy, Clone, Debug)]
pub struct Dir {
    ptr: BlockPointerPair,
    pos: usize,
}

impl Default for Dir {
    fn default() -> Self {
        Dir{
            ptr: BlockPointerPair{a: 0, b: 0},
            pos: 0,
        }
    }
}

impl Dir {
    pub fn new(ptr: BlockPointerPair) -> Self {
        Dir{ ptr, pos: 0 }
    }
}

const NAME_MAX_LEN: usize = 256;

/// Wrapper around an array of u8 representing a filename in ASCII encoding.
type Filename = ArrayString<NAME_MAX_LEN>;

#[derive(Debug, PartialEq)]
pub enum EntryType {
    RegularFile,
    Directory,
}

#[derive(Debug, PartialEq)]
pub struct Info {
    entry_type: EntryType,
    size: usize,
    name: Filename,
}

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
    async fn dir_find(&mut self, path: &str) -> Result<Dir, FsError> {
        // TODO: Support a different starting directory.
        let mut dir = Dir::new(self.root_directory_ptr);
        let mut names = path.split("/");
        while let Some(name) = names.next() {
            match name {
                ""|"."|".." => continue,
                _ => (),
            }
            let lookahead = names.clone();
            if lookahead.scan(0, |state, n| {
                *state = *state + match n { "."|"" => 0, ".." => -1, _ => 1 };
                Some(*state)
            }).min().unwrap_or(1) <= 0 {
                // Skipped by a future ..
                continue
            }
            trace!("Looking for directory {:?}", name);
            let dirmeta = self.read_newer_block(dir.ptr).await?;
            for entry in dirmeta {
                match entry {
                    Err(_) => return Err(FsError::Corrupt),
                    Ok(entry) => if entry.name == name {
                        match entry.data {
                            structs::DirEntryData::Directory{directory_ptr} => {
                                dir.ptr = directory_ptr;
                                break;
                            },
                            structs::DirEntryData::File{..} => {
                                return Err(FsError::NotDir);
                            }
                            _ => {
                                return Err(FsError::Corrupt);
                            }
                        }
                    },
                }
            }
            return Err(FsError::Noent);
        }
        Ok(dir)
    }
    pub async fn dir_open(&mut self, dir: &mut Dir, path: &str) -> Result<(), FsError> {
        let found = self.dir_find(path).await?;
        *dir = found;
        Ok(())
    }
    pub async fn dir_read(&mut self, dir: &mut Dir) -> Result<Option<Info>, FsError> {
        // TODO: Cache the state in dir somehow?
        let mut dirmeta;
        loop {
            dirmeta = self.read_newer_block(dir.ptr).await?;
            match dirmeta.into_iter().enumerate().skip(dir.pos).next() {
                Some((pos, Ok(entry))) => {
                    let entry_type = match entry.data {
                        structs::DirEntryData::File{ .. } => EntryType::RegularFile,
                        structs::DirEntryData::Directory { .. } => EntryType::Directory,
                        _ => return Err(FsError::Corrupt),
                    };
                    dir.pos = pos+1;
                    return Ok(Some(Info{
                        name: Filename::from(entry.name).map_err(|_| FsError::Corrupt)?,
                        entry_type,
                        size: 0,
                    }));
                },
                Some((_, Err(_))) => return Err(FsError::Corrupt),
                None => (),
            }
            if dirmeta.continued {
                dir.ptr = dirmeta.tail_ptr;
                dir.pos = 0;
                continue
            }
            return Ok(None);
        }
    }
}

#[cfg(test)]
mod tests {
    use simplelog::*;
    #[ctor::ctor]
    fn init() {
        TermLogger::init(LevelFilter::Trace, Config::default(), TerminalMode::Mixed, ColorChoice::Auto).unwrap();
    }

    use super::*;
    use super::storage::SliceStorage;

    extern crate std;
    use std::println;
    extern crate alloc;
    use alloc::vec::Vec;
    use alloc::string::String;

    use itertools::repeat_n;
    use tokio_test::block_on;

    use byte::{BytesExt, Result};

    const THREE_FILES: &[u8] = include_bytes!("../testdata/three-files.img");

    #[test]
    fn three_files() {
        block_on(async {
            let mut fs: LittleFs<_, 512> = LittleFs::new(SliceStorage::new(THREE_FILES));
            fs.mount().await.unwrap();

            let mut dir = Dir::default();
            fs.dir_open(&mut dir, "/").await.unwrap();
            println!("open dir: {:?}", dir);
            let mut names = Vec::<String>::new();
            while let Some(info) = fs.dir_read(&mut dir).await.unwrap() {
                println!("entry: {:?}", info);
                names.push(info.name.as_ref().into());
            }
            assert_eq!(names, &["static0", "static1", "static2"]);
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
