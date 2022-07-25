#![no_std]
#![feature(generic_associated_types)]
#![feature(type_alias_impl_trait)]

mod storage;
mod structs;

use structs::{AsOffset, BlockPointer, BlockPointerPair};

use arrayvec::ArrayString;
use bitflags::bitflags;
use byte::{BytesExt, LE};
use itertools::Itertools;
use core::cmp::min;

#[cfg(not(test))]
use defmt::{info, trace, warn};
#[cfg(test)]
use log::{info, trace, warn};

use embedded_storage_async::nor_flash::AsyncReadNorFlash;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum FsError {
    Io,
    Corrupt,
    Noent,
    NotDir,
    Inval,
}

pub struct LittleFs<S, const BLOCK_SIZE: usize> {
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

#[derive(Copy, Clone, Debug)]
pub struct Dir {
    ptr: BlockPointerPair,
    pos: usize,
}

impl Default for Dir {
    fn default() -> Self {
        Dir {
            ptr: BlockPointerPair { a: 0, b: 0 },
            pos: 0,
        }
    }
}

impl Dir {
    fn new(ptr: BlockPointerPair) -> Self {
        Dir { ptr, pos: 0 }
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

#[derive(Debug, PartialEq)]
pub struct File {
    /// Pointer to the first block of the file.
    head: BlockPointer,
    /// Total size of file, in bytes.
    size: u32,
    /// Current position within file, in bytes.
    pos: u32,
    // TODO: Track position within a block to save CTZ lookups.
    // /// Block pointer containing pos.
    // block: BlockPointer,
    // /// Offset of pos within block.
    // off: u32,
}

impl Default for File {
    fn default() -> Self {
        File {
            head: 0,
            size: 0,
            pos: 0,
            //off: 0,
            //block: 0,
        }
    }
}

/// Definition of file open flags which can be mixed and matched as appropriate. These definitions
/// are reminiscent of the ones defined by POSIX.
bitflags! {
    pub struct FileOpenFlags: u32 {
        /// Open file in read only mode.
        const RDONLY = 0x1;
        /// Open file in write only mode.
        const WRONLY = 0x2;
        /// Open file for reading and writing.
        const RDWR = Self::RDONLY.bits | Self::WRONLY.bits;
        /// Create the file if it does not exist.
        const CREAT = 0x0100;
        /// Fail if creating a file that already exists.
        const EXCL = 0x0200;
        /// Truncate the file if it already exists.
        const TRUNC = 0x0400;
        /// Open the file in append only mode.
        const APPEND = 0x0800;
    }
}

fn npw2(a: u32) -> u32 {
    32 - (a-1).leading_zeros()
}

impl<S: AsyncReadNorFlash, const BLOCK_SIZE: usize> LittleFs<S, BLOCK_SIZE> {
    async fn read_newer_block(
        &mut self,
        ptr: BlockPointerPair,
    ) -> Result<structs::MetadataBlock, FsError> {
        self.storage
            .read(ptr.a.as_offset(BLOCK_SIZE), &mut self.buf1)
            .await
            .map_err(|_| FsError::Io)?;
        let block1: Option<structs::MetadataBlock> = self.buf1.read(&mut 0).ok();
        self.storage
            .read(ptr.b.as_offset(BLOCK_SIZE), &mut self.buf2)
            .await
            .map_err(|_| FsError::Io)?;
        let block2: Option<structs::MetadataBlock> = self.buf2.read(&mut 0).ok();

        [block1, block2]
            .into_iter()
            .filter_map(|x| x)
            .reduce(|a, b| {
                if a.revision_count > b.revision_count {
                    a
                } else {
                    b
                }
            })
            .ok_or(FsError::Corrupt)
    }
    pub async fn mount(&mut self) -> Result<(), FsError> {
        let sbmeta = self
            .read_newer_block(BlockPointerPair { a: 0, b: 1 })
            .await?;
        let entry = sbmeta
            .into_iter()
            .exactly_one()
            .map_err(|_| FsError::Corrupt)?
            .map_err(|_| FsError::Corrupt)?;
        if entry.name != "littlefs" {
            return Err(FsError::Corrupt);
        }
        match entry.data {
            structs::DirEntryData::Superblock {
                root_directory_ptr,
                block_size,
                block_count,
                ..
            } => {
                self.block_size = block_size;
                if block_size != BLOCK_SIZE as u32 {
                    return Err(FsError::Inval);
                }
                self.root_directory_ptr = root_directory_ptr;
                self.block_count = block_count;
                Ok(())
            }
            _ => Err(FsError::Corrupt),
        }
    }
    async fn dir_find<'a>(&mut self, path: &'a str) -> Result<structs::DirEntryData, FsError> {
        // TODO: Support a different starting directory.
        let mut dir = Dir::new(self.root_directory_ptr);
        let mut names = path.split("/");
        while let Some(name) = names.next() {
            match name {
                "" | "." | ".." => continue,
                _ => (),
            }
            let lookahead = names.clone();
            if lookahead
                .scan(0, |state, n| {
                    *state = *state
                        + match n {
                            "." | "" => 0,
                            ".." => -1,
                            _ => 1,
                        };
                    Some(*state)
                })
                .min()
                .unwrap_or(1)
                < 0
            {
                // Skipped by a future ..
                continue;
            }
            trace!("Looking for component {:?}", name);
            // TODO: Share implementation with dir_read
            let mut continued = true;
            let mut found = false;
            while continued {
                let block = self.read_newer_block(dir.ptr).await?;
                continued = block.continued;
                dir.ptr = block.tail_ptr;
                if let Some(entry) = block.find_entry(name).map_err(|_| FsError::Corrupt)? {
                    if let None = names.clone().next() {
                        return Ok(entry.data);
                    } else {
                        match entry.data {
                            structs::DirEntryData::Directory { directory_ptr } => {
                                dir.ptr = directory_ptr;
                                found = true;
                            }
                            structs::DirEntryData::File { .. } => {
                                return Err(FsError::NotDir)
                            }
                            _ => return Err(FsError::Corrupt),
                        }
                    }
                }
            }
            if !found {
                return Err(FsError::Noent);
            }
        }
        // Fake directory for /
        Ok(structs::DirEntryData::Directory {
            directory_ptr: dir.ptr,
        })
    }
    pub async fn dir_open(&mut self, dir: &mut Dir, path: &str) -> Result<(), FsError> {
        let entry_data = self.dir_find(path).await?;
        match entry_data {
            structs::DirEntryData::Directory { directory_ptr } => {
                dir.ptr = directory_ptr;
                dir.pos = 0;
                Ok(())
            }
            structs::DirEntryData::File { .. } => Err(FsError::NotDir),
            _ => Err(FsError::Corrupt),
        }
    }
    pub async fn dir_read(&mut self, dir: &mut Dir) -> Result<Option<Info>, FsError> {
        // TODO: Cache the state in dir somehow?
        // TODO: Synthesize "." and ".." entries.
        let mut dirmeta;
        loop {
            dirmeta = self.read_newer_block(dir.ptr).await?;
            match dirmeta.into_iter().enumerate().skip(dir.pos).next() {
                Some((pos, Ok(entry))) => {
                    let (entry_type, size) = match entry.data {
                        structs::DirEntryData::File { size, .. } => (EntryType::RegularFile, size),
                        structs::DirEntryData::Directory { .. } => (EntryType::Directory, 0),
                        _ => continue,
                    };
                    dir.pos = pos + 1;
                    return Ok(Some(Info {
                        name: Filename::from(entry.name).map_err(|_| FsError::Corrupt)?,
                        entry_type,
                        size: size as usize,
                    }));
                }
                Some((_, Err(_))) => return Err(FsError::Corrupt),
                None => (),
            }
            if dirmeta.continued {
                dir.ptr = dirmeta.tail_ptr;
                dir.pos = 0;
                continue;
            }
            return Ok(None);
        }
    }
    pub async fn file_open(
        &mut self,
        file: &mut File,
        path: &str,
        flags: FileOpenFlags,
    ) -> Result<(), FsError> {
        // TODO: deorphan once after mount
        if flags != FileOpenFlags::RDONLY {
            return Err(FsError::Inval);
        }
        let entry_data = self.dir_find(path).await?;
        match entry_data {
            structs::DirEntryData::File {
                head,
                size,
            } => {
                *file = File {
                    head,
                    size,
                    .. Default::default()
                };
                Ok(())
            },
            _ => Err(FsError::NotDir),
        }
    }

    pub async fn file_read(&mut self, file: &mut File, buf: &mut [u8]) -> Result<usize, FsError> {
        let size = min(buf.len() as u32, file.size.saturating_sub(file.pos));
        let mut buf = &mut buf[..size as usize];

        // TODO: Cache a whole block at a time for performance (requires saving
        // block and off in file).

        while buf.len() > 0 {
            let (block, off) = self.ctz_find(file.head, file.size, file.pos).await?;
            let diff = min(buf.len() as u32, self.block_size - off) as usize;
            self.storage.read(
                block.as_offset(BLOCK_SIZE) + off,
                &mut buf[..diff]
            ).await.map_err(|_| FsError::Io)?;
            buf = &mut buf[diff..];
        }
        // We only update file here so that the position is not updated on a failed read.
        file.pos = file.pos.saturating_add(size);
        Ok(size as usize)
    }

    fn ctz_index(&self, off: u32) -> (u32, u32) {
        let size = off;
        let b = self.block_size - 2*4;
        let i = size / b;
        if i == 0 {
            return (0, off);
        }
        let i = (size - 4*((i-1).count_ones()+2)) / b;
        (i, size - b*i - 4*i.count_ones())
    }

    async fn ctz_find(
        &mut self,
        head: BlockPointer, size: u32,
        pos: u32,
        ) -> Result<(BlockPointer, u32), FsError> {
        let mut head = head;
        let (mut current, _) = self.ctz_index(size - 1);
        let (target, pos) = self.ctz_index(pos);

        while current > target {
            let skip = min(
                npw2(current-target+1)-1,
                current.trailing_zeros(),
            );
            let mut buf = [0u8; 4];
            self.storage.read(head.as_offset(BLOCK_SIZE), &mut buf[..]).await.map_err(|_| FsError::Io)?;
            head = buf.read_with(&mut 0, LE).map_err(|_| FsError::Corrupt)?;
            if !(head >= 2 && head <= self.block_count) {
                return Err(FsError::Corrupt);
            }
            current -= 1 << skip;
        }
        Ok((head, pos))
    }
}

#[cfg(test)]
mod tests {
    use simplelog::*;
    #[ctor::ctor]
    fn init() {
        TermLogger::init(
            LevelFilter::Trace,
            Config::default(),
            TerminalMode::Mixed,
            ColorChoice::Auto,
        )
        .unwrap();
    }

    use super::storage::SliceStorage;
    use super::*;

    extern crate std;
    use core::iter::repeat;
    use std::println;
    use alloc::format;
    extern crate alloc;
    use alloc::string::String;
    use alloc::vec::Vec;

    use itertools::repeat_n;
    use tokio_test::block_on;

    use byte::{BytesExt, Result};

    const THREE_FILES: &[u8] = include_bytes!("../testdata/three-files.img");
    const LARGE_DIR: &[u8] = include_bytes!("../testdata/large-dir.img");

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
                if info.entry_type == EntryType::RegularFile {
                    names.push(info.name.as_ref().into());
                    assert_eq!(info.size, 512);
                }
            }
            let expected_data: Vec<u8> = b"0123456789abcdef".into_iter().cycle().take(512).cloned().collect();
            assert_eq!(names, &["static0", "static1", "static2"]);
            for path in &["static0", "static1", "static2", "subdir/file"] {
                let mut file = Default::default();
                fs.file_open(&mut file, path, FileOpenFlags::RDONLY).await.unwrap();
                println!("{} file: {:?}", path, file);
                let mut data = [0u8;1024];
                assert_eq!(fs.file_read(&mut file, &mut data).await.unwrap(), 512);
                println!("{} data: {:?}", path, &data[..512]);
                assert_eq!(&data[..512], &expected_data);
            }
        });
    }

    #[test]
    fn three_files_corrupt() {
        block_on(async {
            let data: Vec<u8> = repeat_n(0u8, 512)
                .chain(THREE_FILES[512..].iter().cloned())
                .collect();
            let mut fs: LittleFs<_, 512> = LittleFs::new(SliceStorage::new(&data[..]));
            fs.mount().await.unwrap();
        });
    }

    #[test]
    fn large_dir() {
        block_on(async {
            let mut fs: LittleFs<_, 512> = LittleFs::new(SliceStorage::new(LARGE_DIR));
            fs.mount().await.unwrap();

            let mut dir = Dir::default();
            fs.dir_open(&mut dir, "/").await.unwrap();
            println!("open dir: {:?}", dir);
            let mut names = Vec::<String>::new();
            while let Some(info) = fs.dir_read(&mut dir).await.unwrap() {
                println!("entry: {:?}", info);
                if info.entry_type == EntryType::RegularFile {
                    names.push(info.name.as_ref().into());
                    assert_eq!(info.size, if info.name.as_ref() == "zzz" { 512 } else { 0 });
                }
            }
            let expected_names: Vec<String> = (0..=99).into_iter().map(|n| format!("static{}", n)).chain([String::from("zzz")]).collect();
            assert_eq!(names, expected_names);
            let expected_data: Vec<u8> = b"0123456789abcdef".into_iter().cycle().take(512).cloned().collect();
            let mut file = Default::default();
            let path = "zzz";
            fs.file_open(&mut file, path, FileOpenFlags::RDONLY).await.unwrap();
            println!("{} file: {:?}", path, file);
            let mut data = [0u8;1024];
            assert_eq!(fs.file_read(&mut file, &mut data).await.unwrap(), 512);
            println!("{} data: {:?}", path, &data[..512]);
            assert_eq!(&data[..512], &expected_data);
            let path = "static0";
            fs.file_open(&mut file, path, FileOpenFlags::RDONLY).await.unwrap();
            assert_eq!(fs.file_read(&mut file, &mut data).await.unwrap(), 0);
        });
    }
}
