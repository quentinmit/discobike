#![no_std]
#![feature(generic_const_exprs)]
#![cfg_attr(
    feature = "async",
    feature(generic_associated_types),
    feature(type_alias_impl_trait),
)]

mod block;
mod fmt;
mod free;
mod storage;
mod structs;

use self::block::Block;
use self::structs::{AsOffset, BlockPointer, BlockPointerPair, MetadataBlock};

use arrayvec::{ArrayString, ArrayVec};
use bitflags::bitflags;
use byte::{BytesExt, LE};
use core::cmp::min;
use itertools::Itertools;
use maybe_async_cfg;

#[cfg(not(defmt))]
trait Format {}

#[cfg(feature = "sync")]
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
#[cfg(feature = "async")]
use embedded_storage_async::nor_flash::{AsyncNorFlash, AsyncReadNorFlash};

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FsError<T> {
    /// Input/output error occurred.
    Io(T),
    /// File was corrupt.
    Corrupt,
    /// No entry found with that name.
    Noent,
    /// File or directory already exists.
    Exist,
    /// Path name is not a directory.
    NotDir,
    /// Incorrect value specified to function.
    Inval,
    /// No space left available for operation.
    Nospc,
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct Dir<const BLOCK_SIZE: usize> {
    /// First metadata blocks for dir
    head: BlockPointerPair,
    /// Current position metadata blocks
    ptr: BlockPointerPair,
    /// Cached block at ptr
    block: Option<MetadataBlock<Block<BLOCK_SIZE>>>,
    pos: usize,
}

impl<const BLOCK_SIZE: usize> Default for Dir<BLOCK_SIZE> {
    fn default() -> Self {
        Dir {
            ptr: BlockPointerPair { a: 0, b: 0 },
            head: BlockPointerPair::default(),
            block: None,
            pos: 0,
        }
    }
}

impl<const BLOCK_SIZE: usize> Dir<BLOCK_SIZE> {
    fn new(ptr: BlockPointerPair) -> Self {
        Dir {
            ptr,
            head: ptr,
            ..Default::default()
        }
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
    32 - (a - 1).leading_zeros()
}

#[maybe_async_cfg::maybe(
    sync(self = "LittleFs", feature = "sync"),
    async(keep_self, feature = "async")
)]
pub struct AsyncLittleFs<S, const BLOCK_SIZE: usize> {
    storage: S,
    root_directory_ptr: BlockPointerPair,
    block_size: u32,
    block_count: u32,
    free: free::FreeBlockCache,
}

#[maybe_async_cfg::maybe(
    sync(self = "LittleFs", feature = "sync"),
    async(keep_self, feature = "async")
)]
impl<S, const BLOCK_SIZE: usize> AsyncLittleFs<S, BLOCK_SIZE> {
    pub fn new(storage: S) -> Self {
        AsyncLittleFs {
            storage,
            block_size: 0,
            block_count: 0,
            root_directory_ptr: Default::default(),
            free: free::FreeBlockCache::new(),
        }
    }
}

#[maybe_async_cfg::maybe(
    idents(AsyncReadNorFlash(async, sync = "ReadNorFlash"),),
    sync(self = "LittleFs", feature = "sync"),
    async(keep_self, feature = "async")
)]
impl<S: AsyncReadNorFlash, const BLOCK_SIZE: usize> AsyncLittleFs<S, BLOCK_SIZE> {
    async fn read_block(&mut self, ptr: BlockPointer) -> Result<Block<BLOCK_SIZE>, FsError<S::Error>> {
        let block_size = if self.block_size > 0 {
            self.block_size as usize
        } else {
            BLOCK_SIZE
        };
        let mut buf = Block::<BLOCK_SIZE>::zero(block_size);
        self.storage
            .read(ptr.as_offset(BLOCK_SIZE), buf.as_mut_slice())
            .await
            .map_err(|e| FsError::Io(e))?;
        Ok(buf)
    }
    /// Read the newer revision from ptr. ptr.a will be left as the block
    /// containing the newer revision.
    async fn read_newer_block(
        &mut self,
        ptr: &mut BlockPointerPair,
    ) -> Result<Block<BLOCK_SIZE>, FsError<S::Error>> {
        let buf1 = self.read_block(ptr.a).await?;
        let buf2 = self.read_block(ptr.b).await?;

        [(ptr.a, buf1), (ptr.b, buf2)]
            .into_iter()
            .filter_map(|(ptr, buf)| {
                let revision = buf.as_metadata::<S::Error>().ok().map(|block| block.revision_count);
                revision.map(|revision| (ptr, buf, revision))
            })
            .reduce(|(aptr, a, arev), (bptr, b, brev)| if arev > brev { (aptr, a, arev) } else { (bptr, b, brev) })
            .map(|(goodptr, buf, _)| { if goodptr == ptr.b { ptr.swap() }; buf })
            .ok_or(FsError::Corrupt)
    }
    pub async fn mount(&mut self) -> Result<(), FsError<S::Error>> {
        self.free = free::FreeBlockCache::new();
        let sb = self
            .read_newer_block(&mut BlockPointerPair { a: 0, b: 1 })
            .await?;
        let md = sb.as_metadata()?;
        let entry = md
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
                if block_size > BLOCK_SIZE as u32 {
                    return Err(FsError::Inval);
                }
                self.root_directory_ptr = root_directory_ptr;
                self.block_count = block_count;
                self.free.set_block_count(block_count);
                Ok(())
            }
            _ => Err(FsError::Corrupt),
        }
    }
    /// Find a file or directory by path. Returns the parent Dir and if the file
    /// or directory exists its DirEntryData.
    async fn dir_find<'a>(&mut self, path: &'a str) -> Result<(Dir<BLOCK_SIZE>, Option<structs::DirEntryData>), FsError<S::Error>> {
        // TODO: Support a different starting directory.
        let mut dir = Dir::<BLOCK_SIZE>::new(self.root_directory_ptr);
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
                self.dir_fetch(&mut dir).await?;
                let block = dir.block.clone().unwrap();
                continued = block.continued;
                if let Some(entry) = block.find_entry(name).map_err(|_| FsError::Corrupt)? {
                    if let None = names.clone().next() {
                        return Ok((dir, Some(entry.data)));
                    } else {
                        match entry.data {
                            structs::DirEntryData::Directory { directory_ptr } => {
                                dir.ptr = directory_ptr;
                                dir.block = None;
                                found = true;
                                break;
                            }
                            structs::DirEntryData::File { .. } => return Err(FsError::NotDir),
                            _ => return Err(FsError::Corrupt),
                        }
                    }
                }
                if continued {
                    dir.ptr = block.tail_ptr;
                    dir.block = None;
                }
            }
            if !found {
                return Ok((dir, None));
            }
        }
        // Fake directory for /
        let ptr = dir.ptr;
        Ok((dir, Some(structs::DirEntryData::Directory {
            directory_ptr: ptr,
        })))
    }
    pub async fn dir_open(&mut self, path: &str) -> Result<Dir<BLOCK_SIZE>, FsError<S::Error>> {
        let entry_data = self.dir_find(path).await?.1.ok_or(FsError::Noent)?;
        match entry_data {
            structs::DirEntryData::Directory { directory_ptr } => Ok(Dir {
                head: directory_ptr,
                ptr: directory_ptr,
                ..Default::default()
            }),
            structs::DirEntryData::File { .. } => Err(FsError::NotDir),
            _ => Err(FsError::Corrupt),
        }
    }
    pub async fn dir_read(&mut self, dir: &mut Dir<BLOCK_SIZE>) -> Result<Option<Info>, FsError<S::Error>> {
        // TODO: Cache the state in dir somehow?
        // TODO: Synthesize "." and ".." entries.
        loop {
            self.dir_fetch(dir).await?;
            let dirmeta = dir.block.as_ref().unwrap();
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
                dir.block = None;
                dir.pos = 0;
                continue;
            }
            return Ok(None);
        }
    }
    async fn dir_fetch(&mut self, dir: &mut Dir<BLOCK_SIZE>) -> Result<(), FsError<S::Error>> {
        match dir.block {
            None => {
                dir.block = Some(
                    self.read_newer_block(&mut dir.ptr)
                        .await?
                        .read(&mut 0)
                        .map_err(|_| FsError::Corrupt)?,
                );
            },
            Some(_) => (),
        };
        Ok(())
    }
    pub async fn file_open(
        &mut self,
        file: &mut File,
        path: &str,
        flags: FileOpenFlags,
    ) -> Result<(), FsError<S::Error>> {
        // TODO: deorphan once after mount
        if flags != FileOpenFlags::RDONLY {
            return Err(FsError::Inval);
        }
        let entry_data = self.dir_find(path).await?.1.ok_or(FsError::Noent)?;
        match entry_data {
            structs::DirEntryData::File { head, size } => {
                *file = File {
                    head,
                    size,
                    ..Default::default()
                };
                Ok(())
            }
            _ => Err(FsError::NotDir),
        }
    }

    pub async fn file_read(&mut self, file: &mut File, buf: &mut [u8]) -> Result<usize, FsError<S::Error>> {
        let size = min(buf.len() as u32, file.size.saturating_sub(file.pos));
        let mut buf = &mut buf[..size as usize];

        // TODO: Cache a whole block at a time for performance (requires saving
        // block and off in file).

        while buf.len() > 0 {
            let (block, off) = self.ctz_find(file.head, file.size, file.pos).await?;
            let diff = min(buf.len() as u32, self.block_size - off) as usize;
            self.storage
                .read(
                    block.as_offset(self.block_size as usize) + off,
                    &mut buf[..diff],
                )
                .await
                .map_err(|e| FsError::Io(e))?;
            buf = &mut buf[diff..];
        }
        // We only update file here so that the position is not updated on a failed read.
        file.pos = file.pos.saturating_add(size);
        Ok(size as usize)
    }

    fn ctz_index(&self, off: u32) -> (u32, u32) {
        let size = off;
        let b = self.block_size - 2 * 4;
        let i = size / b;
        if i == 0 {
            return (0, off);
        }
        let i = (size - 4 * ((i - 1).count_ones() + 2)) / b;
        (i, size - b * i - 4 * i.count_ones())
    }

    async fn ctz_find(
        &mut self,
        head: BlockPointer,
        size: u32,
        pos: u32,
    ) -> Result<(BlockPointer, u32), FsError<S::Error>> {
        let mut head = head;
        let (mut current, _) = self.ctz_index(size - 1);
        let (target, pos) = self.ctz_index(pos);

        while current > target {
            let skip = min(npw2(current - target + 1) - 1, current.trailing_zeros());
            let mut buf = [0u8; 4];
            self.storage
                .read(head.as_offset(self.block_size as usize), &mut buf[..])
                .await
                .map_err(|e| FsError::Io(e))?;
            head = buf.read_with(&mut 0, LE).map_err(|_| FsError::Corrupt)?;
            if !(head >= 2 && head <= self.block_count) {
                return Err(FsError::Corrupt);
            }
            current -= 1 << skip;
        }
        Ok((head, pos))
    }

    /// Traverse the entire filesystem, calling `f` with a pointer to every
    /// allocated block.
    async fn traverse<F>(&mut self, mut f: F) -> Result<(), FsError<S::Error>>
    where
        F: FnMut(BlockPointer) -> Result<(), FsError<S::Error>>,
    {
        if self.root_directory_ptr.is_null() {
            return Ok(());
        }
        let mut cwd = BlockPointerPair { a: 0, b: 1 };
        while !cwd.is_null() {
            f(cwd.a)?;
            f(cwd.b)?;

            let buf = self.read_newer_block(&mut cwd).await?;
            let block = buf.as_metadata()?;

            for entry in &block {
                if let structs::DirEntryData::File { head, size } =
                    entry.map_err(|_| FsError::Corrupt)?.data
                {
                    self.ctz_traverse(head, size, &mut f).await?;
                }
            }
            cwd = block.tail_ptr;
        }
        Ok(())
    }
    /// Traverse a file of length `size` beginning at `head`, calling `f` for
    /// every block that is allocated to the file.
    async fn ctz_traverse<F>(
        &mut self,
        head: BlockPointer,
        size: u32,
        mut f: F,
    ) -> Result<(), FsError<S::Error>>
    where
        F: FnMut(BlockPointer) -> Result<(), FsError<S::Error>>,
    {
        if size == 0 {
            return Ok(());
        }
        let (mut index, _) = self.ctz_index(size - 1);
        let mut head = head;
        loop {
            f(head)?;

            if index == 0 {
                return Ok(());
            }

            let mut buf = Block::<8>::zero(2 - (index & 1) as usize);
            self.storage
                .read(head.as_offset(self.block_size as usize), buf.as_mut_slice())
                .await
                .map_err(|e| FsError::Io(e))?;
            let mut offset = 0;
            while offset < buf.len() {
                let ptr: BlockPointer = buf.as_slice().read_with(&mut offset, LE).unwrap();
                f(ptr)?;
                head = ptr;
            }
            index -= buf.len() as u32;
        }
    }
}

#[maybe_async_cfg::maybe(
    idents(AsyncNorFlash(async, sync = "NorFlash"),),
    sync(self = "LittleFs", feature = "sync"),
    async(keep_self, feature = "async")
)]
impl<S: AsyncNorFlash, const BLOCK_SIZE: usize> AsyncLittleFs<S, BLOCK_SIZE> {
    /// Find the next free block and return its pointer.
    async fn alloc(&mut self) -> Result<BlockPointer, FsError<S::Error>> {
        loop {
            trace!("alloc: current free = {:?}", self.free);
            match self.free.next_free()? {
                Some(ptr) => return Ok(ptr),
                None => {
                    let mut free = self.free.advance();
                    self.traverse(|block| {
                        free.mark_used(block);
                        Ok(())
                    })
                    .await?;
                    self.free = free;
                }
            }
        }
    }
    async fn dir_alloc(&mut self) -> Result<Dir<BLOCK_SIZE>, FsError<S::Error>> {
        let ptr = BlockPointerPair {
            a: self.alloc().await?,
            b: self.alloc().await?,
        };
        let mut dir = Dir::<BLOCK_SIZE>::new(ptr);
        dir.block = Some(structs::MetadataBlock {
            // TODO: Read rev from disk if there's already a dir there?
            revision_count: 1,
            continued: false,
            tail_ptr: Default::default(),
            contents: Block::empty(),
        });
        Ok(dir)
    }
    pub async fn format(&mut self) -> Result<(), FsError<S::Error>> {
        let block_count = (self.storage.capacity() / BLOCK_SIZE) as BlockPointer;
        self.block_size = BLOCK_SIZE as u32;
        // Everything is free!
        self.free = free::FreeBlockCache::new();
        self.free.set_block_count(block_count);
        self.free = self.free.advance();

        let mut superdir = self.dir_alloc().await?;
        let mut root = self.dir_alloc().await?;
        info!("superblock and root allocated: {:?}, {:?}", superdir, root);
        self.dir_commit(&mut root).await?;
        info!("root dir committed");

        self.root_directory_ptr = root.ptr;

        let superblock_entry = structs::DirEntry {
            name: "littlefs",
            data: structs::DirEntryData::Superblock {
                root_directory_ptr: self.root_directory_ptr,
                block_size: BLOCK_SIZE as u32,
                block_count,
                version: structs::VERSION,
            },
            attributes: b"",
        };
        let entry_data =
            Block::<BLOCK_SIZE>::try_from(superblock_entry).map_err(|_| FsError::Corrupt)?;

        superdir.block.as_mut().map(|b| {
            b.contents = entry_data;
            b.tail_ptr = root.ptr;
        });
        // write twice so both copies are written
        if ![
            self.dir_commit(&mut superdir).await,
            self.dir_commit(&mut superdir).await,
        ]
        .into_iter()
        .any(|r| r.is_ok())
        {
            return Err(FsError::Corrupt);
        }
        // sanity check that fetch works
        self.read_newer_block(&mut superdir.ptr).await?;
        self.free.ack();
        Ok(())
    }
    async fn write_block(
        &mut self,
        ptr: BlockPointer,
        buf: Block<BLOCK_SIZE>,
    ) -> Result<(), FsError<S::Error>> {
        let start = ptr.as_offset(self.block_size as usize);
        self.storage
            .erase(start, start + self.block_size)
            .await
            .map_err(|e| FsError::Io(e))?;
        self.storage
            .write(start, buf.as_slice())
            .await
            .map_err(|e| FsError::Io(e))?;
        let mut readback: Block<BLOCK_SIZE> = Block::zero(buf.len());
        self.storage
            .read(start, readback.as_mut_slice())
            .await
            .map_err(|e| FsError::Io(e))?;
        if buf != readback {
            Err(FsError::Corrupt)
        } else {
            Ok(())
        }
    }
    async fn dir_commit(&mut self, dir: &mut Dir<BLOCK_SIZE>) -> Result<(), FsError<S::Error>> {
        // keep pairs in order such that pair[0] is most recent
        trace!("before swap: {:?}", dir.ptr);
        dir.ptr.swap();
        dir.block = dir.block.take().map(|mut b| {
            b.revision_count += 1;
            b
        });
        let oldpair = dir.ptr;
        let mut relocated = false;
        loop {
            match dir.block.as_ref() {
                None => panic!("asked to commit block at {:?} without data", dir.ptr),
                Some(block) => {
                    let buf = Block::<BLOCK_SIZE>::try_from(block).map_err(|_| FsError::Inval)?;
                    match self.write_block(dir.ptr.a, buf).await {
                        Ok(()) => break,
                        Err(FsError::Corrupt) => {
                            // If erase or write fails, try to relocate
                            info!("bad block at {:?}", dir.ptr.a);
                            relocated = true;

                            if dir.ptr == BlockPointerPair::new(0, 1) {
                                warn!("Superblock {:?} has become unwritable", dir.ptr.a);
                                return Err(FsError::Corrupt);
                            }

                            dir.ptr.a = self.alloc().await?;
                        }
                        Err(e) => return Err(e),
                    }
                }
            }
        }
        if relocated {
            info!("relocating {:?} to {:?}", oldpair, dir.ptr);
            todo!();
        }
        Ok(())
    }
    async fn dir_append(&mut self, dir: &mut Dir<BLOCK_SIZE>, entry: structs::DirEntry<'_>) -> Result<(), FsError<S::Error>> {
        let entrybuf = Block::<BLOCK_SIZE>::try_from(entry).map_err(|_| FsError::Inval)?;
        let size = entrybuf.len();
        loop {
            self.dir_fetch(dir).await?;
            let mut block = dir.block.clone().unwrap();
            let oldlen = Block::<BLOCK_SIZE>::try_from(&block).map(|b| b.len()).unwrap_or(BLOCK_SIZE);
            trace!("Block {:?} oldlen: {} trying to add {}", dir.ptr, oldlen, size);
            if oldlen + size < self.block_size as usize {
                block.contents.try_extend_from_slice(entrybuf.as_slice()).unwrap();
                dir.block = Some(block);
                trace!("Trying to commit {:?}", dir);
                self.dir_commit(dir).await?;
                trace!("committed");
                return Ok(());
            }
            if !block.continued {
                // Need to allocate a new dir block
                let mut olddir = dir.clone();
                *dir = self.dir_alloc().await?;
                dir.block.as_mut().map(|b| {
                    b.tail_ptr = block.tail_ptr;
                    b.contents = entrybuf;
                });
                self.dir_commit(dir).await?;
                olddir.block.as_mut().map(|b| {
                    b.continued = true;
                    b.tail_ptr = dir.ptr;
                });
                self.dir_commit(&mut olddir).await?;
                return Ok(());
            }
            dir.ptr = block.tail_ptr;
            dir.block = None;
            dir.pos = 0;
        }
    }
    pub async fn mkdir(&mut self, path: &str) -> Result<(), FsError<S::Error>> {
        // TODO: deorphan
        let (mut cwd, entrydata) = self.dir_find(path).await?;
        if entrydata != None {
            return Err(FsError::Exist);
        }

        self.free.ack();
        let mut dir = self.dir_alloc().await?;
        let tail_ptr = cwd.block.as_ref().unwrap().tail_ptr;
        dir.block.as_mut().map(|b| {
            b.tail_ptr = tail_ptr;
        });
        self.dir_commit(&mut dir).await?;

        trace!("New directory committed at {:?}", dir.ptr);

        let entry = structs::DirEntry{
            name: path.rsplit_once('/').map(|p| p.1).unwrap_or(path),
            attributes: &[],
            data: structs::DirEntryData::Directory { directory_ptr: dir.ptr },
        };
        cwd.block.as_mut().map(|b| {
            b.tail_ptr = dir.ptr;
        });
        self.dir_append(&mut cwd, entry).await?;
        self.free.ack();
        Ok(())
    }
}

#[cfg(all(test, feature = "log"))]
mod test_log {
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
}

#[cfg(test)]
#[maybe_async_cfg::maybe(
    idents(
        AsyncLittleFs(async, sync = "LittleFs"),
        do_test(async = "block_on", sync = "block_on_sync"),
        AsyncReadNorFlash(async, sync = "ReadNorFlash"),
    ),
    sync(self = "tests_sync", feature = "sync"),
    async(keep_self, feature = "async")
)]
mod tests_async {
    use super::storage::slice::SliceStorage;
    use super::*;

    extern crate std;
    use alloc::format;
    use std::println;
    use std::collections::HashSet;
    extern crate alloc;
    use alloc::string::String;
    use alloc::vec::Vec;

    use itertools::repeat_n;
    use tokio_test::block_on;

    fn block_on_sync(_: ()) {}

    use byte::{BytesExt, Result};

    const THREE_FILES: &[u8] = include_bytes!("../testdata/three-files.img");
    const LARGE_DIR: &[u8] = include_bytes!("../testdata/large-dir.img");

    async fn dump_blocks<T: AsyncReadNorFlash>(fs: &mut AsyncLittleFs<T, 512>) {
        let mut iter_count = 0;
        let mut iter_blocks = HashSet::new();
        for i in 0..fs.block_count {
            let buf = fs.read_block(i as BlockPointer).await.unwrap();
            let block = buf.as_metadata::<T::Error>();
            match block {
                Ok(b) => {
                    iter_count += 1;
                    iter_blocks.insert(i);
                    info!("Block {}: {:?}", i, b);
                },
                Err(FsError::Corrupt) => (),
                Err(e) => panic!("as_metadata: {:?}", e),
            }
        }
        let mut traverse_count = 0;
        fs.traverse(|ptr| {
            trace!("found {:?}", ptr);
            traverse_count += 1;
            if !iter_blocks.contains(&ptr) {
                warn!("missing block {:?}", ptr);
            }
            Ok(())
        })
            .await
            .unwrap();
        //assert_eq!(iter_count, traverse_count);
    }

    #[test]
    fn three_files() {
        do_test(async {
            let mut fs: AsyncLittleFs<_, 512> = AsyncLittleFs::new(SliceStorage::new(THREE_FILES));
            fs.mount().await.unwrap();

            let mut dir = fs.dir_open("/").await.unwrap();
            println!("open dir: {:?}", dir);
            let mut names = Vec::<String>::new();
            while let Some(info) = fs.dir_read(&mut dir).await.unwrap() {
                println!("entry: {:?}", info);
                if info.entry_type == EntryType::RegularFile {
                    names.push(info.name.as_ref().into());
                    assert_eq!(info.size, 512);
                }
            }
            let expected_data: Vec<u8> = b"0123456789abcdef"
                .into_iter()
                .cycle()
                .take(512)
                .cloned()
                .collect();
            assert_eq!(names, &["static0", "static1", "static2"]);
            for path in &["static0", "static1", "static2", "subdir/file"] {
                let mut file = Default::default();
                fs.file_open(&mut file, path, FileOpenFlags::RDONLY)
                    .await
                    .unwrap();
                println!("{} file: {:?}", path, file);
                let mut data = [0u8; 1024];
                assert_eq!(fs.file_read(&mut file, &mut data).await.unwrap(), 512);
                println!("{} data: {:?}", path, &data[..512]);
                assert_eq!(&data[..512], &expected_data);
            }
        });
    }

    #[test]
    fn three_files_corrupt() {
        do_test(async {
            let data: Vec<u8> = repeat_n(0u8, 512)
                .chain(THREE_FILES[512..].iter().cloned())
                .collect();
            let mut fs: AsyncLittleFs<_, 512> = AsyncLittleFs::new(SliceStorage::new(&data[..]));
            fs.mount().await.unwrap();
        });
    }

    #[test]
    fn large_dir() {
        do_test(async {
            let mut fs: AsyncLittleFs<_, 512> = AsyncLittleFs::new(SliceStorage::new(LARGE_DIR));
            fs.mount().await.unwrap();

            let mut dir = fs.dir_open("/").await.unwrap();
            println!("open dir: {:?}", dir);
            let mut names = Vec::<String>::new();
            while let Some(info) = fs.dir_read(&mut dir).await.unwrap() {
                println!("entry: {:?}", info);
                if info.entry_type == EntryType::RegularFile {
                    names.push(info.name.as_ref().into());
                    assert_eq!(info.size, if info.name.as_ref() == "zzz" { 512 } else { 0 });
                }
            }
            let expected_names: Vec<String> = (0..=99)
                .into_iter()
                .map(|n| format!("static{}", n))
                .chain([String::from("zzz")])
                .collect();
            assert_eq!(names, expected_names);
            let expected_data: Vec<u8> = b"0123456789abcdef"
                .into_iter()
                .cycle()
                .take(512)
                .cloned()
                .collect();
            let mut file = Default::default();
            let path = "zzz";
            fs.file_open(&mut file, path, FileOpenFlags::RDONLY)
                .await
                .unwrap();
            println!("{} file: {:?}", path, file);
            let mut data = [0u8; 1024];
            assert_eq!(fs.file_read(&mut file, &mut data).await.unwrap(), 512);
            println!("{} data: {:?}", path, &data[..512]);
            assert_eq!(&data[..512], &expected_data);
            let path = "static0";
            fs.file_open(&mut file, path, FileOpenFlags::RDONLY)
                .await
                .unwrap();
            assert_eq!(fs.file_read(&mut file, &mut data).await.unwrap(), 0);
        });
    }

    #[test]
    fn traverse() {
        do_test(async {
            let mut fs: AsyncLittleFs<_, 512> = AsyncLittleFs::new(SliceStorage::new(THREE_FILES));
            fs.mount().await.unwrap();
            let mut count = 0;
            fs.traverse(|ptr| {
                trace!("found {:?}", ptr);
                count += 1;
                Ok(())
            })
            .await
            .unwrap();
            assert_eq!(count, 10);
        });
    }

    #[test]
    fn alloc() {
        do_test(async {
            let mut buf: Vec<u8> = Vec::new();
            buf.extend_from_slice(&THREE_FILES);
            let mut fs: AsyncLittleFs<_, 512> =
                AsyncLittleFs::new(SliceStorage::new(buf.as_mut_slice()));
            fs.mount().await.unwrap();
            assert_eq!(fs.alloc().await, Ok(10));
        });
    }

    #[test]
    fn owned_block() {
        let mut length = 0;
        let block: crate::structs::MetadataBlock<Block<512>> =
            crate::structs::tests::DIRECTORY.read(&mut length).unwrap();
        assert_eq!(block.revision_count, 10);
        assert_eq!(block.contents.len(), 154 - 20);
        let entry = block.find_entry("tea").unwrap().unwrap();
        assert_eq!(entry.name, "tea");
        trace!("found tea: {:?}", entry);
    }

    #[test]
    fn format() {
        do_test(async {
        let mut buf = [0u8; 8192];
        let mut fs: AsyncLittleFs<_, 512> =
            AsyncLittleFs::new(SliceStorage::new(buf.as_mut_slice()));
        fs.format().await.unwrap();
        let mount_err = fs.mount().await;
        info!("formatted fs: {:?}", buf);
        mount_err.unwrap();
        });
    }

    #[test]
    fn mkdir() {
        do_test(async {
            let mut buf = [0u8; 8192];
            let mut fs: AsyncLittleFs<_, 512> =
                AsyncLittleFs::new(SliceStorage::new(buf.as_mut_slice()));
            fs.format().await.unwrap();
            fs.mount().await.unwrap();
            for i in 0..6 {
                let name = format!("dir{}", i);
                fs.mkdir(&name).await.unwrap();
            }
            assert_eq!(fs.mkdir("dir5").await, Err(FsError::Exist));
            assert_eq!(fs.mkdir("dir6").await, Err(FsError::Nospc));

            let mut dir = fs.dir_open("/").await.unwrap();
            let mut names = Vec::<String>::new();
            while let Some(info) = fs.dir_read(&mut dir).await.unwrap() {
                if info.entry_type == EntryType::Directory {
                    names.push(info.name.as_ref().into());
                }
            }
            assert_eq!(names.as_slice(), &["dir0", "dir1", "dir2", "dir3", "dir4", "dir5"]);

            dump_blocks(&mut fs).await;
        });
    }

    #[test]
    fn mkdir_continued() {
        do_test(async {
            let mut buf = [0u8; 8192];
            let mut fs: AsyncLittleFs<_, 512> =
                AsyncLittleFs::new(SliceStorage::new(buf.as_mut_slice()));
            fs.format().await.unwrap();
            fs.mount().await.unwrap();
            let mut want_names = Vec::<String>::new();
            for i in 0..5 {
                let name = format!("{}{}", "x".repeat(128), i);
                want_names.push(name.clone());
                fs.mkdir(&name).await.unwrap();
            }
            assert_eq!(fs.mkdir("dir").await, Err(FsError::Nospc));

            let mut dir = fs.dir_open("/").await.unwrap();
            let mut names = Vec::<String>::new();
            while let Some(info) = fs.dir_read(&mut dir).await.unwrap() {
                if info.entry_type == EntryType::Directory {
                    names.push(info.name.as_ref().into());
                }
            }
            assert_eq!(names.as_slice(), want_names.as_slice());

            dump_blocks(&mut fs).await;
        });
    }
}
