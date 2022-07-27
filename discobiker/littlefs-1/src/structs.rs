use byte::ctx::*;
use byte::{BytesExt, Error, Result as ByteResult, TryRead, TryWrite, LE};

use crc::{Crc, CRC_32_JAMCRC};

use enum_kinds::EnumKind;
use num_enum::{IntoPrimitive, TryFromPrimitive};

pub const VERSION: u32 = 0x00010001;

const CRC32: Crc<u32> = Crc::<u32>::new(&CRC_32_JAMCRC);

pub type BlockPointer = u32;

pub trait AsOffset {
    fn as_offset(self, block_size: usize) -> u32;
}

impl AsOffset for BlockPointer {
    fn as_offset(self, block_size: usize) -> u32 {
        self * block_size as u32
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BlockPointerPair {
    pub a: BlockPointer,
    pub b: BlockPointer,
}

impl BlockPointerPair {
    pub fn is_null(&self) -> bool {
        self.a == 0xffffffff && self.b == 0xffffffff
    }
}

impl Default for BlockPointerPair {
    fn default() -> Self {
        BlockPointerPair { a: 0xffffffff, b: 0xffffffff }
    }
}

impl TryRead<'_, Endian> for BlockPointerPair {
    fn try_read(bytes: &[u8], endian: Endian) -> ByteResult<(Self, usize)> {
        let offset = &mut 0;

        let ptr = BlockPointerPair {
            a: bytes.read_with(offset, endian)?,
            b: bytes.read_with(offset, endian)?,
        };
        Ok((ptr, *offset))
    }
}
impl TryWrite<Endian> for BlockPointerPair {
    fn try_write(self, bytes: &mut [u8], endian: Endian) -> ByteResult<usize> {
        let offset = &mut 0;
        bytes.write_with(offset, self.a, endian)?;
        bytes.write_with(offset, self.b, endian)?;
        Ok(*offset)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MetadataBlock<T>
{
    pub revision_count: u32,
    pub continued: bool,
    pub tail_ptr: BlockPointerPair,
    pub contents: T,
}

impl<'a, T> TryRead<'a, ()> for MetadataBlock<T>
    where T: TryRead<'a, Bytes> + AsRef<[u8]> + 'a
{
    fn try_read(bytes: &'a [u8], _ctx: ()) -> ByteResult<(Self, usize)> {
        let offset = &mut 0;
        let endian = LE;

        let revision_count = bytes.read_with(offset, endian)?;
        let dir_size = bytes.read_with::<u32>(offset, endian)?;
        let continued = dir_size & 0x80000000 > 0;
        let dir_size = dir_size & 0x7FFFFFFF;
        let tail_ptr = bytes.read_with(offset, endian)?;
        let contents = bytes.read_with::<T>(
            offset,
            Bytes::Len((dir_size as usize).saturating_sub(*offset + 4)),
        )?;
        let calc_crc = CRC32.checksum(&bytes[..*offset]);
        let crc = bytes.read_with(offset, endian)?;
        if calc_crc != crc {
            return Err(Error::BadInput {
                err: "crc32 incorrect",
            });
        }
        Ok((
            MetadataBlock {
                revision_count: revision_count,
                continued: continued,
                tail_ptr: tail_ptr,
                contents: contents,
            },
            *offset,
        ))
    }
}

impl<T> TryWrite<()> for MetadataBlock<T>
where T: TryWrite
{
    fn try_write(self, bytes: &mut [u8], _ctx: ()) -> ByteResult<usize> {
        let offset = &mut 0;
        let endian = LE;

        bytes.write_with(offset, self.revision_count, endian)?;
        let dir_size_offset = &mut offset.clone();
        // Fill in dir size later
        bytes.write_with::<u32>(offset, 0, endian)?;
        bytes.write_with(offset, self.tail_ptr, endian)?;
        bytes.write(offset, self.contents)?;
        let crc_offset = &mut offset.clone();
        bytes.write_with::<u32>(offset, 0, endian)?;

        // Fill in dir size now that we know it
        let dir_size = *offset as u32 | if self.continued { 0x80000000 } else { 0 };
        bytes.write_with(dir_size_offset, dir_size, endian)?;

        // Calculate CRC
        let calc_crc = CRC32.checksum(&bytes[..*crc_offset]);
        bytes.write(crc_offset, calc_crc)?;
        Ok(*offset)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, EnumKind)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[enum_kind(
    DirEntryType,
    repr(u8),
    derive(IntoPrimitive, TryFromPrimitive),
    cfg_attr(feature = "defmt", derive(defmt::Format))
)]
pub enum DirEntryData {
    #[enum_kind_value(0x11)]
    File { head: BlockPointer, size: u32 },
    #[enum_kind_value(0x22)]
    Directory { directory_ptr: BlockPointerPair },
    #[enum_kind_value(0x2E)]
    Superblock {
        root_directory_ptr: BlockPointerPair,
        block_size: u32,
        block_count: u32,
        version: u32,
    },
}

impl TryRead<'_, ()> for DirEntryType {
    fn try_read(bytes: &[u8], _ctx: ()) -> ByteResult<(Self, usize)> {
        let offset = &mut 0;
        let v: u8 = bytes.read(offset)?;
        Ok((
            Self::try_from(v).map_err(|_| Error::BadInput {
                err: "invalid entry type",
            })?,
            *offset,
        ))
    }
}
impl TryWrite<Endian> for DirEntryType {
    fn try_write(self, bytes: &mut [u8], endian: Endian) -> ByteResult<usize> {
        let offset = &mut 0;
        bytes.write_with(offset, u8::from(self), endian)?;
        Ok(*offset)
    }
}

impl TryRead<'_, DirEntryType> for DirEntryData {
    fn try_read(bytes: &[u8], ty: DirEntryType) -> ByteResult<(Self, usize)> {
        let offset = &mut 0;
        let endian = LE;
        match ty {
            DirEntryType::File => {
                let f = DirEntryData::File {
                    head: bytes.read_with(offset, endian)?,
                    size: bytes.read_with(offset, endian)?,
                };
                Ok((f, *offset))
            }
            DirEntryType::Directory => {
                let d = DirEntryData::Directory {
                    directory_ptr: bytes.read_with(offset, endian)?,
                };
                Ok((d, *offset))
            }
            DirEntryType::Superblock => {
                let s = DirEntryData::Superblock {
                    root_directory_ptr: bytes.read_with(offset, endian)?,
                    block_size: bytes.read_with(offset, endian)?,
                    block_count: bytes.read_with(offset, endian)?,
                    version: bytes.read_with(offset, endian)?,
                };
                match s {
                    DirEntryData::Superblock {
                        version: VERSION, ..
                    } => Ok((s, *offset)),
                    _ => Err(Error::BadInput {
                        err: "unexpected superblock version",
                    }),
                }
            }
        }
    }
}
impl TryWrite<()> for DirEntryData {
    fn try_write(self, bytes: &mut [u8], _ctx: ()) -> ByteResult<usize> {
        let offset = &mut 0;
        let endian = LE;
        match self {
            DirEntryData::File { head, size } => {
                bytes.write_with(offset, head, endian)?;
                bytes.write_with(offset, size, endian)?;
            },
            DirEntryData::Directory { directory_ptr } => {
                bytes.write_with(offset, directory_ptr, endian)?;
            },
            DirEntryData::Superblock { root_directory_ptr, block_size, block_count, version } => {
                bytes.write_with(offset, root_directory_ptr, endian)?;
                bytes.write_with(offset, block_size, endian)?;
                bytes.write_with(offset, block_count, endian)?;
                bytes.write_with(offset, version, endian)?;
            }
        }
        Ok(*offset)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DirEntry<'a> {
    pub name: &'a str,
    pub data: DirEntryData,
    pub attributes: &'a [u8],
}
impl<'a> TryRead<'a, ()> for DirEntry<'a> {
    fn try_read(bytes: &'a [u8], _ctx: ()) -> ByteResult<(Self, usize)> {
        let offset = &mut 0;
        let endian = LE;

        let entry_type: DirEntryType = bytes.read(offset)?;
        let entry_length: u8 = bytes.read_with(offset, endian)?;
        let attribute_length: u8 = bytes.read_with(offset, endian)?;
        let name_length: u8 = bytes.read_with(offset, endian)?;
        let data_offset = *offset;
        let data = bytes.read_with(offset, entry_type)?;
        if *offset != data_offset + (entry_length as usize) {
            return Err(Error::BadInput {
                err: "invalid entry length",
            });
        }
        let attributes = bytes.read_with(offset, Bytes::Len(attribute_length as usize))?;
        let name = bytes.read_with(offset, Str::Len(name_length as usize))?;
        Ok((
            DirEntry {
                data,
                attributes,
                name,
            },
            *offset,
        ))
    }
}
impl<'a> TryWrite<()> for DirEntry<'a> {
    fn try_write(self, bytes: &mut [u8], _ctx: ()) -> ByteResult<usize> {
        let offset = &mut 0;
        let endian = LE;
        bytes.write_with(offset, DirEntryType::from(self.data), endian)?;
        let mut length_offset = *offset;
        bytes.write_with(offset, 0u8, endian)?;
        bytes.write_with(offset, self.attributes.len() as u8, endian)?;
        bytes.write_with(offset, self.name.len() as u8, endian)?;
        let data_offset = *offset;
        bytes.write(offset, self.data)?;
        bytes.write_with(&mut length_offset, (*offset-data_offset) as u8, endian)?;
        bytes.write(offset, self.attributes)?;
        bytes.write(offset, self.name)?;

        Ok(*offset)
    }
}

pub struct DirEntryIterator<'a> {
    offset: usize,
    contents: &'a [u8],
}

impl<'a> Iterator for DirEntryIterator<'a> {
    type Item = ByteResult<DirEntry<'a>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.offset >= self.contents.len() {
            None
        } else {
            let entry: ByteResult<DirEntry> = self.contents.read(&mut self.offset);
            Some(entry)
        }
    }
}
impl<'a, T> IntoIterator for &'a MetadataBlock<T>
where T: AsRef<[u8]> + 'a
{
    type Item = ByteResult<DirEntry<'a>>;
    type IntoIter = DirEntryIterator<'a>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl<'a, T> MetadataBlock<T>
where T: AsRef<[u8]>
{
    pub fn iter(&'a self) -> DirEntryIterator<'a> {
        DirEntryIterator {
            offset: 0,
            contents: self.contents.as_ref(),
        }
    }
    pub fn find_entry(&'a self, name: &str) -> ByteResult<Option<DirEntry<'a>>> {
        self.iter()
            .find_map(|entry| {
                match entry {
                    Err(e) => Some(Err(e)),
                    Ok(entry) => {
                        // TODO: "check that entry has not been moved"
                        if entry.name == name {
                            Some(Ok(entry))
                        } else {
                            None
                        }
                    }
                }
            })
            .transpose()
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;
    use super::*;
    use alloc::vec::Vec;
    use byte::BytesExt;

    const SUPERBLOCK: &[u8] = &[
        0x03, 0x00, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
        0x00, 0x2e, 0x14, 0x00, 0x08, 0x03, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02,
        0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x6c, 0x69, 0x74, 0x74, 0x6c,
        0x65, 0x66, 0x73, 0x68, 0x96, 0x3b,
        0xc8,
        // TODO: CRC from spec seems wrong? 0xfa, 0x74, 0x0b, 0xc5,
    ];

    #[test]
    fn superblock() {
        let mut length = 0;
        let block: MetadataBlock<&[u8]> = SUPERBLOCK.read(&mut length).unwrap();
        assert_eq!(length, SUPERBLOCK.len());
        assert_eq!(block.revision_count, 3);
        assert_eq!(block.continued, false);
        assert_eq!(block.contents.len(), 52-20);
        assert_eq!(block.tail_ptr, BlockPointerPair { a: 3, b: 2 });
        let iter = block.into_iter();
        let dir_entries: Result<Vec<_>, _> = iter.collect();
        let dir_entries = dir_entries.unwrap();
        assert_eq!(dir_entries.len(), 1);
        let entry = dir_entries[0];
        match entry.data {
            DirEntryData::Superblock {
                root_directory_ptr,
                block_size,
                block_count,
                version,
            } => {
                assert_eq!(root_directory_ptr, BlockPointerPair { a: 3, b: 2 });
                assert_eq!(block_size, 512);
                assert_eq!(block_count, 1024);
                assert_eq!(version, VERSION);
            }
            _ => {
                assert_eq!(DirEntryType::from(&entry.data), DirEntryType::Superblock);
            }
        };
        assert_eq!(entry.attributes.len(), 0);
        assert_eq!(entry.name, "littlefs");
        let mut out: [u8; 512] = [0; 512];
        let mut write_length = 0;
        out.write(&mut write_length, entry).unwrap();
        assert_eq!(block.contents.len(), write_length);
        assert_eq!(block.contents, &out[..write_length]);
    }

    const DIRECTORY: &[u8] = &[
        0x0a, 0x00, 0x00, 0x00, 0x9a, 0x00, 0x00, 0x00, 0x25, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00,
        0x00, 0x22, 0x08, 0x00, 0x03, 0x05, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x74, 0x65,
        0x61, 0x22, 0x08, 0x00, 0x06, 0x07, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x63, 0x6f,
        0x66, 0x66, 0x65, 0x65, 0x22, 0x08, 0x00, 0x04, 0x09, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00,
        0x00, 0x73, 0x6f, 0x64, 0x61, 0x22, 0x08, 0x00, 0x05, 0x1d, 0x00, 0x00, 0x00, 0x1c, 0x00,
        0x00, 0x00, 0x6d, 0x69, 0x6c, 0x6b, 0x31, 0x22, 0x08, 0x00, 0x05, 0x1f, 0x00, 0x00, 0x00,
        0x1e, 0x00, 0x00, 0x00, 0x6d, 0x69, 0x6c, 0x6b, 0x32, 0x22, 0x08, 0x00, 0x05, 0x21, 0x00,
        0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x6d, 0x69, 0x6c, 0x6b, 0x33, 0x22, 0x08, 0x00, 0x05,
        0x23, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x6d, 0x69, 0x6c, 0x6b, 0x34, 0x22, 0x08,
        0x00, 0x05, 0x25, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x6d, 0x69, 0x6c, 0x6b, 0x35,
        0x06, 0x31, 0x6e, 0xc8,
    ];

    #[test]
    fn superblock_rewrite() {
        let mut read_length = 0;
        let block: MetadataBlock<&[u8]> = SUPERBLOCK.read(&mut read_length).unwrap();
        let mut out: [u8; 512] = [0; 512];
        let mut write_length = 0;
        out.write(&mut write_length, block).unwrap();
        assert_eq!(read_length, write_length);
        assert_eq!(SUPERBLOCK, &out[..write_length]);
    }

    #[test]
    fn directory() {
        let mut length = 0;
        let block: MetadataBlock<&[u8]> = DIRECTORY.read(&mut length).unwrap();
        assert_eq!(length, DIRECTORY.len());
        assert_eq!(block.revision_count, 10);
        assert_eq!(block.contents.len(), 154-20);
        assert_eq!(block.tail_ptr, BlockPointerPair { a: 37, b: 36 });
        let iter = block.into_iter();
        let dir_entries: Result<Vec<_>, _> = iter.collect();
        let dir_entries = dir_entries.unwrap();
        assert_eq!(dir_entries.len(), 8);
        assert_eq!(
            dir_entries
                .iter()
                .map(|entry| entry.name)
                .collect::<Vec<&str>>(),
            ["tea", "coffee", "soda", "milk1", "milk2", "milk3", "milk4", "milk5"]
        );
    }
}
