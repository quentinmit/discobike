#![no_std]

extern crate alloc;
use alloc::vec::Vec;
use binrw::io::*;
use binrw::*;
use core::str;

#[derive(Copy, Clone, Debug, PartialEq)]
#[binrw]
#[brw(little, repr = u8)]
#[repr(u8)]
enum DirEntryType {
    File = 0x11,
    Directory = 0x22,
    Superblock = 0x2E,
}

#[binrw]
#[br(little, import(ty: DirEntryType))]
#[bw(little)]
#[derive(Debug)]
enum DirEntryData {
    #[br(pre_assert(ty == DirEntryType::File))]
    File { file_head: u32, file_size: u32 },
    #[br(pre_assert(ty == DirEntryType::Directory))]
    Directory { directory_ptr: [u32; 2] },
    #[br(pre_assert(ty == DirEntryType::Superblock))]
    Superblock {
        root_directory: [u32; 2],
        block_size: u32,
        block_count: u32,
        version: u32,
    },
}

impl From<&DirEntryData> for DirEntryType {
    fn from(data: &DirEntryData) -> Self {
        match data {
            DirEntryData::File { .. } => DirEntryType::File,
            DirEntryData::Directory { .. } => DirEntryType::Directory,
            DirEntryData::Superblock { .. } => DirEntryType::Superblock,
        }
    }
}

#[binrw]
#[brw(little)]
#[derive(Debug)]
struct DirEntry {
    #[br(temp)]
    #[bw(calc(entry_data.into()))]
    entry_type: DirEntryType,

    #[br(temp)]
    #[bw(calc(match entry_type { DirEntryType::File => 8, DirEntryType::Directory => 8, DirEntryType::Superblock => 20 }))]
    entry_length: u8,

    #[br(temp)]
    #[bw(calc(attributes.len().try_into().unwrap()))]
    attribute_length: u8,
    #[br(temp)]
    #[bw(calc(name.len().try_into().unwrap()))]
    name_length: u8,

    #[br(args(entry_type))]
    entry_data: DirEntryData,

    #[br(args { count: attribute_length as usize } )]
    attributes: Vec<u8>,

    #[br(args { count: name_length as usize } )]
    name: Vec<u8>,
}

impl DirEntry {
    pub fn entry_type(&self) -> DirEntryType {
        (&self.entry_data).into()
    }

    pub fn name_str(&self) -> &str {
        str::from_utf8(&self.name).unwrap()
    }
}

pub fn byte_size<Reader, T, Arg, Ret>(
    count: usize,
) -> impl Fn(&mut Reader, &ReadOptions, Arg) -> BinResult<Ret>
where
    T: BinRead<Args = Arg>,
    Reader: Read + Seek,
    Arg: Clone,
    Ret: core::iter::FromIterator<T>,
{
    let read = |reader: &mut Reader, ro: &ReadOptions, args: Arg| {
        let mut value = T::read_options(reader, ro, args.clone())?;
        value.after_parse(reader, ro, args)?;
        let pos = reader.stream_position().unwrap_or(0);
        Ok((value, pos))
    };
    move |reader, ro, args| {
        let start = reader.stream_position()?;
        let mut last_cond = true;
        let mut last_error = false;
        core::iter::repeat_with(|| read(reader, ro, args.clone()))
            .take_while(|result| {
                let cont = last_cond && !last_error; //keep the first error we get
                if let Ok((_, pos)) = result {
                    last_cond = (*pos as usize) < (start as usize) + count;
                } else {
                    last_error = true;
                }
                cont
            })
            .map(|result| result.map(|(val, _pos)| val))
            .collect()
    }
}

#[binrw]
#[brw(little)]
struct MetadataBlock {
    revision_count: u32,
    // High bit means continued on next block
    #[br(restore_position, map = |x: u32| x & 0x80000000 > 0)]
    #[bw(ignore)]
    continued: bool,
    #[br(map = |x: u32| x & 0x7FFFFFFF)]
    #[bw(map = |x| if *continued { *x | 0x80000000 } else { *x })]
    dir_size: u32,
    tail_pointer: [u32; 2],
    #[br(parse_with = byte_size((dir_size-5*4) as usize))]
    dir_entries: Vec<DirEntry>,
    crc: u32,
}

mod bytes {
    use byte::ctx::*;
    use byte::*;

    use enum_kinds::EnumKind;
    use num_enum::{IntoPrimitive, TryFromPrimitive};

    #[derive(Debug, PartialEq)]
    struct MetadataBlock<'a> {
        revision_count: u32,
        continued: bool,
        dir_size: u32,
        tail_pointer: [u32; 2],
        contents: &'a [u8],
        crc: u32,
    }

    impl<'a> TryRead<'a, ()> for MetadataBlock<'a> {
        fn try_read(bytes: &'a [u8], _ctx: ()) -> Result<(Self, usize)> {
            let offset = &mut 0;
            let endian = LE;

            let revision_count = bytes.read_with(offset, endian)?;
            let dir_size = bytes.read_with::<u32>(offset, endian)?;
            let continued = dir_size & 0x80000000 > 0;
            let dir_size = dir_size & 0x7FFFFFFF;
            let tail_pointer = [
                bytes.read_with::<u32>(offset, endian)?,
                bytes.read_with::<u32>(offset, endian)?,
            ];
            let contents = bytes.read_with::<&[u8]>(offset, Bytes::Len(dir_size as usize - 16))?;
            let crc = bytes.read_with(offset, endian)?;
            // TODO: Check crc
            Ok((
                MetadataBlock {
                    revision_count: revision_count,
                    dir_size: dir_size,
                    continued: continued,
                    tail_pointer: tail_pointer,
                    contents: contents,
                    crc: crc,
                },
                *offset,
            ))
        }
    }

    impl<'a> TryWrite<()> for MetadataBlock<'a> {
        fn try_write(self, bytes: &mut [u8], _ctx: ()) -> Result<usize> {
            let offset = &mut 0;
            let endian = LE;

            bytes.write_with(offset, self.revision_count, endian)?;
            let dir_size =
                (self.contents.len() + 16) as u32 | if self.continued { 0x80000000 } else { 0 };
            bytes.write_with(offset, dir_size, endian)?;
            bytes.write_with(offset, self.tail_pointer[0], endian)?;
            bytes.write_with(offset, self.tail_pointer[1], endian)?;
            bytes.write(offset, self.contents)?;
            // TODO: Calculate crc
            bytes.write(offset, self.crc)?;
            Ok(*offset)
        }
    }

    #[derive(Debug, EnumKind)]
    #[enum_kind(DirEntryType, repr(u8), derive(IntoPrimitive, TryFromPrimitive))]
    enum DirEntryData {
        #[enum_kind_value(0x11)]
        File { file_head: u32, file_size: u32 },
        #[enum_kind_value(0x22)]
        Directory { directory_ptr: [u32; 2] },
        #[enum_kind_value(0x2E)]
        Superblock {
            root_directory: [u32; 2],
            block_size: u32,
            block_count: u32,
            version: u32,
        },
    }

    impl TryRead<'_, ()> for DirEntryType {
        fn try_read(bytes: &[u8], _ctx: ()) -> Result<(Self, usize)> {
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

    #[derive(Debug, PartialEq)]
    struct DirEntry<'a> {
        entry_data: &'a [u8],

        attributes: &'a [u8],

        name: &'a str,
    }
    impl<'a> TryRead<'a, ()> for DirEntry<'a> {
        fn try_read(bytes: &'a [u8], _ctx: ()) -> Result<(Self, usize)> {
            let offset = &mut 0;
            let endian = LE;

            let entry_type: DirEntryType = bytes.read(offset)?;
            let entry_length: u8 = bytes.read_with(offset, endian)?;
            let attribute_length: u8 = bytes.read_with(offset, endian)?;
            let name_length: u8 = bytes.read_with(offset, endian)?;
            let entry_data = bytes.read_with(offset, Bytes::Len(entry_length as usize))?;
            let attributes = bytes.read_with(offset, Bytes::Len(attribute_length as usize))?;
            let name = bytes.read_with(offset, Str::Len(name_length as usize))?;
            Ok((
                DirEntry {
                    entry_data,
                    attributes,
                    name,
                },
                *offset,
            ))
        }
    }

    struct DirEntryIterator<'a> {
        offset: usize,
        contents: &'a [u8],
    }

    impl<'a> Iterator for DirEntryIterator<'a> {
        type Item = Result<DirEntry<'a>>;

        fn next(&mut self) -> Option<Self::Item> {
            if self.offset >= self.contents.len() {
                None
            } else {
                let entry: Result<DirEntry> = self.contents.read(&mut self.offset);
                Some(entry)
            }
        }
    }
    impl<'a> IntoIterator for MetadataBlock<'a> {
        type Item = Result<DirEntry<'a>>;
        type IntoIter = DirEntryIterator<'a>;

        fn into_iter(self) -> Self::IntoIter {
            DirEntryIterator {
                offset: 0,
                contents: self.contents,
            }
        }
    }

    #[cfg(test)]
    mod test {
        use alloc::vec::Vec;
        use byte::{BytesExt, LE};

        fn superblock() {
            let bytes: &[u8] = &[
                0x03, 0x00, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02, 0x00,
                0x00, 0x00, 0x2e, 0x14, 0x00, 0x08, 0x03, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
                0x00, 0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x6c, 0x69,
                0x74, 0x74, 0x6c, 0x65, 0x66, 0x73, 0xfa, 0x74, 0x0b, 0xc5,
            ];
            let block: super::MetadataBlock = bytes.read(&mut 0).unwrap();
            assert_eq!(block.revision_count, 3);
            assert_eq!(block.continued, false);
            assert_eq!(block.dir_size, 52);
            assert_eq!(block.tail_pointer, [3, 2]);
            assert_eq!(block.crc, 0xc50b74fa);
            let iter = block.into_iter();
            let dir_entries: Result<Vec<_>, _> = iter.collect();
            let dir_entries = dir_entries.unwrap();
            assert_eq!(dir_entries.len(), 1);
            let entry = &dir_entries[0];
            // assert_eq!(entry.entry_type(), crate::DirEntryType::Superblock);
            assert_eq!(entry.attributes.len(), 0);
            assert_eq!(entry.name, "littlefs");
        }
    }
}

#[cfg(test)]
#[macro_use]
extern crate std;

#[cfg(test)]
mod test {
    use alloc::vec::Vec;
    use binrw::io::Cursor;
    use binrw::BinReaderExt;
    #[test]
    fn superblock() {
        let bytes: Vec<u8> = vec![
            0x03, 0x00, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02, 0x00,
            0x00, 0x00, 0x2e, 0x14, 0x00, 0x08, 0x03, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
            0x00, 0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x6c, 0x69,
            0x74, 0x74, 0x6c, 0x65, 0x66, 0x73, 0xfa, 0x74, 0x0b, 0xc5,
        ];
        let mut reader = Cursor::new(bytes);
        let block: crate::MetadataBlock = reader.read_le().unwrap();
        assert_eq!(block.revision_count, 3);
        assert_eq!(block.continued, false);
        assert_eq!(block.dir_size, 52);
        assert_eq!(block.tail_pointer, [3, 2]);
        assert_eq!(block.crc, 0xc50b74fa);
        assert_eq!(block.dir_entries.len(), 1);
        let entry = &block.dir_entries[0];
        assert_eq!(entry.entry_type(), crate::DirEntryType::Superblock);
        assert_eq!(entry.attributes.len(), 0);
        assert_eq!(entry.name_str(), "littlefs");
    }
    #[test]
    fn directory() {
        let bytes: Vec<u8> = vec![
            0x0a, 0x00, 0x00, 0x00, 0x9a, 0x00, 0x00, 0x00, 0x25, 0x00, 0x00, 0x00, 0x24, 0x00,
            0x00, 0x00, 0x22, 0x08, 0x00, 0x03, 0x05, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
            0x74, 0x65, 0x61, 0x22, 0x08, 0x00, 0x06, 0x07, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00,
            0x00, 0x63, 0x6f, 0x66, 0x66, 0x65, 0x65, 0x22, 0x08, 0x00, 0x04, 0x09, 0x00, 0x00,
            0x00, 0x08, 0x00, 0x00, 0x00, 0x73, 0x6f, 0x64, 0x61, 0x22, 0x08, 0x00, 0x05, 0x1d,
            0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x6d, 0x69, 0x6c, 0x6b, 0x31, 0x22, 0x08,
            0x00, 0x05, 0x1f, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x6d, 0x69, 0x6c, 0x6b,
            0x32, 0x22, 0x08, 0x00, 0x05, 0x21, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x6d,
            0x69, 0x6c, 0x6b, 0x33, 0x22, 0x08, 0x00, 0x05, 0x23, 0x00, 0x00, 0x00, 0x22, 0x00,
            0x00, 0x00, 0x6d, 0x69, 0x6c, 0x6b, 0x34, 0x22, 0x08, 0x00, 0x05, 0x25, 0x00, 0x00,
            0x00, 0x24, 0x00, 0x00, 0x00, 0x6d, 0x69, 0x6c, 0x6b, 0x35, 0x06, 0x31, 0x6e, 0xc8,
        ];
        let mut reader = Cursor::new(bytes);
        let block: crate::MetadataBlock = reader.read_le().unwrap();
        assert_eq!(block.revision_count, 10);
        assert_eq!(block.dir_size, 154);
        assert_eq!(block.tail_pointer, [37, 36]);
        assert_eq!(block.crc, 0xc86e3106);
        assert_eq!(block.dir_entries.len(), 8);
        assert_eq!(
            block
                .dir_entries
                .iter()
                .map(|entry| std::str::from_utf8(&entry.name).unwrap())
                .collect::<Vec<&str>>(),
            ["tea", "coffee", "soda", "milk1", "milk2", "milk3", "milk4", "milk5"]
        );
    }
}

pub fn add(left: usize, right: usize) -> usize {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
