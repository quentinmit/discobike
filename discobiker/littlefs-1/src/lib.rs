#![no_std]

extern crate alloc;
use alloc::vec::Vec;
use binrw::io::*;
use binrw::*;

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
#[brw(little)]
#[derive(Debug)]
struct DirEntryFile {
    file_head: u32,
    file_size: u32,
}

#[binrw]
#[brw(little)]
#[derive(Debug)]
struct DirEntrySuperblock {
    root_directory: [u32; 2],
    block_size: u32,
    block_count: u32,
    version: u32,
}

#[binrw]
#[br(little, import(ty: DirEntryType))]
#[bw(little)]
#[derive(Debug)]
enum DirEntryData {
    #[br(pre_assert(ty == DirEntryType::File))]
    File(DirEntryFile),
    #[br(pre_assert(ty == DirEntryType::Directory))]
    Directory([u32; 2]),
    #[br(pre_assert(ty == DirEntryType::Superblock))]
    Superblock(DirEntrySuperblock),
}

#[binrw]
#[brw(little)]
#[derive(Debug)]
struct DirEntry {
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
    dir_size: u32,
    tail_pointer: [u32; 2],
    #[br(parse_with = byte_size((dir_size-5*4) as usize))]
    dir_entries: Vec<DirEntry>,
    crc: u32,
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
        let superblock: crate::MetadataBlock = reader.read_le().unwrap();
        assert_eq!(superblock.revision_count, 3);
        assert_eq!(superblock.dir_size, 52);
        assert_eq!(superblock.tail_pointer, [3, 2]);
        assert_eq!(superblock.crc, 0xc50b74fa);
        assert_eq!(superblock.dir_entries.len(), 1);
        let entry = &superblock.dir_entries[0];
        assert_eq!(entry.entry_type, crate::DirEntryType::Superblock);
        assert_eq!(entry.attributes.len(), 0);
        assert_eq!(std::str::from_utf8(&entry.name).unwrap(), "littlefs");
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
        assert_eq!(block.dir_entries.iter().map(
            |entry| std::str::from_utf8(&entry.name).unwrap()
        ).collect::<Vec<&str>>(), ["tea", "coffee", "soda", "milk1", "milk2", "milk3", "milk4", "milk5"]);
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
