#![no_std]

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
            let contents = bytes.read_with::<&[u8]>(offset, Bytes::Len(dir_size as usize - *offset - 4))?;
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
            let dir_size_offset = &mut offset.clone();
            // Fill in dir size later
            bytes.write_with::<u32>(offset, 0, endian)?;
            bytes.write_with(offset, self.tail_pointer[0], endian)?;
            bytes.write_with(offset, self.tail_pointer[1], endian)?;
            bytes.write(offset, self.contents)?;
            let crc_offset = &mut offset.clone();
            bytes.write_with::<u32>(offset, 0, endian)?;

            // Fill in dir size now that we know it
            let dir_size = *offset as u32 | if self.continued { 0x80000000 } else { 0 };
            bytes.write_with(dir_size_offset, dir_size, endian)?;

            // TODO: Calculate crc
            bytes.write(crc_offset, self.crc)?;
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
    mod tests {
        extern crate alloc;
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
        #[test]
        fn directory() {
            let bytes: &[u8] = &[
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
            let block: super::MetadataBlock = bytes.read(&mut 0).unwrap();
            assert_eq!(block.revision_count, 10);
            assert_eq!(block.dir_size, 154);
            assert_eq!(block.tail_pointer, [37, 36]);
            assert_eq!(block.crc, 0xc86e3106);
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
}
