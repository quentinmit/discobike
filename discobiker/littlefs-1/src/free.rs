use bitvec::prelude::*;

use crate::{BlockPointer, FsError};
use core::cmp::min;

const LOOKAHEAD: usize = 128;

/// FreeBlockCache tracks the free blocks in the filesystem.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FreeBlockCache {
    /// off represents the block pointer that buf[0] corresponds to.
    off: BlockPointer,
    /// size is the number of blocks being tracked in buf.
    size: usize,
    /// i is the current index in buf
    i: usize,
    /// block_count is the total number of blocks in the filesystem
    block_count: BlockPointer,
    /// ack is the number of blocks to check before looping
    ack: usize,
    buf: BitArr!(for LOOKAHEAD),
}

impl FreeBlockCache {
    pub fn new() -> Self {
        FreeBlockCache {
            off: 0,
            size: 0,
            i: 0,
            block_count: 0,
            ack: 0,
            buf: <BitArr!(for LOOKAHEAD)>::ZERO,
        }
    }
    pub fn set_block_count(&mut self, block_count: BlockPointer) {
        self.block_count = block_count;
        self.ack = block_count as usize;
        // Ignore any data we already have.
        self.size = 0;
    }
    pub fn next_free<T>(&mut self) -> Result<Option<BlockPointer>, FsError<T>> {
        trace!(
            "next_free searching [{}+{}, {})",
            self.off,
            self.i,
            self.off as usize + self.size
        );
        let off = self.buf[self.i..self.size].first_zero();
        if let Some(off) = off {
            self.i += off + 1;
            self.ack -= off + 1;
            return Ok(Some(
                (self.off + self.i as BlockPointer - 1) % self.block_count,
            ));
        }
        self.i += self.size;
        self.ack -= self.size;
        if self.ack == 0 {
            Err(FsError::Nospc)
        } else {
            Ok(None)
        }
    }
    pub fn advance(&self) -> Self {
        Self {
            off: (self.off + self.size as BlockPointer) % self.block_count,
            size: min(self.buf.len(), self.ack),
            i: 0,
            block_count: self.block_count,
            ack: self.ack,
            buf: <BitArr!(for LOOKAHEAD)>::ZERO,
        }
    }
    pub fn mark_used(&mut self, ptr: BlockPointer) {
        let off = (((ptr - self.off) + self.block_count) % self.block_count) as usize;
        if off < self.size {
            self.buf.set(off, true);
        }
    }
    pub fn ack(&mut self) {
        self.ack = self.block_count as usize;
    }
}
