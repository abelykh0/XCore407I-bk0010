//! The `Storage`, `Read`, `Write` and `Seek` driver.
#![allow(non_camel_case_types)]

#[allow(deprecated)]
use generic_array::ArrayLength;

use crate::io::Result;

/// Users of this library provide a "storage driver" by implementing this trait.
///
/// The `write` method may leave changes pending in the storage driver.
/// littlefs invokes `sync()` at filesystem synchronization boundaries.
/// Due to filesystem caches, files must still be synchronized explicitly;
/// this cannot be done automatically in `drop`, which would need mutable
/// references to both filesystem and storage.
// Local XCore407I modification: expose the LittleFS sync callback to drivers.
///
/// The `*_SIZE` types must be `generic_array::typenume::consts` such as `U256`.
///
/// Why? Currently, associated constants can not be used (as constants...) to define
/// arrays. This "will be fixed" as part of const generics.
/// Once that's done, we can get rid of `generic-array`s, and replace the
/// `*_SIZE` types with `usize`s.
pub trait Storage {
    // /// Error type for user-provided read/write/erase methods
    // type Error = usize;

    /// Minimum size of block read in bytes. Not in superblock
    const READ_SIZE: usize;

    /// Minimum size of block write in bytes. Not in superblock
    const WRITE_SIZE: usize;

    /// Size of an erasable block in bytes, as unsigned typenum.
    /// Must be a multiple of both `READ_SIZE` and `WRITE_SIZE`.
    /// [At least 128](https://github.com/littlefs-project/littlefs/issues/264#issuecomment-519963153). Stored in superblock.
    const BLOCK_SIZE: usize;

    /// Number of erasable blocks.
    /// Hence storage capacity is `BLOCK_COUNT * BLOCK_SIZE`
    const BLOCK_COUNT: usize;

    /// Suggested values are 100-1000, higher is more performant but
    /// less wear-leveled.  Default of -1 disables wear-leveling.
    /// Value zero is invalid, must be positive or -1.
    const BLOCK_CYCLES: isize = -1;

    /// littlefs uses a read cache, a write cache, and one cache per per file.
    /// Must be a multiple of `READ_SIZE` and `WRITE_SIZE`.
    /// Must be a factor of `BLOCK_SIZE`.
    #[allow(deprecated)]
    type CACHE_SIZE: ArrayLength<u8>;

    /// Size of the lookahead buffer used by littlefs, measured in multiples of 8 bytes.
    #[allow(deprecated)]
    type LOOKAHEAD_SIZE: ArrayLength<u64>;

    ///// Maximum length of a filename plus one. Stored in superblock.
    ///// Should default to 255+1, but associated type defaults don't exist currently.
    ///// At most 1_022+1.
    /////
    ///// TODO: We can't actually change this - need to pass on as compile flag
    ///// to the C backend.
    //type FILENAME_MAX_PLUS_ONE: ArrayLength<u8>;

    // /// Maximum length of a path plus one. Necessary to convert Rust string slices
    // /// to C strings, which requires an allocation for the terminating
    // /// zero-byte. If in doubt, set to `FILENAME_MAX_PLUS_ONE`.
    // /// Must be larger than `FILENAME_MAX_PLUS_ONE`.
    // type PATH_MAX_PLUS_ONE: ArrayLength<u8>;

    ///// Maximum size of file. Stored in superblock.
    ///// Defaults to 2_147_483_647 (or u31, to avoid sign issues in the C code).
    ///// At most 2_147_483_647.
    /////
    ///// TODO: We can't actually change this - need to pass on as compile flag
    ///// to the C backend.
    //const FILEBYTES_MAX: usize = ll::LFS_FILE_MAX as _;

    ///// Maximum size of custom attributes.
    ///// Should default to 1_022, but associated type defaults don't exists currently.
    ///// At most 1_022.
    /////
    ///// TODO: We can't actually change this - need to pass on as compile flag
    ///// to the C backend.
    //type ATTRBYTES_MAX: ArrayLength<u8>;

    /// Read data from the storage device.
    /// Guaranteed to be called only with bufs of length a multiple of READ_SIZE.
    fn read(&mut self, off: usize, buf: &mut [u8]) -> Result<usize>;
    /// Write data to the storage device.
    /// Guaranteed to be called only with bufs of length a multiple of WRITE_SIZE.
    fn write(&mut self, off: usize, data: &[u8]) -> Result<usize>;
    /// Erase data from the storage device.
    /// Guaranteed to be called only with bufs of length a multiple of BLOCK_SIZE.
    fn erase(&mut self, off: usize, len: usize) -> Result<usize>;
    /// Synchronize pending writes to the storage device.
    ///
    /// Called by littlefs at filesystem synchronization boundaries. Drivers
    /// that commit each write immediately may keep the default implementation.
    fn sync(&mut self) -> Result<()> {
        Ok(())
    }
}
