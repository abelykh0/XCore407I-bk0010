//! Deterministic MTP transport test backend.
//!
//! This module intentionally does not mount or access NAND, Dhara, or LittleFS.
//! It exposes one in-memory test object so USB/MTP protocol behavior can be
//! tested independently of the storage stack.


use core::sync::atomic::{AtomicU32, Ordering};

static STORAGE_TOTAL: AtomicU32 = AtomicU32::new(80 * 1024 * 1024);
static STORAGE_FREE: AtomicU32 = AtomicU32::new(70 * 1024 * 1024);
const FILE_NAME: &[u8] = b"readme.txtx";
const FILE_CONTENT: &[u8] = b"test MTP";

/// Refresh real LittleFS capacity/free-space statistics for the MTP session.
/// The test object remains in memory; no files are created or modified.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_refresh() -> bool {
    match crate::nand_fs::mtp_storage_stats() {
        Ok((total, free)) => {
            STORAGE_TOTAL.store(total, Ordering::Release);
            STORAGE_FREE.store(free, Ordering::Release);
            true
        }
        Err(_) => false,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_object_count() -> u32 {
    1
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_storage_stats(total: *mut u32, free: *mut u32) {
    if !total.is_null() {
        unsafe { total.write(STORAGE_TOTAL.load(Ordering::Acquire)); }
    }
    if !free.is_null() {
        unsafe { free.write(STORAGE_FREE.load(Ordering::Acquire)); }
    }
}

/// The sole object has handle 1 and is named "readme.txtx".
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_object_info(
    handle: u32,
    name_out: *mut u16,
    max_name_units: u32,
    size_out: *mut u32,
) -> bool {
    if handle != 1 {
        return false;
    }
    if !size_out.is_null() {
        unsafe { size_out.write(FILE_CONTENT.len() as u32); }
    }
    if !name_out.is_null() && max_name_units != 0 {
        let count = FILE_NAME.len().min(max_name_units as usize - 1);
        for (index, byte) in FILE_NAME[..count].iter().enumerate() {
            unsafe { name_out.add(index).write(*byte as u16); }
        }
        unsafe { name_out.add(count).write(0); }
    }
    true
}

/// Copy a requested range from the fixed in-memory object.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_read_object_chunk(
    handle: u32,
    offset: u32,
    destination: *mut u8,
    capacity: u32,
) -> i32 {
    if handle != 1 || destination.is_null() || offset as usize > FILE_CONTENT.len() {
        return -1;
    }
    let start = offset as usize;
    let count = (FILE_CONTENT.len() - start).min(capacity as usize);
    if count != 0 {
        unsafe {
            core::ptr::copy_nonoverlapping(
                FILE_CONTENT.as_ptr().add(start),
                destination,
                count,
            );
        }
    }
    count as i32
}

/// Return handle 1 only for the synthetic test filename.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_find_object(name: *const u8, name_len: u32) -> u32 {
    if name.is_null() || name_len as usize != FILE_NAME.len() {
        return 0;
    }
    let bytes = unsafe { core::slice::from_raw_parts(name, name_len as usize) };
    if bytes == FILE_NAME { 1 } else { 0 }
}

// Upload is deliberately disabled in this read-only protocol experiment.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_begin_upload(
    _name: *const u8,
    _name_len: u32,
    _expected_size: u32,
) -> bool {
    false
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_write_upload_chunk(
    _offset: u32,
    _data: *const u8,
    _len: u32,
) -> bool {
    false
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_finish_upload() -> bool {
    false
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_abort_upload() {}
