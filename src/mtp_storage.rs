//! MTP storage backend. Root-directory enumeration is prepared in the main
//! loop, never from a TinyUSB request callback.

use core::sync::atomic::{AtomicU32, Ordering};

const STORAGE_TOTAL: u32 = 80 * 1024 * 1024;
const STORAGE_FREE: u32 = 70 * 1024 * 1024;
const FILE_NAME: &[u8] = b"readme.txtx";
const FILE_CONTENT: &[u8] = b"test MTP";

static OBJECT_COUNT: AtomicU32 = AtomicU32::new(1);

/// Request a refresh. The diagnostic build intentionally exposes only the
/// synthetic object so NAND/LittleFS scanning cannot delay TinyUSB task work.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_refresh() -> bool {
    OBJECT_COUNT.store(1, Ordering::Release);
    true
}

/// Kept as a main-loop hook; real NAND/LittleFS scanning is disabled for this
/// discovery diagnostic.
pub fn service_background() {}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_object_count() -> u32 {
    OBJECT_COUNT.load(Ordering::Acquire)
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_storage_stats(total: *mut u32, free: *mut u32) {
    if !total.is_null() { unsafe { total.write(STORAGE_TOTAL); } }
    if !free.is_null() { unsafe { free.write(STORAGE_FREE); } }
}

/// The sole visible object is the synthetic test file, at handle 1.
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
    if !size_out.is_null() { unsafe { size_out.write(FILE_CONTENT.len() as u32); } }
    if !name_out.is_null() && max_name_units != 0 {
        let n = FILE_NAME.len().min(max_name_units as usize - 1);
        for (i, byte) in FILE_NAME[..n].iter().enumerate() {
            unsafe { name_out.add(i).write(*byte as u16); }
        }
        unsafe { name_out.add(n).write(0); }
    }
    true
}

/// Read the synthetic test object in bounded chunks.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_read_object_chunk(
    handle: u32, offset: u32, destination: *mut u8, capacity: u32,
) -> i32 {
    if handle != 1 || offset as usize > FILE_CONTENT.len() {
        return -1;
    }
    let start = offset as usize;
    let count = (FILE_CONTENT.len() - start).min(capacity as usize);
    if count != 0 {
        if destination.is_null() {
            return -1;
        }
        unsafe {
            core::ptr::copy_nonoverlapping(FILE_CONTENT.as_ptr().add(start), destination, count);
        }
    }
    count as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_find_object(name: *const u8, name_len: u32) -> u32 {
    if name.is_null() { return 0; }
    let bytes = unsafe { core::slice::from_raw_parts(name, name_len as usize) };
    if bytes == FILE_NAME { 1 } else { 0 }
}

// Upload is deliberately disabled in this read-only experiment.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_begin_upload(_name: *const u8, _name_len: u32, _expected_size: u32) -> bool { false }
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_write_upload_chunk(_offset: u32, _data: *const u8, _len: u32) -> bool { false }
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_finish_upload() -> bool { false }
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_abort_upload() {}
