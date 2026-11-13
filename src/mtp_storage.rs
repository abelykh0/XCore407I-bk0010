//! MTP storage backend. Root-directory enumeration is prepared in the main
//! loop, never from a TinyUSB request callback.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use littlefs2::{fs::Filesystem, io::Result as LfsResult};

use crate::dhara_fs::DharaStorage;

const STORAGE_TOTAL: u32 = 80 * 1024 * 1024;
const STORAGE_FREE: u32 = 70 * 1024 * 1024;
const FILE_NAME: &[u8] = b"readme.txtx";
const FILE_CONTENT: &[u8] = b"test MTP";
const MAX_REAL_FILES: usize = 25;
const MAX_NAME: usize = 127;
const SYNTHETIC_HANDLE: u32 = MAX_REAL_FILES as u32 + 1;

#[derive(Clone, Copy)]
struct Object {
    name: [u8; MAX_NAME],
    name_len: u8,
    size: u32,
}

const EMPTY_OBJECT: Object = Object {
    name: [0; MAX_NAME],
    name_len: 0,
    size: 0,
};

static REFRESH_PENDING: AtomicBool = AtomicBool::new(false);
static OBJECT_COUNT: AtomicU32 = AtomicU32::new(1);
static mut OBJECTS: [Object; MAX_REAL_FILES] = [EMPTY_OBJECT; MAX_REAL_FILES];

/// Request a background refresh. This function is called by TinyUSB while
/// processing OpenSession, so it must not touch NAND or LittleFS.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_refresh() -> bool {
    OBJECT_COUNT.store(1, Ordering::Release);
    REFRESH_PENDING.store(true, Ordering::Release);
    true
}

/// Run once from the MTP-mode main loop, after TinyUSB has been serviced.
/// Only regular files in the root directory are exposed.
pub fn service_background() {
    if !REFRESH_PENDING.swap(false, Ordering::AcqRel) {
        return;
    }

    let result = Filesystem::<DharaStorage>::mount_and_then(
        &mut match DharaStorage::new_mtp() {
            Ok(storage) => storage,
            Err(_) => return,
        },
        |fs| {
            fs.read_dir_and_then(&littlefs2::path!("/"), |directory| {
                let mut count = 0usize;
                let objects = unsafe { &mut *core::ptr::addr_of_mut!(OBJECTS) };
                for item in directory {
                    let entry = item?;
                    if !entry.file_type().is_file() || count == MAX_REAL_FILES {
                        continue;
                    }
                    let path = entry.path();
                    let bytes = path.as_str_ref_with_trailing_nul().as_bytes();
                    if bytes.len() < 3 || bytes[0] != b'/' || bytes.last() != Some(&0) {
                        continue;
                    }
                    let name = &bytes[1..bytes.len() - 1];
                    if name.is_empty()
                        || name.len() > MAX_NAME
                        || !name.iter().all(|b| b.is_ascii_graphic() || *b == b' ')
                        || name == FILE_NAME
                    {
                        continue;
                    }
                    // Windows reads ObjectCompressedSize from GetObjectInfo.
                    // Cache the real LittleFS length during the background scan;
                    // USB callbacks only return this cached value.
                    let size = fs.metadata(path)?.len();
                    let mut object = EMPTY_OBJECT;
                    object.name[..name.len()].copy_from_slice(name);
                    object.name_len = name.len() as u8;
                    object.size = size as u32;
                    objects[count] = object;
                    count += 1;
                }
                Ok(count)
            })
        },
    );

    if let Ok(real_count) = result {
        OBJECT_COUNT.store((real_count + 1) as u32, Ordering::Release);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_object_count() -> u32 {
    OBJECT_COUNT.load(Ordering::Acquire)
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_storage_stats(total: *mut u32, free: *mut u32) {
    if !total.is_null() { unsafe { total.write(STORAGE_TOTAL); } }
    if !free.is_null() { unsafe { free.write(STORAGE_FREE); } }
}

/// Handles 1..=25 refer to real root files; the final handle is synthetic.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_object_info(
    handle: u32,
    name_out: *mut u16,
    max_name_units: u32,
    size_out: *mut u32,
) -> bool {
    let count = OBJECT_COUNT.load(Ordering::Acquire) as usize;
    let (name, size): (&[u8], u32) = if handle == count as u32 {
        (FILE_NAME, FILE_CONTENT.len() as u32)
    } else if handle >= 1 && (handle as usize) <= count.saturating_sub(1) {
        let objects = unsafe { &*core::ptr::addr_of!(OBJECTS) };
        let object = &objects[handle as usize - 1];
        (&object.name[..object.name_len as usize], object.size)
    } else {
        return false;
    };

    if !size_out.is_null() { unsafe { size_out.write(size); } }
    if !name_out.is_null() && max_name_units != 0 {
        let n = name.len().min(max_name_units as usize - 1);
        for (i, byte) in name[..n].iter().enumerate() {
            unsafe { name_out.add(i).write(*byte as u16); }
        }
        unsafe { name_out.add(n).write(0); }
    }
    true
}

/// The synthetic test object remains the only readable object in this step.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_read_object_chunk(
    handle: u32, offset: u32, destination: *mut u8, capacity: u32,
) -> i32 {
    let synthetic = OBJECT_COUNT.load(Ordering::Acquire);
    if handle != synthetic || destination.is_null() || offset as usize > FILE_CONTENT.len() {
        return -1;
    }
    let start = offset as usize;
    let count = (FILE_CONTENT.len() - start).min(capacity as usize);
    if count != 0 {
        unsafe { core::ptr::copy_nonoverlapping(FILE_CONTENT.as_ptr().add(start), destination, count); }
    }
    count as i32
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_find_object(name: *const u8, name_len: u32) -> u32 {
    if name.is_null() { return 0; }
    let bytes = unsafe { core::slice::from_raw_parts(name, name_len as usize) };
    let count = OBJECT_COUNT.load(Ordering::Acquire) as usize;
    if bytes == FILE_NAME { return count as u32; }
    let objects = unsafe { &*core::ptr::addr_of!(OBJECTS) };
    for index in 0..count.saturating_sub(1).min(MAX_REAL_FILES) {
        let object = &objects[index];
        if bytes == &object.name[..object.name_len as usize] {
            return index as u32 + 1;
        }
    }
    0
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
