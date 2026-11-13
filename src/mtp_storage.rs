//! MTP storage listing backed by the existing LittleFS root directory.
//!
//! Capacity remains synthetic and file data remains synthetic for this step.

use core::sync::atomic::{AtomicU32, Ordering};

const STORAGE_TOTAL: u32 = 80 * 1024 * 1024;
const STORAGE_FREE: u32 = 70 * 1024 * 1024;
const MAX_OBJECTS: usize = 32;
const MAX_NAME_BYTES: usize = 128;
const FILE_NAME: &[u8] = b"readme.txtx";
const FILE_CONTENT: &[u8] = b"test MTP";

#[derive(Clone, Copy)]
struct MtpObject {
    name: [u8; MAX_NAME_BYTES],
    name_len: u16,
    size: u32,
    synthetic: bool,
}

const EMPTY_OBJECT: MtpObject = MtpObject {
    name: [0; MAX_NAME_BYTES],
    name_len: 0,
    size: 0,
    synthetic: false,
};

static mut OBJECTS: [MtpObject; MAX_OBJECTS] = [EMPTY_OBJECT; MAX_OBJECTS];
static OBJECT_COUNT: AtomicU32 = AtomicU32::new(1);

fn synthetic_object() -> MtpObject {
    let mut object = EMPTY_OBJECT;
    object.name[..FILE_NAME.len()].copy_from_slice(FILE_NAME);
    object.name_len = FILE_NAME.len() as u16;
    object.size = FILE_CONTENT.len() as u32;
    object.synthetic = true;
    object
}

fn object_at(handle: u32) -> Option<MtpObject> {
    if handle == 0 || handle > OBJECT_COUNT.load(Ordering::Acquire) {
        return None;
    }
    let objects = unsafe { &*core::ptr::addr_of!(OBJECTS) };
    Some(objects[handle as usize - 1])
}

/// Refresh the cached root listing when the host opens an MTP session.
/// The reported capacity remains fixed; the synthetic test file is appended
/// after real root files and remains readable independently of NAND.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_refresh() -> bool {
    let objects = unsafe { &mut *core::ptr::addr_of_mut!(OBJECTS) };
    objects.fill(EMPTY_OBJECT);
    objects[0] = synthetic_object();
    OBJECT_COUNT.store(1, Ordering::Release);

    let listed = crate::nand_fs::with_mounted_mtp(|fs| {
        fs.read_dir_and_then(&littlefs2::path!("/"), |directory| {
            let mut count = 0usize;
            for item in directory {
                let entry = item?;
                if !entry.file_type().is_file() || count >= MAX_OBJECTS - 1 {
                    continue;
                }

                let full_path = entry.path();
                let path_bytes = full_path.as_str_ref_with_trailing_nul().as_bytes();
                if path_bytes.len() < 3 || path_bytes.len() > MAX_NAME_BYTES + 2 {
                    continue;
                }
                let filename = &path_bytes[1..path_bytes.len() - 1];
                if filename == FILE_NAME
                    || !filename.iter().all(|byte| byte.is_ascii_graphic() || *byte == b' ')
                {
                    continue;
                }

                let Ok(metadata) = fs.metadata(&full_path) else {
                    continue;
                };
                if !metadata.is_file() {
                    continue;
                }

                let object = &mut objects[count];
                object.name[..filename.len()].copy_from_slice(filename);
                object.name_len = filename.len() as u16;
                object.size = metadata.len().min(u32::MAX as usize) as u32;
                object.synthetic = false;
                count += 1;
            }
            Ok(count)
        })
    });

    if let Ok(real_count) = listed {
        objects[real_count] = synthetic_object();
        OBJECT_COUNT.store((real_count + 1) as u32, Ordering::Release);
    }
    // If NAND initialization or listing fails, retain the synthetic object
    // so MTP discovery and the existing read test continue to work.
    true
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_object_count() -> u32 {
    OBJECT_COUNT.load(Ordering::Acquire)
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_storage_stats(total: *mut u32, free: *mut u32) {
    if !total.is_null() {
        unsafe { total.write(STORAGE_TOTAL); }
    }
    if !free.is_null() {
        unsafe { free.write(STORAGE_FREE); }
    }
}

/// Return the cached filename and size for a one-based MTP object handle.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_object_info(
    handle: u32,
    name_out: *mut u16,
    max_name_units: u32,
    size_out: *mut u32,
) -> bool {
    let Some(object) = object_at(handle) else {
        return false;
    };
    if !size_out.is_null() {
        unsafe { size_out.write(object.size); }
    }
    if !name_out.is_null() && max_name_units != 0 {
        let count = (object.name_len as usize).min(max_name_units as usize - 1);
        for index in 0..count {
            unsafe { name_out.add(index).write(object.name[index] as u16); }
        }
        unsafe { name_out.add(count).write(0); }
    }
    true
}

/// Only the synthetic test object is readable in this listing-only step.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_read_object_chunk(
    handle: u32,
    offset: u32,
    destination: *mut u8,
    capacity: u32,
) -> i32 {
    let Some(object) = object_at(handle) else {
        return -1;
    };
    if !object.synthetic || destination.is_null() || offset as usize > FILE_CONTENT.len() {
        return -1;
    }
    let start = offset as usize;
    let count = (FILE_CONTENT.len() - start).min(capacity as usize);
    if count != 0 {
        unsafe {
            core::ptr::copy_nonoverlapping(FILE_CONTENT.as_ptr().add(start), destination, count);
        }
    }
    count as i32
}

/// Resolve a cached filename to its one-based MTP object handle.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_find_object(name: *const u8, name_len: u32) -> u32 {
    if name.is_null() || name_len == 0 || name_len as usize > MAX_NAME_BYTES {
        return 0;
    }
    let bytes = unsafe { core::slice::from_raw_parts(name, name_len as usize) };
    let count = OBJECT_COUNT.load(Ordering::Acquire) as usize;
    let objects = unsafe { &*core::ptr::addr_of!(OBJECTS) };
    for (index, object) in objects[..count].iter().enumerate() {
        if object.name_len as usize == bytes.len()
            && &object.name[..bytes.len()] == bytes
        {
            return index as u32 + 1;
        }
    }
    0
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
