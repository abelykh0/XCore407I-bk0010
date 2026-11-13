//! MTP storage backend. NAND/LittleFS enumeration is completed before the
//! OTG_HS device is connected to the host, so it cannot starve MTP servicing.

use core::sync::atomic::{AtomicU32, Ordering};
use littlefs2::fs::Filesystem;

use crate::dhara_fs::DharaStorage;

const STORAGE_TOTAL: u32 = 80 * 1024 * 1024;
const STORAGE_FREE: u32 = 70 * 1024 * 1024;
const FILE_NAME: &[u8] = b"readme.txtx";
const FILE_CONTENT: &[u8] = b"test MTP";
const MAX_REAL_FILES: usize = 25;
const MAX_NAME: usize = 127;

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

static OBJECT_COUNT: AtomicU32 = AtomicU32::new(1);
static mut OBJECTS: [Object; MAX_REAL_FILES] = [EMPTY_OBJECT; MAX_REAL_FILES];

/// Build the MTP object table with the same root-file enumerator used by F2.
/// MTP includes every regular root file and also records its size.
pub fn prepare_storage() {
    let objects = unsafe { &mut *core::ptr::addr_of_mut!(OBJECTS) };
    let mut count = 0usize;
    let result = crate::nand_fs::enumerate_root_files(
        MAX_REAL_FILES,
        true,
        |name| name != FILE_NAME,
        |path, size| {
            let name = &path[1..path.len() - 1];
            let object = &mut objects[count];
            object.name[..name.len()].copy_from_slice(name);
            object.name_len = name.len() as u8;
            object.size = size.unwrap_or(0);
            count += 1;
        },
    );

    if result.is_ok() {
        OBJECT_COUNT.store((count + 1) as u32, Ordering::Release);
    } else {
        OBJECT_COUNT.store(1, Ordering::Release);
    }
}

/// OpenSession refresh is intentionally a no-op: the object table was prepared
/// before the device was connected, and rescanning here would block MTP.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_refresh() -> bool {
    true
}

/// No storage work is scheduled after USB device initialization.
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

/// Read one bounded chunk from either a real root file or the synthetic test object.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_read_object_chunk(
    handle: u32, offset: u32, destination: *mut u8, capacity: u32,
) -> i32 {
    let object_count = OBJECT_COUNT.load(Ordering::Acquire);
    if handle == object_count {
        if offset as usize > FILE_CONTENT.len() {
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
        return count as i32;
    }

    if handle == 0 || handle >= object_count || handle as usize > MAX_REAL_FILES {
        return -1;
    }
    let objects = unsafe { &*core::ptr::addr_of!(OBJECTS) };
    let object = &objects[handle as usize - 1];
    let size = object.size as usize;
    if offset as usize > size {
        return -1;
    }
    let count = (size - offset as usize).min(capacity as usize);
    if count == 0 {
        return 0;
    }
    if destination.is_null() {
        return -1;
    }

    let name = match core::str::from_utf8(&object.name[..object.name_len as usize]) {
        Ok(name) => name,
        Err(_) => return -1,
    };
    let relative_path = match littlefs2::path::PathBuf::try_from(name) {
        Ok(path) => path,
        Err(_) => return -1,
    };
    let full_path = littlefs2::path!("/").join(relative_path.as_path());
    let output = unsafe { core::slice::from_raw_parts_mut(destination, count) };

    let mut storage = match DharaStorage::new_mtp() {
        Ok(storage) => storage,
        Err(_) => return -1,
    };
    match Filesystem::<DharaStorage>::mount_and_then(&mut storage, |fs| {
        fs.open_file_and_then(&full_path, |file| {
            littlefs2::io::Seek::seek(file, littlefs2::io::SeekFrom::Start(offset))?;
            littlefs2::io::Read::read(file, output)
        })
    }) {
        Ok(read) if read == count => read as i32,
        _ => -1,
    }
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
