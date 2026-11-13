//! MTP storage backend. NAND/LittleFS enumeration is completed before the
//! OTG_HS device is connected to the host, so it cannot starve MTP servicing.

use core::mem::MaybeUninit;
use core::ptr::{addr_of, addr_of_mut};
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
use littlefs2::fs::{File, FileAllocation, Filesystem};
use littlefs2::io::SeekFrom;

use crate::dhara_fs::DharaStorage;

const STORAGE_TOTAL: u32 = 80 * 1024 * 1024;
const STORAGE_FREE: u32 = 70 * 1024 * 1024;
const FILE_NAME: &[u8] = b"readme.txtx";
const FILE_CONTENT: &[u8] = b"test MTP";
const MAX_REAL_FILES: usize = 25;
const MAX_NAME: usize = 127;

// littlefs2 retains pointers to its Allocation and Storage for the lifetime
// of the mounted filesystem. These objects therefore live at fixed addresses
// for the entire MTP device lifetime and are never moved after initialization.
type MtpFilesystem = Filesystem<'static, DharaStorage>;
type MtpFile = File<'static, 'static, DharaStorage>;

static mut NAND_STORAGE: MaybeUninit<DharaStorage> = MaybeUninit::uninit();
static mut FS_ALLOCATION: MaybeUninit<littlefs2::fs::Allocation<DharaStorage>> =
    MaybeUninit::uninit();
static mut FILESYSTEM: MaybeUninit<MtpFilesystem> = MaybeUninit::uninit();
static mut FILE_ALLOCATION: MaybeUninit<FileAllocation<DharaStorage>> =
    MaybeUninit::uninit();
static mut OPEN_FILE: MaybeUninit<MtpFile> = MaybeUninit::uninit();

// 0 = not initialized, 1 = mounted, 2 = initialization failed.
// Initialization and all filesystem operations run only in the MTP main loop.
static FILESYSTEM_STATE: AtomicU8 = AtomicU8::new(0);
static FILE_OPEN: AtomicBool = AtomicBool::new(false);
static CLOSE_FILE_REQUESTED: AtomicBool = AtomicBool::new(false);
static REFRESH_REQUESTED: AtomicBool = AtomicBool::new(false);
static ACTIVE_HANDLE: AtomicU32 = AtomicU32::new(0);
static NEXT_FILE_OFFSET: AtomicU32 = AtomicU32::new(0);

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

/// Mount LittleFS once and keep its storage and allocation at stable addresses.
/// The explicit 'static references are valid because these statics are never
/// moved, reinitialized, or accessed outside this single-threaded MTP backend.
fn ensure_filesystem() -> bool {
    match FILESYSTEM_STATE.load(Ordering::Acquire) {
        1 => return true,
        2 => return false,
        _ => {}
    }

    let storage = match DharaStorage::new_mtp() {
        Ok(storage) => storage,
        Err(_) => {
            FILESYSTEM_STATE.store(2, Ordering::Release);
            return false;
        }
    };

    unsafe {
        let storage_ptr = addr_of_mut!(NAND_STORAGE).cast::<DharaStorage>();
        storage_ptr.write(storage);

        let allocation_ptr =
            addr_of_mut!(FS_ALLOCATION).cast::<littlefs2::fs::Allocation<DharaStorage>>();
        allocation_ptr.write(Filesystem::<DharaStorage>::allocate());

        // SAFETY: both backing objects are static, initialized above, and are
        // never moved or mutably accessed except through this Filesystem.
        let storage_ref: &'static mut DharaStorage = &mut *storage_ptr;
        let allocation_ref: &'static mut littlefs2::fs::Allocation<DharaStorage> =
            &mut *allocation_ptr;

        match Filesystem::mount(allocation_ref, storage_ref) {
            Ok(filesystem) => {
                addr_of_mut!(FILESYSTEM)
                    .cast::<MtpFilesystem>()
                    .write(filesystem);
                FILESYSTEM_STATE.store(1, Ordering::Release);
                true
            }
            Err(_) => {
                FILESYSTEM_STATE.store(2, Ordering::Release);
                false
            }
        }
    }
}

/// Close the current file exactly once. Called only from MTP main-loop context,
/// never from a USB callback or interrupt.
fn close_active_file() {
    if !FILE_OPEN.swap(false, Ordering::AcqRel) {
        return;
    }

    ACTIVE_HANDLE.store(0, Ordering::Release);
    NEXT_FILE_OFFSET.store(0, Ordering::Release);

    // SAFETY: FILE_OPEN was atomically cleared above, so this is the unique
    // move of the initialized File out of its static slot. File::close consumes
    // it and removes its node from littlefs's internal open-file list.
    let file: MtpFile = unsafe {
        addr_of!(OPEN_FILE)
            .cast::<MtpFile>()
            .read()
    };
    let _ = unsafe { file.close() };
}

/// Keep startup identical to the known-good in-memory backend. Real-file
/// enumeration is requested later by OpenSession, after USB is servicing IRQs.
pub fn prepare_storage() {
    OBJECT_COUNT.store(1, Ordering::Release);
}

/// OpenSession requests a root refresh. The actual NAND/LittleFS work is
/// deferred to service_background(), never performed in a TinyUSB callback.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_refresh() -> bool {
    REFRESH_REQUESTED.store(true, Ordering::Release);
    true
}

/// Drain deferred close and refresh requests from MTP main-loop context.
pub fn service_background() {
    if CLOSE_FILE_REQUESTED.swap(false, Ordering::AcqRel) {
        close_active_file();
    }

    if !REFRESH_REQUESTED.swap(false, Ordering::AcqRel) {
        return;
    }

    // Publish only the synthetic object while rebuilding the real-file table.
    // The object count is updated after enumeration has completed.
    OBJECT_COUNT.store(1, Ordering::Release);
    let result = crate::nand_fs::enumerate_root_files(
        MAX_REAL_FILES,
        true,
        |name| name != FILE_NAME,
        |path, size| {
            if path.len() < 3 || path.last() != Some(&0) {
                return;
            }
            let name = &path[1..path.len() - 1];
            if name.len() > MAX_NAME {
                return;
            }

            // The enumerator is capped at MAX_REAL_FILES, and this callback
            // runs synchronously before the completed count is published.
            let objects = unsafe { &mut *addr_of_mut!(OBJECTS) };
            let index = OBJECT_COUNT.load(Ordering::Relaxed) as usize - 1;
            if index >= MAX_REAL_FILES {
                return;
            }
            let object = &mut objects[index];
            object.name[..name.len()].copy_from_slice(name);
            object.name_len = name.len() as u8;
            object.size = size.unwrap_or(0);
            OBJECT_COUNT.store((index + 2) as u32, Ordering::Relaxed);
        },
    );

    // On an enumeration error, leave the synthetic test object available.
    if result.is_err() {
        OBJECT_COUNT.store(1, Ordering::Release);
    } else {
        // The callback count includes only real files; add the synthetic
        // object as the final handle.
        let real_count = OBJECT_COUNT.load(Ordering::Relaxed);
        OBJECT_COUNT.store(real_count, Ordering::Release);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_end_object_read() {
    CLOSE_FILE_REQUESTED.store(true, Ordering::Release);
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
        let objects = unsafe { &*addr_of!(OBJECTS) };
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

/// Read a bounded chunk. A real file is opened and positioned only for the
/// first chunk, then read sequentially until the MTP transfer completes.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_read_object_chunk(
    handle: u32, offset: u32, destination: *mut u8, capacity: u32,
) -> i32 {
    // A completion/cancel callback may have queued closure immediately before
    // another command is dispatched in the same TinyUSB task pass.
    if CLOSE_FILE_REQUESTED.swap(false, Ordering::AcqRel) {
        close_active_file();
    }

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
    let objects = unsafe { &*addr_of!(OBJECTS) };
    let object = &objects[handle as usize - 1];
    let size = object.size as usize;
    if offset as usize > size || destination.is_null() {
        return if offset as usize == size { 0 } else { -1 };
    }
    let count = (size - offset as usize).min(capacity as usize);
    if count == 0 {
        return 0;
    }
    if !ensure_filesystem() {
        return -1;
    }

    if FILE_OPEN.load(Ordering::Acquire) {
        if ACTIVE_HANDLE.load(Ordering::Acquire) != handle
            || NEXT_FILE_OFFSET.load(Ordering::Acquire) != offset
        {
            close_active_file();
            return -1;
        }
    } else {
        let name = match core::str::from_utf8(&object.name[..object.name_len as usize]) {
            Ok(name) => name,
            Err(_) => return -1,
        };
        let relative_path = match littlefs2::path::PathBuf::try_from(name) {
            Ok(path) => path,
            Err(_) => return -1,
        };
        let full_path = littlefs2::path!("/").join(relative_path.as_path());

        // SAFETY: the mounted filesystem is static and remains alive for the
        // device lifetime. The file allocation is static and is not reused
        // until the previous File has been explicitly closed.
        let filesystem: &'static MtpFilesystem =
            unsafe { &*addr_of!(FILESYSTEM).cast::<MtpFilesystem>() };
        let allocation_ptr =
            addr_of_mut!(FILE_ALLOCATION).cast::<FileAllocation<DharaStorage>>();
        unsafe { allocation_ptr.write(FileAllocation::new()); }
        let allocation: &'static mut FileAllocation<DharaStorage> =
            unsafe { &mut *allocation_ptr };

        let file = match unsafe { File::open(filesystem, allocation, full_path.as_path()) } {
            Ok(file) => file,
            Err(_) => return -1,
        };
        if file.seek(SeekFrom::Start(offset)).is_err() {
            let _ = unsafe { file.close() };
            return -1;
        }

        unsafe {
            addr_of_mut!(OPEN_FILE).cast::<MtpFile>().write(file);
        }
        ACTIVE_HANDLE.store(handle, Ordering::Release);
        NEXT_FILE_OFFSET.store(offset, Ordering::Release);
        FILE_OPEN.store(true, Ordering::Release);
    }

    let output = unsafe { core::slice::from_raw_parts_mut(destination, count) };
    // SAFETY: FILE_OPEN guarantees that OPEN_FILE contains the one live File.
    let file = unsafe { &*addr_of!(OPEN_FILE).cast::<MtpFile>() };
    match file.read(output) {
        Ok(read) if read == count => {
            NEXT_FILE_OFFSET.store(offset + read as u32, Ordering::Release);
            read as i32
        }
        _ => {
            close_active_file();
            -1
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_find_object(name: *const u8, name_len: u32) -> u32 {
    if name.is_null() { return 0; }
    let bytes = unsafe { core::slice::from_raw_parts(name, name_len as usize) };
    let count = OBJECT_COUNT.load(Ordering::Acquire) as usize;
    if bytes == FILE_NAME { return count as u32; }
    let objects = unsafe { &*addr_of!(OBJECTS) };
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
