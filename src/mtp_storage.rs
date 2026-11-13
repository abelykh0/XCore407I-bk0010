//! MTP storage backend. Root-directory enumeration is prepared in the main
//! loop, never from a TinyUSB request callback.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use littlefs2::{
    fs::{Allocation, File, FileAllocation, Filesystem},
    io::Result as LfsResult,
};
use core::mem::MaybeUninit;

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
static CLOSE_FILE_PENDING: AtomicBool = AtomicBool::new(false);
static FILESYSTEM_READY: AtomicBool = AtomicBool::new(false);
static FILE_ALLOCATION_READY: AtomicBool = AtomicBool::new(false);
static FILE_OPEN: AtomicBool = AtomicBool::new(false);
static ACTIVE_FILE_HANDLE: AtomicU32 = AtomicU32::new(0);
static NEXT_FILE_OFFSET: AtomicU32 = AtomicU32::new(0);

// Keep the storage driver, LittleFS allocation, mounted filesystem, and
// active file in separate static objects. The File borrows the mounted
// Filesystem, but neither object owns the other, so this is not self-referential.
static mut MTP_STORAGE: MaybeUninit<DharaStorage> = MaybeUninit::uninit();
static mut MTP_ALLOCATION: MaybeUninit<Allocation<DharaStorage>> = MaybeUninit::uninit();
static mut MTP_FILESYSTEM: MaybeUninit<Filesystem<'static, DharaStorage>> = MaybeUninit::uninit();
static mut MTP_FILE_ALLOCATION: MaybeUninit<FileAllocation<DharaStorage>> = MaybeUninit::uninit();
static mut MTP_FILE: MaybeUninit<File<'static, 'static, DharaStorage>> = MaybeUninit::uninit();
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

/// Return the persistent mounted filesystem. Its backing storage and
/// allocation are separate statics and remain alive for the entire MTP boot.
unsafe fn mtp_filesystem() -> &'static Filesystem<'static, DharaStorage> {
    unsafe { (&*core::ptr::addr_of!(MTP_FILESYSTEM)).assume_init_ref() }
}

fn ensure_mtp_filesystem() -> bool {
    if FILESYSTEM_READY.load(Ordering::Acquire) {
        return true;
    }

    unsafe {
        let storage = match DharaStorage::new_mtp() {
            Ok(storage) => (&mut *core::ptr::addr_of_mut!(MTP_STORAGE)).write(storage),
            Err(_) => return false,
        };
        let allocation = (&mut *core::ptr::addr_of_mut!(MTP_ALLOCATION))
            .write(Filesystem::<DharaStorage>::allocate());
        let filesystem = match Filesystem::mount(allocation, storage) {
            Ok(filesystem) => filesystem,
            Err(_) => return false,
        };
        (&mut *core::ptr::addr_of_mut!(MTP_FILESYSTEM)).write(filesystem);
    }

    FILESYSTEM_READY.store(true, Ordering::Release);
    true
}

fn close_active_file() {
    if !FILE_OPEN.swap(false, Ordering::AcqRel) {
        return;
    }

    // File::close consumes the handle and removes it from LittleFS's internal
    // open-file list. The file allocation itself remains available for reuse.
    unsafe {
        let file = (&*core::ptr::addr_of!(MTP_FILE)).assume_init_read();
        let _ = file.close();
    }
    ACTIVE_FILE_HANDLE.store(0, Ordering::Release);
    NEXT_FILE_OFFSET.store(0, Ordering::Release);
}

fn open_or_seek_active_file(handle: u32, offset: u32) -> bool {
    if FILE_OPEN.load(Ordering::Acquire)
        && ACTIVE_FILE_HANDLE.load(Ordering::Relaxed) != handle
    {
        close_active_file();
    }

    if !FILE_OPEN.load(Ordering::Acquire) {
        let count = OBJECT_COUNT.load(Ordering::Acquire) as usize;
        if handle == 0 || handle as usize > count.saturating_sub(1) {
            return false;
        }
        let objects = unsafe { &*core::ptr::addr_of!(OBJECTS) };
        let object = &objects[handle as usize - 1];
        let name = match core::str::from_utf8(&object.name[..object.name_len as usize]) {
            Ok(name) => name,
            Err(_) => return false,
        };
        let relative_path = match littlefs2::path::PathBuf::try_from(name) {
            Ok(path) => path,
            Err(_) => return false,
        };
        let full_path = littlefs2::path!("/").join(relative_path.as_path());

        if !ensure_mtp_filesystem() {
            return false;
        }

        unsafe {
            if !FILE_ALLOCATION_READY.load(Ordering::Acquire) {
                (&mut *core::ptr::addr_of_mut!(MTP_FILE_ALLOCATION))
                    .write(FileAllocation::<DharaStorage>::new());
                FILE_ALLOCATION_READY.store(true, Ordering::Release);
            }
            let filesystem = mtp_filesystem();
            let file_allocation =
                (&mut *core::ptr::addr_of_mut!(MTP_FILE_ALLOCATION)).assume_init_mut();
            let file = match File::open(filesystem, file_allocation, full_path.as_path()) {
                Ok(file) => file,
                Err(_) => return false,
            };
            (&mut *core::ptr::addr_of_mut!(MTP_FILE)).write(file);
        }
        ACTIVE_FILE_HANDLE.store(handle, Ordering::Release);
        NEXT_FILE_OFFSET.store(0, Ordering::Release);
        FILE_OPEN.store(true, Ordering::Release);
    }

    if NEXT_FILE_OFFSET.load(Ordering::Acquire) != offset {
        let file = unsafe { (&*core::ptr::addr_of!(MTP_FILE)).assume_init_ref() };
        if littlefs2::io::Seek::seek(file, littlefs2::io::SeekFrom::Start(offset as u64)).is_err() {
            close_active_file();
            return false;
        }
        NEXT_FILE_OFFSET.store(offset, Ordering::Release);
    }
    true
}

/// Run from the MTP-mode main loop, after TinyUSB has been serviced.
/// Mount once and retain the filesystem; process file-close requests here,
/// never in a USB callback or interrupt.
pub fn service_background() {
    if CLOSE_FILE_PENDING.swap(false, Ordering::AcqRel) {
        close_active_file();
    }

    if !REFRESH_PENDING.swap(false, Ordering::AcqRel) {
        return;
    }
    if !ensure_mtp_filesystem() {
        return;
    }

    let result = unsafe {
        mtp_filesystem().read_dir_and_then(&littlefs2::path!("/"), |directory| {
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
                let size = mtp_filesystem().metadata(path)?.len();
                let mut object = EMPTY_OBJECT;
                object.name[..name.len()].copy_from_slice(name);
                object.name_len = name.len() as u8;
                object.size = size as u32;
                objects[count] = object;
                count += 1;
            }
            Ok(count)
        })
    };

    if let Ok(real_count) = result {
        OBJECT_COUNT.store((real_count + 1) as u32, Ordering::Release);
    }
}

/// Called by TinyUSB callbacks only to request deferred cleanup.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_close_file() {
    CLOSE_FILE_PENDING.store(true, Ordering::Release);
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

    if !open_or_seek_active_file(handle, offset) {
        return -1;
    }
    let output = unsafe { core::slice::from_raw_parts_mut(destination, count) };
    let file = unsafe { (&*core::ptr::addr_of!(MTP_FILE)).assume_init_ref() };
    match littlefs2::io::Read::read(file, output) {
        Ok(read) if read == count => {
            NEXT_FILE_OFFSET.store(offset.saturating_add(read as u32), Ordering::Release);
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
