use core::sync::atomic::{AtomicU32, Ordering};
use littlefs2::io::prelude::{Read, Seek, Write};

const MAX_NAME: usize = 220;

#[derive(Clone, Copy)]
struct MtpFile {
    name: [u8; MAX_NAME],
    name_len: usize,
    size: u32,
}

impl MtpFile {
    const EMPTY: Self = Self { name: [0; MAX_NAME], name_len: 0, size: 0 };
}

static FILE_COUNT: AtomicU32 = AtomicU32::new(0);
static STORAGE_TOTAL: AtomicU32 = AtomicU32::new(0);
static STORAGE_FREE: AtomicU32 = AtomicU32::new(0);
static mut UPLOAD_NAME: [u8; MAX_NAME] = [0; MAX_NAME];
static mut UPLOAD_NAME_LEN: usize = 0;
static mut UPLOAD_EXPECTED: u32 = 0;
static mut UPLOAD_ACTIVE: bool = false;

fn supported_name(name: &[u8]) -> bool {
    !name.is_empty()
        && name.len() <= MAX_NAME
        && name != b"."
        && name != b".."
        && !name.starts_with(b".mtp-")
        && name.iter().all(|byte| {
            byte.is_ascii_alphanumeric()
                || matches!(*byte, b'_' | b'-' | b'.' | b' ')
        })
}

fn file_at(index: u32) -> Result<Option<MtpFile>, xcore407i_blinker::nand_fs::InitError> {
    let mut current = 0u32;
    let mut found = None;
    xcore407i_blinker::nand_fs::with_mounted(|fs| {
        fs.read_dir_and_then(&littlefs2::path!("/"), |directory| -> littlefs2::io::Result<()> {
            // Ignore individual directory-read errors here; the enclosing
            // read_dir_and_then still reports open/close failures.
            for entry in directory.flatten() {
                if !entry.file_type().is_file() {
                    continue;
                }
                let path = entry.path().as_str_ref_with_trailing_nul();
                let bytes = path.as_bytes();
                if bytes.len() < 3 || bytes[0] != b'/' || bytes[bytes.len() - 1] != 0 {
                    continue;
                }
                let name = &bytes[1..bytes.len() - 1];
                if !supported_name(name) {
                    continue;
                }
                if current == index {
                    let mut file = MtpFile::EMPTY;
                    file.name[..name.len()].copy_from_slice(name);
                    file.name_len = name.len();
                    file.size = entry.metadata().len().min(u32::MAX as usize) as u32;
                    found = Some(file);
                    break;
                }
                current += 1;
            }
            Ok(())
        })
    })?;
    Ok(found)
}

/// Refresh root-directory object count and storage statistics.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_refresh() -> bool {
    let mut count = 0u32;
    let mut total = 0u32;
    let mut free = 0u32;
    let result = xcore407i_blinker::nand_fs::with_mounted(|fs| {
        total = fs.total_space().min(u32::MAX as usize) as u32;
        free = fs.available_space().unwrap_or(0).min(u32::MAX as usize) as u32;
        fs.read_dir_and_then(&littlefs2::path!("/"), |directory| -> littlefs2::io::Result<()> {
            // Ignore individual directory-read errors here; the enclosing
            // read_dir_and_then still reports open/close failures.
            for entry in directory.flatten() {
                if !entry.file_type().is_file() {
                    continue;
                }
                let path = entry.path().as_str_ref_with_trailing_nul();
                let bytes = path.as_bytes();
                if bytes.len() >= 3 && bytes[0] == b'/' && bytes[bytes.len() - 1] == 0
                    && supported_name(&bytes[1..bytes.len() - 1]) {
                    count = count.saturating_add(1);
                }
            }
            Ok(())
        })
    });
    if result.is_ok() {
        FILE_COUNT.store(count, Ordering::Release);
        STORAGE_TOTAL.store(total, Ordering::Release);
        STORAGE_FREE.store(free, Ordering::Release);
        true
    } else {
        FILE_COUNT.store(0, Ordering::Release);
        STORAGE_TOTAL.store(0, Ordering::Release);
        STORAGE_FREE.store(0, Ordering::Release);
        false
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_object_count() -> u32 {
    FILE_COUNT.load(Ordering::Acquire)
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

/// Copy an object's ASCII name as UTF-16 and return its size.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_object_info(
    handle: u32,
    name_out: *mut u16,
    max_name_units: u32,
    size_out: *mut u32,
) -> bool {
    if handle == 0 {
        return false;
    }
    let file = match file_at(handle - 1) {
        Ok(Some(file)) => file,
        _ => return false,
    };
    if !size_out.is_null() {
        unsafe { size_out.write(file.size); }
    }
    if !name_out.is_null() && max_name_units != 0 {
        let count = file.name_len.min(max_name_units as usize - 1);
        for index in 0..count {
            unsafe { name_out.add(index).write(file.name[index] as u16); }
        }
        unsafe { name_out.add(count).write(0); }
    }
    true
}

/// Read one bounded chunk of an object directly from LittleFS.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_read_object_chunk(
    handle: u32,
    offset: u32,
    destination: *mut u8,
    capacity: u32,
) -> i32 {
    if handle == 0 || destination.is_null() || capacity == 0 {
        return -1;
    }
    let file = match file_at(handle - 1) {
        Ok(Some(file)) => file,
        _ => return -1,
    };
    if offset > file.size {
        return -1;
    }
    let name = match core::str::from_utf8(&file.name[..file.name_len]) {
        Ok(name) => name,
        Err(_) => return -1,
    };
    let relative = match littlefs2::path::PathBuf::try_from(name) {
        Ok(path) => path,
        Err(_) => return -1,
    };
    let full_path = littlefs2::path!("/").join(relative.as_path());
    let output = unsafe { core::slice::from_raw_parts_mut(destination, capacity as usize) };
    let mut read_count = 0usize;
    let result = xcore407i_blinker::nand_fs::with_mounted(|fs| {
        fs.open_file_and_then(&full_path, |input| {
            input.seek(littlefs2::io::SeekFrom::Start(offset))?;
            read_count = input.read(output)?;
            Ok(())
        })
    });
    if result.is_ok() { read_count as i32 } else { -1 }
}

/// Return the existing handle for a name, or the next handle for a new file.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_find_object(name: *const u8, name_len: u32) -> u32 {
    if name.is_null() || name_len == 0 || name_len as usize > MAX_NAME {
        return 0;
    }
    let bytes = unsafe { core::slice::from_raw_parts(name, name_len as usize) };
    if !supported_name(bytes) {
        return 0;
    }
    let count = FILE_COUNT.load(Ordering::Acquire);
    for index in 0..count {
        match file_at(index) {
            Ok(Some(file)) if file.name_len == bytes.len()
                && &file.name[..file.name_len] == bytes => return index + 1,
            Ok(_) => {}
            Err(_) => return 0,
        }
    }
    count.saturating_add(1)
}

/// Begin a streamed upload using a temporary file; an existing destination is
/// preserved until the completed temporary file is renamed into place.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_begin_upload(name: *const u8, name_len: u32, expected_size: u32) -> bool {
    if name.is_null() || name_len == 0 || name_len as usize > MAX_NAME {
        return false;
    }
    let bytes = unsafe { core::slice::from_raw_parts(name, name_len as usize) };
    if !supported_name(bytes) {
        return false;
    }
    if unsafe { UPLOAD_ACTIVE } {
        xcore_mtp_abort_upload();
    }

    let result = xcore407i_blinker::nand_fs::with_mounted(|fs| {
        let temp = &littlefs2::path!("/.mtp-upload.tmp");
        if fs.exists(temp) {
            fs.remove(temp)?;
        }
        if expected_size as usize > fs.total_space()
            || expected_size as usize > fs.available_space().unwrap_or(0) {
            return Err(littlefs2::io::Error::IO);
        }
        fs.create_file_and_then(temp, |_| Ok(()))
    });
    if result.is_err() {
        return false;
    }

    let target = core::ptr::addr_of_mut!(UPLOAD_NAME).cast::<u8>();
    for index in 0..name_len as usize {
        unsafe { target.add(index).write(bytes[index]); }
    }
    unsafe {
        UPLOAD_NAME_LEN = name_len as usize;
        UPLOAD_EXPECTED = expected_size;
        UPLOAD_ACTIVE = true;
    }
    true
}

/// Persist one MTP payload packet at its absolute file offset.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_write_upload_chunk(offset: u32, data: *const u8, len: u32) -> bool {
    if data.is_null() || len == 0 {
        return len == 0;
    }
    let (active, expected) = unsafe { (UPLOAD_ACTIVE, UPLOAD_EXPECTED) };
    if !active || offset > expected || len > expected - offset {
        return false;
    }
    let input = unsafe { core::slice::from_raw_parts(data, len as usize) };
    xcore407i_blinker::nand_fs::with_mounted(|fs| {
        fs.open_file_with_options_and_then(
            |options| options.write(true),
            &littlefs2::path!("/.mtp-upload.tmp"),
            |file| {
                file.seek(littlefs2::io::SeekFrom::Start(offset))?;
                file.write_all(input)?;
                file.flush()
            },
        )
    }).is_ok()
}

/// Atomically publish the completed temporary file under its requested name.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_finish_upload() -> bool {
    let (active, name_len, expected) = unsafe {
        (UPLOAD_ACTIVE, UPLOAD_NAME_LEN, UPLOAD_EXPECTED)
    };
    if !active || name_len == 0 {
        return false;
    }
    let name_ptr = core::ptr::addr_of!(UPLOAD_NAME).cast::<u8>();
    let mut name_buf = [0u8; MAX_NAME];
    for index in 0..name_len {
        name_buf[index] = unsafe { name_ptr.add(index).read() };
    }
    let name = match core::str::from_utf8(&name_buf[..name_len]) {
        Ok(name) => name,
        Err(_) => return false,
    };
    let relative = match littlefs2::path::PathBuf::try_from(name) {
        Ok(path) => path,
        Err(_) => return false,
    };
    let destination = littlefs2::path!("/").join(relative.as_path());
    let result = xcore407i_blinker::nand_fs::with_mounted(|fs| {
        let metadata = fs.metadata(&littlefs2::path!("/.mtp-upload.tmp"))?;
        if metadata.len() != expected as usize {
            return Err(littlefs2::io::Error::IO);
        }
        fs.rename(&littlefs2::path!("/.mtp-upload.tmp"), &destination)
    });
    if result.is_ok() {
        unsafe { UPLOAD_ACTIVE = false; }
        xcore_mtp_refresh();
        true
    } else {
        xcore_mtp_abort_upload();
        false
    }
}

/// Abandon a pending upload without modifying its destination file.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_abort_upload() {
    if unsafe { UPLOAD_ACTIVE } {
        let _ = xcore407i_blinker::nand_fs::with_mounted(|fs| {
            let temp = &littlefs2::path!("/.mtp-upload.tmp");
            if fs.exists(temp) { fs.remove(temp) } else { Ok(()) }
        });
    }
    unsafe {
        UPLOAD_ACTIVE = false;
        UPLOAD_NAME_LEN = 0;
        UPLOAD_EXPECTED = 0;
    }
}
