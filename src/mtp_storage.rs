use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

const MAX_OBJECTS: usize = 16;
const MAX_NAME: usize = 64;
const MTP_READ_LIMIT: usize = xcore407i_blinker::camera::CANVAS_MAIN_BYTES - 1_024;

#[derive(Clone, Copy)]
struct MtpFile {
    name: [u8; MAX_NAME],
    name_len: u8,
    size: u32,
}

impl MtpFile {
    const EMPTY: Self = Self {
        name: [0; MAX_NAME],
        name_len: 0,
        size: 0,
    };
}

static mut FILES: [MtpFile; MAX_OBJECTS] = [MtpFile::EMPTY; MAX_OBJECTS];
static FILE_COUNT: AtomicU8 = AtomicU8::new(0);
static STORAGE_TOTAL: AtomicU32 = AtomicU32::new(0);
static STORAGE_FREE: AtomicU32 = AtomicU32::new(0);

/// Refresh the bounded root-only MTP view. File contents remain in NAND.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_refresh() -> bool {
    let files = core::ptr::addr_of_mut!(FILES).cast::<MtpFile>();
    for index in 0..MAX_OBJECTS {
        unsafe { files.add(index).write(MtpFile::EMPTY); }
    }

    let mut count = 0usize;
    let mut total = 0u32;
    let mut free = 0u32;
    let result = xcore407i_blinker::nand_fs::with_mounted(|fs| {
        total = fs.total_space().min(u32::MAX as usize) as u32;
        free = fs.available_space().unwrap_or(0).min(u32::MAX as usize) as u32;
        fs.read_dir_and_then(&littlefs2::path!("/"), |directory| {
            for item in directory {
                let entry = item?;
                if !entry.file_type().is_file() || count == MAX_OBJECTS {
                    continue;
                }

                let path = entry.path().as_str_ref_with_trailing_nul();
                let bytes = path.as_bytes();
                if bytes.len() < 3 || bytes[0] != b'/' || bytes[bytes.len() - 1] != 0 {
                    continue;
                }
                let name = &bytes[1..bytes.len() - 1];
                if name.is_empty()
                    || name == b"."
                    || name == b".."
                    || name.len() > MAX_NAME
                    || !name.iter().all(|byte| {
                        byte.is_ascii_alphanumeric()
                            || matches!(*byte, b'_' | b'-' | b'.' | b' ')
                    })
                {
                    continue;
                }

                let mut file = MtpFile::EMPTY;
                file.name[..name.len()].copy_from_slice(name);
                file.name_len = name.len() as u8;
                file.size = entry.metadata().len().min(u32::MAX as usize) as u32;
                unsafe { files.add(count).write(file); }
                count += 1;
            }
            Ok(())
        })
    });

    if result.is_ok() {
        STORAGE_TOTAL.store(total, Ordering::Release);
        STORAGE_FREE.store(free, Ordering::Release);
        FILE_COUNT.store(count as u8, Ordering::Release);
        true
    } else {
        STORAGE_TOTAL.store(0, Ordering::Release);
        STORAGE_FREE.store(0, Ordering::Release);
        FILE_COUNT.store(0, Ordering::Release);
        false
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_object_count() -> u32 {
    FILE_COUNT.load(Ordering::Acquire) as u32
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
    if handle == 0 || handle > FILE_COUNT.load(Ordering::Acquire) as u32 {
        return false;
    }
    let file = unsafe {
        core::ptr::addr_of!(FILES)
            .cast::<MtpFile>()
            .add((handle - 1) as usize)
            .read()
    };
    if file.name_len == 0 {
        return false;
    }
    if !size_out.is_null() {
        unsafe { size_out.write(file.size); }
    }
    if !name_out.is_null() && max_name_units != 0 {
        let count = (file.name_len as usize).min(max_name_units as usize - 1);
        for index in 0..count {
            unsafe { name_out.add(index).write(file.name[index] as u16); }
        }
        unsafe { name_out.add(count).write(0); }
    }
    true
}

/// Read one complete object into the caller-provided MTP workspace.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_load_object(
    handle: u32,
    destination: *mut u8,
    capacity: u32,
) -> i32 {
    if destination.is_null() || handle == 0
        || handle > FILE_COUNT.load(Ordering::Acquire) as u32
    {
        return -1;
    }

    let file = unsafe {
        core::ptr::addr_of!(FILES)
            .cast::<MtpFile>()
            .add((handle - 1) as usize)
            .read()
    };
    let size = file.size as usize;
    if file.name_len == 0 || size > MTP_READ_LIMIT || size > capacity as usize {
        return -1;
    }

    let name = match core::str::from_utf8(&file.name[..file.name_len as usize]) {
        Ok(name) => name,
        Err(_) => return -1,
    };
    let relative = match littlefs2::path::PathBuf::try_from(name) {
        Ok(path) => path,
        Err(_) => return -1,
    };
    let full_path = littlefs2::path!("/").join(relative.as_path());
    let output = unsafe { core::slice::from_raw_parts_mut(destination, size) };
    let mut total_read = 0usize;

    let result = xcore407i_blinker::nand_fs::with_mounted(|fs| {
        fs.open_file_and_then(&full_path, |file| {
            while total_read < size {
                let count = file.read(&mut output[total_read..])?;
                if count == 0 {
                    return Err(littlefs2::io::Error::IO);
                }
                total_read += count;
            }
            Ok(())
        })
    });

    if result.is_ok() && total_read == size {
        size as i32
    } else {
        -1
    }
}


const MTP_UPLOAD_TEMP: &str = "/.mtp-upload.tmp";
static mut UPLOAD_NAME: [u8; MAX_NAME] = [0; MAX_NAME];
static mut UPLOAD_NAME_LEN: usize = 0;
static mut UPLOAD_EXPECTED: u32 = 0;
static mut UPLOAD_ACTIVE: bool = false;

/// Start a streamed upload into a temporary LittleFS file. The existing
/// destination is not touched until xcore_mtp_finish_upload succeeds.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_begin_upload(name: *const u8, name_len: u32, expected_size: u32) -> bool {
    if name.is_null() || name_len == 0 || name_len as usize > MAX_NAME {
        return false;
    }
    let bytes = unsafe { core::slice::from_raw_parts(name, name_len as usize) };
    if !bytes.iter().all(|byte| {
        byte.is_ascii_alphanumeric() || matches!(*byte, b'_' | b'-' | b'.' | b' ')
    }) || bytes == b"." || bytes == b".." || bytes.starts_with(b".mtp-") {
        return false;
    }

    let mut total = 0u32;
    let mut free = 0u32;
    let result = xcore407i_blinker::nand_fs::with_mounted(|fs| {
        total = fs.total_space().min(u32::MAX as usize) as u32;
        free = fs.available_space().unwrap_or(0).min(u32::MAX as usize) as u32;
        if expected_size > total || expected_size > free {
            return Err(littlefs2::io::Error::IO);
        }
        fs.create_file_and_then(&littlefs2::path!("/.mtp-upload.tmp"), |_| Ok(()))
    });
    if result.is_err() {
        return false;
    }

    let target = unsafe { core::ptr::addr_of_mut!(UPLOAD_NAME).cast::<u8>() };
    for index in 0..name_len as usize {
        unsafe { target.add(index).write(bytes[index]); }
    }
    unsafe {
        UPLOAD_NAME_LEN = name_len as usize;
        UPLOAD_EXPECTED = expected_size;
        UPLOAD_ACTIVE = true;
    }
    let _ = (total, free);
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
                file.seek(littlefs2::io::SeekFrom::Start(offset as u64))?;
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
    let mut name = [0u8; MAX_NAME + 1];
    for index in 0..name_len {
        name[index] = unsafe { name_ptr.add(index).read() };
    }
    let name = match core::str::from_utf8(&name[..name_len]) {
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
    unsafe { UPLOAD_ACTIVE = false; }
    if result.is_ok() {
        xcore_mtp_refresh();
        true
    } else {
        false
    }
}

/// Abandon the pending upload without changing the destination file.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_mtp_abort_upload() {
    if unsafe { UPLOAD_ACTIVE } {
        let _ = xcore407i_blinker::nand_fs::with_mounted(|fs| {
            fs.remove(&littlefs2::path!("/.mtp-upload.tmp"))
        });
    }
    unsafe {
        UPLOAD_ACTIVE = false;
        UPLOAD_NAME_LEN = 0;
        UPLOAD_EXPECTED = 0;
    }
}
