use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, Ordering};

use xcore407i_blinker::camera::{clear_demo_canvas, show_file_browser_line, show_usb_status};

pub(crate) const FILE_BROWSER_OPEN: u8 = 1;
pub(crate) const FILE_BROWSER_UP: u8 = 2;
pub(crate) const FILE_BROWSER_DOWN: u8 = 3;
pub(crate) const FILE_BROWSER_ENTER: u8 = 4;
pub(crate) const FILE_BROWSER_ESCAPE: u8 = 5;
pub(crate) const SAVE_DIALOG_IDLE: u8 = 0;
pub(crate) const SAVE_DIALOG_INPUT: u8 = 1;
pub(crate) const SAVE_DIALOG_CHECK: u8 = 2;
pub(crate) const SAVE_DIALOG_CONFIRM: u8 = 3;
pub(crate) const SAVE_DIALOG_WRITE: u8 = 4;
pub(crate) const SAVE_RESULT_CANCELLED: u8 = 1;
pub(crate) const SAVE_RESULT_OK: u8 = 2;
pub(crate) const SAVE_RESULT_ERROR: u8 = 3;
pub(crate) const FORMAT_IDLE: u8 = 0;
pub(crate) const FORMAT_CONFIRM: u8 = 1;
pub(crate) const FORMAT_CONFIRM_FINAL: u8 = 2;
pub(crate) const FORMAT_REQUEST: u8 = 3;
pub(crate) const FORMAT_CANCELLED: u8 = 4;
pub(crate) const FORMAT_DONE: u8 = 5;
pub(crate) const FORMAT_FAILED: u8 = 6;
pub(crate) static FORMAT_STATE: AtomicU8 = AtomicU8::new(FORMAT_IDLE);
pub(crate) static FORMAT_ERROR: AtomicI32 = AtomicI32::new(0);
pub(crate) const INIT_IDLE: u8 = 0;
pub(crate) const INIT_CONFIRM: u8 = 1;
pub(crate) const INIT_CONFIRM_FINAL: u8 = 2;
pub(crate) const INIT_REQUEST: u8 = 3;
pub(crate) const INIT_CANCELLED: u8 = 4;
pub(crate) const INIT_DONE: u8 = 5;
pub(crate) const INIT_FAILED: u8 = 6;
pub(crate) static INIT_STATE: AtomicU8 = AtomicU8::new(INIT_IDLE);
pub(crate) const SAVE_RAM_START: usize = 0x0200;
pub(crate) const SAVE_RAM_NORMAL_LENGTH: usize = 0x3E00; // 0x0200..0x3FFF
pub(crate) const SAVE_RAM_EXTENDED_LENGTH: usize = 0x6E00; // 0x0200..0x6FFF
pub(crate) const MAX_BROWSER_FILENAME: usize = 30; // includes the .bin extension
pub(crate) const SAVE_NAME_CAPACITY: usize = MAX_BROWSER_FILENAME - 4;
pub(crate) const MAX_BROWSER_FILES: usize = 30;
pub(crate) const MAX_BROWSER_PATH: usize = MAX_BROWSER_FILENAME + 2; // '/' + name + NUL

// Keep the browser's file table in ordinary .bss, not on the limited stack.
#[derive(Clone, Copy)]
pub(crate) struct BrowserFile {
    pub(crate) path: [u8; MAX_BROWSER_PATH],
    pub(crate) len: u8,
}

impl BrowserFile {
    const EMPTY: Self = Self {
        path: [0; MAX_BROWSER_PATH],
        len: 0,
    };
}

pub(crate) static mut BROWSER_FILES: [BrowserFile; MAX_BROWSER_FILES] =
    [BrowserFile::EMPTY; MAX_BROWSER_FILES];

// Shared by normal and CDC modes; their filesystem operations are serialized.
static mut FILE_BROWSER_FILE_ALLOCATION: core::mem::MaybeUninit<
    littlefs2::fs::FileAllocation<xcore407i_blinker::nand_fs::ActiveStorage>,
> = core::mem::MaybeUninit::uninit();

fn with_file_browser_file_allocation<R>(
    operation: impl FnOnce(
        &mut littlefs2::fs::FileAllocation<xcore407i_blinker::nand_fs::ActiveStorage>,
    ) -> R,
) -> R {
    // Normal and CDC modes are mutually exclusive and file operations run in
    // the main context, so this shared workspace has only one active user.
    unsafe {
        let allocation_ptr = core::ptr::addr_of_mut!(FILE_BROWSER_FILE_ALLOCATION)
            .cast::<littlefs2::fs::FileAllocation<xcore407i_blinker::nand_fs::ActiveStorage>>();
        allocation_ptr.write(littlefs2::fs::FileAllocation::new());
        operation(&mut *allocation_ptr)
    }
}
pub(crate) static FILE_BROWSER_ACTIVE: AtomicBool = AtomicBool::new(false);
pub(crate) static FILE_BROWSER_ACTION: AtomicU8 = AtomicU8::new(0);
pub(crate) static SAVE_DIALOG_STATE: AtomicU8 = AtomicU8::new(SAVE_DIALOG_IDLE);
pub(crate) static SAVE_DIALOG_DIRTY: AtomicBool = AtomicBool::new(false);
pub(crate) static SAVE_DIALOG_RESULT: AtomicU8 = AtomicU8::new(0);
pub(crate) static SAVE_ERROR: AtomicI32 = AtomicI32::new(0);
pub(crate) static SAVE_OVERWRITE: AtomicBool = AtomicBool::new(false);
pub(crate) static mut SAVE_FILENAME: [u8; SAVE_NAME_CAPACITY] = [0; SAVE_NAME_CAPACITY];
pub(crate) static mut SAVE_FILENAME_LEN: usize = 0;

/// Cancel every host-side modal operation associated with the BK emulator.
/// This deliberately leaves the NAND filesystem and USB transports intact.
pub(crate) fn reset_emulator_ui() {
    FILE_BROWSER_ACTIVE.store(false, Ordering::Release);
    FILE_BROWSER_ACTION.store(0, Ordering::Release);
    SAVE_DIALOG_STATE.store(SAVE_DIALOG_IDLE, Ordering::Release);
    SAVE_DIALOG_DIRTY.store(false, Ordering::Release);
    SAVE_DIALOG_RESULT.store(0, Ordering::Release);
    SAVE_ERROR.store(0, Ordering::Release);
    SAVE_OVERWRITE.store(false, Ordering::Release);
    unsafe {
        core::ptr::addr_of_mut!(SAVE_FILENAME).cast::<u8>().write_bytes(0, SAVE_NAME_CAPACITY);
        SAVE_FILENAME_LEN = 0;
    }
    FORMAT_STATE.store(FORMAT_IDLE, Ordering::Release);
    FORMAT_ERROR.store(0, Ordering::Release);
    INIT_STATE.store(INIT_IDLE, Ordering::Release);
}
 
unsafe extern "C" {
    safe fn bk_load_bin_prepare(load_addr: u16, data_len: u16) -> i32;
    safe fn bk_load_bin_write(load_addr: u16, offset: u32, data: *const u8, length: usize) -> i32;
    safe fn bk_load_bin_finish(load_addr: u16, data_len: u16) -> i32;
    static mut RamBuffer: [u8; 0x4000];
}

/// Compare root-level filenames lexicographically without regard to ASCII case.
fn compare_browser_names(
    left: &[u8; MAX_BROWSER_PATH],
    left_len: u8,
    right: &[u8; MAX_BROWSER_PATH],
    right_len: u8,
) -> core::cmp::Ordering {
    let left = &left[1..left_len as usize - 1];
    let right = &right[1..right_len as usize - 1];
    for (&a, &b) in left.iter().zip(right.iter()) {
        match a.to_ascii_lowercase().cmp(&b.to_ascii_lowercase()) {
            core::cmp::Ordering::Equal => {}
            ordering => return ordering,
        }
    }
    left.len().cmp(&right.len())
}

/// Sort complete entries in place; the comparator preserves ASCII case-insensitive order.
fn sort_browser_files(files: &mut [BrowserFile], count: usize) {
    files[..count].sort_unstable_by(|left, right| {
        compare_browser_names(&left.path, left.len, &right.path, right.len)
    });
}

/// Read up to MAX_BROWSER_FILES regular files from the LittleFS root directory.
/// Paths are retained as NUL-terminated ASCII strings for the later load.
pub(crate) fn list_root_files(
    files: &mut [BrowserFile; MAX_BROWSER_FILES],
    progress: impl FnMut(u32, u32),
) -> Result<usize, xcore407i_blinker::nand_fs::FsAccessError> {
    xcore407i_blinker::nand_fs::with_mounted_detailed_with_progress(|fs| {
        let mut count = 0usize;
        xcore407i_blinker::nand_fs::read_root_directory(fs, false, |path_bytes, is_dir, _size| {
            if is_dir || count == MAX_BROWSER_FILES {
                return Ok(());
            }
            if path_bytes.len() < 3
                || path_bytes.len() > MAX_BROWSER_PATH
                || !path_bytes[..path_bytes.len() - 1]
                    .iter()
                    .all(|byte| byte.is_ascii_graphic() || *byte == b' ')
            {
                return Ok(());
            }

            // Only expose .bin files in the browser (case-insensitive).
            let filename = &path_bytes[1..path_bytes.len() - 1];
            if filename.len() < 4
                || filename.len() > MAX_BROWSER_FILENAME
                || !filename[filename.len() - 4..].eq_ignore_ascii_case(b".bin")
            {
                return Ok(());
            }

            files[count].path[..path_bytes.len()].copy_from_slice(path_bytes);
            files[count].len = path_bytes.len() as u8;
            count += 1;
            Ok(())
        })?;
        sort_browser_files(files, count);
        Ok(count)
    }, progress)
}

pub(crate) fn draw_file_browser(
    files: &[BrowserFile; MAX_BROWSER_FILES],
    count: usize,
    selected: usize,
) {
    clear_demo_canvas(0);
    show_file_browser_line(0, b"NAND FILES - SELECT PROGRAM", false);

    for row in 0..MAX_BROWSER_FILES {
        let mut line = [b' '; 64];
        if row < count {
            line[0] = if row == selected { b'>' } else { b' ' };
            line[1] = b' ';
            let path_len = files[row].len as usize;
            // Root paths have the form "/filename\\0".
            let name = &files[row].path[1..path_len - 1];
            let copy_len = name.len().min(line.len() - 2);
            line[2..2 + copy_len].copy_from_slice(&name[..copy_len]);
        } else if row == 0 && count == 0 {
            line[..8].copy_from_slice(b"NO FILES");
        }
        show_file_browser_line(row + 1, &line, row == selected && count != 0);
    }

    show_file_browser_line(31, b"\x18/\x19 MOVE  ENTER LOAD  ESC CANCEL", false);
}

/// Build a root-level filename from the name entered in the F3 save dialog.
pub(crate) unsafe fn build_save_path(path: &mut [u8; MAX_BROWSER_PATH]) -> usize {
    let name_len = unsafe { SAVE_FILENAME_LEN };
    if name_len == 0 || name_len > SAVE_NAME_CAPACITY {
        return 0;
    }
    let total = 1 + name_len + 4 + 1;
    if total > path.len() {
        return 0;
    }
    path[0] = b'/';
    unsafe {
        core::ptr::copy_nonoverlapping(
            core::ptr::addr_of!(SAVE_FILENAME).cast::<u8>(),
            path.as_mut_ptr().add(1),
            name_len,
        );
    }
    path[1 + name_len..1 + name_len + 4].copy_from_slice(b".bin");
    path[total - 1] = 0;
    total
}

pub(crate) fn save_file_exists(path_bytes: &[u8]) -> bool {
    let Ok(path) = littlefs2::path::Path::from_bytes_with_nul(path_bytes) else {
        return false;
    };
    xcore407i_blinker::nand_fs::with_mounted(|fs| Ok(fs.exists(path)))
        .unwrap_or(false)
}

/// Save the complete non-video user RAM range in the loader's 4-byte
/// little-endian Start/Size format, without allocating a second RAM image.
pub(crate) fn save_ram_to_path(
    path_bytes: &[u8],
    overwrite: bool,
) -> Result<(), xcore407i_blinker::nand_fs::FsAccessError> {
    // Normal mode exposes RAM through 0x3FFF. Extended mode adds
    // 12 KiB at 0x4000..0x6FFF, backed by the hidden camera-canvas storage.
    let extended_mode = xcore407i_blinker::emulator::bk_screen::extended_memory_enabled();
    let save_length = if extended_mode {
        SAVE_RAM_EXTENDED_LENGTH
    } else {
        SAVE_RAM_NORMAL_LENGTH
    };
    let path = littlefs2::path::Path::from_bytes_with_nul(path_bytes)
        .map_err(|_| xcore407i_blinker::nand_fs::FsAccessError::Operation(-22))?;
    let mut already_exists = false;
    let result = xcore407i_blinker::nand_fs::with_mounted_detailed(|fs| {
        if fs.exists(path) && !overwrite {
            already_exists = true;
            return Err(littlefs2::io::Error::IO);
        }

        with_file_browser_file_allocation(|allocation| {
            fs.open_file_with_options_and_then_with_allocation(
            |options| options.write(true).create(true).truncate(true),
            path,
            allocation,
            |file| {
                let start = (SAVE_RAM_START as u16).to_le_bytes();
                let size = (save_length as u16).to_le_bytes();
                let header = [start[0], start[1], size[0], size[1]];
                let mut header_written = 0usize;
                while header_written < header.len() {
                    let count = file.write(&header[header_written..])?;
                    if count == 0 {
                        return Err(littlefs2::io::Error::IO);
                    }
                    header_written += count;
                }

                // The first part lives in RamBuffer; the extended 12 KiB
                // window is backed by the hidden camera-canvas storage.
                let base_length = 0x4000 - SAVE_RAM_START;
                let ram = unsafe {
                    core::slice::from_raw_parts(
                        core::ptr::addr_of!(RamBuffer).cast::<u8>().add(SAVE_RAM_START),
                        base_length,
                    )
                };
                let mut total = 0usize;
                while total < ram.len() {
                    let count = file.write(&ram[total..])?;
                    if count == 0 {
                        return Err(littlefs2::io::Error::IO);
                    }
                    total += count;
                }

                if extended_mode {
                    let mut extended = [0u8; 128];
                    let mut offset = 0usize;
                    while offset < 0x3000 {
                        let count = (0x3000 - offset).min(extended.len());
                        for (index, byte) in extended[..count].iter_mut().enumerate() {
                            *byte = xcore407i_blinker::camera::bk_extended_ram_read(offset + index);
                        }
                        let mut written = 0usize;
                        while written < count {
                            let amount = file.write(&extended[written..count])?;
                            if amount == 0 {
                                return Err(littlefs2::io::Error::IO);
                            }
                            written += amount;
                        }
                        offset += count;
                    }
                }
                Ok(())
            },
        )
        })
    });
    if already_exists {
        Err(xcore407i_blinker::nand_fs::FsAccessError::Operation(-17))
    } else {
        result
    }
}

/// Draw the F3 filename entry or overwrite confirmation screen.
pub(crate) unsafe fn draw_save_dialog(state: u8) {
    clear_demo_canvas(0);
    show_file_browser_line(0, b"SAVE RAM IMAGE", false);
    if state == SAVE_DIALOG_CONFIRM {
        let mut line = [b' '; 64];
        line[..6].copy_from_slice(b"FILE: ");
        let len = unsafe { SAVE_FILENAME_LEN }.min(SAVE_NAME_CAPACITY);
        let filename = unsafe {
            core::slice::from_raw_parts(
                core::ptr::addr_of!(SAVE_FILENAME).cast::<u8>(),
                len,
            )
        };
        line[6..6 + len].copy_from_slice(filename);
        line[6 + len..10 + len].copy_from_slice(b".bin");
        show_file_browser_line(2, &line, false);
        show_file_browser_line(4, b"FILE EXISTS - OVERWRITE? Y/N", true);
        show_file_browser_line(27, b"Y OVERWRITE  N/ESC CANCEL", false);
    } else {
        show_file_browser_line(2, b"ENTER FILE NAME (NO EXTENSION)", false);
        let mut line = [b' '; 64];
        line[..6].copy_from_slice(b"FILE: ");
        let len = unsafe { SAVE_FILENAME_LEN }.min(SAVE_NAME_CAPACITY);
        let filename = unsafe {
            core::slice::from_raw_parts(
                core::ptr::addr_of!(SAVE_FILENAME).cast::<u8>(),
                len,
            )
        };
        line[6..6 + len].copy_from_slice(filename);
        line[6 + len..10 + len].copy_from_slice(b".bin");
        show_file_browser_line(4, &line, true);
        show_file_browser_line(27, b"ENTER SAVE  BACKSPACE EDIT  ESC CANCEL", false);
    }
}

/// Load a selected .bin file using the BK monitor's tape-image conventions.
pub(crate) fn load_tetris_from_path(path_bytes: &[u8]) -> Result<(), ()> {
    let path = littlefs2::path::Path::from_bytes_with_nul(path_bytes).map_err(|_| ())?;
    xcore407i_blinker::nand_fs::with_mounted(|fs| {
        with_file_browser_file_allocation(|allocation| {
            fs.open_file_with_options_and_then_with_allocation(
                |options| options.read(true),
                path,
                allocation,
                |file| {
                    let file_size = file.len()?;
                    if file_size < 5 {
                        return Err(littlefs2::io::Error::IO);
                    }

                    let mut header = [0u8; 4];
                    let mut header_read = 0usize;
                    while header_read < header.len() {
                        let count = file.read(&mut header[header_read..])?;
                        if count == 0 {
                            return Err(littlefs2::io::Error::IO);
                        }
                        header_read += count;
                    }

                    let load_addr = u16::from_le_bytes([header[0], header[1]]);
                    let data_len = u16::from_le_bytes([header[2], header[3]]) as usize;

                    // This browser targets BK-0010 short-format images: a
                    // four-byte address/length header followed by the payload.
                    // Some short images carry a two-byte trailing checksum.
                    // Do not guess a long-header offset or silently truncate a
                    // payload when the declared length disagrees with the file.
                    let payload_end = 4usize.checked_add(data_len)
                        .ok_or(littlefs2::io::Error::IO)?;
                    if file_size != payload_end && file_size != payload_end + 2 {
                        return Err(littlefs2::io::Error::IO);
                    }

                    // Validate the complete destination range before writing
                    // any payload bytes. BK-0010 writable RAM + video memory
                    // occupies 000000..0100000 (octal).
                    if (load_addr as usize) + data_len > 0o100000 {
                        return Err(littlefs2::io::Error::IO);
                    }
                    // The complete F3 image (0x0200..0x6FFF) is recognized by
                    // bk_load_bin_prepare(), which selects extended memory before
                    // the payload reaches the 0x4000 boundary.

                    // The BK monitor stores the received header in system
                    // cells 264/266 before receiving the payload. Keep that
                    // ordering so payload bytes targeting those cells win.
                    if bk_load_bin_prepare(load_addr, data_len as u16) != 0 {
                        return Err(littlefs2::io::Error::IO);
                    }

                    let mut remaining = data_len;
                    let mut written = 0u32;
                    while remaining != 0 {
                        let wanted = remaining.min(header.len());
                        let count = file.read(&mut header[..wanted])?;
                        if count == 0 {
                            return Err(littlefs2::io::Error::IO);
                        }
                        if bk_load_bin_write(load_addr, written, header.as_ptr(), count) != 0 {
                            return Err(littlefs2::io::Error::IO);
                        }
                        written += count as u32;
                        remaining -= count;
                    }

                    if bk_load_bin_finish(load_addr, data_len as u16) < 0 {
                        return Err(littlefs2::io::Error::IO);
                    }
                    Ok(())
                },
            )
        })
    })
    .map_err(|_| ())
}


/// Display the exact LittleFS mount failure and the two-step destructive confirmation.
pub(crate) fn draw_format_confirmation(final_confirmation: bool, error: i32) {
    clear_demo_canvas(0);
    show_file_browser_line(0, b"LITTLEFS MOUNT FAILED", true);
    show_file_browser_line(3, b"STAGE: LITTLEFS MOUNT", false);
    let detail: &[u8] = match error {
        -5 => b"LFS_ERR_IO (-5): device I/O failure",
        -84 => b"LFS_ERR_CORRUPTION (-84): invalid filesystem",
        -2 => b"LFS_ERR_NO_SUCH_ENTRY (-2)",
        -17 => b"LFS_ERR_ENTRY_ALREADY_EXISTED (-17)",
        -20 => b"LFS_ERR_PATH_NOT_DIR (-20)",
        -21 => b"LFS_ERR_PATH_IS_DIR (-21)",
        -39 => b"LFS_ERR_DIR_NOT_EMPTY (-39)",
        -9 => b"LFS_ERR_BAD_FILE_DESCRIPTOR (-9)",
        -27 => b"LFS_ERR_FILE_TOO_BIG (-27)",
        -22 => b"LFS_ERR_INVALID (-22)",
        -28 => b"LFS_ERR_NO_SPACE (-28)",
        -12 => b"LFS_ERR_NO_MEMORY (-12)",
        -61 => b"LFS_ERR_NO_ATTRIBUTE (-61)",
        -36 => b"LFS_ERR_FILENAME_TOO_LONG (-36)",
        _ => b"",
    };
    if !detail.is_empty() {
        show_file_browser_line(4, detail, false);
    } else {
        let mut line = [b' '; 64];
        let prefix = b"LFS ERROR CODE: ";
        line[..prefix.len()].copy_from_slice(prefix);
        let value = if error < 0 { error.wrapping_neg() as u32 } else { error as u32 };
        let mut number = [0u8; 10];
        let digits = xcore407i_blinker::format::u32_to_decimal(value, &mut number);
        let mut len = prefix.len();
        if error < 0 {
            line[len] = b'-';
            len += 1;
        }
        line[len..len + digits.len()].copy_from_slice(digits);
        len += digits.len();
        show_file_browser_line(4, &line[..len], false);
    }
    if final_confirmation {
        show_file_browser_line(7, b"ARE YOU REALLY SURE?", true);
        show_file_browser_line(9, b"ALL LITTLEFS FILES WILL BE LOST", false);
        show_file_browser_line(27, b"Y FORMAT NOW  N/ESC CANCEL", false);
    } else {
        show_file_browser_line(7, b"FORMAT THE LITTLEFS VOLUME?", true);
        show_file_browser_line(9, b"THIS DELETES ALL FILES", false);
        show_file_browser_line(27, b"Y CONTINUE  N/ESC CANCEL", false);
    }
}


/// Display the two-step confirmation for first-time NAND provisioning.
pub(crate) fn draw_init_confirmation(final_confirmation: bool) {
    clear_demo_canvas(0);
    show_file_browser_line(0, b"MANUAL NAND REINITIALIZATION", true);
    show_file_browser_line(3, b"ERASES EVEN A VALID DHARA VOLUME", false);
    if final_confirmation {
        show_file_browser_line(7, b"ERASE AND INITIALIZE NAND?", true);
        show_file_browser_line(9, b"ALL DATA ON NAND WILL BE LOST", false);
        show_file_browser_line(11, b"THIS CANNOT BE UNDONE", false);
        show_file_browser_line(27, b"Y INITIALIZE  N/ESC CANCEL", false);
    } else {
        show_file_browser_line(7, b"REINITIALIZE NAND STORAGE?", true);
        show_file_browser_line(9, b"ERASES ALL USABLE NAND BLOCKS", false);
        show_file_browser_line(27, b"Y CONTINUE  N/ESC CANCEL", false);
    }
}

pub(crate) fn process_save_dialog() {
    let save_state = SAVE_DIALOG_STATE.load(Ordering::Acquire);
if save_state == SAVE_DIALOG_CHECK {
    show_file_browser_line(27, b"F3: BUILDING SAVE PATH", false);
    let mut path = [0u8; MAX_BROWSER_PATH];
    let path_len = unsafe { build_save_path(&mut path) };
    if path_len == 0 {
        show_file_browser_line(27, b"F3: INVALID SAVE PATH", false);
        SAVE_DIALOG_STATE.store(SAVE_DIALOG_IDLE, Ordering::Release);
        SAVE_DIALOG_RESULT.store(SAVE_RESULT_ERROR, Ordering::Release);
    } else {
        show_file_browser_line(27, b"F3: CHECKING FILE ON NAND", false);
        let exists = save_file_exists(&path[..path_len]);
        show_file_browser_line(27, b"F3: FILE CHECK COMPLETE", false);
        if exists {
            SAVE_DIALOG_STATE.store(SAVE_DIALOG_CONFIRM, Ordering::Release);
            SAVE_DIALOG_DIRTY.store(true, Ordering::Release);
        } else {
            SAVE_OVERWRITE.store(false, Ordering::Release);
            SAVE_DIALOG_STATE.store(SAVE_DIALOG_WRITE, Ordering::Release);
        }
    }
}

if SAVE_DIALOG_STATE.load(Ordering::Acquire) == SAVE_DIALOG_WRITE {
    show_file_browser_line(27, b"F3: PREPARING NAND WRITE", false);
    let mut path = [0u8; MAX_BROWSER_PATH];
    let path_len = unsafe { build_save_path(&mut path) };
    let overwrite = SAVE_OVERWRITE.load(Ordering::Acquire);
    show_file_browser_line(27, b"F3: WRITING RAM TO NAND", false);
    let result = if path_len == 0 {
        Err(xcore407i_blinker::nand_fs::FsAccessError::Operation(-22))
    } else {
        save_ram_to_path(&path[..path_len], overwrite)
    };
    let saved = result.is_ok();
    if let Err(error) = result {
        let code = match error {
            xcore407i_blinker::nand_fs::FsAccessError::Init(kind) => match kind {
                xcore407i_blinker::nand_fs::InitError::Timeout => -1001,
                xcore407i_blinker::nand_fs::InitError::WrongDevice => -1002,
                xcore407i_blinker::nand_fs::InitError::MarkerRead => -1003,
                xcore407i_blinker::nand_fs::InitError::MountFailed => -1004,
                xcore407i_blinker::nand_fs::InitError::DharaNoJournal => -1005,
                xcore407i_blinker::nand_fs::InitError::DharaCapacity => -1006,
                xcore407i_blinker::nand_fs::InitError::ProvisionFailed => -1007,
            },
            xcore407i_blinker::nand_fs::FsAccessError::Mount(code) => -2000 + code,
            xcore407i_blinker::nand_fs::FsAccessError::Operation(code) => code,
        };
        SAVE_ERROR.store(code, Ordering::Release);
    } else {
        SAVE_ERROR.store(0, Ordering::Release);
    }
    show_file_browser_line(
        27,
        if saved { b"F3: NAND WRITE COMPLETE" } else { b"F3: SAVE FAILED - SEE ERROR" },
        false,
    );
    SAVE_DIALOG_STATE.store(SAVE_DIALOG_IDLE, Ordering::Release);
    SAVE_DIALOG_RESULT.store(
        if saved { SAVE_RESULT_OK } else { SAVE_RESULT_ERROR },
        Ordering::Release,
    );
}

if SAVE_DIALOG_DIRTY.swap(false, Ordering::AcqRel) {
    let state = SAVE_DIALOG_STATE.load(Ordering::Acquire);
    if state == SAVE_DIALOG_INPUT || state == SAVE_DIALOG_CONFIRM {
        unsafe { draw_save_dialog(state); }
    }
}

match SAVE_DIALOG_RESULT.swap(0, Ordering::AcqRel) {
    SAVE_RESULT_CANCELLED => {
        clear_demo_canvas(0x10);
        show_usb_status(b"SAVE CANCELLED");
    }
    SAVE_RESULT_OK => {
        clear_demo_canvas(0x10);
        show_usb_status(b"RAM SAVED");
    }
    SAVE_RESULT_ERROR => {
        clear_demo_canvas(0x10);
        let error = SAVE_ERROR.load(Ordering::Acquire);
        let message: &[u8] = match error {
            -1001 => b"SAVE FAILED: NAND TIMEOUT",
            -1002 => b"SAVE FAILED: WRONG NAND DEVICE",
            -1003 => b"SAVE FAILED: BAD BLOCK MARKER READ",
            -1004 => b"SAVE FAILED: STORAGE INIT MOUNT",
            -1005 => b"SAVE FAILED: DHARA JOURNAL MISSING",
            -1006 => b"SAVE FAILED: DHARA CAPACITY",
            -1007 => b"SAVE FAILED: DHARA PROVISION",
            -2005 => b"SAVE FAILED: LFS MOUNT I/O",
            -2084 => b"SAVE FAILED: LFS MOUNT CORRUPT",
            -28 => b"SAVE FAILED: NAND/LFS FULL",
            -84 => b"SAVE FAILED: LFS CORRUPTION",
            -5 => b"SAVE FAILED: NAND I/O ERROR",
            -12 => b"SAVE FAILED: OUT OF MEMORY",
            -22 => b"SAVE FAILED: INVALID MODE/PATH",
            -27 => b"SAVE FAILED: FILE TOO LARGE",
            -17 => b"SAVE FAILED: FILE ALREADY EXISTS",
            _ => b"SAVE FAILED: STORAGE ERROR",
        };
        show_usb_status(message);
    }
    _ => {}
}

}

pub(crate) fn process_open_action(
    browser_action: u8,
    browser_open: &mut bool,
    browser_count: &mut usize,
    browser_selected: &mut usize,
    browser_files: &mut [BrowserFile; MAX_BROWSER_FILES],
    service_io: fn(),
) {
if browser_action == FILE_BROWSER_OPEN && !*browser_open {
    // Enter the browser visibly before any NAND/Dhara/LittleFS work.
    // If mounting stalls or faults, the user can distinguish that
    // from a failure to receive F2.
    clear_demo_canvas(0);
    show_file_browser_line(0, b"NAND FILES - SELECT PROGRAM", false);
    show_file_browser_line(3, b"OPENING NAND STORAGE...", true);
    show_file_browser_line(5, b"MOUNTING LITTLEFS", false);
    show_file_browser_line(27, b"PLEASE WAIT", false);


    // NAND/LittleFS operations are synchronous. Install a narrowly
    // scoped hook so USB and the UVC producer continue running even
    // inside NAND ready/FSMC wait loops and ECC page processing.
    let listing_result = list_root_files(
        &mut *browser_files,
        |done, total| {
            if done == u32::MAX {
                show_file_browser_line(
                    27,
                    if total == 0 {
                        b"F2: LITTLEFS MOUNT" as &[u8]
                    } else {
                        b"F2: ROOT DIRECTORY"
                    },
                    false,
                );
            } else if total == 0 {
                show_file_browser_line(27, b"F2: NAND INIT", false);
            } else if done == 1 || done % 32 == 0 || done == total {
                let percent = done.saturating_mul(100) / total;
                let mut line = [b' '; 32];
                line[..13].copy_from_slice(b"F2: NAND SCAN");
                line[13] = b'0' + ((percent / 100) % 10) as u8;
                line[14] = b'0' + ((percent / 10) % 10) as u8;
                line[15] = b'0' + (percent % 10) as u8;
                line[16] = b'%';
                show_file_browser_line(27, &line, false);
            }
            service_io();
        },
    );

    match listing_result {
        Ok(count) => {
            *browser_count = count;
            *browser_selected = 0;
            *browser_open = true;
            FILE_BROWSER_ACTIVE.store(true, Ordering::Release);
            draw_file_browser(
                &*browser_files,
                *browser_count,
                *browser_selected,
            );
        }
        Err(error) => {
            *browser_open = false;
            FILE_BROWSER_ACTIVE.store(false, Ordering::Release);
            if matches!(
                error,
                xcore407i_blinker::nand_fs::FsAccessError::Init(
                    xcore407i_blinker::nand_fs::InitError::DharaNoJournal
                )
            ) {
                INIT_STATE.store(INIT_CONFIRM, Ordering::Release);
            } else if matches!(
                error,
                xcore407i_blinker::nand_fs::FsAccessError::Mount(_)
            ) {
                let detail = match error {
                    xcore407i_blinker::nand_fs::FsAccessError::Mount(code) => code,
                    _ => 0,
                };
                FORMAT_ERROR.store(detail, Ordering::Release);
                FORMAT_STATE.store(FORMAT_CONFIRM, Ordering::Release);
            } else {
            let message: &[u8] = match error {
                xcore407i_blinker::nand_fs::FsAccessError::Init(error) => {
                    match error {
                        xcore407i_blinker::nand_fs::InitError::Timeout =>
                            b"NAND TIMEOUT",
                        xcore407i_blinker::nand_fs::InitError::WrongDevice =>
                            b"NAND WRONG DEVICE",
                        xcore407i_blinker::nand_fs::InitError::MarkerRead =>
                            b"NAND MARKER READ",
                        xcore407i_blinker::nand_fs::InitError::MountFailed =>
                            b"NAND INIT MOUNT FAIL",
                        xcore407i_blinker::nand_fs::InitError::DharaNoJournal =>
                            b"DHARA JOURNAL MISSING",
                        xcore407i_blinker::nand_fs::InitError::DharaCapacity =>
                            b"DHARA CAPACITY LOW",
                        xcore407i_blinker::nand_fs::InitError::ProvisionFailed =>
                            b"DHARA PROVISION FAILED",
                    }
                },
                xcore407i_blinker::nand_fs::FsAccessError::Mount(_) =>
                    b"LFS MOUNT FAILED",
                xcore407i_blinker::nand_fs::FsAccessError::Operation(_) =>
                    b"LFS ROOT READ FAILED",
            };
            show_usb_status(message);
            }
        }
    }
}

}
