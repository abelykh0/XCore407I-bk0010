use core::sync::atomic::{AtomicBool, AtomicU8};

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
pub(crate) const SAVE_RAM_START: usize = 0x0200;
pub(crate) const SAVE_RAM_LENGTH: usize = 15_872;
pub(crate) const SAVE_NAME_CAPACITY: usize = 24;
pub(crate) const MAX_BROWSER_FILES: usize = 25;
pub(crate) const MAX_BROWSER_PATH: usize = 256;

// Keep the browser's path table in ordinary .bss, not on the limited stack.
pub(crate) static mut BROWSER_PATHS: [[u8; MAX_BROWSER_PATH]; MAX_BROWSER_FILES] =
    [[0; MAX_BROWSER_PATH]; MAX_BROWSER_FILES];
pub(crate) static mut BROWSER_PATH_LENGTHS: [u16; MAX_BROWSER_FILES] = [0; MAX_BROWSER_FILES];
pub(crate) static FILE_BROWSER_ACTIVE: AtomicBool = AtomicBool::new(false);
pub(crate) static FILE_BROWSER_ACTION: AtomicU8 = AtomicU8::new(0);
pub(crate) static SAVE_DIALOG_STATE: AtomicU8 = AtomicU8::new(SAVE_DIALOG_IDLE);
pub(crate) static SAVE_DIALOG_DIRTY: AtomicBool = AtomicBool::new(false);
pub(crate) static SAVE_DIALOG_RESULT: AtomicU8 = AtomicU8::new(0);
pub(crate) static SAVE_OVERWRITE: AtomicBool = AtomicBool::new(false);
pub(crate) static mut SAVE_FILENAME: [u8; SAVE_NAME_CAPACITY] = [0; SAVE_NAME_CAPACITY];
pub(crate) static mut SAVE_FILENAME_LEN: usize = 0;
 
unsafe extern "C" {
    fn bk_set_pc(start: u16);
    static mut RamBuffer: [u8; 0x4000];
}

/// Read up to 25 regular files from the LittleFS root directory.
/// Paths are retained as NUL-terminated ASCII strings for the later load.
pub(crate) fn list_root_files(
    paths: &mut [[u8; MAX_BROWSER_PATH]; MAX_BROWSER_FILES],
    path_lengths: &mut [u16; MAX_BROWSER_FILES],
) -> Result<usize, ()> {
    xcore407i_blinker::nand_fs::with_mounted(|fs| {
        fs.read_dir_and_then(&littlefs2::path!("/"), |directory| {
            let mut count = 0usize;
            for item in directory {
                let entry = item?;
                if !entry.file_type().is_file() {
                    continue;
                }

                let full_path = entry.path();
                let path_bytes = full_path.as_str_ref_with_trailing_nul().as_bytes();
                if path_bytes.len() < 3
                    || path_bytes.len() > MAX_BROWSER_PATH
                    || !path_bytes[..path_bytes.len() - 1]
                        .iter()
                        .all(|byte| byte.is_ascii_graphic() || *byte == b' ')
                {
                    continue;
                }

                // Only expose .bin files in the browser (case-insensitive).
                // The stored path is NUL-terminated and begins with '/'.
                let filename = &path_bytes[1..path_bytes.len() - 1];
                if filename.len() < 4
                    || !filename[filename.len() - 4..].eq_ignore_ascii_case(b".bin")
                {
                    continue;
                }

                paths[count][..path_bytes.len()].copy_from_slice(path_bytes);
                path_lengths[count] = path_bytes.len() as u16;
                count += 1;
                if count == MAX_BROWSER_FILES {
                    break;
                }
            }
            Ok(count)
        })
    })
    .map_err(|_| ())
}

pub(crate) fn draw_file_browser(
    paths: &[[u8; MAX_BROWSER_PATH]; MAX_BROWSER_FILES],
    path_lengths: &[u16; MAX_BROWSER_FILES],
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
            let path_len = path_lengths[row] as usize;
            // Root paths have the form "/filename\\0".
            let name = &paths[row][1..path_len - 1];
            let copy_len = name.len().min(line.len() - 2);
            line[2..2 + copy_len].copy_from_slice(&name[..copy_len]);
        } else if row == 0 && count == 0 {
            line[..8].copy_from_slice(b"NO FILES");
        }
        show_file_browser_line(row + 1, &line, row == selected && count != 0);
    }

    show_file_browser_line(27, b"UP/DOWN MOVE  ENTER LOAD  ESC CANCEL", false);
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
pub(crate) fn save_ram_to_path(path_bytes: &[u8], overwrite: bool) -> Result<(), ()> {
    let path = littlefs2::path::Path::from_bytes_with_nul(path_bytes).map_err(|_| ())?;
    xcore407i_blinker::nand_fs::with_mounted(|fs| {
        if fs.exists(path) && !overwrite {
            return Err(littlefs2::io::Error::IO);
        }

        fs.open_file_with_options_and_then(
            |options| options.write(true).create(true).truncate(true),
            path,
            |file| {
                let start = (SAVE_RAM_START as u16).to_le_bytes();
                let size = (SAVE_RAM_LENGTH as u16).to_le_bytes();
                let header = [start[0], start[1], size[0], size[1]];
                let mut header_written = 0usize;
                while header_written < header.len() {
                    let count = file.write(&header[header_written..])?;
                    if count == 0 {
                        return Err(littlefs2::io::Error::IO);
                    }
                    header_written += count;
                }

                let ram = unsafe {
                    core::slice::from_raw_parts(
                        core::ptr::addr_of!(RamBuffer).cast::<u8>().add(SAVE_RAM_START),
                        SAVE_RAM_LENGTH,
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
                Ok(())
            },
        )
    })
    .map_err(|_| ())
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
        let len = SAVE_FILENAME_LEN.min(SAVE_NAME_CAPACITY);
        line[6..6 + len].copy_from_slice(&SAVE_FILENAME[..len]);
        line[6 + len..10 + len].copy_from_slice(b".bin");
        show_file_browser_line(4, &line, true);
        show_file_browser_line(27, b"ENTER SAVE  BACKSPACE EDIT  ESC CANCEL", false);
    }
}

/// Load a selected file containing the 4-byte Start/Size header and payload.
pub(crate) fn load_tetris_from_path(path_bytes: &[u8]) -> Result<(), ()> {
    let path = littlefs2::path::Path::from_bytes_with_nul(path_bytes).map_err(|_| ())?;
    let (start, _size) = xcore407i_blinker::nand_fs::with_mounted(|fs| {
        fs.open_file_and_then(path, |file| {
            let mut header = [0u8; 4];
            let mut header_read = 0usize;
            while header_read < header.len() {
                let count = file.read(&mut header[header_read..])?;
                if count == 0 {
                    return Err(littlefs2::io::Error::IO);
                }
                header_read += count;
            }

            let start = u16::from_le_bytes([header[0], header[1]]);
            let size = u16::from_le_bytes([header[2], header[3]]);
            if size == 0 || start as u32 + size as u32 > 0x4000 {
                return Err(littlefs2::io::Error::IO);
            }

            let destination = unsafe {
                core::slice::from_raw_parts_mut(
                    core::ptr::addr_of_mut!(RamBuffer).cast::<u8>().add(start as usize),
                    size as usize,
                )
            };
            let mut total = 0usize;
            while total < size as usize {
                let count = file.read(&mut destination[total..])?;
                if count == 0 {
                    return Err(littlefs2::io::Error::IO);
                }
                total += count;
            }
            Ok((start, size))
        })
    }).map_err(|_| ())?;

    unsafe { bk_set_pc(start); }
    Ok(())
}

