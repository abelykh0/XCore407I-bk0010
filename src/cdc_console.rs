//! Minimal CDC command console and ZMODEM upload entry point.

use core::ffi::c_void;

use littlefs2::fs::Filesystem;

use crate::{cdc_zmodem, nand_fs::ActiveStorage, tusb};

const LINE_CAPACITY: usize = 80;

static mut LINE: [u8; LINE_CAPACITY] = [0; LINE_CAPACITY];
static mut LINE_LEN: usize = 0;
static mut WAS_CONNECTED: bool = false;
static mut STORAGE_ERROR_REPORTED: bool = false;

const BANNER: &[u8] = b"\r\nXCore407I CDC console\r\nType HELP for commands.\r\n> ";
const HELP: &[u8] = b"\r\nCommands:\r\n  HELP       Show this help\r\n  STATUS     Show USB/storage status\r\n  VERSION    Show firmware target\r\n  RECV       Receive a file with ZMODEM\r\n  FORMAT     Show NAND format warning\r\n  FORMAT YES Erase and reinitialize NAND\r\n> ";
const STATUS_MOUNTED: &[u8] = b"\r\nUSB: High-Speed UVC camera + CDC serial\r\nAudio: disabled\r\nUSB host / keyboard: disabled\r\nEmulator: disabled\r\nStorage: LittleFS mounted for CDC session\r\n> ";
const STATUS_UNAVAILABLE: &[u8] = b"\r\nUSB: High-Speed UVC camera + CDC serial\r\nAudio: disabled\r\nUSB host / keyboard: disabled\r\nEmulator: disabled\r\nStorage: unavailable\r\n> ";
const VERSION: &[u8] = b"\r\nXCore407I BK-0010 firmware\r\n> ";

/// Print the exact non-destructive storage initialization or mount failure.
pub fn report_storage_error(error: crate::nand_fs::FsAccessError) {
    use crate::nand_fs::{FsAccessError, InitError};

    // The CDC terminal may not have asserted DTR when storage initialization
    // fails. Defer the diagnostic until the host opens the port.
    unsafe {
        if !tusb::tud_cdc_n_connected(0) || STORAGE_ERROR_REPORTED {
            return;
        }
        STORAGE_ERROR_REPORTED = true;
    }

    match error {
        FsAccessError::Init(kind) => {
            let message: &[u8] = match kind {
                InitError::Timeout => b"\r\nStorage init failed: NAND timeout. NAND was not modified.\r\n",
                InitError::WrongDevice => b"\r\nStorage init failed: unexpected NAND device. NAND was not modified.\r\n",
                InitError::MarkerRead => b"\r\nStorage init failed: NAND bad-block marker read error. NAND was not modified.\r\n",
                InitError::MountFailed => b"\r\nStorage init failed: LittleFS mount failed. NAND was not modified.\r\n",
                InitError::DharaNoJournal => b"\r\nStorage init failed: Dhara journal/volume marker missing. NAND was not modified.\r\n",
                InitError::DharaCapacity => b"\r\nStorage init failed: Dhara capacity too small. NAND was not modified.\r\n",
                InitError::ProvisionFailed => b"\r\nStorage init failed: Dhara provisioning error. NAND was not modified.\r\n",
            };
            write_status(message);
        }
        FsAccessError::Mount(code) => {
            write_status(b"\r\nLittleFS mount failed; error code 0x");
            write_hex_u32(code as u32);
            write_status(b"\r\n");
            let (operation, location, detail) = crate::dhara_fs::storage_io_diagnostic();
            if operation == 0 {
                write_status(b"No Dhara Storage callback failure was recorded.\r\n");
            } else {
                let name: &[u8] = match operation {
                    1 => b"read",
                    2 => b"write",
                    3 => b"sync",
                    4 => b"trim",
                    5 => b"erase-sync",
                    _ => b"unknown",
                };
                write_status(b"Dhara ");
                write_status(name);
                write_status(b" failed at logical sector 0x");
                write_hex_u32(location);
                write_status(b", error 0x");
                write_hex_u32(detail);
                write_status(b"\r\n");
            }
            let (stage, detail, page, stored_ecc, calculated_ecc) = crate::nand_fs::nand_read_diagnostic();
            if stage != 0 {
                let description: &[u8] = match stage {
                    1 => b"NAND data-page ready/busy timeout; physical page 0x",
                    2 => b"NAND OOB read ready/busy timeout; physical page 0x",
                    3 => b"Invalid ECC tag; observed OOB tag 0x",
                    4 => b"Uncorrectable ECC syndrome (chunk in high byte); detail 0x",
                    _ => b"Unknown NAND read failure stage; detail 0x",
                };
                write_status(description);
                write_hex_u32(detail);
                write_status(b"\r\n");
                if stage == 4 {
                    write_status(b"Failing physical page: 0x");
                    write_hex_u32(page);
                    write_status(b"\r\nStored ECC: 0x");
                    write_hex_u32(stored_ecc);
                    write_status(b"\r\nCalculated ECC: 0x");
                    write_hex_u32(calculated_ecc);
                    write_status(b"\r\n");
                }
            } else {
                write_status(b"No lower-level NAND read stage was recorded.\r\n");
            }
            write_status(b"NAND was not modified by this diagnostic.\r\n");
        }
        FsAccessError::Operation => {
            write_status(b"\r\nStorage service operation failed. NAND was not modified.\r\n");
        }
    }
}


fn write_hex_u32(value: u32) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let mut digits = [b'0'; 8];
    for (index, digit) in digits.iter_mut().enumerate() {
        let shift = (7 - index) * 4;
        *digit = HEX[((value >> shift) & 0x0F) as usize];
    }
    write_status(&digits);
}

pub(crate) fn write_status(bytes: &[u8]) {
    if bytes.is_empty() {
        return;
    }
    let _ = unsafe {
        tusb::tud_cdc_n_write(0, bytes.as_ptr().cast::<c_void>(), bytes.len() as u32)
    };
    let _ = unsafe { tusb::tud_cdc_n_write_flush(0) };
}

fn command_is(line: &[u8], command: &[u8]) -> bool {
    line.len() == command.len()
        && line.iter().zip(command).all(|(a, b)| a.to_ascii_uppercase() == *b)
}

/// Service CDC input/output and the optional ZMODEM receiver.
/// TinyUSB's device task must be serviced by the caller first.
pub fn service(
    fs: Option<&Filesystem<'_, ActiveStorage>>,
    zmodem: &mut cdc_zmodem::State,
    now_ms: u32,
) {
    unsafe {
        let connected = tusb::tud_cdc_n_connected(0);
        if connected && !WAS_CONNECTED {
            LINE_LEN = 0;
            write_status(BANNER);
        }
        if !connected {
            LINE_LEN = 0;
            WAS_CONNECTED = false;
            STORAGE_ERROR_REPORTED = false;
            cdc_zmodem::cancel(zmodem);
            return;
        }
        WAS_CONNECTED = true;

        if zmodem.active() {
            if let Some(fs) = fs {
                cdc_zmodem::service(fs, zmodem, now_ms);
            }
            return;
        }

        let mut byte = 0u8;
        while tusb::tud_cdc_n_read(0, (&mut byte as *mut u8).cast::<c_void>(), 1) != 0 {
            match byte {
                b'\r' | b'\n' => {
                    write_status(b"\r\n");
                    let line = core::slice::from_raw_parts(
                        core::ptr::addr_of!(LINE).cast::<u8>(),
                        LINE_LEN,
                    );
                    if command_is(line, b"HELP") {
                        write_status(HELP);
                    } else if command_is(line, b"STATUS") {
                        write_status(if fs.is_some() {
                            STATUS_MOUNTED
                        } else {
                            STATUS_UNAVAILABLE
                        });
                    } else if command_is(line, b"VERSION") {
                        write_status(VERSION);
                    } else if command_is(line, b"RECV") {
                        if fs.is_none() {
                            write_status(b"Storage unavailable; ZMODEM receive disabled.\r\n> ");
                        } else if cdc_zmodem::start(zmodem, true, now_ms).is_ok() {
                            write_status(b"ZMODEM receiver ready. In Tera Term choose Transfer > ZMODEM > Send.\r\n");
                        } else {
                            write_status(b"Unable to start ZMODEM receiver.\r\n> ");
                        }
                    } else if command_is(line, b"FORMAT") {
                        write_status(
                            b"WARNING: FORMAT YES erases the NAND and creates a new Dhara/LittleFS volume.\r\nType FORMAT YES to confirm.\r\n> ",
                        );
                    } else if command_is(line, b"FORMAT YES") {
                        if fs.is_some() {
                            write_status(
                                b"Refusing: filesystem is mounted. Reboot into CDC mode with storage unavailable before formatting.\r\n> ",
                            );
                        } else {
                            write_status(b"Formatting NAND; all files will be lost...\r\n");
                            match crate::nand_fs::initialize_nand_volume() {
                                Ok(()) => write_status(
                                    b"NAND format complete. Reboot the device to mount the new volume.\r\n> ",
                                ),
                                Err(error) => {
                                    use crate::nand_fs::InitError;
                                    let message: &[u8] = match error {
                                        InitError::Timeout => b"NAND format failed: timeout.\r\n> ",
                                        InitError::WrongDevice => b"NAND format failed: unexpected NAND device.\r\n> ",
                                        InitError::MarkerRead => b"NAND format failed: bad-block marker read error.\r\n> ",
                                        InitError::MountFailed => b"NAND format failed: LittleFS format/mount error.\r\n> ",
                                        InitError::DharaNoJournal => b"NAND format failed: Dhara journal missing.\r\n> ",
                                        InitError::DharaCapacity => b"NAND format failed: insufficient Dhara capacity.\r\n> ",
                                        InitError::ProvisionFailed => b"NAND format failed: Dhara provisioning error.\r\n> ",
                                    };
                                    write_status(message);
                                }
                            }
                        }
                    } else if LINE_LEN == 0 {
                        write_status(b"> ");
                    } else {
                        write_status(b"Unknown command. Type HELP.\r\n> ");
                    }
                    LINE_LEN = 0;
                }
                0x08 | 0x7f => {
                    if LINE_LEN != 0 {
                        LINE_LEN -= 1;
                        write_status(b"\x08 \x08");
                    }
                }
                0x20..=0x7e => {
                    if LINE_LEN < LINE_CAPACITY {
                        LINE[LINE_LEN] = byte;
                        LINE_LEN += 1;
                        write_status(&[byte]);
                    } else {
                        write_status(b"\x07");
                    }
                }
                _ => {}
            }
        }
    }
}
