//! Minimal CDC command console and ZMODEM upload entry point.

use core::ffi::c_void;

use littlefs2::{fs::Filesystem, path::Path};

use crate::{cdc_zmodem, nand_fs::ActiveStorage, tusb};

const LINE_CAPACITY: usize = 80;

static mut LINE: [u8; LINE_CAPACITY] = [0; LINE_CAPACITY];
static mut LINE_LEN: usize = 0;
static mut ZMODEM_PROBE: [u8; 6] = [0; 6];
static mut ZMODEM_PROBE_LEN: usize = 0;
static mut ZMODEM_PROBE_MATCHED: bool = false;
static mut ZMODEM_START_FAILED: bool = false;
static mut WAS_CONNECTED: bool = false;
static mut STORAGE_ERROR_REPORTED: bool = false;

const BANNER: &[u8] = b"\r\nXCore407I CDC console\r\nType HELP for commands.\r\n> ";
const HELP: &[u8] = b"\r\nCommands:\r\n  HELP       Show this help\r\n  STATUS     Show USB/storage status\r\n  LIST       List files in the LittleFS root (including hidden files)\r\n  VERSION    Show firmware target\r\n  FORMAT     Show NAND format warning\r\n  FORMAT YES Erase and reinitialize NAND\r\n> ";
const STATUS_MOUNTED: &[u8] = b"\r\nUSB: High-Speed UVC camera + CDC serial\r\nAudio: disabled\r\nUSB host / keyboard: disabled\r\nEmulator: disabled\r\nStorage: LittleFS mounted for CDC session\r\n";
const STATUS_UNAVAILABLE: &[u8] = b"\r\nUSB: High-Speed UVC camera + CDC serial\r\nAudio: disabled\r\nUSB host / keyboard: disabled\r\nEmulator: disabled\r\nStorage: unavailable\r\n";
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

pub(crate) fn write_append_failure(lfs_error: i32, operation: u32, location: u32, storage_error: u32) {
    write_status(b"\r\nZMODEM append failed; LittleFS error: 0x");
    write_hex_u32(lfs_error as u32);
    write_status(b"\r\nDhara operation: ");
    write_hex_u32(operation);
    write_status(b" logical sector: 0x");
    write_hex_u32(location);
    write_status(b" error: 0x");
    write_hex_u32(storage_error);
    write_status(b"\r\n");
}

fn write_decimal(mut value: usize) {
    let mut digits = [0u8; 10];
    let mut start = digits.len();
    loop {
        start -= 1;
        digits[start] = b'0' + (value % 10) as u8;
        value /= 10;
        if value == 0 {
            break;
        }
    }
    write_status(&digits[start..]);
}

fn list_files(fs: &Filesystem<'_, ActiveStorage>) {
    write_status(b"\r\nLittleFS root:\r\n");
    let Ok(root) = Path::from_bytes_with_nul(b"/\0") else {
        write_status(b"Cannot construct root path.\r\n> ");
        return;
    };

    let result = fs.read_dir_and_then(root, |entries| {
        for entry in entries {
            let entry = entry?;
            let path = entry.path();
            write_status(b"  ");
            write_status(path.as_str().as_bytes());
            let metadata = entry.metadata();
            if metadata.is_dir() {
                write_status(b" [DIR]");
            } else {
                write_status(b"  ");
                write_decimal(metadata.len());
                write_status(b" bytes");
            }
            write_status(b"\r\n");
        }
        Ok(())
    });

    if let Err(error) = result {
        write_status(b"Directory read failed; LittleFS error: 0x");
        write_hex_u32(error.code() as u32);
        write_status(b"\r\n");
        let (operation, location, storage_error) =
            crate::dhara_fs::storage_io_diagnostic();
        if operation != 0 {
            write_status(b"Dhara operation: ");
            write_hex_u32(operation);
            write_status(b" logical sector: 0x");
            write_hex_u32(location);
            write_status(b" error: 0x");
            write_hex_u32(storage_error);
            write_status(b"\r\n");
        }
    }
    write_status(b"> ");
}

pub fn report_format_status(bytes: &[u8]) {
    write_status(bytes);
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
    service_background: &mut impl FnMut(),
) -> bool {
    let mut format_requested = false;
    unsafe {
        let connected = tusb::tud_cdc_n_connected(0);
        if connected && !WAS_CONNECTED {
            LINE_LEN = 0;
            ZMODEM_PROBE_LEN = 0;
            ZMODEM_PROBE_MATCHED = false;
            ZMODEM_START_FAILED = false;
            write_status(BANNER);
        }
        if !connected {
            LINE_LEN = 0;
            ZMODEM_PROBE_LEN = 0;
            ZMODEM_PROBE_MATCHED = false;
            ZMODEM_START_FAILED = false;
            WAS_CONNECTED = false;
            STORAGE_ERROR_REPORTED = false;
            cdc_zmodem::cancel(zmodem, fs);
            return false;
        }
        WAS_CONNECTED = true;

        if zmodem.active() {
            if let Some(fs) = fs {
                cdc_zmodem::service(fs, zmodem, now_ms);
            }
            return false;
        }

        let mut byte = 0u8;
        while tusb::tud_cdc_n_read(0, (&mut byte as *mut u8).cast::<c_void>(), 1) != 0 {
            // ZMODEM senders initiate a session with a ZRQINIT hex header:
            // "**" + ZDLE + "B00". Hold this short prefix out of the console
            // parser until it either matches or proves to be ordinary text.
            const ZRQINIT_PREFIX: &[u8; 6] = b"**\x18B00";
            if ZMODEM_PROBE_LEN != 0 || byte == b'*' {
                ZMODEM_PROBE[ZMODEM_PROBE_LEN] = byte;
                ZMODEM_PROBE_LEN += 1;
                if ZRQINIT_PREFIX.starts_with(&ZMODEM_PROBE[..ZMODEM_PROBE_LEN]) {
                    if ZMODEM_PROBE_LEN == ZRQINIT_PREFIX.len() {
                        ZMODEM_PROBE_MATCHED = true;
                        let started = if fs.is_some()
                            && cdc_zmodem::start(zmodem, true, now_ms).is_ok()
                        {
                            cdc_zmodem::seed_input(zmodem, &ZMODEM_PROBE[..ZMODEM_PROBE_LEN])
                        } else {
                            false
                        };
                        ZMODEM_START_FAILED = !started;
                        ZMODEM_PROBE_LEN = 0;
                        if started {
                            // Leave remaining CDC OUT bytes queued for the
                            // protocol receiver on the next foreground pass.
                            break;
                        }
                    }
                    continue;
                }

                // Not a ZMODEM startup header. Replay held printable bytes
                // as console input, then process the current byte normally.
                for index in 0..ZMODEM_PROBE_LEN.saturating_sub(1) {
                    let held = ZMODEM_PROBE[index];
                    if (0x20..=0x7e).contains(&held) && LINE_LEN < LINE_CAPACITY {
                        LINE[LINE_LEN] = held;
                        LINE_LEN += 1;
                        write_status(&[held]);
                    }
                }
                ZMODEM_PROBE_LEN = 0;
                if byte == b'*' {
                    ZMODEM_PROBE[0] = byte;
                    ZMODEM_PROBE_LEN = 1;
                    continue;
                }
            }
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
                        write_status(if ZMODEM_START_FAILED {
                            b"ZMODEM startup: header detected, receiver did not start.\r\n"
                        } else if ZMODEM_PROBE_MATCHED {
                            b"ZMODEM startup: header detected; receiver started.\r\n"
                        } else {
                            b"ZMODEM startup: header not detected.\r\n"
                        });
                        let (input, queued, sent) = zmodem.handshake_status();
                        write_status(if input {
                            b"ZMODEM input: accepted. "
                        } else {
                            b"ZMODEM input: not yet accepted. "
                        });
                        write_status(if queued {
                            b"Response: queued. "
                        } else {
                            b"Response: not queued. "
                        });
                        write_status(if sent {
                            b"Response: sent to CDC IN.\r\n"
                        } else {
                            b"Response: not sent.\r\n"
                        });
                        let failure = match zmodem.last_error() {
                            0 => b"ZMODEM last failure: none recorded.\r\n" as &[u8],
                            1 => b"ZMODEM last failure: temp file not active.\r\n",
                            2 => b"ZMODEM last failure: LittleFS append/write/flush.\r\n",
                            3 => b"ZMODEM last failure: receiver rejected file_written.\r\n",
                            4 => b"ZMODEM last failure: receiver rejected accept_file_at.\r\n",
                            5 => b"ZMODEM last failure: final file commit/rename.\r\n",
                            6 => b"ZMODEM last failure: protocol input error.\r\n",
                            8 => b"ZMODEM last failure: temp-file cleanup failed.\r\n",
                            _ => b"ZMODEM last failure: unknown stage.\r\n",
                        };
                        write_status(failure);
                        if zmodem.last_error() == 2 {
                            let (
                                lfs_error, operation, location, storage_error,
                                nand_stage, nand_detail, nand_page,
                                nand_stored_ecc, nand_calculated_ecc,
                            ) = zmodem.append_failure_details();
                            write_status(b"Append LittleFS error: 0x");
                            write_hex_u32(lfs_error as u32);
                            write_status(b"\r\nDhara operation: 0x");
                            write_hex_u32(operation);
                            write_status(b" logical sector: 0x");
                            write_hex_u32(location);
                            write_status(b" error: 0x");
                            write_hex_u32(storage_error);
                            write_status(b"\r\nNAND read stage: 0x");
                            write_hex_u32(nand_stage);
                            write_status(b" detail: 0x");
                            write_hex_u32(nand_detail);
                            write_status(b" physical page: 0x");
                            write_hex_u32(nand_page);
                            write_status(b"\r\nStored ECC: 0x");
                            write_hex_u32(nand_stored_ecc);
                            write_status(b" calculated ECC: 0x");
                            write_hex_u32(nand_calculated_ecc);
                            write_status(b"\r\n");
                        }
                        write_status(b"> ");
                    } else if command_is(line, b"LIST") {
                        if let Some(fs) = fs {
                            list_files(fs);
                        } else {
                            write_status(b"Storage unavailable.\r\n> ");
                        }
                    } else if command_is(line, b"VERSION") {
                        write_status(VERSION);
                    } else if command_is(line, b"FORMAT") {
                        write_status(
                            b"WARNING: FORMAT YES erases the NAND and creates a new Dhara/LittleFS volume.\r\nType FORMAT YES to confirm.\r\n> ",
                        );
                    } else if command_is(line, b"FORMAT YES") {
                        if fs.is_some() {
                            write_status(
                                b"Unmounting LittleFS before NAND format...\r\n",
                            );
                            format_requested = true;
                        } else {
                            write_status(b"Formatting NAND; all files will be lost...\r\n");
                            let mut last_reported = 0u32;
                            match crate::nand_fs::initialize_nand_volume_with_progress(
                                |completed, total| {
                                    service_background();
                                    if completed == total || completed >= last_reported + 32 {
                                        last_reported = completed;
                                        write_status(b"Format progress: ");
                                        write_hex_u32(completed);
                                        write_status(b" / ");
                                        write_hex_u32(total);
                                        write_status(b" blocks\r\n");
                                    }
                                },
                            ) {
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
                    if format_requested {
                        break;
                    }
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
    format_requested
}
