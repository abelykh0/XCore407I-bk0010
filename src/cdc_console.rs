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
const HELP: &[u8] = b"\r\nCommands:\r\n  HELP       Show this help\r\n  STATUS     Show USB/storage status\r\n  LIST       List files in the LittleFS root (including hidden files)\r\n  ECCSCAN    Scan all physical NAND blocks for ECC mismatches (read-only)\r\n  ECCPAGE    Compare one physical page repeatedly: ECC + raw-data stability (read-only)\r\n  NANDRESETTEST Repeat raw page reads with NAND RESET before each sample (read-only)\r\n  VERSION    Show firmware target\r\n  FORMAT     Show NAND format warning\r\n  FORMAT YES Erase and reinitialize NAND\r\n> ";
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
                InitError::Timeout => b"\r\nStorage init failed: NAND timeout. NAND state was not verified.\r\n",
                InitError::WrongDevice => b"\r\nStorage init failed: unexpected NAND device. NAND state was not verified.\r\n",
                InitError::MarkerRead => b"\r\nStorage init failed: NAND bad-block marker read error. NAND state was not verified.\r\n",
                InitError::MountFailed => b"\r\nStorage init failed: LittleFS mount failed. NAND state was not verified.\r\n",
                InitError::DharaNoJournal => b"\r\nStorage init failed: Dhara journal/volume marker missing. NAND state was not verified.\r\n",
                InitError::DharaCapacity => b"\r\nStorage init failed: Dhara capacity too small. NAND state was not verified.\r\n",
                InitError::ProvisionFailed => b"\r\nStorage init failed: Dhara provisioning error. NAND state was not verified.\r\n",
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
                    5 => b"FSMC FIFO did not empty before ECC capture; physical page 0x",
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
            let (device_stage, device_location, device_status) = crate::nand_fs::nand_device_diagnostic();
            if device_stage != 0 {
                write_status(b"Raw NAND command failure: stage=");
                write_decimal(device_stage as usize);
                write_status(b" location=0x");
                write_hex_u32(device_location);
                write_status(b" status=0x");
                write_hex_u32(device_status);
                write_status(b"\r\n");
            }
            write_status(b"NAND was not modified by this diagnostic.\r\n");
        }
        FsAccessError::Operation => {
            write_status(b"\r\nStorage service operation failed. NAND state was not verified.\r\n");
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

pub fn report_format_failure(error: crate::nand_fs::InitError) {
    use crate::nand_fs::InitError;

    let reason: &[u8] = match error {
        InitError::Timeout => b"NAND timeout",
        InitError::WrongDevice => b"unexpected NAND device",
        InitError::MarkerRead => b"bad-block marker read failed",
        InitError::MountFailed => b"LittleFS format or verification mount failed",
        InitError::DharaNoJournal => b"Dhara journal/volume marker missing",
        InitError::DharaCapacity => b"insufficient Dhara capacity",
        InitError::ProvisionFailed => b"Dhara provisioning or block retirement failed",
    };
    write_status(b"\r\nNAND format failed: ");
    write_status(reason);
    write_status(b".\r\n");

    let (
        format_stage, format_location, format_detail, format_return,
        provision_phase, bad_calls, first_bad_block, bad_reason, is_bad_calls,
    ) = crate::dhara_fs::format_diagnostic();
    if format_stage != 0 {
        write_status(b"Format stage 0x");
        write_hex_u32(format_stage);
        write_status(b", location 0x");
        write_hex_u32(format_location);
        write_status(b", detail 0x");
        write_hex_u32(format_detail);
        write_status(b", return 0x");
        write_hex_u32(format_return);
        write_status(b"\r\n");
        write_status(b"Provision phase (1=write, 2=sync): 0x");
        write_hex_u32(provision_phase);
        write_status(b", is_bad calls: 0x");
        write_hex_u32(is_bad_calls);
        write_status(b", bad results: 0x");
        write_hex_u32(bad_calls);
        write_status(b", first bad block: 0x");
        write_hex_u32(first_bad_block);
        write_status(b", bad reason: 0x");
        write_hex_u32(bad_reason);
        write_status(b"\r\n");
    }

    let (operation, location, storage_error) =
        crate::dhara_fs::storage_io_diagnostic();
    if operation != 0 {
        write_status(b"Dhara operation 0x");
        write_hex_u32(operation);
        write_status(b", logical sector 0x");
        write_hex_u32(location);
        write_status(b", error 0x");
        write_hex_u32(storage_error);
        write_status(b"\r\n");
    }

    let (stage, detail, page, stored_ecc, calculated_ecc) =
        crate::nand_fs::nand_read_diagnostic();
    if stage != 0 {
        write_status(b"NAND read stage 0x");
        write_hex_u32(stage);
        write_status(b", detail 0x");
        write_hex_u32(detail);
        write_status(b", physical page 0x");
        write_hex_u32(page);
        write_status(b"\r\n");
        if stage == 4 {
            write_status(b"Stored ECC 0x");
            write_hex_u32(stored_ecc);
            write_status(b", calculated ECC 0x");
            write_hex_u32(calculated_ecc);
            write_status(b"\r\n");
        }
    }
}

pub fn report_format_complete(report: crate::nand_fs::FormatReport) {
    write_status(b"\r\nNAND format complete; new LittleFS volume verified.\r\n");
    write_status(b"Physical blocks: ");
    write_decimal(report.physical_blocks as usize);
    write_status(b"  Good physical: ");
    write_decimal(report.good_blocks as usize);
    write_status(b"  Bad (factory/retired): ");
    write_decimal(report.bad_blocks as usize);
    write_status(b"\r\nNAND geometry: ");
    write_decimal(report.page_size_bytes as usize);
    write_status(b"-byte pages, ");
    write_decimal(report.pages_per_block as usize);
    write_status(b" pages/block, ");
    write_decimal(report.erase_block_bytes as usize);
    write_status(b"-byte erase blocks.\r\nLittleFS: ");
    write_decimal(report.littlefs_blocks as usize);
    write_status(b" logical blocks, ");
    write_decimal(report.littlefs_capacity_bytes as usize);
    write_status(b" bytes.\r\n");
    write_status(b"Storage remounting...\r\n");
}

pub(crate) fn write_status(bytes: &[u8]) {
    if bytes.is_empty() {
        return;
    }

    // TinyUSB's CDC write may accept fewer bytes than requested when its TX
    // FIFO is full. Do not silently discard the tail of diagnostic reports:
    // flush and service the device task until all bytes are queued.
    let mut offset = 0usize;
    let mut stalled = 0u32;
    while offset < bytes.len() && stalled < 4096 {
        let written = unsafe {
            tusb::tud_cdc_n_write(
                0,
                bytes[offset..].as_ptr().cast::<c_void>(),
                (bytes.len() - offset) as u32,
            )
        } as usize;
        offset += written;
        let _ = unsafe { tusb::tud_cdc_n_write_flush(0) };

        if written == 0 {
            stalled += 1;
            unsafe {
                let _ = tusb::tud_task_ext(0, false);
            }
            core::hint::spin_loop();
        } else {
            stalled = 0;
        }
    }
    let _ = unsafe { tusb::tud_cdc_n_write_flush(0) };
}

fn command_is(line: &[u8], command: &[u8]) -> bool {
    line.len() == command.len()
        && line.iter().zip(command).all(|(a, b)| a.to_ascii_uppercase() == *b)
}

fn parse_hex_u32(bytes: &[u8]) -> Option<u32> {
    let digits = if bytes.starts_with(b"0x") || bytes.starts_with(b"0X") {
        &bytes[2..]
    } else {
        bytes
    };
    if digits.is_empty() || digits.len() > 8 {
        return None;
    }
    let mut value = 0u32;
    for &byte in digits {
        let digit = match byte {
            b'0'..=b'9' => (byte - b'0') as u32,
            b'a'..=b'f' => (byte - b'a' + 10) as u32,
            b'A'..=b'F' => (byte - b'A' + 10) as u32,
            _ => return None,
        };
        value = value.checked_mul(16)?.checked_add(digit)?;
    }
    Some(value)
}

/// Service CDC input/output and the optional ZMODEM receiver.
/// TinyUSB's device task must be serviced by the caller first.
pub fn service(
    fs: Option<&Filesystem<'_, ActiveStorage>>,
    zmodem: &mut cdc_zmodem::State,
    now_ms: u32,
    _service_background: &mut impl FnMut(),
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
        if zmodem.discard_trailing_input(now_ms) {
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
                        if zmodem.last_error() == 6 {
                            let (code, detail, received, consumed) = zmodem.protocol_diagnostics();
                            let description: &[u8] = match code {
                                1 => b"Malformed encoding", 2 => b"Malformed file size",
                                3 => b"Malformed filename", 4 => b"Malformed frame",
                                5 => b"Malformed header", 6 => b"Malformed packet",
                                7 => b"Invalid receiver state", 8 => b"Backpressure",
                                9 => b"Receiver buffer exhausted", 10 => b"CRC-16 mismatch",
                                11 => b"CRC-32 mismatch", 12 => b"Unexpected EOF",
                                13 => b"Unsupported feature", 14 => b"Data subpacket exceeded receiver capacity (ZRPOS retry)", _ => b"Unclassified protocol error",
                            };
                            write_status(b"Protocol detail: ");
                            write_status(description);
                            write_status(b" (code 0x");
                            write_hex_u32(code as u32);
                            write_status(b", value 0x");
                            write_hex_u32(detail as u32);
                            write_status(b")\r\nCDC bytes received: 0x");
                            write_hex_u32(received);
                            write_status(b" decoder-consumed: 0x");
                            write_hex_u32(consumed);
                            write_status(b"\r\n");
                        }
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
                    } else if command_is(line, b"NANDPAGEPROBE") {
                        write_status(b"\r\nNANDPAGEPROBE: read-only raw page reads; no NAND writes or erases.\r\n");
                        let (ok, data) = crate::nand_fs::ecc_scan_page_probe();
                        let labels = [
                            b"Page 0 col 0: " as &[u8], b"Page 0 col 1: ",
                            b"Page 0 col 512: ", b"Page 0x83 col 0: ",
                            b"Page 0x83 col 1: ", b"Page 0x83 col 512: ",
                            b"Page 0 spare: ", b"Page 0x83 spare: ",
                        ];
                        for i in 0..8 {
                            write_status(labels[i]);
                            if ok[i] {
                                for (j, byte) in data[i].iter().enumerate() {
                                    if j != 0 { write_status(b" "); }
                                    write_hex_u32(*byte as u32);
                                }
                            } else {
                                write_status(b"READ FAILED");
                            }
                            write_status(b"\r\n");
                        }
                        write_status(b"> ");
                    } else if command_is(line, b"NANDRESETTEST") {
                        write_status(b"\r\nNANDRESETTEST: three raw reads per location; NAND RESET + STATUS before each read. No NAND writes or erases.\r\n");
                        let (reset_ok, read_ok, data) = crate::nand_fs::ecc_scan_reset_page_probe();
                        let labels = [
                            b"Page 0 col 0" as &[u8], b"Page 0 col 1",
                            b"Page 0 col 512", b"Page 0 spare",
                            b"Page 0x83 col 0", b"Page 0x83 col 1",
                            b"Page 0x83 col 512", b"Page 0x83 spare",
                        ];
                        for i in 0..8 {
                            for repeat in 0..3 {
                                write_status(labels[i]);
                                write_status(b" run ");
                                write_decimal(repeat + 1);
                                write_status(b": ");
                                if !reset_ok[i][repeat] {
                                    write_status(b"RESET/STATUS FAILED");
                                } else if !read_ok[i][repeat] {
                                    write_status(b"PAGE READ FAILED");
                                } else {
                                    for (j, byte) in data[i][repeat].iter().enumerate() {
                                        if j != 0 { write_status(b" "); }
                                        write_hex_u32(*byte as u32);
                                    }
                                }
                                write_status(b"\r\n");
                            }
                        }
                        write_status(b"> ");
                    } else if command_is(line, b"NANDPROBE") {
                        write_status(b"\r\nNANDPROBE: read-only READ ID / STATUS; no NAND writes or erases.\r\n");
                        let (id, status, ready) = crate::nand_fs::ecc_scan_interface_probe();
                        write_status(b"READ ID:");
                        for byte in id {
                            write_status(b" 0x");
                            write_hex_u32(byte as u32);
                        }
                        write_status(b"\r\nSTATUS: 0x");
                        write_hex_u32(status as u32);
                        write_status(if ready { b"  R/B=READY\r\n" } else { b"  R/B=BUSY\r\n" });
                        write_status(b"> ");
                    } else if line.len() > 8
                        && line[..8].eq_ignore_ascii_case(b"ECCPAGE ")
                    {
                        let Some(page) = parse_hex_u32(&line[8..]) else {
                            write_status(b"ECCPAGE: use ECCPAGE followed by a hexadecimal physical page number.\r\n> ");
                            LINE_LEN = 0;
                            continue;
                        };
                        if page >= crate::nand_fs::PHYSICAL_BLOCKS * crate::nand_fs::PAGES_PER_BLOCK {
                            write_status(b"ECCPAGE: physical page is out of range.\r\n> ");
                        } else {
                            write_status(b"\r\nECCPAGE: read-only; no NAND writes or erases. FSMC ECC disabled. Three independent raw reads.\r\nPage: 0x");
                            write_hex_u32(page);
                            write_status(b"\r\n");
                            let mut previous: Option<crate::nand_fs::EccPageComparison> = None;
                            for pass in 0..3 {
                                match crate::nand_fs::ecc_software_compare_page(page) {
                                    Some(result) => {
                                        write_status(b"Read ");
                                        write_decimal(pass + 1);
                                        write_status(b":\r\n");
                                        for chunk in 0..4 {
                                            write_status(b"  chunk ");
                                            write_decimal(chunk);
                                            write_status(b" stored=0x");
                                            write_hex_u32(result.stored[chunk]);
                                            write_status(b" software=0x");
                                            write_hex_u32(result.software[chunk]);
                                            write_status(b" data-fingerprint=0x");
                                            write_hex_u32(result.data_fingerprint[chunk]);
                                            if result.stored[chunk] == result.software[chunk] {
                                                write_status(b" ECC=MATCH");
                                            } else {
                                                write_status(b" ECC=MISMATCH");
                                            }
                                            if let Some(old) = previous.as_ref() {
                                                if result.stored[chunk] != old.stored[chunk]
                                                    || result.software[chunk] != old.software[chunk]
                                                    || result.data_fingerprint[chunk] != old.data_fingerprint[chunk]
                                                {
                                                    write_status(b" READ-CHANGED");
                                                }
                                            }
                                            write_status(b"\r\n");
                                        }
                                        previous = Some(result);
                                    }
                                    None => {
                                        write_status(b"Read ");
                                        write_decimal(pass + 1);
                                        write_status(b": READ FAILED, page untagged, or ECC tag invalid.\r\n");
                                        previous = None;
                                    }
                                }
                            }
                            write_status(b"ECCPAGE complete. Compare all three reads; repeat command if needed.\r\n> ");
                        }
                    } else if command_is(line, b"ECCSCAN") {
                        write_status(b"\r\nECCSCAN: read-only full physical NAND ECC scan. No NAND writes or erases.\r\n");
                        let vectors = crate::nand_fs::ecc_software_selftest();
                        let vectors_ok = vectors == [0x00FF_FFFF, 0x00FF_FFFF, 0x00AA_AAAA, 0x0069_5659];
                        write_status(if vectors_ok { b"Software ECC reference vectors: PASS\r\n" } else { b"Software ECC reference vectors: FAIL\r\n" });

                        let mut good_blocks = 0u32;
                        let mut bad_blocks = 0u32;
                        let mut block_read_errors = 0u32;
                        let mut suspect_blocks = 0u32;
                        let mut tagged_pages = 0u32;
                        let mut untagged_pages = 0u32;
                        let mut page_read_errors = 0u32;
                        let mut matching_chunks = 0u32;
                        let mut mismatching_chunks = 0u32;

                        write_status(b"Block map ('.'=clean, '!'=suspect):\r\n");
                        for block in 0..crate::nand_fs::PHYSICAL_BLOCKS {
                            let first = block * crate::nand_fs::PAGES_PER_BLOCK;
                            let mut block_bad = false;
                            let mut block_suspect = false;
                            let mut marker_error = false;

                            for marker_page in [first, first + 1] {
                                _service_background();
                                match crate::nand_fs::ecc_scan_block_marker(marker_page) {
                                    Some(marker) => {
                                        if marker != 0xFF {
                                            block_bad = true;
                                        }
                                    }
                                    None => {
                                        marker_error = true;
                                        break;
                                    }
                                }
                            }

                            if marker_error {
                                block_read_errors += 1;
                                block_suspect = true;
                            } else if block_bad {
                                bad_blocks += 1;
                                block_suspect = true;
                            } else {
                                good_blocks += 1;
                                for page in first..first + crate::nand_fs::PAGES_PER_BLOCK {
                                    match crate::nand_fs::ecc_scan_page_tagged(page) {
                                        Some(false) => {
                                            untagged_pages += 1;
                                            continue;
                                        }
                                        None => {
                                            page_read_errors += 1;
                                            block_suspect = true;
                                            continue;
                                        }
                                        Some(true) => tagged_pages += 1,
                                    }

                                    match crate::nand_fs::ecc_software_compare_page(page) {
                                        Some(result) => {
                                            for chunk in 0..4 {
                                                let stored = result.stored[chunk];
                                                let software = result.software[chunk];
                                                if stored == software {
                                                    matching_chunks += 1;
                                                } else {
                                                    mismatching_chunks += 1;
                                                    block_suspect = true;
                                                    write_status(b"\r\nECC mismatch: page=0x");
                                                    write_hex_u32(page);
                                                    write_status(b" chunk=");
                                                    write_decimal(chunk);
                                                    write_status(b" stored=0x");
                                                    write_hex_u32(stored);
                                                    write_status(b" software=0x");
                                                    write_hex_u32(software);
                                                    write_status(b"\r\n");
                                                }
                                            }
                                        }
                                        None => {
                                            page_read_errors += 1;
                                            block_suspect = true;
                                        }
                                    }
                                }
                            }

                            if block_suspect {
                                suspect_blocks += 1;
                                write_status(b"!");
                            } else {
                                write_status(b".");
                            }
                            if (block + 1) % 64 == 0 {
                                write_status(b"\r\n");
                            }
                            _service_background();
                        }

                        write_status(b"\r\nECCSCAN SUMMARY\r\nGood blocks: ");
                        write_decimal(good_blocks as usize);
                        write_status(b"  Bad blocks: ");
                        write_decimal(bad_blocks as usize);
                        write_status(b"  Marker read errors: ");
                        write_decimal(block_read_errors as usize);
                        write_status(b"\r\nSuspect blocks: ");
                        write_decimal(suspect_blocks as usize);
                        write_status(b"\r\nTagged pages: ");
                        write_decimal(tagged_pages as usize);
                        write_status(b"  Untagged/erased pages: ");
                        write_decimal(untagged_pages as usize);
                        write_status(b"  Page read errors: ");
                        write_decimal(page_read_errors as usize);
                        write_status(b"\r\nMatching chunks: ");
                        write_decimal(matching_chunks as usize);
                        write_status(b"  Mismatching chunks: ");
                        write_decimal(mismatching_chunks as usize);
                        write_status(b"\r\nFSMC ECC: disabled for this scan\r\n");

                        // A full-device scan is valid only if every physical
                        // block was classified as good. A nonzero count alone can
                        // hide a broken marker-read path (for example, 1023 false
                        // bad-block results and one apparently good block).
                        let marker_classification_sane =
                            good_blocks == crate::nand_fs::PHYSICAL_BLOCKS
                                && bad_blocks == 0
                                && block_read_errors == 0;
                        if !marker_classification_sane {
                            write_status(b"WARNING: marker scan classified every physical block as bad (or found no good blocks). Treat marker results as INVALID; do not format or retire blocks from this scan.\r\n");
                        }
                        write_status(if vectors_ok
                            && marker_classification_sane
                            && mismatching_chunks == 0
                            && page_read_errors == 0
                            && block_read_errors == 0
                        {
                            b"ECCSCAN PASSED. NAND was not modified.\r\n> "
                        } else {
                            b"ECCSCAN completed with diagnostic findings. NAND was not modified.\r\n> "
                        });
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
                        } else {
                            write_status(
                                b"Storage is unavailable. Formatting NAND...\r\n",
                            );
                        }
                        format_requested = true;
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
