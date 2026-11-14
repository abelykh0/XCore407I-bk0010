//! Minimal CDC command console and ZMODEM upload entry point.

use core::ffi::c_void;

use littlefs2::{fs::Filesystem, io::{Error as LfsError, Read, Write}, path::Path};

use crate::{cdc_zmodem, nand_fs::ActiveStorage, tusb};

const LINE_CAPACITY: usize = 80;
const STORAGE_TEST_PATH: &[u8] = b"/.storage-test\0";
const STORAGE_TEST_LENGTH: usize = 8192;
const STORAGE_TEST_CHUNK: usize = 256;

static mut LINE: [u8; LINE_CAPACITY] = [0; LINE_CAPACITY];
static mut LINE_LEN: usize = 0;
static mut ZMODEM_PROBE: [u8; 6] = [0; 6];
static mut ZMODEM_PROBE_LEN: usize = 0;
static mut ZMODEM_PROBE_MATCHED: bool = false;
static mut ZMODEM_START_FAILED: bool = false;
static mut WAS_CONNECTED: bool = false;
static mut STORAGE_ERROR_REPORTED: bool = false;

const BANNER: &[u8] = b"\r\nXCore407I CDC console\r\nType HELP for commands.\r\n> ";
const HELP: &[u8] = b"\r\nCommands:\r\n  HELP       Show this help\r\n  STATUS     Show USB/storage status\r\n  LIST       List files in the LittleFS root (including hidden files)\r\n  STORAGETEST Write, read, and compare a test file (video paused)\r\n  VERSION    Show firmware target\r\n  FORMAT     Show NAND format warning\r\n  FORMAT YES Erase and reinitialize NAND\r\n> ";
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



fn report_storage_space(fs: &Filesystem<'_, ActiveStorage>, stage: &[u8]) {
    write_status(stage);
    write_status(b" LittleFS free space: ");
    match fs.available_space() {
        Ok(bytes) => {
            write_decimal(bytes);
            write_status(b" bytes (");
            write_decimal(bytes / crate::nand_fs::ERASE_BLOCK_SIZE);
            write_status(b" whole blocks)");
        }
        Err(error) => { write_status(b"unavailable (LittleFS error 0x"); write_hex_u32(error.code() as u32); write_status(b")"); },
    }
    write_status(b"; total: ");
    write_decimal(fs.total_space());
    write_status(b" bytes; configured blocks: ");
    write_decimal(crate::dhara_fs::LITTLEFS_BLOCK_COUNT);
    write_status(b"; block size: ");
    write_decimal(crate::nand_fs::ERASE_BLOCK_SIZE);
    write_status(b" bytes\r\n");
}

fn storage_test_byte(offset: usize) -> u8 {
    (offset.wrapping_mul(37) ^ (offset >> 3) ^ 0xA5) as u8
}

fn report_storage_test_io_failure(fs: &Filesystem<'_, ActiveStorage>, stage: &[u8], lfs_error: i32) {
    write_status(b"\r\nSTORAGETEST ");
    write_status(stage);
    write_status(b" failed; LittleFS error: 0x");
    write_hex_u32(lfs_error as u32);
    write_status(b"\r\n");

    write_status(b"Free-space traversal skipped here to avoid an additional full NAND scan.\r\n");
    let alloc = crate::dhara_fs::littlefs_alloc_diagnostic();
    let (rb_calls, rb_iterations, rb_timeouts) = crate::nand_fs::nand_rb_wait_diagnostic();
    write_status(b"LittleFS allocator at NOSPC: captured=");
    write_decimal(alloc[7] as usize);
    write_status(b" block_count=");
    write_decimal(alloc[0] as usize);
    write_status(b" lookahead_start=");
    write_decimal(alloc[1] as usize);
    write_status(b" next=");
    write_decimal(alloc[2] as usize);
    write_status(b" size=");
    write_decimal(alloc[3] as usize);
    write_status(b" checkpoint=");
    write_decimal(alloc[4] as usize);
    write_status(b" used_bits=");
    write_decimal(alloc[5] as usize);
    write_status(b" lookahead_bytes=");
    write_decimal(alloc[6] as usize);
    write_status(b" alloc_scans=");
    write_decimal(alloc[8] as usize);
    write_status(b" traversed_blocks=");
    write_decimal(alloc[9] as usize);
    let (alloc_stats, alloc_trace) = crate::dhara_fs::littlefs_alloc_trace_diagnostic();
    write_status(b"LittleFS allocator activity: calls=");
    write_decimal(alloc_stats[0] as usize);
    write_status(b" successes=");
    write_decimal(alloc_stats[1] as usize);
    write_status(b" checkpoint_resets=");
    write_decimal(alloc_stats[2] as usize);
    write_status(b" scans=");
    write_decimal(alloc_stats[3] as usize);
    write_status(b" traversed_blocks=");
    write_decimal(alloc_stats[4] as usize);
    write_status(b" trace_events=");
    write_decimal(alloc_stats[5] as usize);
    write_status(b"\r\nRecent LittleFS allocator events (oldest first):\r\n");
    let trace_count = alloc_stats[5].min(16) as usize;
    let trace_head = (alloc_stats[6] as usize) % 16;
    let trace_start = (trace_head + 16 - trace_count) % 16;
    for index in 0..trace_count {
        let slot = (trace_start + index) % 16;
        let base = slot * 7;
        let event_name: &[u8] = match alloc_trace[base] {
            1 => b"checkpoint",
            2 => b"drop",
            3 => b"scan",
            4 => b"allocate",
            5 => b"NOSPC",
            _ => b"unknown",
        };
        write_status(b"  ");
        write_status(event_name);
        write_status(b" start=");
        write_decimal(alloc_trace[base + 1] as usize);
        write_status(b" next=");
        write_decimal(alloc_trace[base + 2] as usize);
        write_status(b" size=");
        write_decimal(alloc_trace[base + 3] as usize);
        write_status(b" checkpoint=");
        write_decimal(alloc_trace[base + 4] as usize);
        write_status(b" used_bits=");
        write_decimal(alloc_trace[base + 5] as usize);
        write_status(b" scans=");
        write_decimal(alloc_trace[base + 6] as usize);
        write_status(b"\r\n");
    }

    write_status(b"\r\nNAND R/B waits: calls=");
    write_decimal(rb_calls as usize);
    write_status(b" total_poll_iterations=");
    write_decimal(rb_iterations as usize);
    write_status(b" timeouts=");
    write_decimal(rb_timeouts as usize);
    write_status(b"\r\n");
    let dhara_nand = crate::dhara_fs::dhara_nand_diagnostic();
    write_status(b"Dhara NAND callbacks: is_bad=");
    write_decimal(dhara_nand[0] as usize);
    write_status(b" mark_bad=");
    write_decimal(dhara_nand[1] as usize);
    write_status(b" erase=");
    write_decimal(dhara_nand[2] as usize);
    write_status(b" program=");
    write_decimal(dhara_nand[3] as usize);
    write_status(b" is_free=");
    write_decimal(dhara_nand[4] as usize);
    write_status(b" read=");
    write_decimal(dhara_nand[5] as usize);
    write_status(b" copy=");
    write_decimal(dhara_nand[6] as usize);
    write_status(b" last_op=");
    write_decimal(dhara_nand[7] as usize);
    write_status(b" last_location=0x");
    write_hex_u32(dhara_nand[8]);
    let phases = crate::dhara_fs::dhara_phase_diagnostic();
    for (index, phase) in phases.iter().enumerate() {
        let label: &[u8] = match index {
            0 => b"map_write callbacks: ",
            1 => b"map_trim callbacks: ",
            _ => b"map_sync callbacks: ",
        };
        write_status(label);
        write_status(b"erase=");
        write_decimal(phase[2] as usize);
        write_status(b" program=");
        write_decimal(phase[3] as usize);
        write_status(b" read=");
        write_decimal(phase[5] as usize);
        write_status(b" copy=");
        write_decimal(phase[6] as usize);
        write_status(b"\r\n");
    }
    write_status(b"\r\n");
    let (operation, location, storage_error) = crate::dhara_fs::storage_io_diagnostic();
    write_status(b"Dhara operation: 0x");
    write_hex_u32(operation);
    write_status(match operation {
        1 => b" (read), logical sector: 0x" as &[u8],
        2 => b" (write), logical sector: 0x",
        3 => b" (sync), logical sector: 0x",
        4 => b" (erase), logical sector: 0x",
        _ => b", logical sector: 0x",
    });
    write_hex_u32(location);
    write_status(b" error: 0x");
    write_hex_u32(storage_error);
    if storage_error == 3 {
        write_status(b" (Dhara retry/recovery budget exhausted)");
    } else if storage_error == 2 {
        write_status(b" (NAND ECC failure)");
    } else if storage_error == 1 {
        write_status(b" (NAND block operation failed)");
    }
    write_status(b"\r\n");

    let (nand_stage, detail, page, stored_ecc, calculated_ecc) =
        crate::nand_fs::nand_read_diagnostic();
    write_status(b"NAND read stage: 0x");
    write_hex_u32(nand_stage);
    write_status(b" detail: 0x");
    write_hex_u32(detail);
    write_status(b" physical page: 0x");
    write_hex_u32(page);
    write_status(b"\r\nStored ECC: 0x");
    write_hex_u32(stored_ecc);
    write_status(b" calculated ECC: 0x");
    write_hex_u32(calculated_ecc);
    write_status(b"\r\nTest file retained for inspection.\r\n> ");
}

fn report_storage_phase_counters(stage: &[u8]) {
    write_status(stage);
    let counts = crate::dhara_fs::dhara_nand_diagnostic();
    write_status(b"Dhara NAND: is_bad=");
    write_decimal(counts[0] as usize);
    write_status(b" mark_bad=");
    write_decimal(counts[1] as usize);
    write_status(b" erase=");
    write_decimal(counts[2] as usize);
    write_status(b" program=");
    write_decimal(counts[3] as usize);
    write_status(b" is_free=");
    write_decimal(counts[4] as usize);
    write_status(b" read=");
    write_decimal(counts[5] as usize);
    write_status(b" copy=");
    write_decimal(counts[6] as usize);
    write_status(b" last_op=");
    write_decimal(counts[7] as usize);
    write_status(b" last_location=0x");
    write_hex_u32(counts[8]);
    let (calls, iterations, timeouts) = crate::nand_fs::nand_rb_wait_diagnostic();
    write_status(b"\r\nNAND R/B: calls=");
    write_decimal(calls as usize);
    write_status(b" poll_iterations=");
    write_decimal(iterations as usize);
    write_status(b" timeouts=");
    write_decimal(timeouts as usize);
    write_status(b"\r\n");
}

fn storage_test(
    fs: &Filesystem<'_, ActiveStorage>,
    service_usb_only: &mut impl FnMut(),
) {
    let Ok(path) = Path::from_bytes_with_nul(STORAGE_TEST_PATH) else {
        write_status(b"\r\nSTORAGETEST: invalid test path.\r\n> ");
        return;
    };

    write_status(b"\r\nSTORAGETEST starting; UVC/video servicing is paused.\r\n");
    service_usb_only();
    // Keep allocator history from the start of the test, including cleanup
    // and free-space traversal, rather than clearing it just before writes.
    crate::dhara_fs::clear_littlefs_alloc_diagnostic();

    report_storage_phase_counters(b"At STORAGETEST entry (since storage init):\r\n");
    report_storage_space(fs, b"Before test:");
    write_status(b"CLEANUP: checking previous test file...\r\n");
    service_usb_only();
    if fs.exists(path) && fs.remove(path).is_err() {
        write_status(b"STORAGETEST: cannot remove previous test file.\r\n> ");
        return;
    }

    report_storage_phase_counters(b"Before test write (init + preparation):\r\n");
    crate::dhara_fs::clear_storage_io_diagnostic();
    crate::nand_fs::clear_nand_read_diagnostic();
    crate::nand_fs::clear_nand_rb_wait_diagnostic();
    write_status(b"WRITE: opening test file...\r\n");
    service_usb_only();

    let write_result = fs.open_file_with_options_and_then(
        |options| options.write(true).create_new(true),
        path,
        |file| {
            let mut offset = 0usize;
            let mut buffer = [0u8; STORAGE_TEST_CHUNK];
            while offset < STORAGE_TEST_LENGTH {
                let count = core::cmp::min(buffer.len(), STORAGE_TEST_LENGTH - offset);
                for (index, byte) in buffer[..count].iter_mut().enumerate() {
                    *byte = storage_test_byte(offset + index);
                }

                write_status(b"WRITE ");
                write_decimal(offset);
                write_status(b"/");
                write_decimal(STORAGE_TEST_LENGTH);
                write_status(b"\r\n");
                service_usb_only();

                let mut written = 0usize;
                while written < count {
                    let amount = file.write(&buffer[written..count])?;
                    if amount == 0 {
                        return Err(LfsError::IO);
                    }
                    written += amount;
                }
                offset += count;
            }
            write_status(b"FLUSH: committing file...\r\n");
            service_usb_only();
            file.flush()
        },
    );

    if let Err(error) = write_result {
        report_storage_test_io_failure(fs, b"write/flush", error.code());
        return;
    }

    write_status(b"WRITE complete. Reopening file for readback...\r\n");
    service_usb_only();
    crate::dhara_fs::clear_storage_io_diagnostic();
    crate::nand_fs::clear_nand_read_diagnostic();

    let mut mismatch: Option<(usize, u8, u8)> = None;
    let mut short_read_at: Option<usize> = None;
    let mut extra_byte: Option<u8> = None;
    let read_result = fs.open_file_with_options_and_then(
        |options| options.read(true),
        path,
        |file| {
            let mut offset = 0usize;
            let mut buffer = [0u8; STORAGE_TEST_CHUNK];
            while offset < STORAGE_TEST_LENGTH {
                let requested = core::cmp::min(buffer.len(), STORAGE_TEST_LENGTH - offset);
                write_status(b"READ ");
                write_decimal(offset);
                write_status(b"/");
                write_decimal(STORAGE_TEST_LENGTH);
                write_status(b"\r\n");
                service_usb_only();

                let count = file.read(&mut buffer[..requested])?;
                if count == 0 {
                    short_read_at = Some(offset);
                    return Ok(());
                }
                for (index, actual) in buffer[..count].iter().copied().enumerate() {
                    let expected = storage_test_byte(offset + index);
                    if actual != expected {
                        mismatch = Some((offset + index, expected, actual));
                        return Ok(());
                    }
                }
                offset += count;
            }

            write_status(b"READ complete. Checking for extra data...\r\n");
            service_usb_only();
            let mut extra = [0u8; 1];
            if file.read(&mut extra)? != 0 {
                extra_byte = Some(extra[0]);
            }
            Ok(())
        },
    );

    if let Err(error) = read_result {
        report_storage_test_io_failure(fs, b"read", error.code());
        return;
    }

    if let Some((offset, expected, actual)) = mismatch {
        write_status(b"\r\nSTORAGETEST mismatch at byte ");
        write_decimal(offset);
        write_status(b": expected 0x");
        write_hex_u32(expected as u32);
        write_status(b", read 0x");
        write_hex_u32(actual as u32);
        write_status(b". Test file retained.\r\n> ");
        return;
    }
    if let Some(offset) = short_read_at {
        write_status(b"\r\nSTORAGETEST unexpected EOF at byte ");
        write_decimal(offset);
        write_status(b". Test file retained.\r\n> ");
        return;
    }
    if let Some(byte) = extra_byte {
        write_status(b"\r\nSTORAGETEST file longer than expected; extra byte 0x");
        write_hex_u32(byte as u32);
        write_status(b". Test file retained.\r\n> ");
        return;
    }

    write_status(b"\r\nSTORAGETEST PASSED: wrote, flushed, reopened, and verified ");
    write_decimal(STORAGE_TEST_LENGTH);
    write_status(b" bytes. File retained as /.storage-test.\r\n> ");
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
                    } else if command_is(line, b"STORAGETEST") {
                        if let Some(fs) = fs {
                            storage_test(fs, _service_background);
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
