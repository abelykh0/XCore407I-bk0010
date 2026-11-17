//! Minimal CDC command console and ZMODEM upload entry point.

use core::ffi::c_void;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

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
static DTR_ASSERTED: AtomicBool = AtomicBool::new(false);
static BANNER_PENDING: AtomicBool = AtomicBool::new(false);

// Temporary CDC connection/TX diagnostics, reported by STATUS.
static CDC_LINE_CB_COUNT: AtomicU32 = AtomicU32::new(0);
static CDC_LINE_STATE: AtomicU32 = AtomicU32::new(0);
static CDC_SERVICE_COUNT: AtomicU32 = AtomicU32::new(0);
static CDC_CONNECTED_COUNT: AtomicU32 = AtomicU32::new(0);
static CDC_READY_COUNT: AtomicU32 = AtomicU32::new(0);
static CDC_FLUSH_COUNT: AtomicU32 = AtomicU32::new(0);
static CDC_FLUSH_STARTED: AtomicU32 = AtomicU32::new(0);
static CDC_LAST_FLUSH: AtomicU32 = AtomicU32::new(0);
static CDC_BANNER_ATTEMPTS: AtomicU32 = AtomicU32::new(0);
static CDC_BANNER_ACCEPTED: AtomicU32 = AtomicU32::new(0);
static CDC_BANNER_TX_PENDING: AtomicBool = AtomicBool::new(false);
static CDC_BANNER_TX_COMPLETE: AtomicU32 = AtomicU32::new(0);
static CDC_TX_COMPLETE_COUNT: AtomicU32 = AtomicU32::new(0);
static CDC_RX_CALLBACK_COUNT: AtomicU32 = AtomicU32::new(0);
static CDC_BANNER_COMPLETE_RX_COUNT: AtomicU32 = AtomicU32::new(u32::MAX);
static mut STORAGE_ERROR_REPORTED: bool = false;

const BANNER: &[u8] = b"\r\nXCore407I CDC console\r\nType HELP for commands.\r\n> ";
const HELP: &[u8] = b"\r\nCommands:\r\n  HELP       Show this help\r\n  STATUS     Show USB/storage status\r\n  LIST       List files with sizes (including hidden files)\r\n  LIST SHORT List files without sizes (faster comparison)\r\n  LIST PROFILE Profile LittleFS, Dhara, NAND, and CDC time\r\n  LIST PROFILE SHORT Profile listing without file sizes\r\n  DELETE name Delete a file from the LittleFS root\r\n  SEND name   Send a LittleFS file to the terminal via ZMODEM\r\n  ECCSCAN    Scan all physical NAND blocks for ECC mismatches (read-only)\r\n  VERSION    Show firmware target\r\n  FORMAT       Show NAND format warning\r\n  FORMAT YES   Erase and reinitialize NAND (destructive)\r\n  FORMAT LFS YES  Reformat LittleFS only; preserve Dhara journal\r\n> ";
#[cfg(feature = "nand-diagnostics")]
const HELP_NAND_DIAGNOSTICS: &[u8] = b"  ECCFILETEST Write/read a 4 KiB ECC test file\r\n  ECCPAGE    Compare one physical page repeatedly\r\n  ECCPAGEHW  Compare stored, hardware, and software ECC\r\n  ECCSEQ ON/OFF Arm/disarm program-sequence trace\r\n  ECCSEQSTATUS Show captured FSMC ECC/FIFO sequence\r\n  ECCWRITE ON/OFF Enable/disable page-program verification\r\n  ECCWRITESTATUS Show last verified page program\r\n  NANDRESETTEST Repeat raw reads with NAND RESET\r\n  NANDPAGEPROBE / NANDPROBE Low-level NAND diagnostics\r\n";
const STATUS_MOUNTED: &[u8] = b"\r\nUSB: High-Speed UVC camera + CDC serial\r\nAudio: disabled\r\nUSB host / keyboard: disabled\r\nEmulator: disabled\r\nStorage: LittleFS mounted for CDC session\r\n";
const STATUS_UNAVAILABLE: &[u8] = b"\r\nUSB: High-Speed UVC camera + CDC serial\r\nAudio: disabled\r\nUSB host / keyboard: disabled\r\nEmulator: disabled\r\nStorage: unavailable\r\n";
const VERSION: &[u8] = b"\r\nXCore407I BK-0010 firmware\r\n> ";

/// TinyUSB reports DTR changes while processing SET_CONTROL_LINE_STATE.
/// Keep the callback limited to publishing state; all console I/O stays in
/// the foreground service below.
#[unsafe(no_mangle)]
pub extern "C" fn tud_cdc_line_state_cb(itf: u8, dtr: bool, _rts: bool) {
    if itf != 0 {
        return;
    }

    CDC_LINE_CB_COUNT.fetch_add(1, Ordering::Relaxed);
    CDC_LINE_STATE.store((dtr as u32) | ((_rts as u32) << 1), Ordering::Release);
    let was_asserted = DTR_ASSERTED.swap(dtr, Ordering::AcqRel);
    if dtr && !was_asserted {
        BANNER_PENDING.store(true, Ordering::Release);
    } else if !dtr {
        BANNER_PENDING.store(false, Ordering::Release);
    }
}

/// Record host-to-device CDC traffic so banner completion can be ordered
/// against the first received character.
#[unsafe(no_mangle)]
pub extern "C" fn tud_cdc_rx_cb(itf: u8) {
    if itf == 0 {
        CDC_RX_CALLBACK_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

/// TinyUSB calls this after a CDC IN transfer has completed successfully.
#[unsafe(no_mangle)]
pub extern "C" fn tud_cdc_tx_complete_cb(itf: u8) {
    if itf == 0 {
        CDC_TX_COMPLETE_COUNT.fetch_add(1, Ordering::Relaxed);
        if CDC_BANNER_TX_PENDING.swap(false, Ordering::AcqRel) {
            CDC_BANNER_TX_COMPLETE.fetch_add(1, Ordering::Relaxed);
            CDC_BANNER_COMPLETE_RX_COUNT.store(
                CDC_RX_CALLBACK_COUNT.load(Ordering::Acquire),
                Ordering::Release,
            );
        }
    }
}

/// Detect a host opening the CDC port and print the banner once its
/// configuration has opened both CDC data endpoints.
pub fn service_connection() {
    CDC_SERVICE_COUNT.fetch_add(1, Ordering::Relaxed);
    unsafe {
        let connected = tusb::tud_cdc_n_connected(0);
        if connected {
            CDC_CONNECTED_COUNT.fetch_add(1, Ordering::Relaxed);
        }
        if connected && !WAS_CONNECTED {
            LINE_LEN = 0;
            ZMODEM_PROBE_LEN = 0;
            ZMODEM_PROBE_MATCHED = false;
            ZMODEM_START_FAILED = false;
            WAS_CONNECTED = true;
            // Fallback for a DTR edge observed before this foreground pass.
            BANNER_PENDING.store(true, Ordering::Release);
        } else if !connected {
            WAS_CONNECTED = false;
            DTR_ASSERTED.store(false, Ordering::Release);
            BANNER_PENDING.store(false, Ordering::Release);
        }

        let ready = tusb::tud_cdc_n_ready(0);
        if connected && ready {
            CDC_READY_COUNT.fetch_add(1, Ordering::Relaxed);
            // Queue the banner only once. A failed endpoint claim does not
            // discard TinyUSB's TX FIFO, so keep retrying flush below.
            if BANNER_PENDING.load(Ordering::Acquire) {
                CDC_BANNER_ATTEMPTS.fetch_add(1, Ordering::Relaxed);
                let accepted = write_status_counted(BANNER);
                CDC_BANNER_ACCEPTED.fetch_add(accepted as u32, Ordering::Relaxed);
                // Arm completion tracking only after the banner bytes have
                // been accepted into the CDC TX FIFO. write_status_counted()
                // can service TinyUSB while waiting for FIFO space; arming
                // before it returns could misattribute an older transfer's
                // completion to this banner.
                if accepted != 0 {
                    CDC_BANNER_TX_PENDING.store(true, Ordering::Release);
                }
                BANNER_PENDING.store(false, Ordering::Release);

                // The banner is queued after the caller's normal TinyUSB task
                // pass. Run one device-task pass now so the newly queued CDC IN
                // data is serviced without waiting for host input to trigger
                // another console write.
                let _ = tusb::tud_task_ext(1, false);
            }

            // CDC write_flush() may return zero while the IN endpoint is
            // busy. Retry from the foreground loop until the FIFO drains;
            // do not wait for another character to trigger the next flush.
            let flushed = tusb::tud_cdc_n_write_flush(0);
            CDC_FLUSH_COUNT.fetch_add(1, Ordering::Relaxed);
            CDC_LAST_FLUSH.store(flushed, Ordering::Release);
            if flushed != 0 {
                CDC_FLUSH_STARTED.fetch_add(flushed, Ordering::Relaxed);
            }
        }
    }
}

/// Append a snapshot of CDC connection and transmit-path diagnostics to STATUS.
fn write_cdc_diagnostics() {
    let connected = unsafe { tusb::tud_cdc_n_connected(0) };
    let ready = unsafe { tusb::tud_cdc_n_ready(0) };
    write_status(b"CDC diagnostics: connected=");
    write_decimal(connected as usize);
    write_status(b" ready=");
    write_decimal(ready as usize);
    write_status(b" DTR=");
    write_decimal(DTR_ASSERTED.load(Ordering::Acquire) as usize);
    write_status(b" RTS=");
    write_decimal(((CDC_LINE_STATE.load(Ordering::Acquire) >> 1) & 1) as usize);
    write_status(b" banner_pending=");
    write_decimal(BANNER_PENDING.load(Ordering::Acquire) as usize);
    write_status(b"\r\n  line_cb=");
    write_decimal(CDC_LINE_CB_COUNT.load(Ordering::Relaxed) as usize);
    write_status(b" line_state=0x");
    write_hex_u32(CDC_LINE_STATE.load(Ordering::Acquire));
    write_status(b" service=");
    write_decimal(CDC_SERVICE_COUNT.load(Ordering::Relaxed) as usize);
    write_status(b" connected_passes=");
    write_decimal(CDC_CONNECTED_COUNT.load(Ordering::Relaxed) as usize);
    write_status(b" ready_passes=");
    write_decimal(CDC_READY_COUNT.load(Ordering::Relaxed) as usize);
    write_status(b"\r\n  flush_calls=");
    write_decimal(CDC_FLUSH_COUNT.load(Ordering::Relaxed) as usize);
    write_status(b" flush_started_bytes=");
    write_decimal(CDC_FLUSH_STARTED.load(Ordering::Relaxed) as usize);
    write_status(b" last_flush_return=");
    write_decimal(CDC_LAST_FLUSH.load(Ordering::Acquire) as usize);
    write_status(b"\r\n  banner_attempts=");
    write_decimal(CDC_BANNER_ATTEMPTS.load(Ordering::Relaxed) as usize);
    write_status(b" banner_accepted_bytes=");
    write_decimal(CDC_BANNER_ACCEPTED.load(Ordering::Relaxed) as usize);
    write_status(b" banner_tx_complete=");
    write_decimal(CDC_BANNER_TX_COMPLETE.load(Ordering::Relaxed) as usize);
    write_status(b" banner_tx_pending=");
    write_decimal(CDC_BANNER_TX_PENDING.load(Ordering::Acquire) as usize);
    write_status(b" tx_complete=");
    write_decimal(CDC_TX_COMPLETE_COUNT.load(Ordering::Relaxed) as usize);
    write_status(b" rx_callbacks=");
    write_decimal(CDC_RX_CALLBACK_COUNT.load(Ordering::Relaxed) as usize);
    write_status(b" banner_complete_rx_callbacks=");
    let banner_rx = CDC_BANNER_COMPLETE_RX_COUNT.load(Ordering::Acquire);
    if banner_rx == u32::MAX {
        write_status(b"n/a");
    } else {
        write_decimal(banner_rx as usize);
    }
    write_status(b"\r\n");
}

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

fn write_decimal_grouped(mut value: usize) {
    // A 32-bit usize needs at most 10 digits and 3 separators.
    let mut digits = [0u8; 13];
    let mut start = digits.len();
    let mut group_digits = 0usize;

    loop {
        start -= 1;
        digits[start] = b'0' + (value % 10) as u8;
        value /= 10;
        group_digits += 1;

        if value == 0 {
            break;
        }
        if group_digits == 3 {
            start -= 1;
            digits[start] = b',';
            group_digits = 0;
        }
    }

    write_status(&digits[start..]);
}

pub(crate) fn write_send_complete(name: &[u8], size: u32) {
    write_status(b"\r\nSent file ");
    write_status(name);
    write_status(b" size ");
    write_decimal_grouped(size as usize);
    write_status(b" bytes.\r\n> ");
}

fn delete_file(fs: &Filesystem<'_, ActiveStorage>, name: &[u8]) {
    // DELETE accepts one root-level filename only. Keep the same conservative
    // character policy as ZMODEM uploads; never accept paths or separators.
    if name.is_empty()
        || name.len() > 64
        || name == b"."
        || name == b".."
        || !name.iter().all(|byte| {
            byte.is_ascii_alphanumeric()
                || matches!(*byte, b'.' | b'_' | b'-' | b'+' | b' ' | b'(' | b')')
        })
    {
        write_status(b"DELETE: invalid filename. Use a root-level filename without a path.\r\n> ");
        return;
    }

    let mut path_bytes = [0u8; 66];
    path_bytes[0] = b'/';
    path_bytes[1..1 + name.len()].copy_from_slice(name);
    path_bytes[1 + name.len()] = 0;
    let Ok(path) = Path::from_bytes_with_nul(&path_bytes[..name.len() + 2]) else {
        write_status(b"DELETE: invalid path.\r\n> ");
        return;
    };

    write_status(b"DELETE: checking /");
    write_status(name);
    write_status(b"...\r\n");
    if !fs.exists(path) {
        write_status(b"DELETE: file not found.\r\n> ");
        return;
    }
    // Isolate this operation's lower-level failure from earlier filesystem
    // diagnostics so an LFS_ERR_IO can be traced to a Dhara callback.
    crate::dhara_fs::clear_storage_io_diagnostic();
    crate::nand_fs::clear_nand_read_diagnostic();
    match fs.remove(path) {
        Ok(()) => {
            write_status(b"DELETE: removed /");
            write_status(name);
            write_status(b".\r\n> ");
        }
        Err(error) => {
            write_status(b"DELETE: LittleFS remove failed; error 0x");
            write_hex_u32(error.code() as u32);
            write_status(b"\r\n");
            let (operation, location, storage_error) =
                crate::dhara_fs::storage_io_diagnostic();
            if operation == 0 {
                write_status(b"DELETE: no Dhara Storage callback failure was recorded.\r\n");
            } else {
                let operation_name: &[u8] = match operation {
                    1 => b"read",
                    2 => b"write",
                    3 => b"sync",
                    4 => b"trim",
                    _ => b"unknown",
                };
                write_status(b"DELETE: Dhara ");
                write_status(operation_name);
                write_status(b" failed at logical sector 0x");
                write_hex_u32(location);
                write_status(b", error 0x");
                write_hex_u32(storage_error);
                write_status(b"\r\n");
                if operation == 3 {
                    let (stage, page, dhara_error, steps) = crate::dhara_fs::map_sync_diagnostic();
                    let stage_name: &[u8] = match stage {
                        1 => b"step limit",
                        2 => b"pad_queue",
                        3 => b"raw_gc",
                        4 => b"recovery",
                        _ => b"unknown/not captured",
                    };
                    write_status(b"DELETE: map_sync stage ");
                    write_status(stage_name);
                    write_status(b", journal page 0x");
                    write_hex_u32(page);
                    write_status(b", Dhara error 0x");
                    write_hex_u32(dhara_error);
                    write_status(b", steps ");
                    write_decimal(steps as usize);
                    write_status(b"\r\n");
                    let (nand_stage, detail, nand_page, stored, calculated) =
                        crate::nand_fs::nand_read_diagnostic();
                    if nand_stage == 4 {
                        write_status(b"DELETE: ECC failure at physical page 0x");
                        write_hex_u32(nand_page);
                        write_status(b", detail 0x");
                        write_hex_u32(detail);
                        write_status(b", stored ECC 0x");
                        write_hex_u32(stored);
                        write_status(b", calculated ECC 0x");
                        write_hex_u32(calculated);
                        write_status(b"\r\n");
                    } else if nand_stage != 0 {
                        write_status(b"DELETE: NAND read failure stage ");
                        write_decimal(nand_stage as usize);
                        write_status(b", detail 0x");
                        write_hex_u32(detail);
                        write_status(b"\r\n");
                    } else {
                        write_status(b"DELETE: no lower-level NAND read failure recorded.\r\n");
                    }
                }
            }
            write_status(b"> ");
        }
    }
}

struct ListOutput {
    buffer: [u8; 256],
    len: usize,
}

impl ListOutput {
    fn new() -> Self {
        Self {
            buffer: [0; 256],
            len: 0,
        }
    }

    fn push(&mut self, mut bytes: &[u8]) {
        while !bytes.is_empty() {
            let available = self.buffer.len() - self.len;
            if available == 0 {
                self.flush();
                continue;
            }
            let count = available.min(bytes.len());
            self.buffer[self.len..self.len + count].copy_from_slice(&bytes[..count]);
            self.len += count;
            bytes = &bytes[count..];
        }
    }

    fn push_decimal_grouped(&mut self, mut value: usize) {
        let mut digits = [0u8; 13];
        let mut start = digits.len();
        let mut group_digits = 0usize;
        loop {
            start -= 1;
            digits[start] = b'0' + (value % 10) as u8;
            value /= 10;
            group_digits += 1;
            if value == 0 {
                break;
            }
            if group_digits == 3 {
                start -= 1;
                digits[start] = b',';
                group_digits = 0;
            }
        }
        self.push(&digits[start..]);
    }

    fn flush(&mut self) {
        if self.len != 0 {
            write_status(&self.buffer[..self.len]);
            self.len = 0;
        }
    }
}

fn list_files(fs: &Filesystem<'_, ActiveStorage>, show_sizes: bool, profile: bool) {
    let _ = profile;
    #[cfg(feature = "list-profile")]
    if profile {
        crate::list_diag::begin();
    }

    #[cfg(feature = "list-profile")]
    let total_started = if profile { crate::list_diag::cycle_count() } else { 0 };
    write_status(b"\r\nLIST: command accepted.\r\n");
    write_status(b"LittleFS total size: ");
    #[cfg(feature = "list-profile")]
    let total_space_started = if profile { crate::list_diag::cycle_count() } else { 0 };
    let total_space = fs.total_space();
    #[cfg(feature = "list-profile")]
    let total_space_cycles = if profile {
        crate::list_diag::cycle_count().wrapping_sub(total_space_started)
    } else { 0 };
    write_decimal_grouped(total_space);
    write_status(b" bytes\r\n");
    write_status(b"LIST: reading root directory...\r\n");

    #[cfg(feature = "list-profile")]
    let mut file_count = 0u32;
    #[cfg(feature = "list-profile")]
    let mut directory_count = 0u32;
    let mut output = ListOutput::new();
    #[cfg(feature = "list-profile")]
    if profile {
        crate::list_diag::set_directory_active(true);
    }
    #[cfg(feature = "list-profile")]
    let directory_started = if profile { crate::list_diag::cycle_count() } else { 0 };
    let result = crate::nand_fs::read_root_directory(fs, show_sizes, |path_bytes, is_dir, size| {
        #[cfg(feature = "list-profile")]
        if is_dir { directory_count = directory_count.wrapping_add(1); }
        #[cfg(feature = "list-profile")]
        if !is_dir { file_count = file_count.wrapping_add(1); }
        let path = &path_bytes[..path_bytes.len() - 1];
        output.push(b"  ");
        output.push(path);
        if is_dir {
            output.push(b" [DIR]");
        } else if show_sizes {
            output.push(b"  ");
            output.push_decimal_grouped(size);
            output.push(b" bytes");
        }
        output.push(b"\r\n");
        Ok(())
    });
    output.flush();
    #[cfg(feature = "list-profile")]
    let directory_cycles = if profile {
        crate::list_diag::cycle_count().wrapping_sub(directory_started)
    } else { 0 };
    #[cfg(feature = "list-profile")]
    if profile {
        crate::list_diag::set_directory_active(false);
    }
    #[cfg(feature = "list-profile")]
    let directory_stats = if profile {
        crate::list_diag::snapshot()
    } else {
        crate::list_diag::Snapshot {
            dhara_reads: 0, dhara_cycles: 0, nand_reads: 0, nand_bytes: 0,
            nand_cycles: 0, cdc_calls: 0, cdc_bytes: 0, cdc_cycles: 0,
            directory_cdc_cycles: 0,
        }
    };

    match &result {
        Ok(()) => write_status(b"LIST: directory read complete.\r\n"),
        Err(error) => {
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
    }

    #[cfg(feature = "list-profile")]
    let mut available_cycles = 0u32;
    if result.is_ok() {
        write_status(b"LittleFS free space: ");
        #[cfg(feature = "list-profile")]
        let available_started = if profile { crate::list_diag::cycle_count() } else { 0 };
        let available = fs.available_space();
        #[cfg(feature = "list-profile")]
        if profile {
            available_cycles = crate::list_diag::cycle_count().wrapping_sub(available_started);
        }
        match available {
            Ok(bytes) => {
                write_decimal_grouped(bytes);
                write_status(b" bytes\r\n");
            }
            Err(error) => {
                write_status(b"query failed; LittleFS error 0x");
                write_hex_u32(error.code() as u32);
                write_status(b"\r\n");
            }
        }
    }
    #[cfg(feature = "list-profile")]
    if profile {
        let total_cycles = crate::list_diag::cycle_count().wrapping_sub(total_started);
        let stats = crate::list_diag::end();
        report_list_profile(
            show_sizes, file_count, directory_count, total_cycles,
            total_space_cycles, directory_cycles, available_cycles,
            directory_stats, stats,
        );
    } else {
        write_status(b"> ");
    }
    #[cfg(not(feature = "list-profile"))]
    write_status(b"> ");
}

#[cfg(feature = "list-profile")]
fn report_list_profile(
    show_sizes: bool, files: u32, directories: u32, total_cycles: u32,
    total_space_cycles: u32, directory_cycles: u32, available_cycles: u32,
    directory_stats: crate::list_diag::Snapshot,
    stats: crate::list_diag::Snapshot,
) {
    fn cycles_to_us(cycles: u32) -> usize {
        ((cycles as u64 * 1_000_000) / 168_000_000) as usize
    }
    fn line(label: &[u8], cycles: u32) {
        write_status(label);
        write_decimal(cycles_to_us(cycles));
        write_status(b" us\r\n");
    }

    let dhara_overhead = stats.dhara_cycles.saturating_sub(stats.nand_cycles);
    let directory_residual = directory_cycles
        .saturating_sub(stats.dhara_cycles)
        .saturating_sub(stats.directory_cdc_cycles);

    write_status(b"\r\nLIST PROFILE (");
    write_status(if show_sizes { b"with sizes" } else { b"SHORT" });
    write_status(b")\r\nEntries: ");
    write_decimal(files as usize);
    write_status(b" files, ");
    write_decimal(directories as usize);
    write_status(b" directories\r\n");
    line(b"Total LIST: ", total_cycles);
    line(b"LittleFS total_space: ", total_space_cycles);
    line(b"Directory traversal + formatting: ", directory_cycles);
    line(b"LittleFS available_space: ", available_cycles);

    write_status(b"Dhara map reads: ");
    write_decimal(stats.dhara_reads as usize);
    write_status(b" calls, ");
    write_decimal(cycles_to_us(stats.dhara_cycles));
    write_status(b" us total\r\n");
    write_status(b"  Dhara excluding NAND (estimate): ");
    write_decimal(cycles_to_us(dhara_overhead));
    write_status(b" us\r\n");

    write_status(b"NAND page reads: ");
    write_decimal(stats.nand_reads as usize);
    write_status(b", ");
    write_decimal(stats.nand_bytes as usize);
    write_status(b" bytes, ");
    write_decimal(cycles_to_us(stats.nand_cycles));
    write_status(b" us total\r\n");

    write_status(b"CDC writes: ");
    write_decimal(stats.cdc_calls as usize);
    write_status(b", ");
    write_decimal(stats.cdc_bytes as usize);
    write_status(b" bytes, ");
    write_decimal(cycles_to_us(stats.cdc_cycles));
    write_status(b" us total\r\n");
    line(b"  CDC during directory: ", stats.directory_cdc_cycles);
    line(b"Directory residual (LittleFS + formatting estimate): ", directory_residual);
    let available_dhara_reads = stats.dhara_reads.saturating_sub(directory_stats.dhara_reads);
    let available_dhara_cycles = stats.dhara_cycles.saturating_sub(directory_stats.dhara_cycles);
    let available_nand_reads = stats.nand_reads.saturating_sub(directory_stats.nand_reads);
    let available_nand_bytes = stats.nand_bytes.saturating_sub(directory_stats.nand_bytes);
    let available_nand_cycles = stats.nand_cycles.saturating_sub(directory_stats.nand_cycles);
    write_status(b"Directory I/O: Dhara ");
    write_decimal(directory_stats.dhara_reads as usize);
    write_status(b" calls, ");
    write_decimal(cycles_to_us(directory_stats.dhara_cycles));
    write_status(b" us; NAND ");
    write_decimal(directory_stats.nand_reads as usize);
    write_status(b" reads, ");
    write_decimal(cycles_to_us(directory_stats.nand_cycles));
    write_status(b" us\r\n");
    write_status(b"available_space I/O: Dhara ");
    write_decimal(available_dhara_reads as usize);
    write_status(b" calls, ");
    write_decimal(cycles_to_us(available_dhara_cycles));
    write_status(b" us; NAND ");
    write_decimal(available_nand_reads as usize);
    write_status(b" reads, ");
    write_decimal(cycles_to_us(available_nand_cycles));
    write_status(b" us (");
    write_decimal(available_nand_bytes as usize);
    write_status(b" bytes)\r\n");
    write_status(b"Compare LIST PROFILE with LIST PROFILE SHORT; repeat both to reduce cache/order effects.\r\n> ");
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
    write_status(b"\r\nStorage format failed: ");
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
    let _ = write_status_counted(bytes);
}

/// Queue bytes through TinyUSB and return the number accepted into its TX FIFO.
fn write_status_counted(bytes: &[u8]) -> usize {
    if bytes.is_empty() {
        return 0;
    }
    #[cfg(feature = "list-profile")]
    let profile_started = if crate::list_diag::is_active() {
        Some(crate::list_diag::cycle_count())
    } else {
        None
    };

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
                let _ = tusb::tud_task_ext(1, false);
            }
            core::hint::spin_loop();
        } else {
            stalled = 0;
        }
    }
    let _ = unsafe { tusb::tud_cdc_n_write_flush(0) };
    #[cfg(feature = "list-profile")]
    if let Some(started) = profile_started {
        crate::list_diag::record_cdc_write(
            crate::list_diag::cycle_count().wrapping_sub(started),
            offset,
        );
    }
    offset
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
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FormatRequest {
    None,
    LittleFs,
    WholeNand,
}

pub fn service(
    fs: Option<&Filesystem<'_, ActiveStorage>>,
    zmodem: &mut cdc_zmodem::State,
    now_ms: u32,
    _service_background: &mut impl FnMut(),
) -> FormatRequest {
    let mut format_requested = FormatRequest::None;
    unsafe {
        let connected = tusb::tud_cdc_n_connected(0);
        service_connection();
        if !connected {
            LINE_LEN = 0;
            ZMODEM_PROBE_LEN = 0;
            ZMODEM_PROBE_MATCHED = false;
            ZMODEM_START_FAILED = false;
            WAS_CONNECTED = false;
            STORAGE_ERROR_REPORTED = false;
            cdc_zmodem::cancel(zmodem, fs);
            return FormatRequest::None;
        }
        WAS_CONNECTED = true;

        if zmodem.active() {
            if let Some(fs) = fs {
                cdc_zmodem::service(fs, zmodem, now_ms);
            }
            return FormatRequest::None;
        }
        if zmodem.discard_trailing_input(now_ms) {
            return FormatRequest::None;
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
                        #[cfg(feature = "nand-diagnostics")]
                        write_status(HELP_NAND_DIAGNOSTICS);
                    } else if line.len() > 5
                        && line[..5].eq_ignore_ascii_case(b"SEND ")
                    {
                        if let Some(fs) = fs {
                            match cdc_zmodem::start_send(fs, zmodem, &line[5..], now_ms) {
                                Ok(()) => write_status(b"\r\nZMODEM send starting; waiting for terminal receiver...\r\n"),
                                Err(1) => write_status(b"\r\nSEND: another ZMODEM transfer is active.\r\n> "),
                                Err(2) => write_status(b"\r\nSEND: invalid filename. Use a root-level filename without a path.\r\n> "),
                                Err(3) => write_status(b"\r\nSEND: file not found or metadata unavailable.\r\n> "),
                                Err(4) => write_status(b"\r\nSEND: file is too large for ZMODEM.\r\n> "),
                                _ => write_status(b"\r\nSEND: unable to initialize ZMODEM sender.\r\n> "),
                            }
                        } else {
                            write_status(b"\r\nStorage unavailable.\r\n> ");
                        }
                    } else if command_is(line, b"SEND") {
                        write_status(b"\r\nUsage: SEND filename\r\n> ");
                    } else if command_is(line, b"STATUS") {
                        write_status(if fs.is_some() {
                            STATUS_MOUNTED
                        } else {
                            STATUS_UNAVAILABLE
                        });
                        write_cdc_diagnostics();
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
                        if zmodem.last_error() == 5 {
                            let (stage, error) = zmodem.commit_failure_details();
                            let description: &[u8] = match stage {
                                1 => b"Temporary path construction failed",
                                2 => b"Destination path construction failed",
                                3 => b"Destination already exists",
                                4 => b"LittleFS rename returned an error",
                                _ => b"Unclassified commit failure",
                            };
                            write_status(b"Commit stage: ");
                            write_status(description);
                            write_status(b" (0x");
                            write_hex_u32(stage);
                            write_status(b") LittleFS error: 0x");
                            write_hex_u32(error as u32);
                            write_status(b"\r\nTemporary upload: /.zmodem-upload (preserved)\r\n");
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
                    } else if cfg!(feature = "nand-diagnostics") && command_is(line, b"ECCSEQ ON") {
                        crate::nand_fs::set_ecc_sequence_trace_enabled(true);
                        write_status(b"\r\nECCSEQ: armed for the next successful page program; no readback will be performed.\r\n> ");
                    } else if cfg!(feature = "nand-diagnostics") && command_is(line, b"ECCSEQ OFF") {
                        crate::nand_fs::set_ecc_sequence_trace_enabled(false);
                        write_status(b"\r\nECCSEQ: disabled.\r\n> ");
                    } else if cfg!(feature = "nand-diagnostics") && command_is(line, b"ECCSEQSTATUS") {
                        let trace = crate::nand_fs::ecc_sequence_trace();
                        if trace.page == u32::MAX {
                            write_status(b"\r\nECCSEQSTATUS: no page program captured. Enable ECCSEQ, then create/write a file.\r\n> ");
                        } else {
                            write_status(b"\r\nECCSEQSTATUS: FSMC program sequence (no NAND readback)\r\nPage: 0x");
                            write_hex_u32(trace.page);
                            write_status(b" flags=0x");
                            write_hex_u32(trace.flags);
                            write_status(b" (bit0=OOB FIFO drained bit1=program/status success)\r\n");
                            for chunk in 0..4 {
                                write_status(b"  chunk ");
                                write_decimal(chunk);
                                write_status(b" PCR-before=0x");
                                write_hex_u32(trace.pcr_before[chunk]);
                                write_status(b" PCR-enabled=0x");
                                write_hex_u32(trace.pcr_enabled[chunk]);
                                write_status(b" ECC-before-data=0x");
                                write_hex_u32(trace.ecc_before_data[chunk]);
                                write_status(b" input-non-FF=");
                                write_decimal(trace.input_non_ff[chunk] as usize);
                                write_status(b" input-fp=0x");
                                write_hex_u32(trace.input_fingerprint[chunk]);
                                write_status(b" SR-before-wait=0x");
                                write_hex_u32(trace.status_before_wait[chunk]);
                                write_status(b" SR-after-wait=0x");
                                write_hex_u32(trace.status_after_wait[chunk]);
                                write_status(b" FIFO-polls=");
                                write_decimal(trace.fifo_poll_count[chunk] as usize);
                                write_status(b" ECC-after-wait=0x");
                                write_hex_u32(trace.ecc_after_wait[chunk]);
                                write_status(b" SW-ECC=0x");
                                write_hex_u32(trace.software_ecc[chunk]);
                                write_status(b" PCR-after-disable=0x");
                                write_hex_u32(trace.pcr_after_disable[chunk]);
                                write_status(b" ECC-after-disable=0x");
                                write_hex_u32(trace.ecc_after_disable[chunk]);
                                write_status(b"\r\n");
                            }
                            write_status(b"  OOB SR-before-wait=0x");
                            write_hex_u32(trace.oob_status_before_wait);
                            write_status(b" SR-after-wait=0x");
                            write_hex_u32(trace.oob_status_after_wait);
                            write_status(b" FIFO-polls=");
                            write_decimal(trace.oob_fifo_poll_count as usize);
                            write_status(b"\r\nECCSEQSTATUS complete.\r\n> ");
                        }
                    } else if cfg!(feature = "nand-diagnostics") && command_is(line, b"ECCWRITE ON") {
                        crate::nand_fs::set_ecc_write_trace_enabled(true);
                        write_status(b"\r\nECCWRITE: enabled. Subsequent NAND page programs will be read back and checked.\r\n> ");
                    } else if cfg!(feature = "nand-diagnostics") && command_is(line, b"ECCWRITE OFF") {
                        crate::nand_fs::set_ecc_write_trace_enabled(false);
                        write_status(b"\r\nECCWRITE: disabled.\r\n> ");
                    } else if cfg!(feature = "nand-diagnostics") && command_is(line, b"ECCWRITESTATUS") {
                        let trace = crate::nand_fs::ecc_write_trace();
                        if trace.page == u32::MAX {
                            write_status(b"\r\nECCWRITESTATUS: no page program captured. Enable ECCWRITE before creating/writing a file.\r\n> ");
                        } else {
                            write_status(b"\r\nECCWRITESTATUS: last page program verification\r\nPage: 0x");
                            write_hex_u32(trace.page);
                            write_status(b" flags=0x");
                            write_hex_u32(trace.flags);
                            write_status(b" program-count=");
                            write_decimal(trace.program_count as usize);
                            write_status(b" mismatch-chunks=0x");
                            write_hex_u32(trace.mismatch_chunks);
                            write_status(b"\r\n  (flags: bit0=program-ok bit1=data-readback-ok bit2=OOB-readback-ok bit3=tag-ok)\r\n");
                            for chunk in 0..4 {
                                write_status(b"  chunk ");
                                write_decimal(chunk);
                                write_status(b" write-FSMC=0x");
                                write_hex_u32(trace.hardware[chunk]);
                                write_status(b" input-SW=0x");
                                write_hex_u32(trace.input_software[chunk]);
                                write_status(b" OOB-stored=0x");
                                write_hex_u32(trace.stored[chunk]);
                                write_status(b" readback-SW=0x");
                                write_hex_u32(trace.readback_software[chunk]);
                                write_status(b"\r\n    input-fp=0x");
                                write_hex_u32(trace.input_fingerprint[chunk]);
                                write_status(b" readback-fp=0x");
                                write_hex_u32(trace.readback_fingerprint[chunk]);
                                write_status(b"\r\n");
                            }
                            write_status(b"ECCWRITESTATUS complete. Diagnostic reads do not change NAND contents.\r\n> ");
                        }
                    } else if cfg!(feature = "nand-diagnostics") && command_is(line, b"NANDPAGEPROBE") {
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
                    } else if cfg!(feature = "nand-diagnostics") && command_is(line, b"NANDRESETTEST") {
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
                    } else if cfg!(feature = "nand-diagnostics") && command_is(line, b"NANDPROBE") {
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
                    } else if cfg!(feature = "nand-diagnostics") && line.len() > 8
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
                                write_status(b"Starting raw read ");
                                write_decimal(pass + 1);
                                write_status(b"...\r\n");
                                match crate::nand_fs::ecc_software_compare_page(page) {
                                    Ok(result) => {
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
                                    Err((stage, detail)) => {
                                        let description: &[u8] = match stage {
                                            0 => b"invalid physical page",
                                            1 => b"data-area page read failed or timed out",
                                            2 => b"OOB-area page read failed or timed out",
                                            3 => b"ECC tag mismatch (observed tag)",
                                            _ => b"unknown raw-read failure",
                                        };
                                        write_status(b"Read ");
                                        write_decimal(pass + 1);
                                        write_status(b" FAILED: ");
                                        write_status(description);
                                        write_status(b" detail=0x");
                                        write_hex_u32(detail);
                                        write_status(b"\r\n");
                                        break;
                                    }
                                }
                            }
                            write_status(b"ECCPAGE complete. Compare all three reads; repeat command if needed.\r\n> ");
                        }
                    } else if cfg!(feature = "nand-diagnostics") && line.len() > 10
                        && line[..10].eq_ignore_ascii_case(b"ECCPAGEHW ")
                    {
                        let Some(page) = parse_hex_u32(&line[10..]) else {
                            write_status(b"ECCPAGEHW: use ECCPAGEHW followed by a hexadecimal physical page number.\r\n> ");
                            LINE_LEN = 0;
                            continue;
                        };
                        if page >= crate::nand_fs::PHYSICAL_BLOCKS * crate::nand_fs::PAGES_PER_BLOCK {
                            write_status(b"ECCPAGEHW: physical page is out of range.\r\n> ");
                        } else {
                            write_status(b"\r\nECCPAGEHW: read-only; NAND RESET before each sample; no NAND writes or erases.\r\nPage: 0x");
                            write_hex_u32(page);
                            write_status(b"\r\n");
                            for pass in 0..3 {
                                write_status(b"Starting hardware-ECC read ");
                                write_decimal(pass + 1);
                                write_status(b"...\r\n");
                                match crate::nand_fs::ecc_hardware_compare_page(page) {
                                    Ok(result) => {
                                        write_status(b"Read ");
                                        write_decimal(pass + 1);
                                        write_status(b" reset-status=0x");
                                        write_hex_u32(result.reset_status as u32);
                                        write_status(b":\r\n");
                                        for chunk in 0..4 {
                                            write_status(b"  chunk ");
                                            write_decimal(chunk);
                                            write_status(b" stored=0x");
                                            write_hex_u32(result.stored[chunk]);
                                            write_status(b" FSMC=0x");
                                            write_hex_u32(result.hardware[chunk]);
                                            write_status(b" software=0x");
                                            write_hex_u32(result.software[chunk]);
                                            write_status(b" fingerprint=0x");
                                            write_hex_u32(result.data_fingerprint[chunk]);
                                            if result.stored[chunk] == result.hardware[chunk]
                                                && result.hardware[chunk] == result.software[chunk] {
                                                write_status(b" ALL-MATCH");
                                            } else if result.hardware[chunk] == result.software[chunk] {
                                                write_status(b" HW=SW");
                                            } else if result.stored[chunk] == result.hardware[chunk] {
                                                write_status(b" STORED=HW");
                                            } else if result.stored[chunk] == result.software[chunk] {
                                                write_status(b" STORED=SW");
                                            } else {
                                                write_status(b" ALL-DIFFER");
                                            }
                                            write_status(b"\r\n");
                                        }
                                    }
                                    Err((stage, detail)) => {
                                        let description: &[u8] = match stage {
                                            1 => b"RESET ready/busy timeout",
                                            2 => b"RESET status not ready",
                                            3 => b"page-read ready/busy timeout",
                                            4 => b"data FIFO wait failed (chunk index)",
                                            5 => b"OOB read ready/busy timeout",
                                            6 => b"OOB ECC tag mismatch (observed tag)",
                                            _ => b"invalid page or unknown diagnostic failure",
                                        };
                                        write_status(b"Read ");
                                        write_decimal(pass + 1);
                                        write_status(b" FAILED: ");
                                        write_status(description);
                                        write_status(b" detail=0x");
                                        write_hex_u32(detail);
                                        write_status(b"\r\n");
                                        break;
                                    }
                                }
                            }
                            write_status(b"ECCPAGEHW complete. Compare the three samples, especially chunk 3.\r\n> ");
                        }
                    } else if cfg!(feature = "nand-diagnostics") && command_is(line, b"ECCFILETEST") {
                        if let Some(fs) = fs {
                            write_status(b"\r\nECCFILETEST: writing deterministic 4096-byte file /eccdiag.bin...\r\n");
                            crate::dhara_fs::clear_storage_io_diagnostic();
                            crate::nand_fs::clear_nand_read_diagnostic();
                            let result = cdc_zmodem::run_ecc_storage_diagnostic(fs, zmodem);
                            match result {
                                0 => write_status(b"ECCFILETEST PASS: write, ECC-bypassed read, and ECC-checked read all matched. Temporary file removed.\r\n> "),
                                1 => write_status(b"ECCFILETEST FAIL stage 1: invalid test path.\r\n> "),
                                2 => write_status(b"ECCFILETEST FAIL stage 2: could not remove previous test file.\r\n> "),
                                3 => write_status(b"ECCFILETEST FAIL stage 3: create test file failed.\r\n> "),
                                4 => write_status(b"ECCFILETEST FAIL stage 4: write/flush failed.\r\n> "),
                                5 => write_status(b"ECCFILETEST FAIL stage 5: ECC-bypassed read failed or data mismatch.\r\n> "),
                                6 => write_status(b"ECCFILETEST FAIL stage 6: ECC-checked read failed or data mismatch.\r\n> "),
                                _ => write_status(b"ECCFILETEST: data checks passed, but temporary-file cleanup failed.\r\n> "),
                            }
                            let (stage, detail, page, stored, calculated) = crate::nand_fs::nand_read_diagnostic();
                            if stage != 0 {
                                write_status(b"NAND read stage: 0x");
                                write_hex_u32(stage);
                                write_status(b" detail: 0x");
                                write_hex_u32(detail);
                                write_status(b" physical page: 0x");
                                write_hex_u32(page);
                                write_status(b" stored ECC: 0x");
                                write_hex_u32(stored);
                                write_status(b" calculated ECC: 0x");
                                write_hex_u32(calculated);
                                write_status(b"\r\n");
                            }
                            let (operation, location, error) = crate::dhara_fs::storage_io_diagnostic();
                            if operation != 0 {
                                write_status(b"Dhara operation: 0x");
                                write_hex_u32(operation);
                                write_status(b" logical sector: 0x");
                                write_hex_u32(location);
                                write_status(b" error: 0x");
                                write_hex_u32(error);
                                write_status(b"\r\n> ");
                            }
                        } else {
                            write_status(b"Storage unavailable.\r\n> ");
                        }
                    } else if command_is(line, b"ECCSCAN") {
                        write_status(b"\r\nECCSCAN: read-only full physical NAND ECC scan. No NAND writes or erases.\r\n");
                        let vectors = crate::nand_ecc::ecc_software_selftest();
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
                                        Ok(result) => {
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
                                        Err(_) => {
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
                    } else if line.len() >= 7
                        && line[..7].iter().zip(b"DELETE").all(|(a, b)| a.to_ascii_uppercase() == *b)
                        && line[6] == b' '
                    {
                        if let Some(fs) = fs {
                            delete_file(fs, &line[7..]);
                        } else {
                            write_status(b"Storage unavailable.\r\n> ");
                        }
                    } else if command_is(line, b"LIST PROFILE") || command_is(line, b"LIST PROFILE SHORT") {
                        if let Some(fs) = fs {
                            #[cfg(feature = "list-profile")]
                            list_files(fs, !command_is(line, b"LIST PROFILE SHORT"), true);
                            #[cfg(not(feature = "list-profile"))]
                            write_status(b"LIST PROFILE requires firmware built with --features list-profile.\r\n> ");
                        } else {
                            write_status(b"Storage unavailable.\r\n> ");
                        }
                    } else if command_is(line, b"LIST") || command_is(line, b"LIST SHORT") {
                        if let Some(fs) = fs {
                            list_files(fs, !command_is(line, b"LIST SHORT"), false);
                        } else {
                            write_status(b"Storage unavailable.\r\n> ");
                        }
                    } else if command_is(line, b"VERSION") {
                        write_status(VERSION);
                    } else if command_is(line, b"FORMAT") {
                        write_status(
                            b"WARNING: FORMAT YES erases the NAND and creates a new Dhara/LittleFS volume.\r\nType FORMAT YES to confirm.\r\n> ",
                        );
                    } else if command_is(line, b"FORMAT LFS YES") {
                        write_status(b"LittleFS-only format requested. Dhara journal will be preserved; operation aborts if Dhara cannot resume.\r\n");
                        format_requested = FormatRequest::LittleFs;
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
                        format_requested = FormatRequest::WholeNand;
                    } else if LINE_LEN == 0 {
                        write_status(b"> ");
                    } else {
                        write_status(b"Unknown command. Type HELP.\r\n> ");
                    }
                    LINE_LEN = 0;
                    if format_requested != FormatRequest::None {
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
