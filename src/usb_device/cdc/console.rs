//! CDC console lifecycle and command service.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::tusb;

mod commands;
mod diagnostic_commands;
mod diagnostics;
mod listing;
mod output;

#[cfg(feature = "list-profile")]
pub(crate) mod list_diag;
#[cfg(feature = "list-profile")]
pub(crate) mod write_diag;

pub use commands::{FormatRequest, service};
pub use diagnostics::{
    report_format_complete, report_format_failure, report_format_status, report_storage_error,
};
use output::write_status_counted;
pub(crate) use output::{
    write_receive_complete, write_receive_summary, write_send_complete, write_status,
};

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
const HELP: &[u8] = br#"
Commands:
  HELP                Show this help
  STATUS              Show USB/storage status
  IPCONFIG            Show Ethernet link and IP configuration
  LIST                List files with sizes (including hidden files)
  LIST SHORT          List files without sizes (faster comparison)
  LIST PROFILE        Profile LittleFS, Dhara, NAND, and CDC time
  LIST PROFILE SHORT  Profile listing without file sizes
  DELETE name         Delete a file from the LittleFS root
  SEND name           Send a LittleFS file to the terminal via ZMODEM
  DHARA STATUS        Show LittleFS/Dhara allocation and NAND operation counters
  DHARA RECLAIM       Trim unused LittleFS blocks and synchronize Dhara
  ECCSCAN             Scan all physical NAND blocks for ECC mismatches (read-only)
  VERSION             Show firmware target
  FORMAT              Show NAND format warning
  FORMAT YES          Erase and reinitialize NAND (destructive)
  FORMAT LFS YES      Reformat LittleFS only; preserve Dhara journal
> "#;
fn write_help() {
    let mut start = 0;
    for (index, byte) in HELP.iter().enumerate() {
        if *byte == b'\n' {
            if index > start {
                write_status(&HELP[start..index]);
            }
            write_status(b"\r\n");
            start = index + 1;
        }
    }
    if start < HELP.len() {
        write_status(&HELP[start..]);
    }
}

#[cfg(feature = "list-profile")]
const HELP_WRITE_PROFILE: &[u8] = b"  WRITE PROFILE Show last ZMODEM receive timing breakdown\r\n";
#[cfg(feature = "nand-diagnostics")]
const HELP_NAND_DIAGNOSTICS: &[u8] = b"  ECCFILETEST Write/read a 4 KiB ECC test file\r\n  ECCPAGE    Compare one physical page repeatedly\r\n  ECCPAGEHW  Compare stored, hardware, and software ECC\r\n  ECCSEQ ON/OFF Arm/disarm program-sequence trace\r\n  ECCSEQSTATUS Show captured FSMC ECC/FIFO sequence\r\n  ECCWRITE ON/OFF Enable/disable page-program verification\r\n  ECCWRITESTATUS Show last verified page program\r\n  NANDRESETTEST Repeat raw reads with NAND RESET\r\n  NANDPAGEPROBE / NANDPROBE Low-level NAND diagnostics\r\n";
const STATUS_MOUNTED: &[u8] =
    b"\r\nUSB: High-Speed UVC camera + CDC serial\r\nStorage: LittleFS mounted for CDC session\r\n";
const STATUS_UNAVAILABLE: &[u8] =
    b"\r\nUSB: High-Speed UVC camera + CDC serial\r\nStorage: unavailable\r\n";

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
