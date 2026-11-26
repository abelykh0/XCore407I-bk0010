use super::output::{write_decimal, write_hex_u32, write_status};
use super::*;

#[cfg(feature = "list-profile")]
use super::super::zmodem as cdc_zmodem;

/// Append a snapshot of CDC connection and transmit-path diagnostics to STATUS.
pub(super) fn write_cdc_diagnostics() {
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
            let (stage, detail, page, stored_ecc, calculated_ecc) =
                crate::nand_fs::nand_read_diagnostic();
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
            let (device_stage, device_location, device_status) =
                crate::nand_fs::nand_device_diagnostic();
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
        FsAccessError::Operation(code) => {
            write_status(b"\r\nStorage service operation failed; LittleFS error code 0x");
            write_hex_u32(code as u32);
            write_status(b". NAND state was not verified.\r\n");
        }
    }
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
        format_stage,
        format_location,
        format_detail,
        format_return,
        provision_phase,
        bad_calls,
        first_bad_block,
        bad_reason,
        is_bad_calls,
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

    let (operation, location, storage_error) = crate::dhara_fs::storage_io_diagnostic();
    if operation != 0 {
        write_status(b"Dhara operation 0x");
        write_hex_u32(operation);
        write_status(b", logical sector 0x");
        write_hex_u32(location);
        write_status(b", error 0x");
        write_hex_u32(storage_error);
        write_status(b"\r\n");
    }

    let (stage, detail, page, stored_ecc, calculated_ecc) = crate::nand_fs::nand_read_diagnostic();
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

#[cfg(feature = "list-profile")]
pub(super) fn write_write_profile(zmodem: &cdc_zmodem::State) {
    let p = crate::usb_device::cdc::console::write_diag::snapshot();
    let (error, detail, wire_received, wire_consumed) = zmodem.protocol_diagnostics();
    let dhara_residual = p
        .dhara_write_us
        .saturating_sub(p.nand_program_us)
        .saturating_sub(p.nand_erase_us);
    write_status(b"\r\nWRITE PROFILE (last completed receive)\r\n");
    write_status(b"Transfer: ");
    write_decimal((p.transfer_us / 1_000) as usize);
    write_status(b" ms  ZMODEM wire RX/consumed: ");
    write_decimal(wire_received as usize);
    write_status(b"/");
    write_decimal(wire_consumed as usize);
    write_status(b" bytes\r\n");
    write_status(b"ZMODEM poll: ");
    write_decimal(p.zmodem_poll_calls as usize);
    write_status(b" calls, ");
    write_decimal(p.zmodem_poll_us as usize);
    write_status(b" us; submit: ");
    write_decimal(p.zmodem_submit_calls as usize);
    write_status(b" calls, ");
    write_decimal(p.zmodem_submit_us as usize);
    write_status(b" us\r\n");
    write_status(b"LittleFS create: ");
    write_decimal(p.lfs_create_us as usize);
    write_status(b" us; append writes: ");
    write_decimal(p.lfs_append_calls as usize);
    write_status(b" calls / ");
    write_decimal(p.lfs_append_us as usize);
    write_status(b" us total\r\n");
    write_status(b"  append open: ");
    write_decimal(p.lfs_open_us as usize);
    write_status(b" us; write callback: ");
    write_decimal(p.lfs_callback_us as usize);
    write_status(b" us; close: ");
    write_decimal(p.lfs_close_us as usize);
    write_status(b" us; commit/rename: ");
    write_decimal(p.lfs_commit_us as usize);
    write_status(b" us\r\n");
    write_status(b"LittleFS write: ");
    write_decimal(p.lfs_write_calls as usize);
    write_status(b" calls / ");
    write_decimal(p.lfs_write_bytes as usize);
    write_status(b" bytes / ");
    write_decimal(p.lfs_write_us as usize);
    write_status(b" us\r\n");
    write_status(b"Dhara map_write: ");
    write_decimal(p.dhara_write_calls as usize);
    write_status(b" calls / ");
    write_decimal(p.dhara_write_pages as usize);
    write_status(b" pages / ");
    write_decimal(p.dhara_write_us as usize);
    write_status(b" us\r\n");
    write_status(b"NAND page program: ");
    write_decimal(p.nand_program_calls as usize);
    write_status(b" calls / ");
    write_decimal(p.nand_program_us as usize);
    write_status(b" us; FSMC FIFO waits: ");
    write_decimal(p.nand_fifo_wait_us as usize);
    write_status(b" us; NAND busy: ");
    write_decimal(p.nand_busy_wait_us as usize);
    write_status(b" us\r\n");
    let ecc_fsmc_us = p
        .nand_program_us
        .saturating_sub(p.nand_fifo_wait_us)
        .saturating_sub(p.nand_busy_wait_us);
    write_status(b"  ECC/FSMC data feed + setup (derived): ");
    write_decimal(ecc_fsmc_us as usize);
    write_status(b" us\r\n");
    write_status(b"NAND erase: ");
    write_decimal(p.nand_erase_calls as usize);
    write_status(b" calls / ");
    write_decimal(p.nand_erase_us as usize);
    write_status(b" us\r\n");
    write_status(b"Dhara residual (map time minus program/erase): ");
    write_decimal(dhara_residual as usize);
    write_status(b" us (approx.)\r\n");
    if error != 0 {
        write_status(b"Last protocol error: ");
        write_decimal(error as usize);
        write_status(b" detail=");
        write_decimal(detail as usize);
        write_status(b"\r\n");
    }
    write_status(b"Times are accumulated per operation; LittleFS/Dhara/NAND rows overlap and must not be added.\r\n> ");
}
