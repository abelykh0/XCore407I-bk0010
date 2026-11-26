use super::super::zmodem as cdc_zmodem;
use super::diagnostic_commands;
use super::diagnostics::write_cdc_diagnostics;
use super::listing::list_files;
use super::output::{write_decimal, write_hex_u32, write_status};
use super::*;
use crate::{nand_fs::ActiveStorage, tusb};
use core::ffi::c_void;
use littlefs2::{fs::Filesystem, path::Path};

#[cfg(feature = "list-profile")]
use super::diagnostics::write_write_profile;

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
            let (operation, location, storage_error) = crate::dhara_fs::storage_io_diagnostic();
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

pub(super) fn command_is(line: &[u8], command: &[u8]) -> bool {
    line.len() == command.len()
        && line
            .iter()
            .zip(command)
            .all(|(a, b)| a.to_ascii_uppercase() == *b)
}

pub(super) fn parse_hex_u32(bytes: &[u8]) -> Option<u32> {
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

        // Match the startup banner's lifecycle: report only from the normal
        // foreground console path, once CDC is connected and its data endpoints
        // are ready. The ZMODEM state retains the record until the whole line
        // has been accepted into the CDC TX FIFO.
        if tusb::tud_cdc_n_ready(0) {
            let _ = zmodem.report_receive_complete();
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
                        let started =
                            if fs.is_some() && cdc_zmodem::start(zmodem, true, now_ms).is_ok() {
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
                        #[cfg(feature = "list-profile")]
                        write_status(HELP_WRITE_PROFILE);
                    } else if line.len() > 5 && line[..5].eq_ignore_ascii_case(b"SEND ") {
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
                    } else if cfg!(feature = "list-profile") && command_is(line, b"WRITE PROFILE") {
                        #[cfg(feature = "list-profile")]
                        write_write_profile(zmodem);
                    } else if command_is(line, b"IPCONFIG") {
                        let phy = crate::ethernet::probe_dp83848();
                        if phy.mdio_timeout {
                            write_status(
                                b"\r\nEthernet: MDIO timeout; PHY status unavailable.\r\n> ",
                            );
                        } else if !phy.found {
                            write_status(b"\r\nEthernet: DP83848 not detected. Check PHY power and MDIO wiring.\r\n> ");
                        } else {
                            write_status(b"\r\nEthernet: link ");
                            write_status(if phy.physts & 1 != 0 { b"up" } else { b"down" });
                            if phy.physts & 1 != 0 {
                                write_status(if phy.physts & (1 << 1) != 0 {
                                    b", 10 Mb/s" as &[u8]
                                } else {
                                    b", 100 Mb/s" as &[u8]
                                });
                                write_status(if phy.physts & (1 << 2) != 0 {
                                    b", full duplex" as &[u8]
                                } else {
                                    b", half duplex" as &[u8]
                                });
                            }
                            write_status(b"\r\nMAC/DMA: ");
                            write_status(if crate::ethernet::is_initialized() {
                                b"initialized"
                            } else {
                                b"not initialized"
                            });
                            write_status(b"\r\nIPv4: ");
                            if let Some((address, prefix)) = crate::ethernet::ipv4_config() {
                                for (index, octet) in address.iter().enumerate() {
                                    if index != 0 {
                                        write_status(b".");
                                    }
                                    write_decimal(*octet as usize);
                                }
                                write_status(b"/");
                                write_decimal(prefix as usize);
                            } else {
                                write_status(b"acquiring via DHCP");
                            }
                            write_status(b"\r\n> ");
                        }
                    } else if command_is(line, b"STATUS") {
                        write_status(if fs.is_some() {
                            STATUS_MOUNTED
                        } else {
                            STATUS_UNAVAILABLE
                        });
                        write_cdc_diagnostics();
                        let (stage, offset, total, length, written, sends) =
                            crate::ethernet::http_upload_diagnostics();
                        write_status(b"HTTP upload: stage=");
                        write_decimal(stage as usize);
                        write_status(b" offset=");
                        write_decimal(offset as usize);
                        write_status(b"/");
                        write_decimal(total as usize);
                        write_status(b" chunk=");
                        write_decimal(length as usize);
                        write_status(b" written=");
                        write_decimal(written as usize);
                        write_status(b" send=");
                        write_decimal(sends as usize);
                        write_status(b"\r\n");
                        let (
                            poll_stage,
                            polls,
                            receives,
                            rx_bytes,
                            no_send,
                            no_recv,
                            expected,
                            queue,
                            tcp_state,
                        ) = crate::ethernet::http_poll_diagnostics();
                        write_status(b"HTTP socket: stage=");
                        write_decimal(poll_stage as usize);
                        write_status(b" polls=");
                        write_decimal(polls as usize);
                        write_status(b" recv_calls=");
                        write_decimal(receives as usize);
                        write_status(b" rx_bytes=");
                        write_decimal(rx_bytes as usize);
                        write_status(b" no_send=");
                        write_decimal(no_send as usize);
                        write_status(b" no_recv=");
                        write_decimal(no_recv as usize);
                        write_status(b" expected=");
                        write_decimal(expected as usize);
                        write_status(b" queue=");
                        write_decimal(queue as usize);
                        write_status(b" tcp_state=");
                        write_decimal(tcp_state as usize);
                        write_status(b"\r\n");
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
                                1 => b"Malformed encoding",
                                2 => b"Malformed file size",
                                3 => b"Malformed filename",
                                4 => b"Malformed frame",
                                5 => b"Malformed header",
                                6 => b"Malformed packet",
                                7 => b"Invalid receiver state",
                                8 => b"Backpressure",
                                9 => b"Receiver buffer exhausted",
                                10 => b"CRC-16 mismatch",
                                11 => b"CRC-32 mismatch",
                                12 => b"Unexpected EOF",
                                13 => b"Unsupported feature",
                                14 => b"Data subpacket exceeded receiver capacity (ZRPOS retry)",
                                _ => b"Unclassified protocol error",
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
                                lfs_error,
                                operation,
                                location,
                                storage_error,
                                nand_stage,
                                nand_detail,
                                nand_page,
                                nand_stored_ecc,
                                nand_calculated_ecc,
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
                    } else if diagnostic_commands::handle(line, fs, zmodem, _service_background) {
                        // Diagnostic handler printed its response.
                    } else if line.len() >= 7
                        && line[..7]
                            .iter()
                            .zip(b"DELETE")
                            .all(|(a, b)| a.to_ascii_uppercase() == *b)
                        && line[6] == b' '
                    {
                        if let Some(fs) = fs {
                            delete_file(fs, &line[7..]);
                        } else {
                            write_status(b"Storage unavailable.\r\n> ");
                        }
                    } else if command_is(line, b"LIST PROFILE")
                        || command_is(line, b"LIST PROFILE SHORT")
                    {
                        if let Some(_fs) = fs {
                            #[cfg(feature = "list-profile")]
                            list_files(_fs, !command_is(line, b"LIST PROFILE SHORT"), true);
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
                        write_status(b"\r\nXCore407I BK-0010 firmware v");
                        write_status(env!("CARGO_PKG_VERSION").as_bytes());
                        write_status(b"\r\n> ");
                    } else if command_is(line, b"FORMAT") {
                        write_status(
                            b"WARNING: FORMAT YES erases the NAND and creates a new Dhara/LittleFS volume.\r\nType FORMAT YES to confirm.\r\n> ",
                        );
                    } else if command_is(line, b"FORMAT LFS YES") {
                        write_status(b"LittleFS-only format requested. Dhara journal will be preserved; operation aborts if Dhara cannot resume.\r\n");
                        format_requested = FormatRequest::LittleFs;
                    } else if command_is(line, b"FORMAT YES") {
                        if fs.is_some() {
                            write_status(b"Unmounting LittleFS before NAND format...\r\n");
                        } else {
                            write_status(b"Storage is unavailable. Formatting NAND...\r\n");
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
