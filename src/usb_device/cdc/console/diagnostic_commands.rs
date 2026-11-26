use super::super::zmodem as cdc_zmodem;
use super::commands::{command_is, parse_hex_u32};
use super::output::{write_decimal, write_hex_u32, write_status};
use super::*;
use crate::nand_fs::ActiveStorage;
use littlefs2::fs::Filesystem;

/// Handle console commands that expose low-level NAND/ECC diagnostics.
pub(super) fn handle(
    line: &[u8],
    fs: Option<&Filesystem<'_, ActiveStorage>>,
    zmodem: &mut cdc_zmodem::State,
    _service_background: &mut dyn FnMut(),
) -> bool {
    if command_is(line, b"DHARA STATUS") {
        if fs.is_none() {
            write_status(b"\r\nDHARA STATUS: storage is not mounted.\r\n> ");
            return true;
        }
        let (allocated, capacity) = crate::dhara_fs::map_status();
        write_status(b"\r\nDHARA map: allocated sectors=");
        write_decimal(allocated as usize);
        write_status(b" capacity=");
        write_decimal(capacity as usize);
        write_status(b" logical free sectors=");
        write_decimal(capacity.saturating_sub(allocated) as usize);
        write_status(b"\r\nPhysical free pages are managed internally by Dhara.\r\n> ");
    } else if command_is(line, b"DHARA GC") {
        if fs.is_none() {
            write_status(b"\r\nDHARA GC: storage is not mounted.\r\n> ");
            return true;
        }
        // Reclaim up to four journal blocks before a large sequential write.
        // Each map_gc call relocates live pages and reclaims at most one block.
        let mut completed = 0u32;
        let mut erased_blocks = 0u32;
        let mut copied_pages = 0u32;
        let mut error = 0u32;
        for _ in 0..4 {
            let (ok, gc_error, erased, copied) = crate::dhara_fs::map_gc();
            erased_blocks = erased_blocks.saturating_add(erased);
            copied_pages = copied_pages.saturating_add(copied);
            if !ok {
                error = gc_error;
                break;
            }
            completed += 1;
        }
        write_status(b"\r\nDHARA GC: passes=");
        write_decimal(completed as usize);
        write_status(b" erased_blocks=");
        write_decimal(erased_blocks as usize);
        write_status(b" copied_pages=");
        write_decimal(copied_pages as usize);
        if error != 0 {
            write_status(b" error=0x");
            write_hex_u32(error);
        }
        write_status(b"\r\n> ");
    } else if command_is(line, b"ZCRC HW") {
        zmodem2::set_rx_crc32_hardware(true);
        zmodem2::reset_rx_crc32_diagnostics();
        write_status(b"\r\nZMODEM receive CRC32: batched hardware mode selected; counters reset.\r\n> ");
    } else if command_is(line, b"ZCRC SW") {
        zmodem2::set_rx_crc32_hardware(false);
        zmodem2::reset_rx_crc32_diagnostics();
        write_status(b"\r\nZMODEM receive CRC32: original byte-at-a-time software mode selected; counters reset.\r\n> ");
    } else if command_is(line, b"ZCRC RESET") {
        zmodem2::reset_rx_crc32_diagnostics();
        write_status(b"\r\nZMODEM receive CRC32 counters reset.\r\n> ");
    } else if command_is(line, b"ZCRC STATUS") {
        let (
            hardware,
            crc32_bytes,
            crc16_bytes,
            software_bytes,
            hardware_bytes,
            batches,
            passed,
            failed,
            hardware_words,
            hardware_fallback_bytes,
        ) = zmodem2::rx_crc32_diagnostics();
        write_status(b"\r\nZMODEM receive CRC32: mode=");
        write_status(if hardware { b"hardware\r\n" } else { b"software\r\n" });
        write_status(b"  crc32_input_bytes=");
        write_decimal(crc32_bytes as usize);
        write_status(b" crc16_input_bytes=");
        write_decimal(crc16_bytes as usize);
        write_status(b" software_bytes=");
        write_decimal(software_bytes as usize);
        write_status(b" batched_bytes=");
        write_decimal(hardware_bytes as usize);
        write_status(b" batches=");
        write_decimal(batches as usize);
        write_status(b" crc_ok=");
        write_decimal(passed as usize);
        write_status(b" crc_errors=");
        write_decimal(failed as usize);
        write_status(b"\r\n  CRC_DR_words=");
        write_decimal(hardware_words as usize);
        write_status(b" hardware_fallback_bytes=");
        write_decimal(hardware_fallback_bytes as usize);
        write_status(b"\r\n> ");
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
            write_status(
                b"ECCWRITESTATUS complete. Diagnostic reads do not change NAND contents.\r\n> ",
            );
        }
    } else if cfg!(feature = "nand-diagnostics") && command_is(line, b"NANDPAGEPROBE") {
        write_status(b"\r\nNANDPAGEPROBE: read-only raw page reads; no NAND writes or erases.\r\n");
        let (ok, data) = crate::nand_fs::ecc_scan_page_probe();
        let labels = [
            b"Page 0 col 0: " as &[u8],
            b"Page 0 col 1: ",
            b"Page 0 col 512: ",
            b"Page 0x83 col 0: ",
            b"Page 0x83 col 1: ",
            b"Page 0x83 col 512: ",
            b"Page 0 spare: ",
            b"Page 0x83 spare: ",
        ];
        for i in 0..8 {
            write_status(labels[i]);
            if ok[i] {
                for (j, byte) in data[i].iter().enumerate() {
                    if j != 0 {
                        write_status(b" ");
                    }
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
            b"Page 0 col 0" as &[u8],
            b"Page 0 col 1",
            b"Page 0 col 512",
            b"Page 0 spare",
            b"Page 0x83 col 0",
            b"Page 0x83 col 1",
            b"Page 0x83 col 512",
            b"Page 0x83 spare",
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
                        if j != 0 {
                            write_status(b" ");
                        }
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
        write_status(if ready {
            b"  R/B=READY\r\n"
        } else {
            b"  R/B=BUSY\r\n"
        });
        write_status(b"> ");
    } else if cfg!(feature = "nand-diagnostics")
        && line.len() > 8
        && line[..8].eq_ignore_ascii_case(b"ECCPAGE ")
    {
        let Some(page) = parse_hex_u32(&line[8..]) else {
            write_status(
                b"ECCPAGE: use ECCPAGE followed by a hexadecimal physical page number.\r\n> ",
            );
            unsafe {
                LINE_LEN = 0;
            }
            return true;
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
            write_status(
                b"ECCPAGE complete. Compare all three reads; repeat command if needed.\r\n> ",
            );
        }
    } else if cfg!(feature = "nand-diagnostics")
        && line.len() > 10
        && line[..10].eq_ignore_ascii_case(b"ECCPAGEHW ")
    {
        let Some(page) = parse_hex_u32(&line[10..]) else {
            write_status(
                b"ECCPAGEHW: use ECCPAGEHW followed by a hexadecimal physical page number.\r\n> ",
            );
            unsafe {
                LINE_LEN = 0;
            }
            return true;
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
                                && result.hardware[chunk] == result.software[chunk]
                            {
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
            write_status(
                b"ECCPAGEHW complete. Compare the three samples, especially chunk 3.\r\n> ",
            );
        }
    } else if cfg!(feature = "nand-diagnostics") && command_is(line, b"ECCFILETEST") {
        if let Some(fs) = fs {
            write_status(
                b"\r\nECCFILETEST: writing deterministic 4096-byte file /eccdiag.bin...\r\n",
            );
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
        #[cfg(feature = "nand-diagnostics")]
        {
            write_status(b"\r\nECCSCAN: read-only full physical NAND ECC scan. No NAND writes or erases.\r\n");
            let vectors = crate::nand_ecc::ecc_software_selftest();
            let vectors_ok = vectors == [0x00FF_FFFF, 0x00FF_FFFF, 0x00AA_AAAA, 0x0069_5659];
            write_status(if vectors_ok {
                b"Software ECC reference vectors: PASS\r\n"
            } else {
                b"Software ECC reference vectors: FAIL\r\n"
            });

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
            let marker_classification_sane = good_blocks == crate::nand_fs::PHYSICAL_BLOCKS
                && bad_blocks == 0
                && block_read_errors == 0;
            if !marker_classification_sane {
                write_status(b"WARNING: marker scan classified every physical block as bad (or found no good blocks). Treat marker results as INVALID; do not format or retire blocks from this scan.\r\n");
            }
            write_status(
                if vectors_ok
                    && marker_classification_sane
                    && mismatching_chunks == 0
                    && page_read_errors == 0
                    && block_read_errors == 0
                {
                    b"ECCSCAN PASSED. NAND was not modified.\r\n> "
                } else {
                    b"ECCSCAN completed with diagnostic findings. NAND was not modified.\r\n> "
                },
            );
        }
        #[cfg(not(feature = "nand-diagnostics"))]
        write_status(b"ECCSCAN requires nand-diagnostics.\r\n> ");
    }
    command_is(line, b"ZCRC HW")
        || command_is(line, b"ZCRC SW")
        || command_is(line, b"ZCRC STATUS")
        || command_is(line, b"ZCRC RESET")
}
