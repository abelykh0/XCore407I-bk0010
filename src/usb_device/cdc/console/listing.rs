use super::output::{write_decimal_grouped, write_hex_u32, write_status};
use crate::nand_fs::ActiveStorage;
use littlefs2::fs::Filesystem;

#[cfg(feature = "list-profile")]
use super::output::write_decimal;

struct ListOutput {
    buffer: [u8; 1024],
    len: usize,
}

impl ListOutput {
    fn new() -> Self {
        Self {
            buffer: [0; 1024],
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

pub(super) fn list_files(fs: &Filesystem<'_, ActiveStorage>, show_sizes: bool, profile: bool) {
    let _ = profile;
    #[cfg(feature = "list-profile")]
    if profile {
        crate::usb_device::cdc::console::list_diag::begin();
    }

    #[cfg(feature = "list-profile")]
    let total_started = if profile {
        crate::usb_device::cdc::console::list_diag::cycle_count()
    } else {
        0
    };
    write_status(b"\r\nLIST: command accepted.\r\n");
    write_status(b"LittleFS total size: ");
    #[cfg(feature = "list-profile")]
    let total_space_started = if profile {
        crate::usb_device::cdc::console::list_diag::cycle_count()
    } else {
        0
    };
    let total_space = fs.total_space();
    #[cfg(feature = "list-profile")]
    let total_space_cycles = if profile {
        crate::usb_device::cdc::console::list_diag::cycle_count().wrapping_sub(total_space_started)
    } else {
        0
    };
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
        crate::usb_device::cdc::console::list_diag::set_directory_active(true);
    }
    #[cfg(feature = "list-profile")]
    let directory_started = if profile {
        crate::usb_device::cdc::console::list_diag::cycle_count()
    } else {
        0
    };
    let result = crate::nand_fs::read_root_directory(fs, show_sizes, |path_bytes, is_dir, size| {
        #[cfg(feature = "list-profile")]
        if is_dir {
            directory_count = directory_count.wrapping_add(1);
        }
        #[cfg(feature = "list-profile")]
        if !is_dir {
            file_count = file_count.wrapping_add(1);
        }
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
        crate::usb_device::cdc::console::list_diag::cycle_count().wrapping_sub(directory_started)
    } else {
        0
    };
    #[cfg(feature = "list-profile")]
    if profile {
        crate::usb_device::cdc::console::list_diag::set_directory_active(false);
    }
    #[cfg(feature = "list-profile")]
    let directory_stats = if profile {
        crate::usb_device::cdc::console::list_diag::snapshot()
    } else {
        crate::usb_device::cdc::console::list_diag::Snapshot {
            dhara_reads: 0,
            dhara_cycles: 0,
            nand_reads: 0,
            nand_bytes: 0,
            nand_cycles: 0,
            cdc_calls: 0,
            cdc_bytes: 0,
            cdc_cycles: 0,
            directory_cdc_cycles: 0,
        }
    };

    match &result {
        Ok(()) => write_status(b"LIST: directory read complete.\r\n"),
        Err(error) => {
            write_status(b"Directory read failed; LittleFS error: 0x");
            write_hex_u32(error.code() as u32);
            write_status(b"\r\n");
            let (operation, location, storage_error) = crate::dhara_fs::storage_io_diagnostic();
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
        let available_started = if profile {
            crate::usb_device::cdc::console::list_diag::cycle_count()
        } else {
            0
        };
        let available = fs.available_space();
        #[cfg(feature = "list-profile")]
        if profile {
            available_cycles = crate::usb_device::cdc::console::list_diag::cycle_count()
                .wrapping_sub(available_started);
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
        let total_cycles =
            crate::usb_device::cdc::console::list_diag::cycle_count().wrapping_sub(total_started);
        let stats = crate::usb_device::cdc::console::list_diag::end();
        report_list_profile(
            show_sizes,
            file_count,
            directory_count,
            total_cycles,
            total_space_cycles,
            directory_cycles,
            available_cycles,
            directory_stats,
            stats,
        );
    } else {
        write_status(b"> ");
    }
    #[cfg(not(feature = "list-profile"))]
    write_status(b"> ");
}

#[cfg(feature = "list-profile")]
fn report_list_profile(
    show_sizes: bool,
    files: u32,
    directories: u32,
    total_cycles: u32,
    total_space_cycles: u32,
    directory_cycles: u32,
    available_cycles: u32,
    directory_stats: crate::usb_device::cdc::console::list_diag::Snapshot,
    stats: crate::usb_device::cdc::console::list_diag::Snapshot,
) {
    fn cycles_to_us(cycles: u32) -> usize {
        ((cycles as u64 * 1_000_000) / crate::SYSTEM_CORE_CLOCK_HZ as u64) as usize
    }
    fn line(label: &[u8], cycles: u32) {
        write_status(label);
        write_decimal(cycles_to_us(cycles));
        write_status(b" us\r\n");
    }

    let dhara_overhead = stats.dhara_cycles.saturating_sub(stats.nand_cycles);
    let directory_residual = directory_cycles
        .saturating_sub(directory_stats.dhara_cycles)
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
    line(
        b"Directory residual (LittleFS + formatting estimate): ",
        directory_residual,
    );
    let available_dhara_reads = stats
        .dhara_reads
        .saturating_sub(directory_stats.dhara_reads);
    let available_dhara_cycles = stats
        .dhara_cycles
        .saturating_sub(directory_stats.dhara_cycles);
    let available_nand_reads = stats.nand_reads.saturating_sub(directory_stats.nand_reads);
    let available_nand_bytes = stats.nand_bytes.saturating_sub(directory_stats.nand_bytes);
    let available_nand_cycles = stats
        .nand_cycles
        .saturating_sub(directory_stats.nand_cycles);
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
