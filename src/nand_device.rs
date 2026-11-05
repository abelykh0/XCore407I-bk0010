//! Read-only raw NAND access for the XCore407I K9F1G08U0E.
//!
//! This layer deliberately exposes page reads only. It does not erase or
//! program NAND and does not claim data integrity until ECC is implemented.
//! Callers must configure the FSMC NAND pins (AF12) before constructing it.
//!
//! Geometry: 2 KiB data + 64-byte spare, 64 pages/block, 1,024 blocks.

use core::ptr::{read_volatile, write_volatile};

pub const PAGE_DATA_BYTES: usize = 2048;
pub const PAGE_SPARE_BYTES: usize = 64;
pub const PAGE_TOTAL_BYTES: usize = PAGE_DATA_BYTES + PAGE_SPARE_BYTES;
pub const PAGES_PER_BLOCK: u32 = 64;
pub const BLOCK_COUNT: u32 = 1024;
pub const PAGE_COUNT: u32 = PAGES_PER_BLOCK * BLOCK_COUNT;

const FSMC_BASE: usize = 0xA000_0000;
const NAND_BASE: usize = 0x7000_0000;
const GPIOD_IDR: *const u32 = 0x4002_0C10 as *const u32;
const NAND_RB_MASK: u32 = 1 << 6;
const CMD_ADDR: *mut u8 = (NAND_BASE | (1 << 17)) as *mut u8;
const ADDR_ADDR: *mut u8 = (NAND_BASE | (1 << 16)) as *mut u8;
const DATA_ADDR: *mut u8 = NAND_BASE as *mut u8;
const TIMEOUT: u32 = 20_000_000;

const CMD_READ0: u8 = 0x00;
const CMD_READSTART: u8 = 0x30;
const CMD_READID: u8 = 0x90;
const CMD_RESET: u8 = 0xFF;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NandError {
    Timeout,
    WrongDevice { manufacturer: u8, device: u8 },
    PageOutOfRange,
    ColumnOutOfRange,
    InvalidBufferLength,
}

/// K9F1G08U0E raw NAND controller.
pub struct Nand {
    id: [u8; 5],
}

impl Nand {
    /// Configure FSMC bank 2, reset the NAND and verify its JEDEC ID.
    ///
    /// GPIO pins must already be configured for FSMC AF12. This function does
    /// not change GPIO configuration and performs no erase/program commands.
    pub fn initialize() -> Result<Self, NandError> {
        configure_fsmc();
        write_cmd(CMD_RESET);
        wait_ready_status()?;

        let mut id = [0u8; 5];
        write_cmd(CMD_READID);
        write_addr(0);
        for byte in &mut id {
            *byte = read_data();
        }

        if id[0] != 0xEC || id[1] != 0xF1 {
            return Err(NandError::WrongDevice {
                manufacturer: id[0],
                device: id[1],
            });
        }

        Ok(Self { id })
    }

    pub fn id(&self) -> [u8; 5] {
        self.id
    }

    /// Read a contiguous range from a NAND page, including its spare area.
    ///
    /// Column is in the range 0..=2111. The output must fit entirely inside
    /// the 2,112-byte page. This is a raw read: no ECC correction is applied.
    pub fn read_bytes(
        &mut self,
        page: u32,
        column: u16,
        out: &mut [u8],
    ) -> Result<(), NandError> {
        if page >= PAGE_COUNT {
            return Err(NandError::PageOutOfRange);
        }
        let start = column as usize;
        if start >= PAGE_TOTAL_BYTES {
            return Err(NandError::ColumnOutOfRange);
        }
        if out.is_empty() || start + out.len() > PAGE_TOTAL_BYTES {
            return Err(NandError::InvalidBufferLength);
        }

        write_cmd(CMD_READ0);
        write_addr(column as u8);
        write_addr((column >> 8) as u8);
        write_addr(page as u8);
        write_addr((page >> 8) as u8);
        write_cmd(CMD_READSTART);
        wait_ready_rb()?;

        for byte in out {
            *byte = read_data();
        }
        Ok(())
    }

    /// Check the factory bad-block marker in the first two pages' spare areas.
    ///
    /// A block is considered factory-good only when both marker bytes are
    /// 0xFF. This does not detect grown bad blocks or perform ECC validation.
    pub fn is_factory_good_block(&mut self, block: u32) -> Result<bool, NandError> {
        if block >= BLOCK_COUNT {
            return Err(NandError::PageOutOfRange);
        }

        let mut marker = [0u8; 1];
        for page_in_block in 0..2 {
            let page = block * PAGES_PER_BLOCK + page_in_block;
            self.read_bytes(page, PAGE_DATA_BYTES as u16, &mut marker)?;
            if marker[0] != 0xFF {
                return Ok(false);
            }
        }
        Ok(true)
    }
}

fn write_cmd(value: u8) {
    unsafe { write_volatile(CMD_ADDR, value) };
}

fn write_addr(value: u8) {
    unsafe { write_volatile(ADDR_ADDR, value) };
}

fn read_data() -> u8 {
    unsafe { read_volatile(DATA_ADDR) }
}

fn wait_ready_rb() -> Result<(), NandError> {
    // Require a complete busy-to-ready transition; an idle-high R/B level
    // alone is not proof that the page-read cycle has completed.
    let mut saw_busy = false;
    for _ in 0..TIMEOUT {
        let ready = unsafe { read_volatile(GPIOD_IDR) } & NAND_RB_MASK != 0;
        if !ready {
            saw_busy = true;
        } else if saw_busy {
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err(NandError::Timeout)
}

fn wait_ready_status() -> Result<(), NandError> {
    for _ in 0..TIMEOUT {
        write_cmd(0x70);
        if read_data() & 0x40 != 0 {
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err(NandError::Timeout)
}

fn configure_fsmc() {
    unsafe {
        let rcc_ahb3enr = 0x4002_3838 as *mut u32;
        write_volatile(rcc_ahb3enr, read_volatile(rcc_ahb3enr) | 1);

        let pcr2 = (FSMC_BASE + 0x60) as *mut u32;
        let timing = 4 | (8 << 8) | (4 << 16) | (2 << 24);
        write_volatile(pcr2, 0);
        write_volatile((FSMC_BASE + 0x68) as *mut u32, timing);
        write_volatile((FSMC_BASE + 0x6C) as *mut u32, timing);

        // NAND bank 2, 8-bit bus, ECC disabled for this raw-read stage,
        // FSMC wait feature enabled for the PD6 R/B/NWAIT connection.
        let control = (1 << 1) | (1 << 2) | (1 << 3) | (2 << 9) | (2 << 13);
        write_volatile(pcr2, control);
        let _ = read_volatile(pcr2);
    }
}

/// Non-destructive startup diagnostic. Checks the device ID and scans factory
/// bad-block markers; it never issues program or erase commands.
pub fn run_read_only_diagnostic() {
    use crate::camera::show_usb_status;

    show_usb_status(b"NAND: READ-ONLY INIT");
    let mut nand = match Nand::initialize() {
        Ok(device) => device,
        Err(NandError::Timeout) => {
            show_usb_status(b"NAND: INIT TIMEOUT");
            return;
        }
        Err(NandError::WrongDevice { .. }) => {
            show_usb_status(b"NAND: WRONG DEVICE");
            return;
        }
        Err(_) => {
            show_usb_status(b"NAND: INIT ERROR");
            return;
        }
    };

    let mut good = 0u16;
    let mut bad = 0u16;
    for block in 0..BLOCK_COUNT {
        match nand.is_factory_good_block(block) {
            Ok(true) => good += 1,
            Ok(false) => bad += 1,
            Err(_) => {
                show_usb_status(b"NAND: MARKER READ FAIL");
                return;
            }
        }
    }

    // Display hexadecimal counts without allocating or formatting strings.
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let mut line = *b"NAND G:0000 B:0000";
    line[7] = HEX[((good >> 12) & 0xF) as usize];
    line[8] = HEX[((good >> 8) & 0xF) as usize];
    line[9] = HEX[((good >> 4) & 0xF) as usize];
    line[10] = HEX[(good & 0xF) as usize];
    line[14] = HEX[((bad >> 12) & 0xF) as usize];
    line[15] = HEX[((bad >> 8) & 0xF) as usize];
    line[16] = HEX[((bad >> 4) & 0xF) as usize];
    line[17] = HEX[(bad & 0xF) as usize];
    show_usb_status(&line);
}
