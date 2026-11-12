//! LittleFS block device for the XCore407I Samsung K9F1G08U0E NAND.
//!
//! Geometry: 2 KiB data pages, 64 pages per 128 KiB erase block, 1,024
//! physical blocks. Sixteen good blocks are held in reserve for bad-block
//! tolerance; LittleFS sees 1,008 logical blocks.
//!
//! This first implementation uses raw NAND data with ECC disabled. It keeps
//! factory bad blocks out of the logical address space but does not yet
//! implement ECC correction or grown-bad-block remapping.

use core::ptr::{read_volatile, write_volatile};
use stm32f4::stm32f407 as pac;

use littlefs2::{
    consts::{U2048, U32},
    driver::Storage,
    fs::Filesystem,
    io::{Error as LfsError, Result as LfsResult},
};

const FSMC_BASE: usize = 0xA000_0000;
const NAND_BASE: usize = 0x7000_0000;

// STM32F407 FSMC NAND Bank 2 registers.
const FSMC_PCR2: *mut u32 = (FSMC_BASE + 0x60) as *mut u32;
const FSMC_ECCR2: *const u32 = (FSMC_BASE + 0x54) as *const u32;
const FSMC_PCR_ECCEN: u32 = 1 << 6;
const FSMC_PCR_ECCPS_MASK: u32 = 0b111 << 17;
const FSMC_ECC_CHUNK_SIZE: usize = 512;
const FSMC_ECC_CHUNKS_PER_PAGE: usize = PAGE_SIZE / FSMC_ECC_CHUNK_SIZE;
const _: () = assert!(FSMC_ECC_CHUNKS_PER_PAGE == 4);

const RB_MASK: u32 = 1 << 6;
const CMD: *mut u8 = (NAND_BASE | (1 << 17)) as *mut u8;
const ADDR: *mut u8 = (NAND_BASE | (1 << 16)) as *mut u8;
const DATA: *mut u8 = NAND_BASE as *mut u8;
const TIMEOUT: u32 = 20_000_000;

pub const PAGE_SIZE: usize = 2048;
pub const SPARE_SIZE: usize = 64;
pub const PAGES_PER_BLOCK: u32 = 64;
pub const PHYSICAL_BLOCKS: u32 = 1024;
pub const ERASE_BLOCK_SIZE: usize = PAGE_SIZE * PAGES_PER_BLOCK as usize;
pub const RESERVED_BLOCKS: usize = 16;
pub const LOGICAL_BLOCKS: usize = PHYSICAL_BLOCKS as usize - RESERVED_BLOCKS;

/// Keep the 2 KiB logical-to-physical map in static RAM, not on the
/// small startup stack while LittleFS is being mounted/formatted.
static mut PHYSICAL_BLOCK_MAP: [u16; LOGICAL_BLOCKS] = [0; LOGICAL_BLOCKS];

pub struct NandStorage {
    physical_blocks: &'static mut [u16; LOGICAL_BLOCKS],
    good_blocks: usize,
    factory_bad_blocks: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InitError {
    Timeout,
    WrongDevice,
    MarkerRead,
    TooManyBadBlocks,
    MountFailed,
}

fn command(value: u8) { unsafe { write_volatile(CMD, value) } }
fn address(value: u8) { unsafe { write_volatile(ADDR, value) } }
fn write_data(value: u8) { unsafe { write_volatile(DATA, value) } }
fn read_data() -> u8 { unsafe { read_volatile(DATA) } }

/// Prepare the FSMC ECC engine for one 512-byte data chunk.
/// Not connected to page I/O until OOB storage and legacy-page handling land.
#[allow(dead_code)]
fn ecc_begin_chunk() {
    unsafe {
        let pcr = read_volatile(FSMC_PCR2);
        write_volatile(FSMC_PCR2, pcr & !FSMC_PCR_ECCEN);
        write_volatile(
            FSMC_PCR2,
            (pcr & !FSMC_PCR_ECCPS_MASK) | (1 << 17) | FSMC_PCR_ECCEN,
        );
        let _ = read_volatile(FSMC_PCR2);
    }
}

/// Read the 24-bit ECC result and disable the engine before reuse.
#[allow(dead_code)]
fn ecc_end_chunk() -> u32 {
    unsafe {
        let ecc = read_volatile(FSMC_ECCR2) & 0x00FF_FFFF;
        let pcr = read_volatile(FSMC_PCR2);
        write_volatile(FSMC_PCR2, pcr & !FSMC_PCR_ECCEN);
        let _ = read_volatile(FSMC_PCR2);
        ecc
    }
}

fn wait_ready_status() -> bool {
    for _ in 0..TIMEOUT {
        command(0x70);
        if read_data() & 0x40 != 0 { return true; }
        core::hint::spin_loop();
    }
    false
}

fn wait_ready_rb() -> bool {
    let mut busy_seen = false;
    for _ in 0..TIMEOUT {
        let ready = unsafe { read_volatile(pac::GPIOD::ptr().cast::<u8>().add(0x10).cast::<u32>()) } & RB_MASK != 0;
        if !ready {
            busy_seen = true;
        } else if busy_seen {
            return true;
        }
        core::hint::spin_loop();
    }
    false
}

fn read_page(page: u32, column: u16, out: &mut [u8]) -> bool {
    if out.is_empty() || column as usize + out.len() > PAGE_SIZE + SPARE_SIZE {
        return false;
    }
    command(0x00);
    address(column as u8);
    address((column >> 8) as u8);
    address(page as u8);
    address((page >> 8) as u8);
    command(0x30);
    if !wait_ready_rb() { return false; }
    for byte in out { *byte = read_data(); }
    true
}

fn erase_physical_block(block: u16) -> bool {
    let first_page = block as u32 * PAGES_PER_BLOCK;
    command(0x60);
    address(first_page as u8);
    address((first_page >> 8) as u8);
    command(0xD0);
    if !wait_ready_rb() { return false; }
    command(0x70);
    read_data() & 0x01 == 0
}

fn program_physical_page(page: u32, data: &[u8]) -> bool {
    if data.len() != PAGE_SIZE { return false; }
    command(0x80);
    address(0);
    address(0);
    address(page as u8);
    address((page >> 8) as u8);
    // K9F1G08U0E tADL minimum is 100 ns.
    cortex_m::asm::delay(1_000);
    for &byte in data { write_data(byte); }
    command(0x10);
    if !wait_ready_rb() { return false; }
    command(0x70);
    read_data() & 0x01 == 0
}

fn configure_fsmc() {
    unsafe {
        let rcc = &*pac::RCC::ptr();
        rcc.ahb3enr().modify(|_, w| w.fsmcen().set_bit());
        let _ = rcc.ahb3enr().read().bits();
        let pcr2 = FSMC_PCR2;
        let timing = 4 | (8 << 8) | (4 << 16) | (2 << 24);
        write_volatile(pcr2, 0);
        write_volatile((FSMC_BASE + 0x68) as *mut u32, timing);
        write_volatile((FSMC_BASE + 0x6C) as *mut u32, timing);
        // NAND bank 2, 8-bit bus, NWAIT enabled. Configure a 512-byte
        // ECC chunk size, but leave ECCEN clear until OOB handling is ready.
        let control = (1 << 1) | (1 << 2) | (1 << 3) | (2 << 9) | (2 << 13) | (1 << 17);
        write_volatile(pcr2, control);
        let _ = read_volatile(pcr2);
    }
}

impl NandStorage {
    /// Reset the NAND and build a logical-to-physical map from factory markers.
    /// Fails closed unless at least 1,008 factory-good blocks are available.
    pub fn new() -> Result<Self, InitError> {
        configure_fsmc();
        command(0xFF);
        if !wait_ready_status() { return Err(InitError::Timeout); }

        command(0x90);
        address(0);
        let id = [read_data(), read_data(), read_data(), read_data(), read_data()];
        if id[0] != 0xEC || id[1] != 0xF1 {
            return Err(InitError::WrongDevice);
        }

        // The map contains 1,008 u16 entries (2,016 bytes). Keep it out of
        // the call stack: this routine runs alongside the USB polling ISR.
        // Only one NandStorage instance is created for this firmware lifetime.
        let physical_blocks = unsafe {
            &mut *core::ptr::addr_of_mut!(PHYSICAL_BLOCK_MAP)
        };
        physical_blocks.fill(0);
        let mut storage = Self {
            physical_blocks,
            good_blocks: 0,
            factory_bad_blocks: 0,
        };
        let mut marker = [0u8; 1];

        for block in 0..PHYSICAL_BLOCKS {
            let first = block * PAGES_PER_BLOCK;
            let second = first + 1;
            if !read_page(first, PAGE_SIZE as u16, &mut marker) {
                return Err(InitError::MarkerRead);
            }
            let first_good = marker[0] == 0xFF;
            if !read_page(second, PAGE_SIZE as u16, &mut marker) {
                return Err(InitError::MarkerRead);
            }
            if first_good && marker[0] == 0xFF {
                if storage.good_blocks < LOGICAL_BLOCKS {
                    storage.physical_blocks[storage.good_blocks] = block as u16;
                    storage.good_blocks += 1;
                }
            } else {
                storage.factory_bad_blocks += 1;
            }
        }

        if storage.good_blocks != LOGICAL_BLOCKS {
            return Err(InitError::TooManyBadBlocks);
        }
        Ok(storage)
    }

    pub fn factory_bad_blocks(&self) -> u16 { self.factory_bad_blocks }
    pub fn logical_capacity(&self) -> usize { LOGICAL_BLOCKS * ERASE_BLOCK_SIZE }

    fn physical_block(&self, logical: usize) -> Option<u16> {
        self.physical_blocks.get(logical).copied()
    }
}

impl Storage for NandStorage {
    type CACHE_SIZE = U2048;
    type LOOKAHEAD_SIZE = U32;

    const READ_SIZE: usize = PAGE_SIZE;
    const WRITE_SIZE: usize = PAGE_SIZE;
    const BLOCK_SIZE: usize = ERASE_BLOCK_SIZE;
    const BLOCK_COUNT: usize = LOGICAL_BLOCKS;
    const BLOCK_CYCLES: isize = 100;

    fn read(&mut self, off: usize, buf: &mut [u8]) -> LfsResult<usize> {
        if off % PAGE_SIZE != 0 || buf.len() % PAGE_SIZE != 0 {
            return Err(LfsError::IO);
        }
        let capacity = Self::BLOCK_COUNT * ERASE_BLOCK_SIZE;
        if off.checked_add(buf.len()).filter(|&end| end <= capacity).is_none() {
            return Err(LfsError::IO);
        }

        for (index, page_buf) in buf.chunks_exact_mut(PAGE_SIZE).enumerate() {
            let absolute = off + index * PAGE_SIZE;
            let logical_block = absolute / ERASE_BLOCK_SIZE;
            let page_in_block = (absolute % ERASE_BLOCK_SIZE) / PAGE_SIZE;
            let physical = self.physical_block(logical_block).ok_or(LfsError::IO)?;
            let page = physical as u32 * PAGES_PER_BLOCK + page_in_block as u32;
            if !read_page(page, 0, page_buf) { return Err(LfsError::IO); }
        }
        Ok(buf.len())
    }

    fn write(&mut self, off: usize, data: &[u8]) -> LfsResult<usize> {
        if off % PAGE_SIZE != 0 || data.is_empty() || data.len() % PAGE_SIZE != 0 {
            return Err(LfsError::IO);
        }
        let capacity = Self::BLOCK_COUNT * Self::BLOCK_SIZE;
        if off.checked_add(data.len()).filter(|&end| end <= capacity).is_none() {
            return Err(LfsError::IO);
        }

        for (index, page_data) in data.chunks_exact(PAGE_SIZE).enumerate() {
            let absolute = off + index * PAGE_SIZE;
            let logical_block = absolute / ERASE_BLOCK_SIZE;
            let page_in_block = (absolute % ERASE_BLOCK_SIZE) / PAGE_SIZE;
            let physical = self.physical_block(logical_block).ok_or(LfsError::IO)?;
            let page = physical as u32 * PAGES_PER_BLOCK + page_in_block as u32;
            if !program_physical_page(page, page_data) { return Err(LfsError::IO); }
        }
        Ok(data.len())
    }

    fn erase(&mut self, off: usize, len: usize) -> LfsResult<usize> {
        if off % ERASE_BLOCK_SIZE != 0 || len == 0 || len % ERASE_BLOCK_SIZE != 0 {
            return Err(LfsError::IO);
        }
        let capacity = Self::BLOCK_COUNT * Self::BLOCK_SIZE;
        if off.checked_add(len).filter(|&end| end <= capacity).is_none() {
            return Err(LfsError::IO);
        }

        for logical in (off / ERASE_BLOCK_SIZE)..((off + len) / ERASE_BLOCK_SIZE) {
            let physical = self.physical_block(logical).ok_or(LfsError::IO)?;
            if !erase_physical_block(physical) { return Err(LfsError::IO); }
        }
        Ok(len)
    }
}

/// Keep the LittleFS mount alive for the duration of the supplied service closure.
///
/// NAND initialization and all filesystem operations run in the caller's
/// context (never in the USB polling ISR). This is deliberately mount-only:
/// it never formats or erases the NAND.
pub fn with_mounted<R>(
    service: impl FnOnce(&Filesystem<'_, NandStorage>) -> LfsResult<R>,
) -> Result<R, InitError> {
    let mut storage = NandStorage::new()?;
    if !Filesystem::<NandStorage>::is_mountable(&mut storage) {
        return Err(InitError::MountFailed);
    }
    Filesystem::<NandStorage>::mount_and_then(&mut storage, service)
        .map_err(|_| InitError::MountFailed)
}

/// Mount the existing volume once, publish its diagnostic report, and keep
/// the filesystem mounted for the entire service closure.
///
/// Unlike initialize_and_mount_report(), this does not mount and immediately
/// unmount before the caller starts using the filesystem. It never formats.
pub fn with_mounted_report<R>(
    service: impl FnOnce(&Filesystem<'_, NandStorage>, &[u8], usize) -> LfsResult<R>,
) -> Result<R, InitError> {
    let mut storage = NandStorage::new()?;
    if !Filesystem::<NandStorage>::is_mountable(&mut storage) {
        return Err(InitError::MountFailed);
    }

    let mut report = [0u8; 192];
    let mut len = 0;
    append(&mut report, &mut len, b"XCore407I NAND / LittleFS\r\n");
    append(&mut report, &mut len, b"Factory bad blocks: ");
    append_hex(&mut report, &mut len, storage.factory_bad_blocks() as u32, 4);
    append(&mut report, &mut len, b"\r\n");
    append(&mut report, &mut len, b"LittleFS: existing volume mounted\r\n");
    append(&mut report, &mut len, b"Capacity: ");

    Filesystem::<NandStorage>::mount_and_then(&mut storage, |fs| {
        append_hex(&mut report, &mut len, fs.total_space() as u32, 8);
        append(&mut report, &mut len, b" bytes\r\nFree: ");
        append_hex(
            &mut report,
            &mut len,
            fs.available_space().unwrap_or(0) as u32,
            8,
        );
        append(&mut report, &mut len, b" bytes\r\n");
        append(&mut report, &mut len, b"ECC: prepared (not yet applied)\r\n");
        service(fs, &report, len)
    })
    .map_err(|_| InitError::MountFailed)
}

/// Set true only for the initial provisioning flash. After README reports
/// "LittleFS: formatted and mounted", change this to false and rebuild.
/// With this false, an absent/corrupt filesystem is never reformatted.
const FORMAT_NAND_ONCE: bool = false;

/// Initialize NAND, perform the explicitly enabled initial format if needed,
/// then mount. Normal boots with FORMAT_NAND_ONCE=false are mount-only.
pub fn initialize_and_mount_report() -> ([u8; 192], usize) {
    let mut report = [0u8; 192];
    let mut len = 0;
    append(&mut report, &mut len, b"XCore407I NAND / LittleFS\r\n");

    let mut storage = match NandStorage::new() {
        Ok(storage) => storage,
        Err(error) => {
            append(&mut report, &mut len, b"NAND init failed: ");
            append_init_error(&mut report, &mut len, error);
            append(&mut report, &mut len, b"\r\n");
            return (report, len);
        }
    };

    append(&mut report, &mut len, b"Factory bad blocks: ");
    append_hex(&mut report, &mut len, storage.factory_bad_blocks() as u32, 4);
    append(&mut report, &mut len, b"\r\n");

    let mountable = Filesystem::<NandStorage>::is_mountable(&mut storage);
    let mut formatted_now = false;

    if !mountable {
        if !FORMAT_NAND_ONCE {
            append(&mut report, &mut len, b"ERROR: no valid LittleFS; not formatted\r\n");
            return (report, len);
        }

        append(&mut report, &mut len, b"Initial LittleFS format...\r\n");
        if Filesystem::<NandStorage>::format(&mut storage).is_err() {
            append(&mut report, &mut len, b"ERROR: format failed\r\n");
            return (report, len);
        }
        formatted_now = true;
    }

    match Filesystem::<NandStorage>::mount_and_then(&mut storage, |fs| {
        Ok((fs.total_space(), fs.available_space().unwrap_or(0)))
    }) {
        Ok((total, free)) => {
            if formatted_now {
                append(&mut report, &mut len, b"LittleFS: formatted and mounted\r\n");
            } else {
                append(&mut report, &mut len, b"LittleFS: existing volume mounted\r\n");
            }
            append(&mut report, &mut len, b"Capacity: ");
            append_hex(&mut report, &mut len, total as u32, 8);
            append(&mut report, &mut len, b" bytes\r\nFree: ");
            append_hex(&mut report, &mut len, free as u32, 8);
            append(&mut report, &mut len, b" bytes\r\n");
            append(&mut report, &mut len, b"ECC: prepared (not yet applied)\r\n");
        }
        Err(_) => append(&mut report, &mut len, b"ERROR: mount failed; NAND unchanged\r\n"),
    }
    (report, len)
}

fn append(out: &mut [u8; 192], len: &mut usize, bytes: &[u8]) {
    let count = bytes.len().min(out.len() - *len);
    out[*len..*len + count].copy_from_slice(&bytes[..count]);
    *len += count;
}

fn append_hex(out: &mut [u8; 192], len: &mut usize, value: u32, digits: usize) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for shift in (0..digits).rev() {
        let nibble = ((value >> (shift * 4)) & 0xF) as usize;
        append(out, len, &HEX[nibble..nibble + 1]);
    }
}

fn append_init_error(out: &mut [u8; 192], len: &mut usize, error: InitError) {
    let message = match error {
        InitError::Timeout => b"timeout" as &[u8],
        InitError::WrongDevice => b"wrong device",
        InitError::MarkerRead => b"bad-block marker read",
        InitError::TooManyBadBlocks => b"not enough good blocks",
        InitError::MountFailed => b"LittleFS mount failed",
    };
    append(out, len, message);
}
