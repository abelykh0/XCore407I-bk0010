//! Low-level raw NAND driver for the XCore407I Samsung K9F1G08U0E.
//!
//! Dhara is the sole logical storage layer and owns physical block mapping,
//! bad-block handling, and wear leveling. This module provides the NAND
//! command, erase, page-program, and page-read primitives used by Dhara.
//!
//! Geometry: 2 KiB data pages, 64 pages per 128 KiB erase block, and 1,024
//! physical blocks. Page programs store the STM32 FSMC's four 512-byte
//! Hamming ECC values and an ECC tag in the spare area. Reads correct
//! single-bit data errors and reject uncorrectable ECC syndromes.

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicU32, Ordering};
use stm32f4::stm32f407 as pac;

use littlefs2::{
    fs::Filesystem,
    io::Result as LfsResult,
};

const FSMC_BASE: usize = 0xA000_0000;
const NAND_BASE: usize = 0x7000_0000;

// STM32F407 FSMC NAND Bank 2 registers.
const FSMC_PCR2: *mut u32 = (FSMC_BASE + 0x60) as *mut u32;
// Bank 2 ECCR2 is at offset 0x74 (PCR2 +0x60, SR2 +0x64,
// PMEM2 +0x68, PATT2 +0x6C, reserved +0x70, ECCR2 +0x74).
const FSMC_ECCR2: *const u32 = (FSMC_BASE + 0x74) as *const u32;
const FSMC_PCR_ECCEN: u32 = 1 << 6;
const FSMC_PCR_ECCPS_MASK: u32 = 0b111 << 17;
const FSMC_ECC_CHUNK_SIZE: usize = 512;
const FSMC_ECC_CHUNKS_PER_PAGE: usize = PAGE_SIZE / FSMC_ECC_CHUNK_SIZE;
const _: () = assert!(FSMC_ECC_CHUNKS_PER_PAGE == 4);

pub(crate) const RB_MASK: u32 = 1 << 6;
pub(crate) const CMD: *mut u8 = (NAND_BASE | (1 << 17)) as *mut u8;
pub(crate) const ADDR: *mut u8 = (NAND_BASE | (1 << 16)) as *mut u8;
pub(crate) const DATA: *mut u8 = NAND_BASE as *mut u8;
const TIMEOUT: u32 = 20_000_000;

pub const PAGE_SIZE: usize = 2048;
pub const SPARE_SIZE: usize = 64;
pub const PAGES_PER_BLOCK: u32 = 64;
pub const PHYSICAL_BLOCKS: u32 = 1024;
pub const ERASE_BLOCK_SIZE: usize = PAGE_SIZE * PAGES_PER_BLOCK as usize;
pub type ActiveStorage = crate::dhara_fs::DharaStorage;

// First failure inside the raw NAND page-read/ECC path.
// Stages: 1=data read ready timeout, 2=OOB read timeout,
// 3=invalid ECC tag, 4=uncorrectable ECC syndrome.
static NAND_READ_FAILURE_STAGE: AtomicU32 = AtomicU32::new(0);
static NAND_READ_FAILURE_DETAIL: AtomicU32 = AtomicU32::new(0);
static NAND_READ_FAILURE_PAGE: AtomicU32 = AtomicU32::new(0);
static NAND_READ_FAILURE_STORED_ECC: AtomicU32 = AtomicU32::new(0);
static NAND_READ_FAILURE_CALCULATED_ECC: AtomicU32 = AtomicU32::new(0);

fn record_nand_read_failure(stage: u32, detail: u32) {
    if NAND_READ_FAILURE_STAGE.compare_exchange(0, stage, Ordering::AcqRel, Ordering::Relaxed).is_ok() {
        NAND_READ_FAILURE_DETAIL.store(detail, Ordering::Release);
    }
}

fn record_ecc_read_failure(page: u32, chunk: u32, stored: u32, calculated: u32) {
    if NAND_READ_FAILURE_STAGE.compare_exchange(0, 4, Ordering::AcqRel, Ordering::Relaxed).is_ok() {
        NAND_READ_FAILURE_PAGE.store(page, Ordering::Relaxed);
        NAND_READ_FAILURE_STORED_ECC.store(stored, Ordering::Relaxed);
        NAND_READ_FAILURE_CALCULATED_ECC.store(calculated, Ordering::Relaxed);
        NAND_READ_FAILURE_DETAIL.store((chunk << 24) | ((stored ^ calculated) & 0x00FF_FFFF), Ordering::Release);
    }
}

pub(crate) fn clear_nand_read_diagnostic() {
    NAND_READ_FAILURE_STAGE.store(0, Ordering::Release);
    NAND_READ_FAILURE_DETAIL.store(0, Ordering::Release);
    NAND_READ_FAILURE_PAGE.store(0, Ordering::Release);
    NAND_READ_FAILURE_STORED_ECC.store(0, Ordering::Release);
    NAND_READ_FAILURE_CALCULATED_ECC.store(0, Ordering::Release);
}

pub(crate) fn nand_read_diagnostic() -> (u32, u32, u32, u32, u32) {
    let stage = NAND_READ_FAILURE_STAGE.load(Ordering::Acquire);
    (
        stage,
        NAND_READ_FAILURE_DETAIL.load(Ordering::Acquire),
        NAND_READ_FAILURE_PAGE.load(Ordering::Relaxed),
        NAND_READ_FAILURE_STORED_ECC.load(Ordering::Relaxed),
        NAND_READ_FAILURE_CALCULATED_ECC.load(Ordering::Relaxed),
    )
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InitError {
    Timeout,
    WrongDevice,
    MarkerRead,
    MountFailed,
    DharaNoJournal,
    DharaCapacity,
    ProvisionFailed,
}

pub(crate) fn command(value: u8) { unsafe { write_volatile(CMD, value) } }
pub(crate) fn address(value: u8) { unsafe { write_volatile(ADDR, value) } }
pub(crate) fn write_data(value: u8) { unsafe { write_volatile(DATA, value) } }
pub(crate) fn read_data() -> u8 { unsafe { read_volatile(DATA) } }

/// Prepare the FSMC ECC engine for one 512-byte data chunk.
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
fn ecc_end_chunk() -> u32 {
    unsafe {
        let ecc = read_volatile(FSMC_ECCR2) & 0x00FF_FFFF;
        let pcr = read_volatile(FSMC_PCR2);
        write_volatile(FSMC_PCR2, pcr & !FSMC_PCR_ECCEN);
        let _ = read_volatile(FSMC_PCR2);
        ecc
    }
}

pub(crate) fn wait_ready_status() -> bool {
    for _ in 0..TIMEOUT {
        command(0x70);
        if read_data() & 0x40 != 0 { return true; }
        core::hint::spin_loop();
    }
    false
}

pub(crate) fn wait_ready_rb() -> bool {
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

pub(crate) fn read_page(page: u32, column: u16, out: &mut [u8]) -> bool {
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

pub(crate) fn erase_physical_block(block: u16) -> bool {
    let first_page = block as u32 * PAGES_PER_BLOCK;
    command(0x60);
    address(first_page as u8);
    address((first_page >> 8) as u8);
    command(0xD0);
    if !wait_ready_rb() { return false; }
    command(0x70);
    read_data() & 0x01 == 0
}

// OOB bytes 0..1 are reserved for the NAND factory bad-block marker.
// Bytes 2..13 hold four 3-byte ECC values; bytes 14..15 identify ECC pages.
const ECC_OOB_OFFSET: u16 = 2;
const ECC_TAG: [u8; 2] = [0xA5, 0x5A];
const ECC_BYTES_PER_CHUNK: usize = 3;
const ECC_OOB_BYTES: usize = FSMC_ECC_CHUNKS_PER_PAGE * ECC_BYTES_PER_CHUNK;

fn encode_ecc(ecc: u32, out: &mut [u8]) {
    out[0] = ecc as u8;
    out[1] = (ecc >> 8) as u8;
    out[2] = (ecc >> 16) as u8;
}

pub(crate) fn program_physical_page(page: u32, data: &[u8]) -> bool {
    if data.len() != PAGE_SIZE { return false; }

    command(0x80);
    address(0);
    address(0);
    address(page as u8);
    address((page >> 8) as u8);
    // K9F1G08U0E tADL minimum is 100 ns.
    cortex_m::asm::delay(1_000);

    let mut ecc = [0u8; ECC_OOB_BYTES];
    for (chunk_index, chunk) in data.chunks_exact(FSMC_ECC_CHUNK_SIZE).enumerate() {
        ecc_begin_chunk();
        for &byte in chunk { write_data(byte); }
        encode_ecc(ecc_end_chunk(), &mut ecc[chunk_index * ECC_BYTES_PER_CHUNK..][..ECC_BYTES_PER_CHUNK]);
    }

    // Continue the same sequential page-program operation into the spare area.
    // The two leading 0xFF bytes leave the factory marker positions untouched.
    write_data(0xFF);
    write_data(0xFF);
    for &byte in &ecc { write_data(byte); }
    for &byte in &ECC_TAG { write_data(byte); }

    command(0x10);
    if !wait_ready_rb() { return false; }
    command(0x70);
    read_data() & 0x01 == 0
}

/// Read a full data page, computing ECC as the four 512-byte chunks are read.
/// Pages without the tag are treated as legacy raw-NAND pages.
///
/// Apply the STM32 FSMC 512-byte Hamming syndrome to a data chunk.
///
/// The 24-bit syndrome is twelve 2-bit parity pairs. The first three pairs
/// encode the data-bit position; the remaining nine encode the byte address.
/// A data-bit error has 01 or 10 in every pair. A one-bit syndrome is an
/// ECC-bit error and does not require changing the data.
fn correct_ecc_chunk(data: &mut [u8], stored: u32, calculated: u32) -> bool {
    if data.len() != FSMC_ECC_CHUNK_SIZE {
        return false;
    }

    let syndrome = (stored ^ calculated) & 0x00FF_FFFF;
    if syndrome == 0 {
        return true;
    }

    // A single changed ECC bit is a parity-bit error, not a data-bit error.
    if syndrome.count_ones() == 1 {
        return true;
    }

    // Decode the three low parity pairs as the bit index (P4, P2, P1).
    let mut bit_index = 0usize;
    for bit in 0..3 {
        match (syndrome >> (bit * 2)) & 0x03 {
            0b01 => {}
            0b10 => bit_index |= 1 << bit,
            _ => return false,
        }
    }

    // The remaining nine pairs encode the byte address (P8 through P1024).
    let mut byte_index = 0usize;
    for bit in 0..9 {
        match (syndrome >> (6 + bit * 2)) & 0x03 {
            0b01 => {}
            0b10 => byte_index |= 1 << bit,
            _ => return false,
        }
    }

    if byte_index >= data.len() {
        return false;
    }

    data[byte_index] ^= 1 << bit_index;
    true
}

pub(crate) fn read_data_page(page: u32, out: &mut [u8]) -> bool {
    if out.len() != PAGE_SIZE {
        return false;
    }

    command(0x00);
    address(0);
    address(0);
    address(page as u8);
    address((page >> 8) as u8);
    command(0x30);
    if !wait_ready_rb() {
        record_nand_read_failure(1, page);
        return false;
    }

    let mut calculated = [0u8; ECC_OOB_BYTES];
    for (chunk_index, chunk) in out.chunks_exact_mut(FSMC_ECC_CHUNK_SIZE).enumerate() {
        ecc_begin_chunk();
        for byte in chunk {
            *byte = read_data();
        }
        encode_ecc(ecc_end_chunk(), &mut calculated[chunk_index * ECC_BYTES_PER_CHUNK..][..ECC_BYTES_PER_CHUNK]);
    }

    let mut stored = [0u8; ECC_OOB_BYTES + ECC_TAG.len()];
    if !read_page(page, PAGE_SIZE as u16 + ECC_OOB_OFFSET, &mut stored) {
        record_nand_read_failure(2, page);
        return false;
    }

    let tag = &stored[ECC_OOB_BYTES..];
    if tag == [0xFF, 0xFF] {
        return true;
    }
    if tag != ECC_TAG {
        record_nand_read_failure(3, u32::from(tag[0]) | (u32::from(tag[1]) << 8));
        return false;
    }

    for (chunk_index, chunk) in out.chunks_exact_mut(FSMC_ECC_CHUNK_SIZE).enumerate() {
        let offset = chunk_index * ECC_BYTES_PER_CHUNK;
        let stored_ecc = u32::from_le_bytes([
            stored[offset],
            stored[offset + 1],
            stored[offset + 2],
            0,
        ]);
        let calculated_ecc = u32::from_le_bytes([
            calculated[offset],
            calculated[offset + 1],
            calculated[offset + 2],
            0,
        ]);
        if !correct_ecc_chunk(chunk, stored_ecc, calculated_ecc) {
            record_ecc_read_failure(page, chunk_index as u32, stored_ecc, calculated_ecc);
            return false;
        }
    }
    true
}


pub(crate) fn configure_fsmc() {
    unsafe {
        let rcc = &*pac::RCC::ptr();
        rcc.ahb3enr().modify(|_, w| w.fsmcen().set_bit());
        let _ = rcc.ahb3enr().read().bits();
        let pcr2 = FSMC_PCR2;
        let timing = 4 | (8 << 8) | (4 << 16) | (2 << 24);
        write_volatile(pcr2, 0);
        write_volatile((FSMC_BASE + 0x68) as *mut u32, timing);
        write_volatile((FSMC_BASE + 0x6C) as *mut u32, timing);
        // NAND bank 2, 8-bit bus, NWAIT enabled. ECC is enabled only around
        // individual 512-byte data chunks; ECCEN remains clear while idle.
        let control = (1 << 1) | (1 << 2) | (1 << 3) | (2 << 9) | (2 << 13) | (1 << 17);
        write_volatile(pcr2, control);
        let _ = read_volatile(pcr2);
    }
}

/// Keep the LittleFS mount alive for the duration of the supplied service closure.
///
/// NAND initialization and filesystem operations run in caller context,
/// never in a USB ISR. Dhara-backed storage is mount-only.
/// Failure stage for a read/write operation on the active LittleFS volume.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FsAccessError {
    Init(InitError),
    Mount(i32),
    Operation,
}

/// Mount the active volume and preserve whether failure occurred during NAND
/// initialization, mount, or the caller's filesystem operation.
pub fn with_mounted_detailed<R>(
    service: impl FnOnce(&Filesystem<'_, ActiveStorage>) -> LfsResult<R>,
) -> Result<R, FsAccessError> {
    let mut storage = ActiveStorage::new().map_err(FsAccessError::Init)?;
    // Mount directly rather than using is_mountable(): the latter only
    // returns a bool and discards LittleFS's actual error code.
    // Nest the service result so a caller-operation error remains distinct.
    let mounted = Filesystem::<ActiveStorage>::mount_and_then(&mut storage, |fs| {
        Ok(service(fs))
    }).map_err(|error| FsAccessError::Mount(error.code()))?;
    mounted.map_err(|_| FsAccessError::Operation)
}

/// Compatibility wrapper for existing callers that only need success/failure.
pub fn with_mounted<R>(
    service: impl FnOnce(&Filesystem<'_, ActiveStorage>) -> LfsResult<R>,
) -> Result<R, InitError> {
    with_mounted_detailed(service).map_err(|_| InitError::MountFailed)
}

/// Mount the existing volume once, publish its diagnostic report, and keep
/// the filesystem mounted for the entire service closure.
///
/// Unlike initialize_and_mount_report(), this does not mount and immediately
/// unmount before the caller starts using the filesystem.
pub fn with_mounted_report<R>(
    service: impl FnOnce(&Filesystem<'_, ActiveStorage>, &[u8], usize) -> LfsResult<R>,
) -> Result<R, InitError> {
    let mut storage = ActiveStorage::new()?;
    if !Filesystem::<ActiveStorage>::is_mountable(&mut storage) {
        return Err(InitError::MountFailed);
    }

    let mut report = [0u8; 256];
    let mut len = 0;
    append(&mut report, &mut len, b"XCore407I NAND / LittleFS\r\n");
    append(&mut report, &mut len, b"Factory bad blocks: ");
    append_hex(&mut report, &mut len, storage.factory_bad_blocks() as u32, 4);
    append(&mut report, &mut len, b"\r\n");
    append(&mut report, &mut len, b"LittleFS: existing volume mounted\r\n");
    append(&mut report, &mut len, b"Capacity: ");

    Filesystem::<ActiveStorage>::mount_and_then(&mut storage, |fs| {
        append_hex(&mut report, &mut len, fs.total_space() as u32, 8);
        append(&mut report, &mut len, b" bytes\r\nFree: ");
        append_hex(
            &mut report,
            &mut len,
            fs.available_space().unwrap_or(0) as u32,
            8,
        );
        append(&mut report, &mut len, b" bytes\r\n");
        append(&mut report, &mut len, b"ECC: single-bit correction; uncorrectable errors rejected\r\n");
        service(fs, &report, len)
    })
    .map_err(|_| InitError::MountFailed)
}

/// Initialize NAND and mount the existing LittleFS volume. This routine
/// never formats or erases NAND.
pub fn initialize_and_mount_report() -> ([u8; 256], usize) {
    let mut report = [0u8; 256];
    let mut len = 0;
    append(&mut report, &mut len, b"XCore407I NAND / LittleFS\r\n");

    let mut storage = match ActiveStorage::new() {
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

    if !Filesystem::<ActiveStorage>::is_mountable(&mut storage) {
        append(&mut report, &mut len, b"ERROR: no valid LittleFS; NAND unchanged\r\n");
        return (report, len);
    }

    match Filesystem::<ActiveStorage>::mount_and_then(&mut storage, |fs| {
        Ok((fs.total_space(), fs.available_space().unwrap_or(0)))
    }) {
        Ok((total, free)) => {
            append(&mut report, &mut len, b"LittleFS: existing volume mounted\r\n");
            append(&mut report, &mut len, b"Capacity: ");
            append_hex(&mut report, &mut len, total as u32, 8);
            append(&mut report, &mut len, b" bytes\r\nFree: ");
            append_hex(&mut report, &mut len, free as u32, 8);
            append(&mut report, &mut len, b" bytes\r\n");
            append(&mut report, &mut len, b"ECC: single-bit correction; uncorrectable errors rejected\r\n");
        }
        Err(_) => append(&mut report, &mut len, b"ERROR: mount failed; NAND unchanged\r\n"),
    }
    (report, len)
}

fn append(out: &mut [u8; 256], len: &mut usize, bytes: &[u8]) {
    let count = bytes.len().min(out.len() - *len);
    out[*len..*len + count].copy_from_slice(&bytes[..count]);
    *len += count;
}

fn append_hex(out: &mut [u8; 256], len: &mut usize, value: u32, digits: usize) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for shift in (0..digits).rev() {
        let nibble = ((value >> (shift * 4)) & 0xF) as usize;
        append(out, len, &HEX[nibble..nibble + 1]);
    }
}

fn append_init_error(out: &mut [u8; 256], len: &mut usize, error: InitError) {
    let message = match error {
        InitError::Timeout => b"timeout" as &[u8],
        InitError::WrongDevice => b"wrong device",
        InitError::MarkerRead => b"bad-block marker read",
        InitError::MountFailed => b"LittleFS mount failed",
        InitError::DharaNoJournal => b"Dhara journal missing",
        InitError::DharaCapacity => b"Dhara capacity too small",
        InitError::ProvisionFailed => b"Dhara provisioning failed",
    };
    append(out, len, message);
}


/// Summary of a completed NAND/Dhara/LittleFS provisioning operation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FormatReport {
    pub physical_blocks: u32,
    pub bad_blocks: u16,
    pub good_blocks: u32,
    pub page_size_bytes: u32,
    pub pages_per_block: u32,
    pub erase_block_bytes: u32,
    pub littlefs_blocks: u32,
    pub littlefs_capacity_bytes: u32,
}

/// Initialize an unprovisioned NAND after explicit user confirmation.
/// This erases every non-bad physical block, creates a new Dhara journal,
/// formats LittleFS, and verifies that the new filesystem can be mounted.
pub fn initialize_nand_volume() -> Result<(), InitError> {
    initialize_nand_volume_with_progress(|_, _| {}).map(|_| ())
}

/// CDC variant: invoke progress after each physical block, allowing the
/// caller to service USB/video between NAND erase operations.
pub fn initialize_nand_volume_with_progress(
    progress: impl FnMut(u32, u32),
 ) -> Result<FormatReport, InitError> {
    crate::dhara_fs::clear_storage_io_diagnostic();
    clear_nand_read_diagnostic();
    let mut storage = ActiveStorage::initialize_new_volume_with_progress(progress)?;
    Filesystem::<ActiveStorage>::format(&mut storage)
        .map_err(|_| InitError::MountFailed)?;
    Filesystem::<ActiveStorage>::mount_and_then(&mut storage, |_| Ok(()))
        .map_err(|_| InitError::MountFailed)?;
    let bad_blocks = storage.bad_block_count();
    Ok(FormatReport {
        physical_blocks: PHYSICAL_BLOCKS,
        bad_blocks,
        good_blocks: PHYSICAL_BLOCKS - bad_blocks as u32,
        page_size_bytes: PAGE_SIZE as u32,
        pages_per_block: PAGES_PER_BLOCK,
        erase_block_bytes: ERASE_BLOCK_SIZE as u32,
        littlefs_blocks: crate::dhara_fs::LITTLEFS_BLOCK_COUNT as u32,
        littlefs_capacity_bytes: storage.logical_capacity() as u32,
    })
}

/// Format only the LittleFS logical volume on an already-valid Dhara journal.
/// DharaStorage::new() verifies NAND identity, resumes Dhara, checks the
/// volume marker, and validates capacity before any LittleFS erase is issued.
pub fn format_existing_littlefs() -> Result<(), InitError> {
    let mut storage = ActiveStorage::new()?;
    Filesystem::<ActiveStorage>::format(&mut storage)
        .map_err(|_| InitError::MountFailed)?;
    Filesystem::<ActiveStorage>::mount_and_then(&mut storage, |_| Ok(()))
        .map_err(|_| InitError::MountFailed)
}
