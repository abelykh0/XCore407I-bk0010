//! LittleFS block device for the XCore407I Samsung K9F1G08U0E NAND.
//!
//! Geometry: 2 KiB data pages, 64 pages per 128 KiB erase block, 1,024
//! physical blocks. Sixteen good blocks are held in reserve for bad-block
//! tolerance; LittleFS sees 1,008 logical blocks. Two additional good blocks
//! hold redundant, CRC-protected physical mapping snapshots.
//!
//! New page writes store the STM32 FSMC's 512-byte Hamming ECC values in the
//! spare area. Reads verify ECC-tagged pages and retain compatibility with
//! legacy pages that have no ECC tag. Reads correct single-bit errors in
//! tagged pages and reject uncorrectable syndromes. Corrected data is returned
//! in RAM; NAND pages are not scrubbed
//! yet, and grown-bad blocks are not remapped.

use core::ptr::{read_volatile, write_volatile};
#[cfg(feature = "dhara-eval")]
use core::sync::atomic::{AtomicU32, Ordering};
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
// Bank 2 ECCR2 is at offset 0x74 (PCR2 +0x60, SR2 +0x64,
// PMEM2 +0x68, PATT2 +0x6C, reserved +0x70, ECCR2 +0x74).
const FSMC_ECCR2: *const u32 = (FSMC_BASE + 0x74) as *const u32;
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

/// Maximum number of factory-good blocks available after LittleFS and its
/// two mapping-metadata blocks. Dhara receives only this isolated tail region.
#[cfg(feature = "dhara-eval")]
pub const DHARA_TEST_MAX_BLOCKS: usize = RESERVED_BLOCKS - 2;

/// Keep the 2 KiB logical-to-physical map in static RAM, not on the
/// small startup stack while LittleFS is being mounted/formatted.
static mut PHYSICAL_BLOCK_MAP: [u16; LOGICAL_BLOCKS] = [0; LOGICAL_BLOCKS];

// A complete mapping record fits in one NAND data page. Keep one scratch page
// in static RAM so NAND initialization does not consume 2 KiB of stack.
static mut MAP_METADATA_PAGE: [u8; PAGE_SIZE] = [0xFF; PAGE_SIZE];

#[cfg(feature = "dhara-eval")]
static DHARA_TEST_BLOCK_COUNT: AtomicU32 = AtomicU32::new(u32::MAX);

#[cfg(feature = "dhara-eval")]
#[repr(C)]
struct DharaNand {
    log2_page_size: u8,
    log2_ppb: u8,
    num_blocks: u32,
}

#[cfg(feature = "dhara-eval")]
#[repr(C)]
struct DharaNandContext {
    nand: DharaNand,
    block_count: u32,
    physical_blocks: [u16; DHARA_TEST_MAX_BLOCKS],
}

#[cfg(feature = "dhara-eval")]
static mut DHARA_NAND_CONTEXT: DharaNandContext = DharaNandContext {
    nand: DharaNand { log2_page_size: 11, log2_ppb: 6, num_blocks: 0 },
    block_count: 0,
    physical_blocks: [u16::MAX; DHARA_TEST_MAX_BLOCKS],
};

#[cfg(feature = "dhara-eval")]
static mut DHARA_READ_SCRATCH: [u8; PAGE_SIZE] = [0; PAGE_SIZE];

#[cfg(feature = "dhara-eval")]
static mut DHARA_FREE_SCRATCH: [u8; PAGE_SIZE + SPARE_SIZE] =
    [0; PAGE_SIZE + SPARE_SIZE];

#[cfg(feature = "dhara-eval")]
static mut DHARA_VERIFY_SCRATCH: [u8; PAGE_SIZE] = [0; PAGE_SIZE];

#[cfg(feature = "dhara-eval")]
static DHARA_READ_OK: AtomicU32 = AtomicU32::new(u32::MAX);

#[cfg(feature = "dhara-eval")]
static DHARA_FIRST_PAGE_FREE: AtomicU32 = AtomicU32::new(u32::MAX);

const MAP_METADATA_MAGIC: [u8; 8] = *b"XCMAP001";
const MAP_METADATA_VERSION: u16 = 1;
const MAP_METADATA_HEADER_SIZE: usize = 20;
const MAP_METADATA_MAP_OFFSET: usize = MAP_METADATA_HEADER_SIZE;
const MAP_METADATA_CRC_OFFSET: usize =
    MAP_METADATA_MAP_OFFSET + LOGICAL_BLOCKS * core::mem::size_of::<u16>();
const _: () = assert!(MAP_METADATA_CRC_OFFSET + 4 <= PAGE_SIZE);

pub struct NandStorage {
    physical_blocks: &'static mut [u16; LOGICAL_BLOCKS],
    good_blocks: usize,
    factory_bad_blocks: u16,
    metadata_blocks: [u16; 2],
    active_metadata_slot: u8,
    mapping_generation: u32,
    /// A spare good block beyond LittleFS and mapping metadata, reserved for ECC self-test.
    #[cfg(feature = "nand-ecc-test")]
    ecc_test_block: Option<u16>,
    /// Factory-good physical blocks excluded from LittleFS and mapping metadata.
    #[cfg(feature = "dhara-eval")]
    dhara_test_blocks: [u16; DHARA_TEST_MAX_BLOCKS],
    #[cfg(feature = "dhara-eval")]
    dhara_test_block_count: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InitError {
    Timeout,
    WrongDevice,
    MarkerRead,
    TooManyBadBlocks,
    MetadataFailure,
    MountFailed,
}

fn command(value: u8) { unsafe { write_volatile(CMD, value) } }
fn address(value: u8) { unsafe { write_volatile(ADDR, value) } }
fn write_data(value: u8) { unsafe { write_volatile(DATA, value) } }
fn read_data() -> u8 { unsafe { read_volatile(DATA) } }

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

fn program_physical_page(page: u32, data: &[u8]) -> bool {
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

fn read_data_page(page: u32, out: &mut [u8]) -> bool {
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
        return false;
    }

    let tag = &stored[ECC_OOB_BYTES..];
    if tag == [0xFF, 0xFF] {
        return true;
    }
    if tag != ECC_TAG {
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
            return false;
        }
    }
    true
}


#[cfg(feature = "dhara-eval")]
unsafe fn dhara_context_for(nand: *const DharaNand) -> Option<*mut DharaNandContext> {
    let context = core::ptr::addr_of_mut!(DHARA_NAND_CONTEXT);
    let expected = core::ptr::addr_of!((*context).nand);
    if nand == expected { Some(context) } else { None }
}

#[cfg(feature = "dhara-eval")]
unsafe fn dhara_physical_page(nand: *const DharaNand, page: u32) -> Option<u32> {
    let context = unsafe { dhara_context_for(nand)? };
    let block_index = (page >> 6) as usize;
    if block_index >= unsafe { (*context).block_count as usize } {
        return None;
    }
    let physical_block = unsafe { (*context).physical_blocks[block_index] };
    if physical_block == u16::MAX {
        return None;
    }
    Some(physical_block as u32 * PAGES_PER_BLOCK + (page & 63))
}

#[cfg(feature = "dhara-eval")]
unsafe fn set_dhara_error(error: *mut i32, value: i32) {
    if !error.is_null() {
        unsafe { *error = value; }
    }
}

/// Read-only Dhara callback: every exposed block is a factory-good spare block.
#[cfg(feature = "dhara-eval")]
#[unsafe(no_mangle)]
pub extern "C" fn dhara_nand_is_bad(nand: *const DharaNand, block: u32) -> i32 {
    let context = unsafe { dhara_context_for(nand) };
    match context {
        Some(context) if block < unsafe { (*context).block_count } => 0,
        _ => 1,
    }
}

/// Read-only Dhara callback. The existing STM32 FSMC ECC path validates and
/// corrects the complete 2 KiB page before the requested slice is returned.
/// Dhara error values: 0 = none, 1 = bad block, 2 = ECC.
#[cfg(feature = "dhara-eval")]
#[unsafe(no_mangle)]
pub extern "C" fn dhara_nand_read(
    nand: *const DharaNand,
    page: u32,
    offset: usize,
    length: usize,
    data: *mut u8,
    error: *mut i32,
) -> i32 {
    if offset > PAGE_SIZE || length > PAGE_SIZE - offset || (length != 0 && data.is_null()) {
        unsafe { set_dhara_error(error, 2); }
        return -1;
    }
    let physical_page = match unsafe { dhara_physical_page(nand, page) } {
        Some(page) => page,
        None => {
            unsafe { set_dhara_error(error, 1); }
            return -1;
        }
    };

    let scratch = unsafe { &mut *core::ptr::addr_of_mut!(DHARA_READ_SCRATCH) };
    if !read_data_page(physical_page, scratch) {
        unsafe { set_dhara_error(error, 2); }
        return -1;
    }
    if length != 0 {
        unsafe {
            core::ptr::copy_nonoverlapping(scratch.as_ptr().add(offset), data, length);
        }
    }
    unsafe { set_dhara_error(error, 0); }
    0
}

/// Read-only free-page callback. It checks the complete data+OOB page, so a
/// page with programmed ECC metadata is not mistaken for erased NAND.
#[cfg(feature = "dhara-eval")]
#[unsafe(no_mangle)]
pub extern "C" fn dhara_nand_is_free(nand: *const DharaNand, page: u32) -> i32 {
    let physical_page = match unsafe { dhara_physical_page(nand, page) } {
        Some(page) => page,
        None => return 0,
    };
    let scratch = unsafe { &mut *core::ptr::addr_of_mut!(DHARA_FREE_SCRATCH) };
    if !read_page(physical_page, 0, scratch) {
        return 0;
    }
    if scratch.iter().all(|&byte| byte == 0xFF) { 1 } else { 0 }
}

/// Verify callback address translation and ECC reads without modifying NAND.
/// The test reads page zero of each isolated block and separately checks
/// whether that page is completely erased (including OOB).
#[cfg(feature = "dhara-eval")]
pub fn verify_dhara_read_only_callbacks() -> Option<(usize, usize)> {
    let context = unsafe { core::ptr::addr_of!(DHARA_NAND_CONTEXT) };
    let nand = unsafe { core::ptr::addr_of!((*context).nand) };
    let count = unsafe { (*context).block_count as usize };
    if count == 0 || count > DHARA_TEST_MAX_BLOCKS {
        return None;
    }

    let mut read_ok = 0usize;
    let mut first_page_free = 0usize;
    let output = unsafe { &mut *core::ptr::addr_of_mut!(DHARA_VERIFY_SCRATCH) };

    for block in 0..count {
        if dhara_nand_is_bad(nand, block as u32) != 0 {
            continue;
        }
        let mut error = 0i32;
        let page = (block as u32) << 6;
        if dhara_nand_read(
            nand,
            page,
            0,
            PAGE_SIZE,
            output.as_mut_ptr(),
            &mut error,
        ) == 0 {
            read_ok += 1;
            if dhara_nand_is_free(nand, page) == 1 {
                first_page_free += 1;
            }
        }
    }

    DHARA_READ_OK.store(read_ok as u32, Ordering::Relaxed);
    DHARA_FIRST_PAGE_FREE.store(first_page_free as u32, Ordering::Relaxed);
    Some((read_ok, first_page_free))
}

#[cfg(feature = "dhara-eval")]
pub fn dhara_read_only_test_cached() -> Option<(usize, usize)> {
    let read_ok = DHARA_READ_OK.load(Ordering::Relaxed);
    let free = DHARA_FIRST_PAGE_FREE.load(Ordering::Relaxed);
    if read_ok == u32::MAX || free == u32::MAX {
        None
    } else {
        Some((read_ok as usize, free as usize))
    }
}

/// CRC-32/IEEE used to reject incomplete or corrupted mapping records.
fn mapping_crc32(bytes: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &byte in bytes {
        crc ^= byte as u32;
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xEDB8_8320 & 0u32.wrapping_sub(crc & 1));
        }
    }
    !crc
}

fn metadata_page() -> &'static mut [u8; PAGE_SIZE] {
    unsafe { &mut *core::ptr::addr_of_mut!(MAP_METADATA_PAGE) }
}

fn build_mapping_record(
    generation: u32,
    physical_blocks: &[u16; LOGICAL_BLOCKS],
) {
    let page = metadata_page();
    page.fill(0xFF);
    page[..8].copy_from_slice(&MAP_METADATA_MAGIC);
    page[8..12].copy_from_slice(&generation.to_le_bytes());
    page[12..14].copy_from_slice(&MAP_METADATA_VERSION.to_le_bytes());
    page[14..16].copy_from_slice(&(LOGICAL_BLOCKS as u16).to_le_bytes());
    page[16..20].copy_from_slice(&0u32.to_le_bytes());

    for (index, &block) in physical_blocks.iter().enumerate() {
        let offset = MAP_METADATA_MAP_OFFSET + index * 2;
        page[offset..offset + 2].copy_from_slice(&block.to_le_bytes());
    }

    let crc = mapping_crc32(&page[..MAP_METADATA_CRC_OFFSET]);
    page[MAP_METADATA_CRC_OFFSET..MAP_METADATA_CRC_OFFSET + 4]
        .copy_from_slice(&crc.to_le_bytes());
}

/// Validate a mapping record and return its generation. The map is checked
/// for range, reserved-block collisions, and duplicate physical blocks.
fn validate_mapping_record(page: &[u8; PAGE_SIZE], metadata_blocks: &[u16; 2]) -> Option<u32> {
    if page[..8] != MAP_METADATA_MAGIC
        || u16::from_le_bytes([page[12], page[13]]) != MAP_METADATA_VERSION
        || u16::from_le_bytes([page[14], page[15]]) as usize != LOGICAL_BLOCKS
    {
        return None;
    }

    let stored_crc = u32::from_le_bytes([
        page[MAP_METADATA_CRC_OFFSET],
        page[MAP_METADATA_CRC_OFFSET + 1],
        page[MAP_METADATA_CRC_OFFSET + 2],
        page[MAP_METADATA_CRC_OFFSET + 3],
    ]);
    if mapping_crc32(&page[..MAP_METADATA_CRC_OFFSET]) != stored_crc {
        return None;
    }

    for index in 0..LOGICAL_BLOCKS {
        let offset = MAP_METADATA_MAP_OFFSET + index * 2;
        let block = u16::from_le_bytes([page[offset], page[offset + 1]]);
        if block as u32 >= PHYSICAL_BLOCKS
            || block == metadata_blocks[0]
            || block == metadata_blocks[1]
        {
            return None;
        }
        for previous in 0..index {
            let previous_offset = MAP_METADATA_MAP_OFFSET + previous * 2;
            let previous_block = u16::from_le_bytes([
                page[previous_offset],
                page[previous_offset + 1],
            ]);
            if block == previous_block {
                return None;
            }
        }
    }

    Some(u32::from_le_bytes([page[8], page[9], page[10], page[11]]))
}

fn read_mapping_record(block: u16, metadata_blocks: &[u16; 2]) -> Option<u32> {
    let page = metadata_page();
    if !read_data_page(block as u32 * PAGES_PER_BLOCK, page) {
        return None;
    }
    validate_mapping_record(page, metadata_blocks)
}

fn load_mapping_record(
    block: u16,
    metadata_blocks: &[u16; 2],
    physical_blocks: &mut [u16; LOGICAL_BLOCKS],
) -> Option<u32> {
    let page = metadata_page();
    if !read_data_page(block as u32 * PAGES_PER_BLOCK, page) {
        return None;
    }
    let generation = validate_mapping_record(page, metadata_blocks)?;
    for (index, entry) in physical_blocks.iter_mut().enumerate() {
        let offset = MAP_METADATA_MAP_OFFSET + index * 2;
        *entry = u16::from_le_bytes([page[offset], page[offset + 1]]);
    }
    Some(generation)
}

/// Write the next complete mapping snapshot to the inactive metadata block.
/// The active copy is never erased by this operation.
fn write_initial_mapping(
    block: u16,
    generation: u32,
    physical_blocks: &[u16; LOGICAL_BLOCKS],
) -> bool {
    build_mapping_record(generation, physical_blocks);
    let page = metadata_page();
    if !erase_physical_block(block)
        || !program_physical_page(block as u32 * PAGES_PER_BLOCK, page)
    {
        return false;
    }
    if !read_data_page(block as u32 * PAGES_PER_BLOCK, page) {
        return false;
    }
    validate_mapping_record(page, &[u16::MAX; 2]) == Some(generation)
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
        // NAND bank 2, 8-bit bus, NWAIT enabled. ECC is enabled only around
        // individual 512-byte data chunks; ECCEN remains clear while idle.
        let control = (1 << 1) | (1 << 2) | (1 << 3) | (2 << 9) | (2 << 13) | (1 << 17);
        write_volatile(pcr2, control);
        let _ = read_volatile(pcr2);
    }
}

impl NandStorage {
    /// Reset the NAND, preserve the legacy logical map, and load or initialize
    /// redundant persistent mapping metadata. Requires 1,010 factory-good blocks.
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
            metadata_blocks: [u16::MAX; 2],
            active_metadata_slot: 0,
            mapping_generation: 0,
            #[cfg(feature = "nand-ecc-test")]
            ecc_test_block: None,
            #[cfg(feature = "dhara-eval")]
            dhara_test_blocks: [u16::MAX; DHARA_TEST_MAX_BLOCKS],
            #[cfg(feature = "dhara-eval")]
            dhara_test_block_count: 0,
        };
        let mut metadata_count = 0usize;
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
                    // Preserve the original physical layout used by the
                    // existing LittleFS volume.
                    storage.physical_blocks[storage.good_blocks] = block as u16;
                    storage.good_blocks += 1;
                } else if metadata_count < 2 {
                    // The first two additional good blocks hold redundant
                    // mapping snapshots and are never exposed to LittleFS.
                    storage.metadata_blocks[metadata_count] = block as u16;
                    metadata_count += 1;
                } else {
                    #[cfg(feature = "nand-ecc-test")]
                    if storage.ecc_test_block.is_none() {
                        // The opt-in destructive test uses a spare, not metadata.
                        storage.ecc_test_block = Some(block as u16);
                    }
                    #[cfg(feature = "dhara-eval")]
                    {
                        // Keep every remaining factory-good block out of the
                        // LittleFS map. Dhara will receive this list as its
                        // complete physical device; it cannot address the
                        // LittleFS or mapping-metadata regions.
                        let index = storage.dhara_test_block_count;
                        if index < DHARA_TEST_MAX_BLOCKS {
                            storage.dhara_test_blocks[index] = block as u16;
                            storage.dhara_test_block_count += 1;
                        }
                    }
                }
            } else {
                storage.factory_bad_blocks += 1;
            }
        }

        if storage.good_blocks != LOGICAL_BLOCKS || metadata_count != 2 {
            return Err(InitError::TooManyBadBlocks);
        }

        #[cfg(feature = "dhara-eval")]
        DHARA_TEST_BLOCK_COUNT.store(storage.dhara_test_block_count as u32, Ordering::Relaxed);

        // Prefer the newest valid snapshot. Generation comparison uses
        // wrapping arithmetic so rollover does not make an older copy win.
        let generation0 = read_mapping_record(storage.metadata_blocks[0], &storage.metadata_blocks);
        let generation1 = read_mapping_record(storage.metadata_blocks[1], &storage.metadata_blocks);
        let selected = match (generation0, generation1) {
            (Some(a), Some(b)) => {
                if b.wrapping_sub(a) < 0x8000_0000 { 1 } else { 0 }
            }
            (Some(_), None) => 0,
            (None, Some(_)) => 1,
            (None, None) => {
                // First boot after adding metadata support: the old mapping
                // remains exactly the first 1,008 factory-good blocks.
                if !write_initial_mapping(storage.metadata_blocks[0], 1, storage.physical_blocks) {
                    return Err(InitError::MetadataFailure);
                }
                storage.active_metadata_slot = 0;
                storage.mapping_generation = 1;
                return Ok(storage);
            }
        };

        let generation = load_mapping_record(
            storage.metadata_blocks[selected],
            &storage.metadata_blocks,
            storage.physical_blocks,
        ).ok_or(InitError::MetadataFailure)?;
        storage.active_metadata_slot = selected as u8;
        storage.mapping_generation = generation;
        Ok(storage)
    }

    pub fn factory_bad_blocks(&self) -> u16 { self.factory_bad_blocks }
    pub fn logical_capacity(&self) -> usize { LOGICAL_BLOCKS * ERASE_BLOCK_SIZE }

    /// Physical blocks available for isolated Dhara testing. This list never
    /// includes LittleFS-mapped blocks or either mapping-metadata block.
    #[cfg(feature = "dhara-eval")]
    pub fn dhara_test_blocks(&self) -> &[u16] {
        &self.dhara_test_blocks[..self.dhara_test_block_count]
    }

    #[cfg(feature = "dhara-eval")]
    pub fn dhara_test_capacity_bytes(&self) -> usize {
        self.dhara_test_block_count * ERASE_BLOCK_SIZE
    }

    /// Cached spare-block count from the most recent NAND inventory scan.
    /// None means NAND has not yet been initialized during this boot.
    #[cfg(feature = "dhara-eval")]
    pub fn dhara_test_block_count_cached() -> Option<usize> {
        let count = DHARA_TEST_BLOCK_COUNT.load(Ordering::Relaxed);
        (count != u32::MAX).then_some(count as usize)
    }

    /// Perform a read-only factory-marker scan to count the physical blocks
    /// excluded from LittleFS and its two mapping metadata blocks.
    /// This deliberately does not initialize or mount either filesystem.
    #[cfg(feature = "dhara-eval")]
    pub fn scan_dhara_spare_inventory() -> Option<usize> {
        configure_fsmc();
        command(0xFF);
        if !wait_ready_status() {
            return None;
        }

        let mut good_blocks = 0usize;
        let mut spare_blocks = 0usize;
        let mut marker = [0u8; 1];
        let dhara_context = unsafe { core::ptr::addr_of_mut!(DHARA_NAND_CONTEXT) };

        for block in 0..PHYSICAL_BLOCKS {
            let first = block * PAGES_PER_BLOCK;
            if !read_page(first, PAGE_SIZE as u16, &mut marker) {
                return None;
            }
            let first_good = marker[0] == 0xFF;

            if !read_page(first + 1, PAGE_SIZE as u16, &mut marker) {
                return None;
            }
            let second_good = marker[0] == 0xFF;

            if first_good && second_good {
                if good_blocks >= LOGICAL_BLOCKS + 2 {
                    if spare_blocks < DHARA_TEST_MAX_BLOCKS {
                        unsafe {
                            (*dhara_context).physical_blocks[spare_blocks] = block as u16;
                        }
                    }
                    spare_blocks += 1;
                }
                good_blocks += 1;
            }
        }

        if good_blocks < LOGICAL_BLOCKS + 2 {
            return None;
        }

        DHARA_TEST_BLOCK_COUNT.store(spare_blocks as u32, Ordering::Relaxed);
        unsafe {
            (*dhara_context).block_count = spare_blocks as u32;
            (*dhara_context).nand.num_blocks = spare_blocks as u32;
        }
        Some(spare_blocks)
    }

    fn physical_block(&self, logical: usize) -> Option<u16> {
        self.physical_blocks.get(logical).copied()
    }

    /// Persist a changed map to the inactive metadata block before callers
    /// switch or retire any physical data block.
    fn persist_mapping(&mut self) -> bool {
        let next_slot = 1usize - self.active_metadata_slot as usize;
        let next_generation = self.mapping_generation.wrapping_add(1);
        let block = self.metadata_blocks[next_slot];

        build_mapping_record(next_generation, self.physical_blocks);
        let page = metadata_page();
        if !erase_physical_block(block)
            || !program_physical_page(block as u32 * PAGES_PER_BLOCK, page)
        {
            return false;
        }
        if !read_data_page(block as u32 * PAGES_PER_BLOCK, page)
            || validate_mapping_record(page, &self.metadata_blocks) != Some(next_generation)
        {
            return false;
        }

        self.active_metadata_slot = next_slot as u8;
        self.mapping_generation = next_generation;
        true
    }
}

/// Result of the destructive ECC fault-injection test.
///
/// The test only erases one factory-good block that is beyond LittleFS's
/// logical capacity. It never selects a block used by the filesystem.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg(feature = "nand-ecc-test")]
pub enum EccSelfTestError {
    NoReservedGoodBlock,
    EraseFailed,
    ProgramFailed,
    SingleBitReadRejected,
    SingleBitDataMismatch,
    DoubleBitDetectionFailed,
    CleanupFailed,
}

/// Program test pages with ECC for an all-0xFF page, then alter selected
/// data-register bytes with ECC disabled before the same page-program confirm.
#[cfg(feature = "nand-ecc-test")]
fn program_ecc_test_page(page: u32, corruptions: &[(u16, u8)]) -> bool {
    command(0x80);
    address(0);
    address(0);
    address(page as u8);
    address((page >> 8) as u8);
    cortex_m::asm::delay(1_000);

    let mut ecc = [0u8; ECC_OOB_BYTES];
    let blank = [0xFFu8; FSMC_ECC_CHUNK_SIZE];
    for chunk_index in 0..FSMC_ECC_CHUNKS_PER_PAGE {
        ecc_begin_chunk();
        for &byte in &blank {
            write_data(byte);
        }
        encode_ecc(
            ecc_end_chunk(),
            &mut ecc[chunk_index * ECC_BYTES_PER_CHUNK..][..ECC_BYTES_PER_CHUNK],
        );
    }

    // Random Data Input changes the NAND page buffer after ECC calculation,
    // but before the single final program-confirm operation.
    for &(column, value) in corruptions {
        if column as usize >= PAGE_SIZE {
            return false;
        }
        command(0x85);
        address(column as u8);
        address((column >> 8) as u8);
        cortex_m::asm::delay(1_000);
        write_data(value);
    }

    // Start at OOB byte 0 because the two 0xFF marker placeholders
    // precede ECC bytes 2..13 and the tag at bytes 14..15.
    command(0x85);
    address(PAGE_SIZE as u16 as u8);
    address(((PAGE_SIZE as u16) >> 8) as u8);
    cortex_m::asm::delay(1_000);
    write_data(0xFF);
    write_data(0xFF);
    for &byte in &ecc {
        write_data(byte);
    }
    for &byte in &ECC_TAG {
        write_data(byte);
    }

    command(0x10);
    if !wait_ready_rb() {
        return false;
    }
    command(0x70);
    if read_data() & 0x01 != 0 {
        return false;
    }

    // Verify the tag with a fresh raw OOB read immediately after programming.
    // This separates a failed OOB program from a later read-path problem.
    let tag_column = PAGE_SIZE as u16 + ECC_OOB_OFFSET + ECC_OOB_BYTES as u16;
    let mut tag_readback = [0u8; ECC_TAG.len()];
    if !read_page(page, tag_column, &mut tag_readback) {
        return false;
    }
    if tag_readback != ECC_TAG {
        return false;
    }
    true
}

/// Exercise one-bit correction and two-bit detection on a reserved NAND block.
///
/// The test erases and reuses only the first factory-good block excluded from
/// LittleFS. It attempts to erase that block again before returning.
#[cfg(feature = "nand-ecc-test")]
pub fn run_ecc_fault_injection_test() -> Result<(), EccSelfTestError> {
    let storage = NandStorage::new().map_err(|_| EccSelfTestError::EraseFailed)?;
    let block = storage.ecc_test_block.ok_or(EccSelfTestError::NoReservedGoodBlock)?;
    if !erase_physical_block(block) {
        return Err(EccSelfTestError::EraseFailed);
    }

    let first_page = block as u32 * PAGES_PER_BLOCK;
    let one_bit = [(0x0125u16, 0xF7u8)]; // Clear bit 3.
    let two_bits = [(0x0125u16, 0xEDu8)]; // Clear bits 1 and 4.

    let test_result = (|| {
        if !program_ecc_test_page(first_page, &one_bit)
            || !program_ecc_test_page(first_page + 1, &two_bits)
        {
            return Err(EccSelfTestError::ProgramFailed);
        }

        let mut readback = [0u8; PAGE_SIZE];
        if !read_data_page(first_page, &mut readback) {
            return Err(EccSelfTestError::SingleBitReadRejected);
        }
        if let Some((index, &value)) = readback
            .iter()
            .enumerate()
            .find(|(_, byte)| **byte != 0xFF)
        {
            let _ = (index, value);
            return Err(EccSelfTestError::SingleBitDataMismatch);
        }

        if read_data_page(first_page + 1, &mut readback) {
            return Err(EccSelfTestError::DoubleBitDetectionFailed);
        }

        Ok(())
    })();

    if !erase_physical_block(block) {
        return Err(EccSelfTestError::CleanupFailed);
    }
    test_result
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
            if !read_data_page(page, page_buf) { return Err(LfsError::IO); }
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

    let mut report = [0u8; 256];
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
        append(&mut report, &mut len, b"ECC: single-bit correction; uncorrectable errors rejected\r\n");
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
pub fn initialize_and_mount_report() -> ([u8; 256], usize) {
    let mut report = [0u8; 256];
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

    #[cfg(feature = "dhara-eval")]
    {
        append(&mut report, &mut len, b"Dhara spare blocks: ");
        append_hex(&mut report, &mut len, storage.dhara_test_blocks().len() as u32, 2);
        append(&mut report, &mut len, b" (");
        append_hex(&mut report, &mut len, storage.dhara_test_capacity_bytes() as u32, 8);
        append(&mut report, &mut len, b" bytes)\r\n");
    }

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
        InitError::TooManyBadBlocks => b"not enough good blocks",
        InitError::MetadataFailure => b"mapping metadata read/write failed",
        InitError::MountFailed => b"LittleFS mount failed",
    };
    append(out, len, message);
}
