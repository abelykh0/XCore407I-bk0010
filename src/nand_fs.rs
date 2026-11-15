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
use core::sync::atomic::{AtomicPtr, AtomicU32, Ordering};
use stm32f4::stm32f407 as pac;

use littlefs2::{
    fs::Filesystem,
    io::Result as LfsResult,
};

const FSMC_BASE: usize = 0xA000_0000;
const NAND_BASE: usize = 0x7000_0000;

// STM32F407 FSMC NAND Bank 2 registers.
const FSMC_PCR2: *mut u32 = (FSMC_BASE + 0x60) as *mut u32;
const FSMC_SR2: *const u32 = (FSMC_BASE + 0x64) as *const u32;
const FSMC_SR_FEMPT: u32 = 1 << 6;
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

// Optional main-context service hook used while synchronous NAND operations
// wait for the device. The hook is installed only around the file-browser
// operation; it must not access NAND or call back into LittleFS.
static NAND_IO_SERVICE_HOOK: AtomicPtr<fn()> = AtomicPtr::new(core::ptr::null_mut());

pub fn set_io_service_hook(hook: Option<&'static fn()>) {
    let pointer = hook.map_or(core::ptr::null_mut(), |callback| {
        callback as *const fn() as *mut fn()
    });
    NAND_IO_SERVICE_HOOK.store(pointer, Ordering::Release);
}

#[inline]
pub(crate) fn service_io_hook() {
    let pointer = NAND_IO_SERVICE_HOOK.load(Ordering::Acquire);
    if !pointer.is_null() {
        unsafe { (*pointer)(); }
    }
}

pub const PAGE_SIZE: usize = 2048;
pub const SPARE_SIZE: usize = 64;
pub const PAGES_PER_BLOCK: u32 = 64;
pub const PHYSICAL_BLOCKS: u32 = 1024;
pub const ERASE_BLOCK_SIZE: usize = PAGE_SIZE * PAGES_PER_BLOCK as usize;
pub type ActiveStorage = crate::dhara_fs::DharaStorage;

// Preserve the first low-level failure for storage error reports.
static NAND_READ_FAILURE_STAGE: AtomicU32 = AtomicU32::new(0);
static NAND_READ_FAILURE_DETAIL: AtomicU32 = AtomicU32::new(0);
static NAND_READ_FAILURE_PAGE: AtomicU32 = AtomicU32::new(0);
static NAND_READ_FAILURE_STORED_ECC: AtomicU32 = AtomicU32::new(0);
static NAND_READ_FAILURE_CALCULATED_ECC: AtomicU32 = AtomicU32::new(0);
static NAND_DEVICE_FAILURE_STAGE: AtomicU32 = AtomicU32::new(0);
static NAND_DEVICE_FAILURE_LOCATION: AtomicU32 = AtomicU32::new(0);
static NAND_DEVICE_FAILURE_STATUS: AtomicU32 = AtomicU32::new(0);

fn record_nand_device_failure(stage: u32, location: u32, status: u32) {
    if NAND_DEVICE_FAILURE_STAGE.compare_exchange(0, stage, Ordering::AcqRel, Ordering::Relaxed).is_ok() {
        NAND_DEVICE_FAILURE_LOCATION.store(location, Ordering::Relaxed);
        NAND_DEVICE_FAILURE_STATUS.store(status, Ordering::Release);
    }
}

pub(crate) fn clear_nand_device_diagnostic() {
    NAND_DEVICE_FAILURE_STAGE.store(0, Ordering::Release);
    NAND_DEVICE_FAILURE_LOCATION.store(0, Ordering::Release);
    NAND_DEVICE_FAILURE_STATUS.store(0, Ordering::Release);
}

pub(crate) fn nand_device_diagnostic() -> (u32, u32, u32) {
    (
        NAND_DEVICE_FAILURE_STAGE.load(Ordering::Acquire),
        NAND_DEVICE_FAILURE_LOCATION.load(Ordering::Acquire),
        NAND_DEVICE_FAILURE_STATUS.load(Ordering::Acquire),
    )
}

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
    clear_nand_device_diagnostic();
}

pub(crate) fn nand_read_diagnostic() -> (u32, u32, u32, u32, u32) {
    (
        NAND_READ_FAILURE_STAGE.load(Ordering::Acquire),
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

/// Wait for the FSMC FIFO to empty before capturing the ECC result.
fn wait_fsmc_fifo_empty() -> bool {
    for _ in 0..TIMEOUT {
        if unsafe { read_volatile(FSMC_SR2) } & FSMC_SR_FEMPT != 0 {
            return true;
        }
        core::hint::spin_loop();
    }
    false
}

/// Read the 24-bit ECC result and disable the engine before reuse.
fn ecc_end_chunk() -> u32 {
    unsafe {
        let pcr = read_volatile(FSMC_PCR2);
        let ecc = read_volatile(FSMC_ECCR2) & 0x00FF_FFFF;
        write_volatile(FSMC_PCR2, pcr & !FSMC_PCR_ECCEN);
        let _ = read_volatile(FSMC_PCR2);
        ecc
    }
}

pub(crate) fn wait_ready_status() -> bool {
    for iteration in 0..TIMEOUT {
        command(0x70);
        if read_data() & 0x40 != 0 { return true; }
        core::hint::spin_loop();
    }
    false
}

pub(crate) fn wait_ready_rb() -> bool {
    for iteration in 0..TIMEOUT {
        if unsafe { read_volatile(pac::GPIOD::ptr().cast::<u8>().add(0x10).cast::<u32>()) } & RB_MASK != 0 {
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
    if !wait_ready_rb() {
        record_nand_device_failure(1, page, 0);
        return false;
    }
    for byte in out { *byte = read_data(); }
    true
}

pub(crate) fn erase_physical_block(block: u16) -> bool {
    let first_page = block as u32 * PAGES_PER_BLOCK;
    command(0x60);
    address(first_page as u8);
    address((first_page >> 8) as u8);
    command(0xD0);
    if !wait_ready_rb() {
        record_nand_device_failure(2, block as u32, 0);
        return false;
    }
    command(0x70);
    let status = read_data();
    if status & 0x01 != 0 {
        record_nand_device_failure(3, block as u32, status as u32);
        return false;
    }
    true
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

/// Linux FSMC driver's 512-byte NAND_ECC_SOFT_HAMMING_SM_ORDER
/// reference algorithm, expressed without alignment-dependent word loads.
fn ecc_software_512(data: &[u8]) -> u32 {
    if data.len() != FSMC_ECC_CHUNK_SIZE {
        return 0;
    }

    #[inline]
    fn inv_parity(value: u8) -> u8 {
        ((!value.count_ones()) & 1) as u8
    }

    let mut rp4 = 0u32;
    let mut rp6 = 0u32;
    let mut rp8 = 0u32;
    let mut rp10 = 0u32;
    let mut rp12 = 0u32;
    let mut rp14 = 0u32;
    let mut rp16 = 0u32;
    let mut par = 0u32;

    // Linux ecc_sw_hamming_calculate(): 8 groups x 16 little-endian words.
    let words: [u32; 128] = core::array::from_fn(|i| {
        u32::from_le_bytes([
            data[i * 4],
            data[i * 4 + 1],
            data[i * 4 + 2],
            data[i * 4 + 3],
        ])
    });

    for group in 0..8 {
        let base = group * 16;
        let mut t = words[base];
        rp4 ^= words[base];
        t ^= words[base + 1];
        rp6 ^= t;
        t ^= words[base + 2];
        rp4 ^= words[base + 2];
        t ^= words[base + 3];
        rp8 ^= t;

        t ^= words[base + 4];
        rp4 ^= words[base + 4];
        rp6 ^= words[base + 4];
        t ^= words[base + 5];
        rp6 ^= words[base + 5];
        t ^= words[base + 6];
        rp4 ^= words[base + 6];
        t ^= words[base + 7];
        rp10 ^= t;

        t ^= words[base + 8];
        rp4 ^= words[base + 8];
        rp6 ^= words[base + 8];
        rp8 ^= words[base + 8];
        t ^= words[base + 9];
        rp6 ^= words[base + 9];
        rp8 ^= words[base + 9];
        t ^= words[base + 10];
        rp4 ^= words[base + 10];
        rp8 ^= words[base + 10];
        t ^= words[base + 11];
        rp8 ^= words[base + 11];

        t ^= words[base + 12];
        rp4 ^= words[base + 12];
        rp6 ^= words[base + 12];
        t ^= words[base + 13];
        rp6 ^= words[base + 13];
        t ^= words[base + 14];
        rp4 ^= words[base + 14];
        t ^= words[base + 15];

        par ^= t;
        if group & 1 == 0 { rp12 ^= t; }
        if group & 2 == 0 { rp14 ^= t; }
        if group & 4 == 0 { rp16 ^= t; }
    }

    let fold = |v: u32| -> u8 {
        let v = v ^ (v >> 16);
        let v = v ^ (v >> 8);
        v as u8
    };
    let rp4 = fold(rp4);
    let rp6 = fold(rp6);
    let rp8 = fold(rp8);
    let rp10 = fold(rp10);
    let rp12 = fold(rp12);
    let rp14 = fold(rp14);
    let rp16 = fold(rp16);

    // Little-endian Linux SM-order parity packing.
    let rp3 = fold(par >> 16);
    let low16 = par & 0xFFFF;
    let rp2 = (low16 ^ (low16 >> 8)) as u8;
    let p16 = par ^ (par >> 16);
    let rp1 = ((p16 >> 8) & 0xFF) as u8;
    let rp0 = p16 as u8;
    let par8 = (p16 ^ (p16 >> 8)) as u8;

    let rp5 = par8 ^ rp4;
    let rp7 = par8 ^ rp6;
    let rp9 = par8 ^ rp8;
    let rp11 = par8 ^ rp10;
    let rp13 = par8 ^ rp12;
    let rp15 = par8 ^ rp14;
    let rp17 = par8 ^ rp16;

    let ecc0 = (inv_parity(rp7) << 7)
        | (inv_parity(rp6) << 6)
        | (inv_parity(rp5) << 5)
        | (inv_parity(rp4) << 4)
        | (inv_parity(rp3) << 3)
        | (inv_parity(rp2) << 2)
        | (inv_parity(rp1) << 1)
        | inv_parity(rp0);
    let ecc1 = (inv_parity(rp15) << 7)
        | (inv_parity(rp14) << 6)
        | (inv_parity(rp13) << 5)
        | (inv_parity(rp12) << 4)
        | (inv_parity(rp11) << 3)
        | (inv_parity(rp10) << 2)
        | (inv_parity(rp9) << 1)
        | inv_parity(rp8);
    let ecc2 = (inv_parity(par8 & 0xF0) << 7)
        | (inv_parity(par8 & 0x0F) << 6)
        | (inv_parity(par8 & 0xCC) << 5)
        | (inv_parity(par8 & 0x33) << 4)
        | (inv_parity(par8 & 0xAA) << 3)
        | (inv_parity(par8 & 0x55) << 2)
        | (inv_parity(rp17) << 1)
        | inv_parity(rp16);

    (ecc0 as u32) | ((ecc1 as u32) << 8) | ((ecc2 as u32) << 16)
}


/// Convert 512-byte Linux SM-order Hamming ECC to the STM32 FSMC ECCR layout.
/// Hardware comparisons establish a six-bit left rotation followed by inversion.
fn ecc_linux_to_fsmc(ecc: u32) -> u32 {
    let ecc = ecc & 0x00FF_FFFF;
    let rotated = ((ecc << 6) | (ecc >> 18)) & 0x00FF_FFFF;
    (!rotated) & 0x00FF_FFFF
}

fn ecc_software_512_fsmc(data: &[u8]) -> u32 {
    ecc_linux_to_fsmc(ecc_software_512(data))
}

/// Program a full page and store four FSMC-generated 512-byte ECC values.
pub(crate) fn program_physical_page(page: u32, data: &[u8]) -> bool {
    if data.len() != PAGE_SIZE { return false; }

    command(0x80);
    address(0);
    address(0);
    address(page as u8);
    address((page >> 8) as u8);
    cortex_m::asm::delay(1_000);

    let mut ecc = [0xFFu8; ECC_OOB_BYTES];
    for (chunk_index, chunk) in data.chunks_exact(FSMC_ECC_CHUNK_SIZE).enumerate() {
        ecc_begin_chunk();
        for &byte in chunk { write_data(byte); }
        if !wait_fsmc_fifo_empty() {
            let status = unsafe { read_volatile(FSMC_SR2) };
            record_nand_device_failure(6, page, status);
            return false;
        }
        let chunk_ecc = ecc_end_chunk();
        encode_ecc(chunk_ecc, &mut ecc[chunk_index * ECC_BYTES_PER_CHUNK..][..ECC_BYTES_PER_CHUNK]);
    }

    write_data(0xFF);
    write_data(0xFF);
    for &byte in &ecc { write_data(byte); }
    for &byte in &ECC_TAG { write_data(byte); }

    command(0x10);
    if !wait_ready_rb() {
        record_nand_device_failure(4, page, 0);
        return false;
    }
    command(0x70);
    let status = read_data();
    if status & 0x01 != 0 {
        record_nand_device_failure(5, page, status as u32);
        return false;
    }
    true
}

/// Decode a 24-bit STM32 FSMC syndrome using Linux's SM-order Hamming
/// decoder. FSMC ECC is Linux ECC rotated left by six bits and complemented,
/// so the syndrome is rotated right by six before decoding.
fn ecc_error_location(syndrome: u32) -> Option<(usize, usize)> {
    let linux = ((syndrome >> 6) | (syndrome << 18)) & 0x00FF_FFFF;
    let b0 = linux as u8;
    let b1 = (linux >> 8) as u8;
    let b2 = (linux >> 16) as u8;

    // A 512-byte data-bit syndrome has alternating parity in every
    // address pair, including both high byte-address bits in b2.
    if ((b0 ^ (b0 >> 1)) & 0x55) != 0x55
        || ((b1 ^ (b1 >> 1)) & 0x55) != 0x55
        || ((b2 ^ (b2 >> 1)) & 0x55) != 0x55
    {
        return None;
    }

    // Linux addressbits[] extracts the odd-positioned bits from each
    // interleaved parity byte: input bits 1,3,5,7 become output bits 0..3.
    #[inline]
    fn address_bits(value: u8) -> usize {
        (((value >> 1) & 1)
            | ((value >> 2) & 2)
            | ((value >> 3) & 4)
            | ((value >> 4) & 8)) as usize
    }

    let byte_index = (address_bits(b2 & 0x03) << 8)
        | (address_bits(b1) << 4)
        | address_bits(b0);
    let bit_index = address_bits(b2 >> 2);

    if byte_index >= FSMC_ECC_CHUNK_SIZE || bit_index >= 8 {
        return None;
    }
    Some((byte_index, bit_index))
}

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

    let Some((byte_index, bit_index)) = ecc_error_location(syndrome) else {
        return false;
    };
    data[byte_index] ^= 1 << bit_index;
    true
}

/// Reference vectors from Linux ecc_sw_hamming_calculate() using
/// NAND_ECC_SOFT_HAMMING_SM_ORDER and 512-byte steps.
pub(crate) fn ecc_software_selftest() -> [u32; 4] {
    let zero = [0u8; FSMC_ECC_CHUNK_SIZE];
    let erased = [0xFFu8; FSMC_ECC_CHUNK_SIZE];
    let mut single_bit = [0u8; FSMC_ECC_CHUNK_SIZE];
    single_bit[0] = 1;
    let mut pseudo_random = [0u8; FSMC_ECC_CHUNK_SIZE];
    let mut state = 0x1234_5678u32;
    for byte in &mut pseudo_random {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        *byte = state as u8;
    }
    [
        ecc_software_512(&zero),
        ecc_software_512(&erased),
        ecc_software_512(&single_bit),
        ecc_software_512(&pseudo_random),
    ]
}


/// Read-only raw NAND interface probe: READ ID and STATUS, without page access.
pub(crate) fn ecc_scan_interface_probe() -> ([u8; 5], u8, bool) {
    // Avoid FSMC NWAIT stalling the CPU while probing the bus. Restore PCR2
    // exactly as found before returning.
    let pcr = unsafe { read_volatile(FSMC_PCR2) };
    unsafe {
        write_volatile(FSMC_PCR2, pcr & !(1 << 1));
        let _ = read_volatile(FSMC_PCR2);
    }

    command(0x90);
    address(0x00);
    let id = [read_data(), read_data(), read_data(), read_data(), read_data()];

    command(0x70);
    let status = read_data();
    let ready = unsafe {
        read_volatile(pac::GPIOD::ptr().cast::<u8>().add(0x10).cast::<u32>())
    } & RB_MASK != 0;

    unsafe {
        write_volatile(FSMC_PCR2, pcr);
        let _ = read_volatile(FSMC_PCR2);
    }
    (id, status, ready)
}

/// Read-only classification helpers used by the whole-device ECC scan.
pub(crate) fn ecc_scan_block_marker(page: u32) -> Option<[u8; 2]> {
    if page >= PHYSICAL_BLOCKS * PAGES_PER_BLOCK { return None; }
    let mut marker = [0xFFu8; 2];

    // ECCSCAN is a standalone diagnostic. Do not run the USB/UVC/video service
    // hook from inside this low-level marker read: it can obscure whether a
    // NAND wait is actually returning. The caller services USB immediately
    // before and after each marker read.
    let hook = NAND_IO_SERVICE_HOOK.swap(core::ptr::null_mut(), Ordering::AcqRel);

    // The FSMC NWAIT input is useful for normal NAND traffic, but it can
    // stall a CPU load/store before the software ready/busy timeout gets a
    // chance to run. For this diagnostic marker read, disable NWAIT and use
    // the NAND R/B GPIO polling in read_page() as the sole readiness check.
    // Restore the complete PCR2 value immediately after the bounded read.
    let pcr = unsafe { read_volatile(FSMC_PCR2) };
    unsafe {
        write_volatile(FSMC_PCR2, pcr & !(1 << 1));
        let _ = read_volatile(FSMC_PCR2);
    }
    let read_ok = read_page(page, PAGE_SIZE as u16, &mut marker);
    unsafe {
        write_volatile(FSMC_PCR2, pcr);
        let _ = read_volatile(FSMC_PCR2);
    }

    NAND_IO_SERVICE_HOOK.store(hook, Ordering::Release);

    if !read_ok { return None; }
    Some(marker)
}

pub(crate) fn ecc_scan_page_tagged(page: u32) -> Option<bool> {
    if page >= PHYSICAL_BLOCKS * PAGES_PER_BLOCK { return None; }
    let mut tag = [0u8; 2];
    if !read_page(page, PAGE_SIZE as u16 + ECC_OOB_OFFSET + ECC_OOB_BYTES as u16, &mut tag) {
        return None;
    }
    Some(tag == ECC_TAG)
}

/// Read one tagged physical page and compare OOB, FSMC, and software ECC.
/// This bypasses Dhara/LittleFS and never writes or alters NAND contents.
pub(crate) struct EccPageComparison {
    pub stored: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub hardware: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub software: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
}

pub(crate) fn ecc_software_compare_page(page: u32) -> Option<EccPageComparison> {
    if page >= PHYSICAL_BLOCKS * PAGES_PER_BLOCK {
        return None;
    }

    let mut data = [0u8; PAGE_SIZE];
    command(0x00);
    address(0);
    address(0);
    address(page as u8);
    address((page >> 8) as u8);
    command(0x30);
    if !wait_ready_rb() {
        return None;
    }

    let mut hardware = [0u32; FSMC_ECC_CHUNKS_PER_PAGE];
    for (index, chunk) in data.chunks_exact_mut(FSMC_ECC_CHUNK_SIZE).enumerate() {
        ecc_begin_chunk();
        for byte in chunk.iter_mut() {
            *byte = read_data();
        }
        if !wait_fsmc_fifo_empty() {
            let _ = ecc_end_chunk();
            return None;
        }
        hardware[index] = ecc_end_chunk() & 0x00FF_FFFF;
    }

    let mut spare = [0u8; ECC_OOB_BYTES + ECC_TAG.len()];
    if !read_page(page, PAGE_SIZE as u16 + ECC_OOB_OFFSET, &mut spare) {
        return None;
    }
    if spare[ECC_OOB_BYTES..] != ECC_TAG {
        return None;
    }

    let mut result = EccPageComparison {
        stored: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        hardware,
        software: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    };
    for (index, chunk) in data.chunks_exact(FSMC_ECC_CHUNK_SIZE).enumerate() {
        let offset = index * ECC_BYTES_PER_CHUNK;
        result.stored[index] = u32::from_le_bytes([
            spare[offset], spare[offset + 1], spare[offset + 2], 0,
        ]);
        result.software[index] = ecc_software_512_fsmc(chunk);
    }
    Some(result)
}

pub(crate) fn read_data_page(page: u32, out: &mut [u8]) -> bool {
    if out.len() != PAGE_SIZE { return false; }

    command(0x00);
    address(0);
    address(0);
    address(page as u8);
    address((page >> 8) as u8);
    command(0x30);
    if !wait_ready_rb() {
        record_nand_read_failure(1, page);
        record_nand_device_failure(1, page, 0);
        return false;
    }

    let mut calculated = [0u8; ECC_OOB_BYTES];
    for (chunk_index, chunk) in out.chunks_exact_mut(FSMC_ECC_CHUNK_SIZE).enumerate() {
        ecc_begin_chunk();
        for byte in chunk.iter_mut() { *byte = read_data(); }
        if !wait_fsmc_fifo_empty() {
            let _ = ecc_end_chunk();
            record_nand_read_failure(5, page);
            return false;
        }
        let chunk_ecc = ecc_end_chunk();
        encode_ecc(chunk_ecc, &mut calculated[chunk_index * ECC_BYTES_PER_CHUNK..][..ECC_BYTES_PER_CHUNK]);
        service_io_hook();
    }

    let mut stored = [0u8; ECC_OOB_BYTES + ECC_TAG.len()];
    if !read_page(page, PAGE_SIZE as u16 + ECC_OOB_OFFSET, &mut stored) {
        record_nand_read_failure(2, page);
        return false;
    }

    let tag = &stored[ECC_OOB_BYTES..];
    if tag == [0xFF, 0xFF] { return true; }
    if tag != ECC_TAG {
        record_nand_read_failure(3, u32::from(tag[0]) | (u32::from(tag[1]) << 8));
        return false;
    }

    for (chunk_index, chunk) in out.chunks_exact_mut(FSMC_ECC_CHUNK_SIZE).enumerate() {
        let offset = chunk_index * ECC_BYTES_PER_CHUNK;
        let stored_ecc = u32::from_le_bytes([
            stored[offset], stored[offset + 1], stored[offset + 2], 0,
        ]);
        let calculated_ecc = u32::from_le_bytes([
            calculated[offset], calculated[offset + 1], calculated[offset + 2], 0,
        ]);
        if !correct_ecc_chunk(chunk, stored_ecc, calculated_ecc) {
            record_ecc_read_failure(page, chunk_index as u32, stored_ecc, calculated_ecc);
            return false;
        }
        service_io_hook();
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
    with_mounted_detailed_with_progress(service, |_, _| {})
}

/// Mount the active volume while periodically allowing time-critical services
/// to run during the physical bad-block marker scan.
pub fn with_mounted_detailed_with_progress<R>(
    service: impl FnOnce(&Filesystem<'_, ActiveStorage>) -> LfsResult<R>,
    progress: impl FnMut(u32, u32),
) -> Result<R, FsAccessError> {
    let mut storage =
        ActiveStorage::new_with_progress(progress).map_err(FsAccessError::Init)?;
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
