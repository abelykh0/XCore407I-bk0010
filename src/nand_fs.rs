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
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
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

// Explicitly scoped raw-data diagnostic mode; normal operation keeps ECC enabled.
static RAW_NAND_TEST_MODE: AtomicBool = AtomicBool::new(false);

pub(crate) fn set_raw_nand_test_mode(enabled: bool) {
    RAW_NAND_TEST_MODE.store(enabled, Ordering::SeqCst);
}

// Diagnostic mode: generate/store ECC on writes, but read data without correction.
static ECC_WRITE_RAW_READ_TEST_MODE: AtomicBool = AtomicBool::new(false);

pub(crate) fn set_ecc_write_raw_read_test_mode(enabled: bool) {
    ECC_WRITE_RAW_READ_TEST_MODE.store(enabled, Ordering::SeqCst);
}

// Diagnostic mode: calculate/compare ECC on reads, but never mutate page data.
static ECC_CHECK_NO_CORRECT_TEST_MODE: AtomicBool = AtomicBool::new(false);
static SOFTWARE_ECC_TEST_MODE: AtomicBool = AtomicBool::new(false);
static SOFTWARE_ECC_COMPARE_TEST_MODE: AtomicBool = AtomicBool::new(false);
static SOFTWARE_ECC_COMPARE_COUNTS: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];
static SOFTWARE_ECC_FIRST_PAGE: AtomicU32 = AtomicU32::new(0);
static SOFTWARE_ECC_FIRST_CHUNK: AtomicU32 = AtomicU32::new(0);
static SOFTWARE_ECC_FIRST_STORED: AtomicU32 = AtomicU32::new(0);
static SOFTWARE_ECC_FIRST_CALCULATED: AtomicU32 = AtomicU32::new(0);

pub(crate) fn set_ecc_check_no_correct_test_mode(enabled: bool) {
    ECC_CHECK_NO_CORRECT_TEST_MODE.store(enabled, Ordering::SeqCst);
}

pub(crate) fn set_software_ecc_compare_test_mode(enabled: bool) {
    SOFTWARE_ECC_COMPARE_TEST_MODE.store(enabled, Ordering::SeqCst);
}

pub(crate) fn software_ecc_compare_diagnostic() -> [u32; 6] {
    [
        SOFTWARE_ECC_COMPARE_COUNTS[0].load(Ordering::Acquire),
        SOFTWARE_ECC_COMPARE_COUNTS[1].load(Ordering::Acquire),
        SOFTWARE_ECC_FIRST_PAGE.load(Ordering::Acquire),
        SOFTWARE_ECC_FIRST_CHUNK.load(Ordering::Acquire),
        SOFTWARE_ECC_FIRST_STORED.load(Ordering::Acquire),
        SOFTWARE_ECC_FIRST_CALCULATED.load(Ordering::Acquire),
    ]
}

pub(crate) fn set_software_ecc_test_mode(enabled: bool) {
    SOFTWARE_ECC_TEST_MODE.store(enabled, Ordering::SeqCst);
}
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
static ECC_READ_COUNTS: [AtomicU32; 5] = [
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0),
    AtomicU32::new(0), AtomicU32::new(0),
];
// Last completed ECC page read: page, tag class (1=valid, 2=legacy),
// chunk, syndrome, stored/calculated ECC, corrected byte and bit values.
static ECC_LAST_PAGE: AtomicU32 = AtomicU32::new(0);
static ECC_LAST_TAG: AtomicU32 = AtomicU32::new(0);
static ECC_LAST_CHUNK: AtomicU32 = AtomicU32::new(0);
static ECC_LAST_SYNDROME: AtomicU32 = AtomicU32::new(0);
static ECC_LAST_STORED: AtomicU32 = AtomicU32::new(0);
static ECC_LAST_CALCULATED: AtomicU32 = AtomicU32::new(0);
// Preserve the first nonzero syndrome in a scan; the "last" values otherwise
// tend to describe an unrelated clean page by the time ECCDIAG finishes.
static ECC_FIRST_PAGE: AtomicU32 = AtomicU32::new(0);
static ECC_FIRST_CHUNK: AtomicU32 = AtomicU32::new(0);
static ECC_FIRST_SYNDROME: AtomicU32 = AtomicU32::new(0);
static ECC_FIRST_STORED: AtomicU32 = AtomicU32::new(0);
static ECC_FIRST_CALCULATED: AtomicU32 = AtomicU32::new(0);
static ECC_LAST_BYTE: AtomicU32 = AtomicU32::new(u32::MAX);
static ECC_LAST_BEFORE: AtomicU32 = AtomicU32::new(0);
static ECC_LAST_AFTER: AtomicU32 = AtomicU32::new(0);
// Calculated ECC for each 512-byte chunk of the most recently read page.
static ECC_LAST_PAGE_CHUNKS: [AtomicU32; FSMC_ECC_CHUNKS_PER_PAGE] = [
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0),
];
// PCR2 read-back immediately after ECC enable and immediately before ECCR2 capture.
static ECC_LAST_PCR_ENABLE: [AtomicU32; FSMC_ECC_CHUNKS_PER_PAGE] = [
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0),
];
static ECC_LAST_PCR_CAPTURE: [AtomicU32; FSMC_ECC_CHUNKS_PER_PAGE] = [
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0),
];


// First raw NAND command failure during a storage operation.
// Stages: 1=page-read timeout, 2=block-erase timeout, 3=block-erase
// status failure, 4=page-program timeout, 5=page-program status failure,
// 6=FSMC write FIFO did not drain before ECC capture.
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
    for count in &ECC_READ_COUNTS { count.store(0, Ordering::Release); }
    for count in &SOFTWARE_ECC_COMPARE_COUNTS { count.store(0, Ordering::Release); }
    SOFTWARE_ECC_FIRST_PAGE.store(0, Ordering::Release);
    SOFTWARE_ECC_FIRST_CHUNK.store(0, Ordering::Release);
    SOFTWARE_ECC_FIRST_STORED.store(0, Ordering::Release);
    SOFTWARE_ECC_FIRST_CALCULATED.store(0, Ordering::Release);
    ECC_LAST_PAGE.store(0, Ordering::Release);
    ECC_LAST_TAG.store(0, Ordering::Release);
    ECC_LAST_CHUNK.store(0, Ordering::Release);
    ECC_LAST_SYNDROME.store(0, Ordering::Release);
    ECC_LAST_STORED.store(0, Ordering::Release);
    ECC_LAST_CALCULATED.store(0, Ordering::Release);
    ECC_FIRST_PAGE.store(0, Ordering::Release);
    ECC_FIRST_CHUNK.store(0, Ordering::Release);
    ECC_FIRST_SYNDROME.store(0, Ordering::Release);
    ECC_FIRST_STORED.store(0, Ordering::Release);
    ECC_FIRST_CALCULATED.store(0, Ordering::Release);
    ECC_LAST_BYTE.store(u32::MAX, Ordering::Release);
    ECC_LAST_BEFORE.store(0, Ordering::Release);
    ECC_LAST_AFTER.store(0, Ordering::Release);
    for value in &ECC_LAST_PAGE_CHUNKS { value.store(0, Ordering::Release); }
    for value in &ECC_LAST_PCR_ENABLE { value.store(0, Ordering::Release); }
    for value in &ECC_LAST_PCR_CAPTURE { value.store(0, Ordering::Release); }
    clear_nand_device_diagnostic();
}


pub(crate) fn ecc_read_diagnostic() -> [u32; 31] {
    [
        ECC_READ_COUNTS[0].load(Ordering::Acquire),
        ECC_READ_COUNTS[1].load(Ordering::Acquire),
        ECC_READ_COUNTS[2].load(Ordering::Acquire),
        ECC_READ_COUNTS[3].load(Ordering::Acquire),
        ECC_READ_COUNTS[4].load(Ordering::Acquire),
        ECC_LAST_PAGE.load(Ordering::Acquire),
        ECC_LAST_TAG.load(Ordering::Acquire),
        ECC_LAST_CHUNK.load(Ordering::Acquire),
        ECC_LAST_SYNDROME.load(Ordering::Acquire),
        ECC_LAST_STORED.load(Ordering::Acquire),
        ECC_LAST_CALCULATED.load(Ordering::Acquire),
        ECC_LAST_BYTE.load(Ordering::Acquire),
        ECC_LAST_BEFORE.load(Ordering::Acquire),
        ECC_LAST_AFTER.load(Ordering::Acquire),
        ECC_LAST_PAGE_CHUNKS[0].load(Ordering::Acquire),
        ECC_LAST_PAGE_CHUNKS[1].load(Ordering::Acquire),
        ECC_LAST_PAGE_CHUNKS[2].load(Ordering::Acquire),
        ECC_LAST_PAGE_CHUNKS[3].load(Ordering::Acquire),
        ECC_LAST_PCR_ENABLE[0].load(Ordering::Acquire),
        ECC_LAST_PCR_ENABLE[1].load(Ordering::Acquire),
        ECC_LAST_PCR_ENABLE[2].load(Ordering::Acquire),
        ECC_LAST_PCR_ENABLE[3].load(Ordering::Acquire),
        ECC_LAST_PCR_CAPTURE[0].load(Ordering::Acquire),
        ECC_LAST_PCR_CAPTURE[1].load(Ordering::Acquire),
        ECC_LAST_PCR_CAPTURE[2].load(Ordering::Acquire),
        ECC_LAST_PCR_CAPTURE[3].load(Ordering::Acquire),
        ECC_FIRST_PAGE.load(Ordering::Acquire),
        ECC_FIRST_CHUNK.load(Ordering::Acquire),
        ECC_FIRST_SYNDROME.load(Ordering::Acquire),
        ECC_FIRST_STORED.load(Ordering::Acquire),
        ECC_FIRST_CALCULATED.load(Ordering::Acquire),
    ]
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

/// Prepare the FSMC ECC engine for one data chunk. ECCPS is the
/// RM0090 encoding: 0=256 bytes, 1=512 bytes, etc.
fn ecc_begin_chunk_size(eccps: u32) -> u32 {
    unsafe {
        let pcr = read_volatile(FSMC_PCR2);
        write_volatile(FSMC_PCR2, pcr & !FSMC_PCR_ECCEN);
        write_volatile(
            FSMC_PCR2,
            (pcr & !FSMC_PCR_ECCPS_MASK) | ((eccps & 0x7) << 17) | FSMC_PCR_ECCEN,
        );
        read_volatile(FSMC_PCR2)
    }
}

/// Prepare the FSMC ECC engine for one 512-byte data chunk.
fn ecc_begin_chunk() -> u32 {
    ecc_begin_chunk_size(1)
}

/// Wait until all buffered FSMC writes have reached the NAND data bus.
/// RM0090 requires FEMPT=1 before reading ECC after a NAND write.
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
fn ecc_end_chunk() -> (u32, u32) {
    unsafe {
        // Capture both registers before disabling ECC so the diagnostic
        // reflects the state at the exact end of the 512-byte transfer.
        let pcr = read_volatile(FSMC_PCR2);
        let ecc = read_volatile(FSMC_ECCR2) & 0x00FF_FFFF;
        write_volatile(FSMC_PCR2, pcr & !FSMC_PCR_ECCEN);
        let _ = read_volatile(FSMC_PCR2);
        (ecc, pcr)
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

static NAND_RB_WAIT_CALLS: AtomicU32 = AtomicU32::new(0);
static NAND_RB_WAIT_ITERATIONS: AtomicU32 = AtomicU32::new(0);
static NAND_RB_WAIT_TIMEOUTS: AtomicU32 = AtomicU32::new(0);

pub(crate) fn clear_nand_rb_wait_diagnostic() {
    NAND_RB_WAIT_CALLS.store(0, Ordering::Release);
    NAND_RB_WAIT_ITERATIONS.store(0, Ordering::Release);
    NAND_RB_WAIT_TIMEOUTS.store(0, Ordering::Release);
}

pub(crate) fn nand_rb_wait_diagnostic() -> (u32, u32, u32) {
    (NAND_RB_WAIT_CALLS.load(Ordering::Acquire), NAND_RB_WAIT_ITERATIONS.load(Ordering::Acquire), NAND_RB_WAIT_TIMEOUTS.load(Ordering::Acquire))
}

pub(crate) fn wait_ready_rb() -> bool {
    let mut busy_seen = false;
    let mut iterations = 0u32;
    for _ in 0..TIMEOUT {
        iterations += 1;
        let ready = unsafe { read_volatile(pac::GPIOD::ptr().cast::<u8>().add(0x10).cast::<u32>()) } & RB_MASK != 0;
        if !ready {
            busy_seen = true;
        } else if busy_seen {
            NAND_RB_WAIT_CALLS.fetch_add(1, Ordering::Relaxed);
            NAND_RB_WAIT_ITERATIONS.fetch_add(iterations, Ordering::Relaxed);
            return true;
        }
        core::hint::spin_loop();
    }
    NAND_RB_WAIT_CALLS.fetch_add(1, Ordering::Relaxed);
    NAND_RB_WAIT_ITERATIONS.fetch_add(iterations, Ordering::Relaxed);
    NAND_RB_WAIT_TIMEOUTS.fetch_add(1, Ordering::Relaxed);
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

pub(crate) fn program_physical_page(page: u32, data: &[u8]) -> bool {
    if data.len() != PAGE_SIZE { return false; }

    command(0x80);
    address(0);
    address(0);
    address(page as u8);
    address((page >> 8) as u8);
    // K9F1G08U0E tADL minimum is 100 ns.
    cortex_m::asm::delay(1_000);

    let raw_mode = RAW_NAND_TEST_MODE.load(Ordering::SeqCst);
    let software_ecc = SOFTWARE_ECC_TEST_MODE.load(Ordering::SeqCst);
    let write_ecc = !raw_mode;
    let mut ecc = [0xFFu8; ECC_OOB_BYTES];
    for (chunk_index, chunk) in data.chunks_exact(FSMC_ECC_CHUNK_SIZE).enumerate() {
        if write_ecc && !software_ecc { let _ = ecc_begin_chunk(); }
        for &byte in chunk { write_data(byte); }
        // The FSMC buffers NAND writes. Wait for the bus transfer to finish
        // before moving to the next chunk or storing its software ECC.
        if !wait_fsmc_fifo_empty() {
            let status = unsafe { read_volatile(FSMC_SR2) };
            record_nand_device_failure(6, page, status);
            return false;
        }
        if write_ecc {
            let chunk_ecc = if software_ecc {
                ecc_software_512(chunk)
            } else {
                ecc_end_chunk().0
            };
            encode_ecc(chunk_ecc, &mut ecc[chunk_index * ECC_BYTES_PER_CHUNK..][..ECC_BYTES_PER_CHUNK]);
        }
    }

    // Continue the same sequential page-program operation into the spare area.
    // The two leading 0xFF bytes leave the factory marker positions untouched.
    write_data(0xFF);
    write_data(0xFF);
    for &byte in &ecc { write_data(byte); }
    let tag = if write_ecc { ECC_TAG } else { [0xFF; 2] };
    for &byte in &tag { write_data(byte); }

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

/// Read a full data page, computing ECC as the four 512-byte chunks are read.
/// Pages without the tag are treated as legacy raw-NAND pages.
///
/// Apply the STM32 FSMC 512-byte Hamming syndrome to a data chunk.
///
/// The 24-bit syndrome is twelve 2-bit parity pairs. The first three pairs
/// encode the data-bit position; the remaining nine encode the byte address.
/// A data-bit error has 01 or 10 in every pair. A one-bit syndrome is an
/// ECC-bit error and does not require changing the data.
fn ecc_error_location(syndrome: u32) -> Option<(usize, usize)> {
    let mut bit_index = 0usize;
    for bit in 0..3 {
        match (syndrome >> (bit * 2)) & 0x03 {
            0b01 => {}
            0b10 => bit_index |= 1 << bit,
            _ => return None,
        }
    }
    let mut byte_index = 0usize;
    for bit in 0..9 {
        match (syndrome >> (6 + bit * 2)) & 0x03 {
            0b01 => {}
            0b10 => byte_index |= 1 << bit,
            _ => return None,
        }
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

/// Per-chunk evidence captured during the direct, read-only ECC diagnostic.
#[derive(Clone, Copy)]
pub(crate) struct EccStandaloneChunk {
    pub calculated: u32,
    pub pcr_enable: u32,
    pub pcr_capture: u32,
    pub first_byte: u8,
    pub last_byte: u8,
    pub first_bytes: [u8; 16],
    pub checksum: u32,
    pub non_ff_bytes: u16,
}

/// Deterministic reference vectors for checking the software ECC port.
/// Expected values were generated with Linux ecc_sw_hamming_calculate()
/// using 512-byte steps and NAND_ECC_SOFT_HAMMING_SM_ORDER.
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

/// Read one physical NAND page directly and compare FSMC ECC with its OOB
/// values. This bypasses Dhara and LittleFS and never changes NAND contents.
pub(crate) fn ecc_standalone_read(
    page: u32,
) -> Option<(
    [u32; 4],
    [EccStandaloneChunk; FSMC_ECC_CHUNKS_PER_PAGE],
    [u32; 8],
    u32,
    [u32; FSMC_ECC_CHUNKS_PER_PAGE],
)> {
    let mut data = [0u8; PAGE_SIZE];

    // Complete NAND command/address/read-cycle setup before enabling ECC.
    // RM0090's ECC procedure enables ECC immediately before the data transfer;
    // command and address accesses must not be part of the ECC window.
    command(0x00);
    address(0);
    address(0);
    address(page as u8);
    address((page >> 8) as u8);
    command(0x30);
    if !wait_ready_rb() {
        return None;
    }

    let mut chunks = [EccStandaloneChunk {
        calculated: 0,
        pcr_enable: 0,
        pcr_capture: 0,
        first_byte: 0,
        last_byte: 0,
        first_bytes: [0; 16],
        checksum: 0,
        non_ff_bytes: 0,
    }; FSMC_ECC_CHUNKS_PER_PAGE];

    for (index, chunk) in data.chunks_exact_mut(FSMC_ECC_CHUNK_SIZE).enumerate() {
        let pcr_enable = ecc_begin_chunk();
        let mut non_ff_bytes = 0u16;
        let mut first_bytes = [0u8; 16];
        let mut checksum = 2_166_136_261u32; // FNV-1a offset basis
        for (offset, byte) in chunk.iter_mut().enumerate() {
            *byte = read_data();
            if offset < first_bytes.len() {
                first_bytes[offset] = *byte;
            }
            checksum = checksum.wrapping_mul(16_777_619) ^ (*byte as u32);
            if *byte != 0xFF {
                non_ff_bytes += 1;
            }
        }
        // Follow the reference sequence used by the STM32 NAND ECC example:
        // drain the FSMC FIFO before reading ECCR2, then disable ECC to reset
        // the accumulator before the next 512-byte chunk.
        if !wait_fsmc_fifo_empty() {
            let _ = ecc_end_chunk();
            return None;
        }
        let (calculated, pcr_capture) = ecc_end_chunk();
        chunks[index] = EccStandaloneChunk {
            calculated: calculated & 0x00FF_FFFF,
            pcr_enable,
            pcr_capture,
            first_byte: chunk[0],
            last_byte: chunk[chunk.len() - 1],
            first_bytes,
            checksum,
            non_ff_bytes,
        };
    }

    // Read the four stored ECC words directly from the spare area.
    let mut spare_ecc = [0u8; ECC_OOB_BYTES];
    if !read_page(page, PAGE_SIZE as u16 + ECC_OOB_OFFSET, &mut spare_ecc) {
        return None;
    }
    let mut stored = [0u32; FSMC_ECC_CHUNKS_PER_PAGE];
    for index in 0..FSMC_ECC_CHUNKS_PER_PAGE {
        let offset = index * ECC_BYTES_PER_CHUNK;
        stored[index] = u32::from_le_bytes([
            spare_ecc[offset],
            spare_ecc[offset + 1],
            spare_ecc[offset + 2],
            0,
        ]);
    }
    // Repeat the direct read using 256-byte ECC windows. This is a
    // read-only probe of ECCPS=000 and does not compare against OOB values.
    command(0x00);
    address(0);
    address(0);
    address(page as u8);
    address((page >> 8) as u8);
    command(0x30);
    if !wait_ready_rb() {
        return None;
    }
    let mut calculated_256 = [0u32; 8];
    let mut remaining = PAGE_SIZE;
    for value in &mut calculated_256 {
        let _ = ecc_begin_chunk_size(0);
        for _ in 0..256 {
            let _ = read_data();
        }
        if !wait_fsmc_fifo_empty() {
            let _ = ecc_end_chunk();
            return None;
        }
        *value = ecc_end_chunk().0 & 0x00FF_FFFF;
        remaining -= 256;
    }
    debug_assert_eq!(remaining, 0);

    // Finally calculate ECC over the entire 2 KiB page in one window.
    // ECCPS=011 selects 2048 bytes, so this avoids all per-512-byte
    // accumulator resets. This is read-only and is not compared with OOB,
    // whose current layout stores four independent 512-byte ECC values.
    command(0x00);
    address(0);
    address(0);
    address(page as u8);
    address((page >> 8) as u8);
    command(0x30);
    if !wait_ready_rb() {
        return None;
    }
    let _ = ecc_begin_chunk_size(3);
    for _ in 0..PAGE_SIZE {
        let _ = read_data();
    }
    if !wait_fsmc_fifo_empty() {
        let _ = ecc_end_chunk();
        return None;
    }
    let calculated_2048 = ecc_end_chunk().0 & 0x0FFF_FFFF;

    let mut calculated_software = [0u32; FSMC_ECC_CHUNKS_PER_PAGE];
    for (index, chunk) in data.chunks_exact(FSMC_ECC_CHUNK_SIZE).enumerate() {
        calculated_software[index] = ecc_software_512(chunk);
    }

    Some((stored, chunks, calculated_256, calculated_2048, calculated_software))
}

pub(crate) fn read_data_page(page: u32, out: &mut [u8]) -> bool {
    if out.len() != PAGE_SIZE {
        return false;
    }

    let raw_read = RAW_NAND_TEST_MODE.load(Ordering::SeqCst)
        || ECC_WRITE_RAW_READ_TEST_MODE.load(Ordering::SeqCst);
    let software_ecc = SOFTWARE_ECC_TEST_MODE.load(Ordering::SeqCst);

    // In the software-ECC experiment, leave the FSMC ECC engine disabled:
    // calculate the reference code from the bytes actually read from NAND.
    if !raw_read && !software_ecc {
        let _ = ecc_begin_chunk();
    }

    command(0x00);
    address(0);
    address(0);
    address(page as u8);
    address((page >> 8) as u8);
    command(0x30);
    if !wait_ready_rb() {
        if !raw_read && !software_ecc {
            let _ = ecc_end_chunk();
        }
        record_nand_read_failure(1, page);
        record_nand_device_failure(1, page, 0);
        return false;
    }

    if raw_read {
        for byte in out.iter_mut() { *byte = read_data(); }
        return true;
    }

    let mut calculated = [0u8; ECC_OOB_BYTES];
    for (chunk_index, chunk) in out.chunks_exact_mut(FSMC_ECC_CHUNK_SIZE).enumerate() {
        if !software_ecc && chunk_index != 0 {
            let _ = ecc_begin_chunk();
        }
        let pcr_enable = unsafe { read_volatile(FSMC_PCR2) };
        ECC_LAST_PCR_ENABLE[chunk_index].store(pcr_enable, Ordering::Relaxed);
        for byte in chunk.iter_mut() {
            *byte = read_data();
        }
        let (chunk_ecc, pcr_capture) = if software_ecc {
            (ecc_software_512(chunk), unsafe { read_volatile(FSMC_PCR2) })
        } else {
            ecc_end_chunk()
        };
        ECC_LAST_PCR_CAPTURE[chunk_index].store(pcr_capture, Ordering::Relaxed);
        ECC_LAST_PAGE_CHUNKS[chunk_index].store(chunk_ecc, Ordering::Relaxed);
        encode_ecc(chunk_ecc, &mut calculated[chunk_index * ECC_BYTES_PER_CHUNK..][..ECC_BYTES_PER_CHUNK]);
    }

    let mut stored = [0u8; ECC_OOB_BYTES + ECC_TAG.len()];
    if !read_page(page, PAGE_SIZE as u16 + ECC_OOB_OFFSET, &mut stored) {
        record_nand_read_failure(2, page);
        return false;
    }

    let tag = &stored[ECC_OOB_BYTES..];
    ECC_LAST_PAGE.store(page, Ordering::Relaxed);
    if tag == [0xFF, 0xFF] {
        ECC_READ_COUNTS[1].fetch_add(1, Ordering::Relaxed);
        ECC_LAST_TAG.store(2, Ordering::Release);
        return true;
    }
    if tag != ECC_TAG {
        record_nand_read_failure(3, u32::from(tag[0]) | (u32::from(tag[1]) << 8));
        return false;
    }

    ECC_READ_COUNTS[0].fetch_add(1, Ordering::Relaxed);
    ECC_LAST_TAG.store(1, Ordering::Release);
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
        let syndrome = (stored_ecc ^ calculated_ecc) & 0x00FF_FFFF;
        if SOFTWARE_ECC_COMPARE_TEST_MODE.load(Ordering::SeqCst) {
            let software_calculated = ecc_software_512(chunk);
            SOFTWARE_ECC_COMPARE_COUNTS[0].fetch_add(1, Ordering::Relaxed);
            if software_calculated != stored_ecc {
                if SOFTWARE_ECC_COMPARE_COUNTS[1].fetch_add(1, Ordering::Relaxed) == 0 {
                    SOFTWARE_ECC_FIRST_PAGE.store(page, Ordering::Relaxed);
                    SOFTWARE_ECC_FIRST_CHUNK.store(chunk_index as u32, Ordering::Relaxed);
                    SOFTWARE_ECC_FIRST_STORED.store(stored_ecc, Ordering::Relaxed);
                    SOFTWARE_ECC_FIRST_CALCULATED.store(software_calculated, Ordering::Release);
                }
            }
        }
        ECC_LAST_CHUNK.store(chunk_index as u32, Ordering::Relaxed);
        ECC_LAST_SYNDROME.store(syndrome, Ordering::Relaxed);
        ECC_LAST_STORED.store(stored_ecc, Ordering::Relaxed);
        ECC_LAST_CALCULATED.store(calculated_ecc, Ordering::Relaxed);
        ECC_LAST_BYTE.store(u32::MAX, Ordering::Relaxed);
        if syndrome != 0 {
            if ECC_READ_COUNTS[2].fetch_add(1, Ordering::Relaxed) == 0 {
                ECC_FIRST_PAGE.store(page, Ordering::Relaxed);
                ECC_FIRST_CHUNK.store(chunk_index as u32, Ordering::Relaxed);
                ECC_FIRST_SYNDROME.store(syndrome, Ordering::Relaxed);
                ECC_FIRST_STORED.store(stored_ecc, Ordering::Relaxed);
                ECC_FIRST_CALCULATED.store(calculated_ecc, Ordering::Relaxed);
            }
            if syndrome.count_ones() == 1 {
                ECC_READ_COUNTS[3].fetch_add(1, Ordering::Relaxed);
            }
        }
        // Decode the candidate correction for telemetry; the actual decoder
        // below remains the sole code that mutates the page buffer.
        let mut changed_byte = u32::MAX;
        let mut before = 0u32;
        let mut after = 0u32;
        if syndrome != 0 && syndrome.count_ones() != 1 {
            if let Some((byte_index, bit_index)) = ecc_error_location(syndrome) {
                if byte_index < chunk.len() {
                    changed_byte = byte_index as u32;
                    before = chunk[byte_index] as u32;
                    after = (chunk[byte_index] ^ (1 << bit_index)) as u32;
                }
            }
        }
        if ECC_CHECK_NO_CORRECT_TEST_MODE.load(Ordering::SeqCst) {
            // Preserve raw data while collecting ECC syndromes for comparison.
            continue;
        }
        if !correct_ecc_chunk(chunk, stored_ecc, calculated_ecc) {
            record_ecc_read_failure(page, chunk_index as u32, stored_ecc, calculated_ecc);
            return false;
        }
        if changed_byte != u32::MAX {
            ECC_READ_COUNTS[4].fetch_add(1, Ordering::Relaxed);
            ECC_LAST_BYTE.store(changed_byte, Ordering::Relaxed);
            ECC_LAST_BEFORE.store(before, Ordering::Relaxed);
            ECC_LAST_AFTER.store(after, Ordering::Relaxed);
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
