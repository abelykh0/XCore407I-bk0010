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

use core::mem::MaybeUninit;
// Switch MSP to a caller-provided, 8-byte-aligned stack for one synchronous call.
// r0 = entry function, r1 = context, r2 = stack top.
core::arch::global_asm!(
    ".syntax unified",
    ".thumb",
    ".global __xcore_call_on_stack",
    ".type __xcore_call_on_stack, %function",
    ".thumb_func",
    "__xcore_call_on_stack:",
    "push {{r4, r5, r6, lr}}",
    "mov r4, sp",
    "mov r5, r0",
    "mov sp, r2",
    "mov r0, r1",
    "blx r5",
    "mov sp, r4",
    "pop {{r4, r5, r6, pc}}",
);

unsafe extern "C" {
    fn __xcore_call_on_stack(
        entry: unsafe extern "C" fn(*mut ()),
        context: *mut (),
        stack_top: *mut u8,
    );
}

struct AlternateStackCall<F, R> {
    function: Option<F>,
    result: MaybeUninit<R>,
}

unsafe extern "C" fn invoke_on_alternate_stack<F, R>(context: *mut ())
where
    F: FnOnce() -> R,
{
    let call = unsafe { &mut *context.cast::<AlternateStackCall<F, R>>() };
    let function = call.function.take().unwrap();
    call.result.write(function());
}

/// Run one synchronous operation on the CCM canvas-tail region. The region is
/// reserved for the stack only in CDC mode; normal mode continues to use it as
/// part of the video canvas.
pub unsafe fn with_cdc_alternate_stack<F, R>(function: F) -> R
where
    F: FnOnce() -> R,
{
    let mut call = AlternateStackCall {
        function: Some(function),
        result: MaybeUninit::uninit(),
    };
    let stack_top = crate::camera::cdc_alternate_stack_top();
    // The tail storage is 8-byte aligned and its size is a multiple of 8.
    unsafe {
        __xcore_call_on_stack(
            invoke_on_alternate_stack::<F, R>,
            (&mut call as *mut AlternateStackCall<F, R>).cast(),
            stack_top,
        );
        call.result.assume_init()
    }
}

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering};
use stm32f4::stm32f407 as pac;
use crate::nand_ecc::{correct_ecc_chunk, ecc_software_512, ecc_software_512_fsmc};

use littlefs2::{
    fs::{Allocation as LittleFsAllocation, Filesystem},
    io::Result as LfsResult,
};

const NAND_BASE: usize = 0x7000_0000;

// Access FSMC NAND Bank 2 through the PAC register block rather than
// duplicating its peripheral base address and register offsets.
#[inline(always)]
fn fsmc_pcr2() -> *mut u32 {
    unsafe { (*pac::FSMC::ptr()).pcr2().as_ptr() }
}

#[inline(always)]
fn fsmc_sr2() -> *const u32 {
    unsafe { (*pac::FSMC::ptr()).sr2().as_ptr() }
}

#[inline(always)]
fn fsmc_pmem2() -> *mut u32 {
    unsafe { (*pac::FSMC::ptr()).pmem2().as_ptr() }
}

#[inline(always)]
fn fsmc_patt2() -> *mut u32 {
    unsafe { (*pac::FSMC::ptr()).patt2().as_ptr() }
}

#[inline(always)]
fn fsmc_eccr2() -> *const u32 {
    unsafe { (*pac::FSMC::ptr()).eccr2().as_ptr() }
}

const FSMC_SR_FEMPT: u32 = 1 << 6;
const FSMC_PCR_ECCEN: u32 = 1 << 6;
const FSMC_ECC_CHUNK_SIZE: usize = 512;
const FSMC_ECC_CHUNKS_PER_PAGE: usize = PAGE_SIZE / FSMC_ECC_CHUNK_SIZE;

const _: () = assert!(FSMC_ECC_CHUNKS_PER_PAGE == 4);

pub(crate) const RB_MASK: u32 = 1 << 6;
pub(crate) const CMD: *mut u8 = (NAND_BASE | (1 << 17)) as *mut u8;
pub(crate) const ADDR: *mut u8 = (NAND_BASE | (1 << 16)) as *mut u8;
// Bank 2 attribute-space alias of the address latch. RM0090 §36.6.5
// uses this for the final address byte so ATTHOLD can keep NCE asserted
// across NAND tWB without penalizing ordinary common-space accesses.
const ATTR_ADDR: *mut u8 = (NAND_BASE | (1 << 27) | (1 << 16)) as *mut u8;
pub(crate) const DATA: *mut u8 = NAND_BASE as *mut u8;
const TIMEOUT: u32 = 20_000_000;
// Keep the interactive ECCPAGE diagnostic responsive if NAND R/B never rises.
const ECC_DIAGNOSTIC_TIMEOUT: u32 = TIMEOUT;

static mut ECC_DIAGNOSTIC_PAGE_BUFFER: [u8; PAGE_SIZE] = [0; PAGE_SIZE];

// Optional main-context service hook used while synchronous NAND operations
// wait for the device. The normal file browser and CDC foreground install it
// around filesystem work; it must not access NAND or call back into LittleFS.
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

// One caller-owned LittleFS mount workspace shared by normal and CDC modes.
// The modes are mutually exclusive, so this allocation is never mounted twice
// concurrently. Keeping it static avoids placing the multi-kilobyte cache and
// LittleFS state on the limited main stack.
static mut LITTLEFS_MOUNT_ALLOCATION: MaybeUninit<LittleFsAllocation<ActiveStorage>> =
    MaybeUninit::uninit();

fn mount_with_shared_allocation<R>(
    storage: &mut ActiveStorage,
    service: impl FnOnce(&Filesystem<'_, ActiveStorage>) -> LfsResult<R>,
) -> LfsResult<R> {
    // SAFETY: firmware runs one main-context filesystem operation at a time;
    // the normal and CDC modes do not run concurrently. Interrupt handlers
    // must not access LittleFS.
    unsafe {
        let allocation_ptr = core::ptr::addr_of_mut!(LITTLEFS_MOUNT_ALLOCATION)
            .cast::<LittleFsAllocation<ActiveStorage>>();
        allocation_ptr.write(LittleFsAllocation::new());
        let allocation = &mut *allocation_ptr;
        let filesystem = Filesystem::<ActiveStorage>::mount(allocation, storage)?;
        service(&filesystem)
    }
}

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
#[inline]
fn address_last(value: u8) { unsafe { write_volatile(ATTR_ADDR, value) } }
pub(crate) fn write_data(value: u8) { unsafe { write_volatile(DATA, value) } }
pub(crate) fn read_data() -> u8 { unsafe { read_volatile(DATA) } }

/// Prepare the FSMC ECC engine for one 512-byte data chunk.
fn ecc_begin_chunk() {
    unsafe {
        let pcr = read_volatile(fsmc_pcr2());
        // ECCPS is configured once in configure_fsmc(). Match ST's HAL
        // sequence here: disable ECC, then re-enable it by changing ECCEN
        // only; do not rewrite the page-size field for every chunk.
        let disabled = pcr & !FSMC_PCR_ECCEN;
        write_volatile(fsmc_pcr2(), disabled);
        let _ = read_volatile(fsmc_pcr2());
        write_volatile(fsmc_pcr2(), disabled | FSMC_PCR_ECCEN);
        let _ = read_volatile(fsmc_pcr2());
    }
}

/// Wait for all preceding NAND data writes to reach the FSMC, then wait
/// until its write FIFO is empty before reading ECCR2.
///
/// NAND data is mapped at 0x7000_0000, in the Cortex-M4 default External RAM
/// (Normal memory) region, while FSMC status registers are at 0xA000_0000
/// (External Device memory). PM0214 §2.2.4 says ordering between these
/// different memory types is not guaranteed without a memory barrier.
fn wait_fsmc_fifo_empty() -> (bool, u32) {
    cortex_m::asm::dsb();
    for iteration in 0..TIMEOUT {
        if unsafe { read_volatile(fsmc_sr2()) } & FSMC_SR_FEMPT != 0 {
            return (true, iteration + 1);
        }
        core::hint::spin_loop();
    }
    (false, TIMEOUT)
}

/// Read the 24-bit ECC result and disable the engine before reuse.
fn ecc_end_chunk() -> u32 {
    unsafe {
        let pcr = read_volatile(fsmc_pcr2());
        let ecc = read_volatile(fsmc_eccr2()) & 0x00FF_FFFF;
        write_volatile(fsmc_pcr2(), pcr & !FSMC_PCR_ECCEN);
        let _ = read_volatile(fsmc_pcr2());
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
    wait_ready_rb_timeout(TIMEOUT)
}

fn wait_ready_rb_timeout(timeout: u32) -> bool {
    // K9F1G08U0E R/B is normally high. After READ, PROGRAM, ERASE, or
    // RESET starts, it first transitions low and returns high on completion.
    // Waiting only for high can therefore accept the idle-high level before
    // the NAND has even entered its busy phase (notably after PROGRAM CONFIRM).
    // GPIOD IDR is accessed through the PAC instead of a raw register offset.
    let gpiod = unsafe { &*pac::GPIOD::ptr() };

    // First observe that the command was accepted and the NAND became busy.
    let mut saw_busy = false;
    for _ in 0..timeout {
        if gpiod.idr().read().bits() & RB_MASK == 0 {
            saw_busy = true;
            break;
        }
        core::hint::spin_loop();
    }
    if !saw_busy {
        return false;
    }

    // Then wait for the actual completion edge.
    for _ in 0..timeout {
        if gpiod.idr().read().bits() & RB_MASK != 0 {
            return true;
        }
        core::hint::spin_loop();
    }
    false
}

pub(crate) fn read_page(page: u32, column: u16, out: &mut [u8]) -> bool {
    read_page_with_timeout(page, column, out, TIMEOUT)
}

fn read_page_with_timeout(page: u32, column: u16, out: &mut [u8], timeout: u32) -> bool {
    if out.is_empty() || column as usize + out.len() > PAGE_SIZE + SPARE_SIZE {
        return false;
    }
    command(0x00);
    address(column as u8);
    address((column >> 8) as u8);
    address(page as u8);
    address_last((page >> 8) as u8);
    command(0x30);
    if !wait_ready_rb_timeout(timeout) {
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
    address_last((first_page >> 8) as u8);
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

/// Snapshot of the most recent page-program ECC verification.
#[derive(Clone, Copy)]
pub(crate) struct EccWriteTrace {
    pub page: u32,
    pub flags: u32,
    pub program_count: u32,
    pub mismatch_chunks: u32,
    pub hardware: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub input_software: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub stored: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub readback_software: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub input_fingerprint: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub readback_fingerprint: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
}

/// Low-overhead trace of the FSMC ECC/FIFO sequence during page programming.
/// Unlike ECCWRITE, this does not perform a NAND readback.
#[derive(Clone, Copy)]
pub(crate) struct EccSequenceTrace {
    pub page: u32,
    pub flags: u32,
    pub pcr_before: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub pcr_enabled: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub ecc_before_data: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub input_fingerprint: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub input_non_ff: [u16; FSMC_ECC_CHUNKS_PER_PAGE],
    pub status_before_wait: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub status_after_wait: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub fifo_poll_count: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub ecc_after_wait: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub software_ecc: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub pcr_after_disable: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub ecc_after_disable: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub oob_status_before_wait: u32,
    pub oob_status_after_wait: u32,
    pub oob_fifo_poll_count: u32,
}
static ECC_SEQUENCE_TRACE_ENABLED: AtomicU32 = AtomicU32::new(0);
static ECC_SEQUENCE_TRACE_CAPTURED: AtomicU32 = AtomicU32::new(0);
static mut LAST_ECC_SEQUENCE_TRACE: EccSequenceTrace = EccSequenceTrace {
    page: u32::MAX, flags: 0,
    pcr_before: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    pcr_enabled: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    ecc_before_data: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    input_fingerprint: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    input_non_ff: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    status_before_wait: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    status_after_wait: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    fifo_poll_count: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    ecc_after_wait: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    software_ecc: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    pcr_after_disable: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    ecc_after_disable: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    oob_status_before_wait: 0, oob_status_after_wait: 0, oob_fifo_poll_count: 0,
};

pub(crate) fn set_ecc_sequence_trace_enabled(enabled: bool) {
    if enabled {
        ECC_SEQUENCE_TRACE_CAPTURED.store(0, Ordering::Release);
        unsafe {
            core::ptr::write_volatile(core::ptr::addr_of_mut!(LAST_ECC_SEQUENCE_TRACE),
                EccSequenceTrace {
                    page: u32::MAX, flags: 0,
                    pcr_before: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                    pcr_enabled: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                    ecc_before_data: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                    input_fingerprint: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                    input_non_ff: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                    status_before_wait: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                    status_after_wait: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                    fifo_poll_count: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                                    ecc_after_wait: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                    software_ecc: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                    pcr_after_disable: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                    ecc_after_disable: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                    oob_status_before_wait: 0, oob_status_after_wait: 0, oob_fifo_poll_count: 0,
                            });
        }
    }
    ECC_SEQUENCE_TRACE_ENABLED.store(enabled as u32, Ordering::Release);
}

pub(crate) fn ecc_sequence_trace() -> EccSequenceTrace {
    unsafe { core::ptr::read_volatile(core::ptr::addr_of!(LAST_ECC_SEQUENCE_TRACE)) }
}

static ECC_WRITE_TRACE_ENABLED: AtomicU32 = AtomicU32::new(0);
static ECC_WRITE_TRACE_PROGRAM_COUNT: AtomicU32 = AtomicU32::new(0);
static ECC_WRITE_TRACE_MISMATCH_SEEN: AtomicU32 = AtomicU32::new(0);
static mut LAST_ECC_WRITE_TRACE: EccWriteTrace = EccWriteTrace {
    page: u32::MAX,
    flags: 0,
    program_count: 0,
    mismatch_chunks: 0,
    hardware: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    input_software: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    stored: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    readback_software: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    input_fingerprint: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    readback_fingerprint: [0; FSMC_ECC_CHUNKS_PER_PAGE],
};

pub(crate) fn set_ecc_write_trace_enabled(enabled: bool) {
    if enabled {
        ECC_WRITE_TRACE_PROGRAM_COUNT.store(0, Ordering::Release);
        ECC_WRITE_TRACE_MISMATCH_SEEN.store(0, Ordering::Release);
        unsafe {
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!(LAST_ECC_WRITE_TRACE),
                EccWriteTrace {
                    page: u32::MAX,
                    flags: 0,
                    program_count: 0,
                    mismatch_chunks: 0,
                    hardware: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                    input_software: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                    stored: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                    readback_software: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                    input_fingerprint: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                    readback_fingerprint: [0; FSMC_ECC_CHUNKS_PER_PAGE],
                },
            );
        }
    }
    ECC_WRITE_TRACE_ENABLED.store(enabled as u32, Ordering::Release);
}

pub(crate) fn ecc_write_trace() -> EccWriteTrace {
    unsafe { core::ptr::read_volatile(core::ptr::addr_of!(LAST_ECC_WRITE_TRACE)) }
}

fn ecc_data_fingerprint(data: &[u8]) -> u32 {
    let mut fingerprint = 0x811C_9DC5u32;
    for &byte in data {
        fingerprint = (fingerprint ^ byte as u32).wrapping_mul(0x0100_0193);
    }
    fingerprint
}

/// Program a full page and store four FSMC-generated 512-byte ECC values.
pub(crate) fn program_physical_page(page: u32, data: &[u8]) -> bool {
    if data.len() != PAGE_SIZE { return false; }

    command(0x80);
    address(0);
    address(0);
    address(page as u8);
    address_last((page >> 8) as u8);
    cortex_m::asm::delay(1_000);

    let trace_enabled = ECC_WRITE_TRACE_ENABLED.load(Ordering::Acquire) != 0;
    let sequence_trace_enabled = ECC_SEQUENCE_TRACE_ENABLED.load(Ordering::Acquire) != 0
        && ECC_SEQUENCE_TRACE_CAPTURED.load(Ordering::Acquire) == 0;
    let mut sequence_trace = EccSequenceTrace {
        page, flags: 0,
        pcr_before: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        pcr_enabled: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        ecc_before_data: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        input_fingerprint: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        input_non_ff: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        status_before_wait: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        status_after_wait: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        fifo_poll_count: [0; FSMC_ECC_CHUNKS_PER_PAGE],
            ecc_after_wait: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        software_ecc: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        pcr_after_disable: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        ecc_after_disable: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        oob_status_before_wait: 0, oob_status_after_wait: 0, oob_fifo_poll_count: 0,
    };
    let mut ecc = [0xFFu8; ECC_OOB_BYTES];
    let mut hardware_ecc = [0u32; FSMC_ECC_CHUNKS_PER_PAGE];
    let mut input_software_ecc = [0u32; FSMC_ECC_CHUNKS_PER_PAGE];
    let mut input_fingerprint = [0u32; FSMC_ECC_CHUNKS_PER_PAGE];
    for (chunk_index, chunk) in data.chunks_exact(FSMC_ECC_CHUNK_SIZE).enumerate() {
        if sequence_trace_enabled {
            sequence_trace.pcr_before[chunk_index] = unsafe { read_volatile(fsmc_pcr2()) };
        }
        ecc_begin_chunk();
        if sequence_trace_enabled {
            sequence_trace.pcr_enabled[chunk_index] = unsafe { read_volatile(fsmc_pcr2()) };
            sequence_trace.ecc_before_data[chunk_index] = unsafe { read_volatile(fsmc_eccr2()) } & 0x00FF_FFFF;
        }
        if sequence_trace_enabled {
            sequence_trace.input_fingerprint[chunk_index] = ecc_data_fingerprint(chunk);
            sequence_trace.input_non_ff[chunk_index] =
                chunk.iter().filter(|&&byte| byte != 0xFF).count() as u16;
        }
        for &byte in chunk { write_data(byte); }
        if sequence_trace_enabled {
            sequence_trace.status_before_wait[chunk_index] = unsafe { read_volatile(fsmc_sr2()) };
        }
        let (fifo_empty, fifo_polls) = wait_fsmc_fifo_empty();
        if sequence_trace_enabled {
            sequence_trace.fifo_poll_count[chunk_index] = fifo_polls;
        }
        if !fifo_empty {
            let status = unsafe { read_volatile(fsmc_sr2()) };
            record_nand_device_failure(6, page, status);
            return false;
        }
        if sequence_trace_enabled {
            sequence_trace.status_after_wait[chunk_index] = unsafe { read_volatile(fsmc_sr2()) };
        }
        let chunk_ecc = ecc_end_chunk();
        if sequence_trace_enabled {
            sequence_trace.ecc_after_wait[chunk_index] = chunk_ecc;
            sequence_trace.pcr_after_disable[chunk_index] = unsafe { read_volatile(fsmc_pcr2()) };
            sequence_trace.ecc_after_disable[chunk_index] = unsafe { read_volatile(fsmc_eccr2()) } & 0x00FF_FFFF;
        }
        hardware_ecc[chunk_index] = chunk_ecc;
        // Software ECC is diagnostic-only; it is not needed to program NAND.
        // Keep it out of the normal page-program path.
        if trace_enabled || sequence_trace_enabled {
            input_software_ecc[chunk_index] = ecc_software_512_fsmc(chunk);
            if sequence_trace_enabled {
                sequence_trace.software_ecc[chunk_index] = input_software_ecc[chunk_index];
            }
        }
        if trace_enabled {
            input_fingerprint[chunk_index] = ecc_data_fingerprint(chunk);
        }
        encode_ecc(chunk_ecc, &mut ecc[chunk_index * ECC_BYTES_PER_CHUNK..][..ECC_BYTES_PER_CHUNK]);

    }

    write_data(0xFF);
    write_data(0xFF);
    for &byte in &ecc { write_data(byte); }
    for &byte in &ECC_TAG { write_data(byte); }

    // Drain the FSMC write FIFO after the spare-area ECC and tag bytes too.
    // The program-confirm command must not be issued while OOB data remains
    // buffered in the FSMC NAND write FIFO.
    if sequence_trace_enabled {
        sequence_trace.oob_status_before_wait = unsafe { read_volatile(fsmc_sr2()) };
    }
    let (oob_fifo_empty, oob_fifo_polls) = wait_fsmc_fifo_empty();
    if sequence_trace_enabled {
        sequence_trace.oob_fifo_poll_count = oob_fifo_polls;
    }
    if !oob_fifo_empty {
        let status = unsafe { read_volatile(fsmc_sr2()) };
        record_nand_device_failure(7, page, status);
        return false;
    }

    if sequence_trace_enabled {
        sequence_trace.oob_status_after_wait = unsafe { read_volatile(fsmc_sr2()) };
        sequence_trace.flags |= 1;
    }
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
    if sequence_trace_enabled {
        sequence_trace.flags |= 1 << 1; // NAND program-confirm/status succeeded.
        if ECC_SEQUENCE_TRACE_CAPTURED.compare_exchange(0, 1, Ordering::AcqRel, Ordering::Relaxed).is_ok() {
            unsafe {
                core::ptr::write_volatile(
                    core::ptr::addr_of_mut!(LAST_ECC_SEQUENCE_TRACE),
                    sequence_trace,
                );
            }
        }
    }
    if trace_enabled {
        // ECCWRITE-only verification reads the programmed data and OOB.
        let readback = unsafe {
            &mut *core::ptr::addr_of_mut!(ECC_DIAGNOSTIC_PAGE_BUFFER)
        };
        let data_ok = read_page(page, 0, readback);
        let mut spare = [0u8; ECC_OOB_BYTES + ECC_TAG.len()];
        let spare_ok = read_page(
            page,
            PAGE_SIZE as u16 + ECC_OOB_OFFSET,
            &mut spare,
        );
        let tag_ok = spare_ok && spare[ECC_OOB_BYTES..] == ECC_TAG;
        let mut trace = EccWriteTrace {
            page,
            flags: 1, // NAND program/status succeeded.
            program_count: ECC_WRITE_TRACE_PROGRAM_COUNT.fetch_add(1, Ordering::AcqRel) + 1,
            mismatch_chunks: 0,
            hardware: hardware_ecc,
            input_software: input_software_ecc,
            stored: [0; FSMC_ECC_CHUNKS_PER_PAGE],
            readback_software: [0; FSMC_ECC_CHUNKS_PER_PAGE],
            input_fingerprint,
            readback_fingerprint: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        };
        if data_ok {
            trace.flags |= 1 << 1;
            for (index, chunk) in readback.chunks_exact(FSMC_ECC_CHUNK_SIZE).enumerate() {
                trace.readback_software[index] = ecc_software_512_fsmc(chunk);
                trace.readback_fingerprint[index] = ecc_data_fingerprint(chunk);
            }
        }
        if spare_ok { trace.flags |= 1 << 2; }
        if tag_ok {
            trace.flags |= 1 << 3;
            for index in 0..FSMC_ECC_CHUNKS_PER_PAGE {
                let offset = index * ECC_BYTES_PER_CHUNK;
                trace.stored[index] = u32::from_le_bytes([
                    spare[offset], spare[offset + 1], spare[offset + 2], 0,
                ]);
            }
        }
        for index in 0..FSMC_ECC_CHUNKS_PER_PAGE {
            let readback_ok = data_ok && tag_ok;
            if trace.hardware[index] != trace.input_software[index]
                || (tag_ok && trace.stored[index] != trace.hardware[index])
                || (readback_ok && trace.readback_software[index] != trace.stored[index])
                || (data_ok && trace.input_fingerprint[index] != trace.readback_fingerprint[index])
            {
                trace.mismatch_chunks |= 1 << index;
            }
        }
        // Preserve the first bad page for diagnosis. If no mismatch has
        // occurred, keep updating the snapshot so status shows the latest
        // successful write.
        let should_store = if trace.mismatch_chunks != 0 {
            ECC_WRITE_TRACE_MISMATCH_SEEN.compare_exchange(
                0, 1, Ordering::AcqRel, Ordering::Relaxed,
            ).is_ok()
        } else {
            ECC_WRITE_TRACE_MISMATCH_SEEN.load(Ordering::Acquire) == 0
        };
        if should_store {
            unsafe {
                core::ptr::write_volatile(
                    core::ptr::addr_of_mut!(LAST_ECC_WRITE_TRACE),
                    trace,
                );
            }
        }
    }
    true
}

/// Read-only raw NAND interface probe: READ ID and STATUS, without page access.
pub(crate) fn ecc_scan_interface_probe() -> ([u8; 5], u8, bool) {
    // Avoid FSMC NWAIT stalling the CPU while probing the bus. Restore PCR2
    // exactly as found before returning.
    let pcr = unsafe { read_volatile(fsmc_pcr2()) };
    unsafe {
        write_volatile(fsmc_pcr2(), pcr & !(1 << 1));
        let _ = read_volatile(fsmc_pcr2());
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
        write_volatile(fsmc_pcr2(), pcr);
        let _ = read_volatile(fsmc_pcr2());
    }
    (id, status, ready)
}

/// Read-only raw page-read probe. Reads main and spare areas from pages 0 and 0x83.
/// NWAIT is bypassed and the USB service hook is suppressed for deterministic bus reads.
pub(crate) fn ecc_scan_page_probe() -> ([bool; 8], [[u8; 16]; 8]) {
    let mut ok = [false; 8];
    let mut data = [[0xEEu8; 16]; 8];
    let hook = NAND_IO_SERVICE_HOOK.swap(core::ptr::null_mut(), Ordering::AcqRel);
    let pcr = unsafe { read_volatile(fsmc_pcr2()) };
    unsafe {
        write_volatile(fsmc_pcr2(), pcr & !(1 << 1));
        let _ = read_volatile(fsmc_pcr2());
    }
    let pages = [0u32, 0, 0, 0x83, 0x83, 0x83, 0, 0x83];
    let columns = [0u16, 1, 512, 0, 1, 512, PAGE_SIZE as u16, PAGE_SIZE as u16];
    for i in 0..8 {
        ok[i] = read_page(pages[i], columns[i], &mut data[i]);
    }
    unsafe {
        write_volatile(fsmc_pcr2(), pcr);
        let _ = read_volatile(fsmc_pcr2());
    }
    NAND_IO_SERVICE_HOOK.store(hook, Ordering::Release);
    (ok, data)
}

/// Repeat representative raw page reads, issuing RESET and polling STATUS before
/// every sample. This is diagnostic-only and never programs or erases the array.
pub(crate) fn ecc_scan_reset_page_probe() -> ([[bool; 3]; 8], [[bool; 3]; 8], [[[u8; 16]; 3]; 8]) {
    let mut reset_ok = [[false; 3]; 8];
    let mut read_ok = [[false; 3]; 8];
    let mut data = [[[0xEEu8; 16]; 3]; 8];
    let hook = NAND_IO_SERVICE_HOOK.swap(core::ptr::null_mut(), Ordering::AcqRel);
    let pcr = unsafe { read_volatile(fsmc_pcr2()) };
    unsafe {
        write_volatile(fsmc_pcr2(), pcr & !(1 << 1) & !FSMC_PCR_ECCEN);
        let _ = read_volatile(fsmc_pcr2());
    }
    let pages = [0u32, 0, 0, 0, 0x83, 0x83, 0x83, 0x83];
    let columns = [0u16, 1, 512, PAGE_SIZE as u16, 0, 1, 512, PAGE_SIZE as u16];
    for i in 0..8 {
        for repeat in 0..3 {
            command(0xFF);
            // Wait only on the physical R/B pin. Do not issue STATUS (0x70):
            // the purpose of this test is to isolate RESET -> READ sequencing.
            reset_ok[i][repeat] = wait_ready_rb();
            if reset_ok[i][repeat] {
                read_ok[i][repeat] = read_page(pages[i], columns[i], &mut data[i][repeat]);
            }
        }
    }
    unsafe {
        write_volatile(fsmc_pcr2(), pcr);
        let _ = read_volatile(fsmc_pcr2());
    }
    NAND_IO_SERVICE_HOOK.store(hook, Ordering::Release);
    (reset_ok, read_ok, data)
}

/// Read-only classification helpers used by the whole-device ECC scan.
pub(crate) fn ecc_scan_block_marker(page: u32) -> Option<u8> {
    if page >= PHYSICAL_BLOCKS * PAGES_PER_BLOCK { return None; }

    // Use exactly the same marker read as Dhara's block_is_bad(): one byte
    // at spare-area column PAGE_SIZE. Do not change FSMC timing or the NAND
    // service hook here; the diagnostic must not use a different bus path.
    let mut marker = [0xFFu8; 1];
    if !read_page(page, PAGE_SIZE as u16, &mut marker) {
        return None;
    }
    Some(marker[0])
}

pub(crate) fn ecc_scan_page_tagged(page: u32) -> Option<bool> {
    if page >= PHYSICAL_BLOCKS * PAGES_PER_BLOCK { return None; }
    let mut tag = [0u8; 2];
    if !read_page(page, PAGE_SIZE as u16 + ECC_OOB_OFFSET + ECC_OOB_BYTES as u16, &mut tag) {
        return None;
    }
    Some(tag == ECC_TAG)
}

/// Read one tagged physical page and compare stored ECC with independent
/// software ECC. The FSMC ECC engine remains disabled throughout this scan,
/// isolating raw NAND reads and software calculation from the hardware ECC/FIFO path.
pub(crate) struct EccPageComparison {
    pub stored: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub software: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    /// FNV-1a fingerprints of each raw 512-byte chunk, before ECC correction.
    pub data_fingerprint: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
}

pub(crate) fn ecc_software_compare_page(page: u32) -> Result<EccPageComparison, (u8, u32)> {
    if page >= PHYSICAL_BLOCKS * PAGES_PER_BLOCK { return Err((0, page)); }
    // SAFETY: ECCPAGE is a synchronous foreground diagnostic and does not
    // run concurrently with a ZMODEM receiver session or from an ISR.
    let data = unsafe {
        core::slice::from_raw_parts_mut(
            core::ptr::addr_of_mut!(ECC_DIAGNOSTIC_PAGE_BUFFER).cast::<u8>(),
            PAGE_SIZE,
        )
    };
    let pcr = unsafe { read_volatile(fsmc_pcr2()) };
    unsafe {
        write_volatile(fsmc_pcr2(), pcr & !FSMC_PCR_ECCEN);
        let _ = read_volatile(fsmc_pcr2());
    }
    if !read_page_with_timeout(page, 0, data, ECC_DIAGNOSTIC_TIMEOUT) {
        unsafe {
            write_volatile(fsmc_pcr2(), pcr & !FSMC_PCR_ECCEN);
            let _ = read_volatile(fsmc_pcr2());
        }
        return Err((1, page));
    }
    let mut spare = [0u8; ECC_OOB_BYTES + ECC_TAG.len()];
    if !read_page_with_timeout(
        page,
        PAGE_SIZE as u16 + ECC_OOB_OFFSET,
        &mut spare,
        ECC_DIAGNOSTIC_TIMEOUT,
    ) {
        unsafe {
            write_volatile(fsmc_pcr2(), pcr & !FSMC_PCR_ECCEN);
            let _ = read_volatile(fsmc_pcr2());
        }
        return Err((2, page));
    }
    unsafe {
        write_volatile(fsmc_pcr2(), pcr & !FSMC_PCR_ECCEN);
        let _ = read_volatile(fsmc_pcr2());
    }
    if spare[ECC_OOB_BYTES..] != ECC_TAG {
        let tag = u32::from(spare[ECC_OOB_BYTES])
            | (u32::from(spare[ECC_OOB_BYTES + 1]) << 8);
        return Err((3, tag));
    }
    let mut result = EccPageComparison {
        stored: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        software: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        data_fingerprint: [0; FSMC_ECC_CHUNKS_PER_PAGE],
    };
    for (index, chunk) in data.chunks_exact(FSMC_ECC_CHUNK_SIZE).enumerate() {
        let offset = index * ECC_BYTES_PER_CHUNK;
        result.stored[index] = u32::from_le_bytes([
            spare[offset], spare[offset + 1], spare[offset + 2], 0,
        ]);
        result.software[index] = ecc_software_512_fsmc(chunk);
        let mut fingerprint = 0x811C_9DC5u32;
        for &byte in chunk {
            fingerprint = (fingerprint ^ byte as u32).wrapping_mul(0x0100_0193);
        }
        result.data_fingerprint[index] = fingerprint;
    }
    Ok(result)
}

/// Result of a raw page read with the FSMC ECC engine enabled per 512-byte chunk.
pub(crate) struct EccPageHardwareComparison {
    pub stored: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub hardware: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub software: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub data_fingerprint: [u32; FSMC_ECC_CHUNKS_PER_PAGE],
    pub reset_status: u8,
}

/// Read-only diagnostic: NAND RESET, raw data read with FSMC ECC capture,
/// then OOB read with ECC disabled. No ECC correction and no NAND writes.
pub(crate) fn ecc_hardware_compare_page(
    page: u32,
) -> Result<EccPageHardwareComparison, (u8, u32)> {
    if page >= PHYSICAL_BLOCKS * PAGES_PER_BLOCK { return Err((0, page)); }

    let data = unsafe {
        core::slice::from_raw_parts_mut(
            core::ptr::addr_of_mut!(ECC_DIAGNOSTIC_PAGE_BUFFER).cast::<u8>(),
            PAGE_SIZE,
        )
    };

    // Start each sample from a known NAND command state.
    command(0xFF);
    if !wait_ready_rb_timeout(ECC_DIAGNOSTIC_TIMEOUT) { return Err((1, 0)); }
    command(0x70);
    let reset_status = read_data();
    if reset_status & 0x40 == 0 { return Err((2, reset_status as u32)); }

    command(0x00);
    address(0);
    address(0);
    address(page as u8);
    address_last((page >> 8) as u8);
    command(0x30);
    if !wait_ready_rb_timeout(ECC_DIAGNOSTIC_TIMEOUT) { return Err((3, page)); }

    let mut hardware = [0u32; FSMC_ECC_CHUNKS_PER_PAGE];
    for (index, chunk) in data.chunks_exact_mut(FSMC_ECC_CHUNK_SIZE).enumerate() {
        ecc_begin_chunk();
        for byte in chunk.iter_mut() { *byte = read_data(); }
        let (fifo_empty, _) = wait_fsmc_fifo_empty();
        if !fifo_empty {
            let _ = ecc_end_chunk();
            return Err((4, index as u32));
        }
        hardware[index] = ecc_end_chunk();
        service_io_hook();
    }

    let mut spare = [0u8; ECC_OOB_BYTES + ECC_TAG.len()];
    if !read_page_with_timeout(
        page,
        PAGE_SIZE as u16 + ECC_OOB_OFFSET,
        &mut spare,
        ECC_DIAGNOSTIC_TIMEOUT,
    ) {
        return Err((5, page));
    }
    if spare[ECC_OOB_BYTES..] != ECC_TAG {
        return Err((6, u32::from_le_bytes([
            spare[ECC_OOB_BYTES], spare[ECC_OOB_BYTES + 1], 0, 0,
        ])));
    }

    let mut result = EccPageHardwareComparison {
        stored: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        hardware,
        software: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        data_fingerprint: [0; FSMC_ECC_CHUNKS_PER_PAGE],
        reset_status,
    };
    for (index, chunk) in data.chunks_exact(FSMC_ECC_CHUNK_SIZE).enumerate() {
        let offset = index * ECC_BYTES_PER_CHUNK;
        result.stored[index] = u32::from_le_bytes([
            spare[offset], spare[offset + 1], spare[offset + 2], 0,
        ]);
        result.software[index] = ecc_software_512_fsmc(chunk);
        let mut fingerprint = 0x811C_9DC5u32;
        for &byte in chunk {
            fingerprint = (fingerprint ^ byte as u32).wrapping_mul(0x0100_0193);
        }
        result.data_fingerprint[index] = fingerprint;
    }
    Ok(result)
}

// Software ECC validation is disabled in normal filesystem operation. It may
// be enabled only by explicit NAND/ECC diagnostic commands.
static NAND_ECC_DIAGNOSTIC_CHECK_ENABLED: AtomicBool = AtomicBool::new(false);

pub(crate) fn set_ecc_check_enabled(enabled: bool) {
    NAND_ECC_DIAGNOSTIC_CHECK_ENABLED.store(enabled, Ordering::Release);
}

pub(crate) fn read_data_page(page: u32, out: &mut [u8]) -> bool {
    if out.len() != PAGE_SIZE { return false; }

    // Read the page with the FSMC ECC engine disabled. The hardware-ECC
    // read path has produced inconsistent syndromes under filesystem load,
    // while raw reads and independent software ECC agree on verified pages.
    if !read_page(page, 0, out) {
        record_nand_read_failure(1, page);
        return false;
    }

    if !NAND_ECC_DIAGNOSTIC_CHECK_ENABLED.load(Ordering::Acquire) {
        return true;
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
        let calculated_ecc = ecc_software_512_fsmc(chunk);
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
        let pcr2 = fsmc_pcr2();
        // NAND timing reference for this board:
        // - Samsung K9F1G08U0E, x8 NAND, 3.3 V.
        // - HCLK = 168 MHz, so one HCLK = 5.952 ns.
        // - Datasheet minima include tCLS/tALS=12 ns, tCS=20 ns,
        //   tWP/tRP=12 ns, tWC/tRC=25 ns, tWH=10 ns, tDH/tALH=5 ns,
        //   tAR/tCLR=10 ns, and tWB(max)=100 ns.
        // The "minimum" values below are register-field lower bounds derived
        // from those AC limits, not guarantees for PCB signal integrity or
        // proof that a more aggressive combination works on this board.
        //
        // Common-space FSMC_PMEM2 fields (each SET/WAIT encoding is N+1 HCLK):
        // MEMSET=2: address/control setup before NWE/NOE; 3 cycles = 17.9 ns.
        //   Datasheet tCS=20 ns implies a theoretical minimum encoding of 3
        //   (4 cycles = 23.8 ns); tCLS/tALS alone would need encoding 2.
        //   Current value 2 is below tCS if this phase is the CE-setup phase.
        // MEMWAIT=2: minimum NWE/NOE assertion phase; 3 cycles = 17.9 ns.
        //   tWP/tRP=12 ns requires at least 3 HCLK cycles, so encoding 2 is
        //   the theoretical minimum at 168 MHz (encoding 1 is only 11.9 ns).
        // MEMHOLD=1: post-command address hold on reads / data hold on writes.
        //   Encoding 0 is reserved; 1 is the minimum legal value. On reads
        //   this is (HOLD+2) cycles = 17.9 ns; on writes it is HOLD cycles.
        // MEMHIZ=1: write-cycle data-bus Hi-Z interval; encoding 0 is the
        //   minimum legal value (1 HCLK). NAND AC tables give no separate
        //   numeric minimum for this FSMC bus-turnaround field.
        let common_timing = 2 | (2 << 8) | (1 << 16) | (1 << 24);
        //
        // Attribute-space FSMC_PATT2 is used for the final NAND address-byte
        // write (see address_last()). Fields have the same SET/WAIT/HOLD
        // meanings, but use the attribute timing register:
        // ATTSET=4: address setup; 5 cycles = 29.8 ns. tCS=20 ns gives a
        //   theoretical minimum encoding of 3 (4 cycles = 23.8 ns).
        // ATTWAIT=8: NWE/NOE assertion; 9 cycles = 53.6 ns. Based on
        //   tWP=12 ns alone, encoding 2 (3 cycles = 17.9 ns) is the minimum.
        // ATTHOLD=18: post-command hold; 18 cycles = 107.1 ns. This is
        //   intentionally longer than tWB(max)=100 ns. If this delay is
        //   relied on to cover tWB, the theoretical minimum is 17 cycles
        //   (encoding 17 = 101.2 ns); encoding 16 would be only 95.2 ns.
        // ATTHIZ=2: write-cycle data-bus Hi-Z interval; encoding 0 is the
        //   controller minimum. No separate NAND AC limit sets this value.
        let attribute_timing = 4 | (8 << 8) | (18 << 16) | (2 << 24);
        write_volatile(pcr2, 0);
        write_volatile(fsmc_pmem2(), common_timing);
        write_volatile(fsmc_patt2(), attribute_timing);
        //
        // FSMC_PCR2 configuration:
        // PWAITEN=1: enable the NAND-bank wait input; 0 is the disabled
        //   alternative, not a timing optimization.
        // PBKEN=1: enable NAND bank 2; it must remain 1 for accesses.
        // PTYP=1: select NAND Flash (0 selects PC Card/CompactFlash).
        // PWID=00: 8-bit data bus, matching this x8 NAND; 16-bit is invalid
        //   for the fitted device.
        // ECCPS=001: 512-byte ECC chunk, matching the four chunks per 2-KiB
        //   page. This is a format/algorithm choice, not a speed parameter.
        // ECCEN=0 while idle; ecc_begin_chunk() enables it only for each
        // 512-byte chunk. Reserved bits remain zero.
        //
        // TCLR=0: CLE-low to RE-low delay:
        //   (TCLR + SET + 2) HCLK. In common space (MEMSET=2), this is
        //   4 cycles = 23.8 ns, above tCLR(min)=10 ns. Zero is the minimum
        //   field encoding and is sufficient by the NAND AC specification.
        // TAR=1: ALE-low to RE-low delay, same formula. With MEMSET=2 this
        //   is 5 cycles = 29.8 ns, above tAR(min)=10 ns. TAR=0 would give
        //   4 cycles = 23.8 ns and is the theoretical minimum.
        // These TCLR/TAR delays are not the NAND tWHR parameter.
        let control = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 13) | (1 << 17);
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
    mut progress: impl FnMut(u32, u32),
) -> Result<R, FsAccessError> {
    // Progress sentinels distinguish the NAND scan, LittleFS mount, and
    // caller operation. Physical scan updates are 1..=PHYSICAL_BLOCKS.
    progress(0, 0);
    let mut storage =
        ActiveStorage::new_with_progress(&mut progress).map_err(FsAccessError::Init)?;
    progress(u32::MAX, 0);
    // Mount directly rather than using is_mountable(): the latter only
    // returns a bool and discards LittleFS's actual error code.
    // Nest the service result so a caller-operation error remains distinct.
    let mounted = mount_with_shared_allocation(&mut storage, |fs| {
        progress(u32::MAX, 1);
        Ok(service(fs))
    }).map_err(|error| FsAccessError::Mount(error.code()))?;
    mounted.map_err(|_| FsAccessError::Operation)
}


/// Visit each entry in the LittleFS root directory using one shared iterator.
/// The path slice includes its trailing NUL; size is meaningful for files.
pub fn read_root_directory(
    fs: &Filesystem<'_, ActiveStorage>,
    include_file_sizes: bool,
    mut visit: impl FnMut(&[u8], bool, usize) -> LfsResult<()>,
) -> LfsResult<()> {
    fs.read_dir_and_then(&littlefs2::path!("/"), |directory| {
        loop {
            let item = if include_file_sizes {
                directory.next()
            } else {
                directory.next_without_size()
            };
            let Some(item) = item else {
                break;
            };
            let entry = item?;
            let is_dir = entry.file_type().is_dir();
            let size = if include_file_sizes && !is_dir {
                entry.metadata().len()
            } else {
                0
            };
            let path = entry.path();
            visit(
                path.as_str_ref_with_trailing_nul().as_bytes(),
                is_dir,
                size,
            )?;
        }
        Ok(())
    })
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
    let mut report = [0u8; 256];
    let mut len = 0;
    append(&mut report, &mut len, b"XCore407I NAND / LittleFS\r\n");
    append(&mut report, &mut len, b"Factory bad blocks: ");
    append_hex(&mut report, &mut len, storage.factory_bad_blocks() as u32, 4);
    append(&mut report, &mut len, b"\r\n");
    append(&mut report, &mut len, b"LittleFS: existing volume mounted\r\n");
    append(&mut report, &mut len, b"Capacity: ");

    mount_with_shared_allocation(&mut storage, |fs| {
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

    match mount_with_shared_allocation(&mut storage, |fs| {
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
    mount_with_shared_allocation(&mut storage, |_| Ok(()))
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
    mount_with_shared_allocation(&mut storage, |_| Ok(()))
        .map_err(|_| InitError::MountFailed)
}
