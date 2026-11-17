//! Cycle and I/O counters used by the LIST PROFILE diagnostic.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

static ACTIVE: AtomicBool = AtomicBool::new(false);
static DIRECTORY_ACTIVE: AtomicBool = AtomicBool::new(false);
static DHARA_READS: AtomicU32 = AtomicU32::new(0);
static DHARA_CYCLES: AtomicU32 = AtomicU32::new(0);
static NAND_READS: AtomicU32 = AtomicU32::new(0);
static NAND_BYTES: AtomicU32 = AtomicU32::new(0);
static NAND_CYCLES: AtomicU32 = AtomicU32::new(0);
static CDC_CALLS: AtomicU32 = AtomicU32::new(0);
static CDC_BYTES: AtomicU32 = AtomicU32::new(0);
static CDC_CYCLES: AtomicU32 = AtomicU32::new(0);
static DIRECTORY_CDC_CYCLES: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy)]
pub(crate) struct Snapshot {
    pub dhara_reads: u32, pub dhara_cycles: u32,
    pub nand_reads: u32, pub nand_bytes: u32, pub nand_cycles: u32,
    pub cdc_calls: u32, pub cdc_bytes: u32, pub cdc_cycles: u32,
    pub directory_cdc_cycles: u32,
}

#[inline]
pub(crate) fn cycle_count() -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
}
pub(crate) fn is_active() -> bool { ACTIVE.load(Ordering::Relaxed) }
pub(crate) fn set_directory_active(active: bool) {
    DIRECTORY_ACTIVE.store(active, Ordering::Relaxed);
}
pub(crate) fn begin() {
    ACTIVE.store(false, Ordering::Release);
    DIRECTORY_ACTIVE.store(false, Ordering::Relaxed);
    DHARA_READS.store(0, Ordering::Relaxed);
    DHARA_CYCLES.store(0, Ordering::Relaxed);
    NAND_READS.store(0, Ordering::Relaxed);
    NAND_BYTES.store(0, Ordering::Relaxed);
    NAND_CYCLES.store(0, Ordering::Relaxed);
    CDC_CALLS.store(0, Ordering::Relaxed);
    CDC_BYTES.store(0, Ordering::Relaxed);
    CDC_CYCLES.store(0, Ordering::Relaxed);
    DIRECTORY_CDC_CYCLES.store(0, Ordering::Relaxed);
    ACTIVE.store(true, Ordering::Release);
}
pub(crate) fn end() -> Snapshot {
    ACTIVE.store(false, Ordering::Release);
    DIRECTORY_ACTIVE.store(false, Ordering::Relaxed);
    Snapshot {
        dhara_reads: DHARA_READS.load(Ordering::Relaxed),
        dhara_cycles: DHARA_CYCLES.load(Ordering::Relaxed),
        nand_reads: NAND_READS.load(Ordering::Relaxed),
        nand_bytes: NAND_BYTES.load(Ordering::Relaxed),
        nand_cycles: NAND_CYCLES.load(Ordering::Relaxed),
        cdc_calls: CDC_CALLS.load(Ordering::Relaxed),
        cdc_bytes: CDC_BYTES.load(Ordering::Relaxed),
        cdc_cycles: CDC_CYCLES.load(Ordering::Relaxed),
        directory_cdc_cycles: DIRECTORY_CDC_CYCLES.load(Ordering::Relaxed),
    }
}
pub(crate) fn record_dhara_read(cycles: u32) {
    if is_active() {
        DHARA_READS.fetch_add(1, Ordering::Relaxed);
        DHARA_CYCLES.fetch_add(cycles, Ordering::Relaxed);
    }
}
pub(crate) fn record_nand_read(cycles: u32, bytes: usize) {
    if is_active() {
        NAND_READS.fetch_add(1, Ordering::Relaxed);
        NAND_BYTES.fetch_add(bytes as u32, Ordering::Relaxed);
        NAND_CYCLES.fetch_add(cycles, Ordering::Relaxed);
    }
}
pub(crate) fn record_cdc_write(cycles: u32, bytes: usize) {
    if is_active() {
        CDC_CALLS.fetch_add(1, Ordering::Relaxed);
        CDC_BYTES.fetch_add(bytes as u32, Ordering::Relaxed);
        CDC_CYCLES.fetch_add(cycles, Ordering::Relaxed);
        if DIRECTORY_ACTIVE.load(Ordering::Relaxed) {
            DIRECTORY_CDC_CYCLES.fetch_add(cycles, Ordering::Relaxed);
        }
    }
}
