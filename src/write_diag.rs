//! Low-overhead timing counters for a single ZMODEM receive transaction.
//! Enabled with the existing `list-profile` feature, which enables DWT CYCCNT.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

const CORE_CLOCK_HZ: u32 = 168_000_000;
static ACTIVE: AtomicBool = AtomicBool::new(false);
static STARTED: AtomicU32 = AtomicU32::new(0);
static TRANSFER_CYCLES: AtomicU32 = AtomicU32::new(0);
static ZMODEM_POLL_CALLS: AtomicU32 = AtomicU32::new(0);
static ZMODEM_POLL_CYCLES: AtomicU32 = AtomicU32::new(0);
static ZMODEM_SUBMIT_CALLS: AtomicU32 = AtomicU32::new(0);
static ZMODEM_SUBMIT_CYCLES: AtomicU32 = AtomicU32::new(0);
static LFS_OPEN_CYCLES: AtomicU32 = AtomicU32::new(0);
static LFS_WRITE_CALLS: AtomicU32 = AtomicU32::new(0);
static LFS_WRITE_BYTES: AtomicU32 = AtomicU32::new(0);
static LFS_WRITE_CYCLES: AtomicU32 = AtomicU32::new(0);
static LFS_FLUSH_CYCLES: AtomicU32 = AtomicU32::new(0);
static DHARA_WRITE_CALLS: AtomicU32 = AtomicU32::new(0);
static DHARA_WRITE_PAGES: AtomicU32 = AtomicU32::new(0);
static DHARA_WRITE_CYCLES: AtomicU32 = AtomicU32::new(0);
static NAND_PROGRAM_CALLS: AtomicU32 = AtomicU32::new(0);
static NAND_PROGRAM_CYCLES: AtomicU32 = AtomicU32::new(0);
static NAND_FIFO_WAIT_CYCLES: AtomicU32 = AtomicU32::new(0);
static NAND_BUSY_WAIT_CYCLES: AtomicU32 = AtomicU32::new(0);
static NAND_ERASE_CALLS: AtomicU32 = AtomicU32::new(0);
static NAND_ERASE_CYCLES: AtomicU32 = AtomicU32::new(0);

#[inline]
pub(crate) fn cycle_count() -> u32 { cortex_m::peripheral::DWT::cycle_count() }

fn add(counter: &AtomicU32, value: u32) {
    if ACTIVE.load(Ordering::Relaxed) { counter.fetch_add(value, Ordering::Relaxed); }
}

pub(crate) fn begin() {
    ACTIVE.store(false, Ordering::Release);
    STARTED.store(cycle_count(), Ordering::Relaxed);
    TRANSFER_CYCLES.store(0, Ordering::Relaxed);
    ZMODEM_POLL_CALLS.store(0, Ordering::Relaxed);
    ZMODEM_POLL_CYCLES.store(0, Ordering::Relaxed);
    ZMODEM_SUBMIT_CALLS.store(0, Ordering::Relaxed);
    ZMODEM_SUBMIT_CYCLES.store(0, Ordering::Relaxed);
    LFS_OPEN_CYCLES.store(0, Ordering::Relaxed);
    LFS_WRITE_CALLS.store(0, Ordering::Relaxed);
    LFS_WRITE_BYTES.store(0, Ordering::Relaxed);
    LFS_WRITE_CYCLES.store(0, Ordering::Relaxed);
    LFS_FLUSH_CYCLES.store(0, Ordering::Relaxed);
    DHARA_WRITE_CALLS.store(0, Ordering::Relaxed);
    DHARA_WRITE_PAGES.store(0, Ordering::Relaxed);
    DHARA_WRITE_CYCLES.store(0, Ordering::Relaxed);
    NAND_PROGRAM_CALLS.store(0, Ordering::Relaxed);
    NAND_PROGRAM_CYCLES.store(0, Ordering::Relaxed);
    NAND_FIFO_WAIT_CYCLES.store(0, Ordering::Relaxed);
    NAND_BUSY_WAIT_CYCLES.store(0, Ordering::Relaxed);
    NAND_ERASE_CALLS.store(0, Ordering::Relaxed);
    NAND_ERASE_CYCLES.store(0, Ordering::Relaxed);
    ACTIVE.store(true, Ordering::Release);
}

pub(crate) fn finish() {
    let elapsed = cycle_count().wrapping_sub(STARTED.load(Ordering::Relaxed));
    TRANSFER_CYCLES.store(elapsed, Ordering::Relaxed);
    ACTIVE.store(false, Ordering::Release);
}

pub(crate) fn active() -> bool { ACTIVE.load(Ordering::Relaxed) }
pub(crate) fn record_zmodem_poll(cycles: u32) { add(&ZMODEM_POLL_CALLS, 1); add(&ZMODEM_POLL_CYCLES, cycles); }
pub(crate) fn record_zmodem_submit(cycles: u32) { add(&ZMODEM_SUBMIT_CALLS, 1); add(&ZMODEM_SUBMIT_CYCLES, cycles); }
pub(crate) fn record_lfs_open(cycles: u32) { add(&LFS_OPEN_CYCLES, cycles); }
pub(crate) fn record_lfs_write(cycles: u32, bytes: usize) { add(&LFS_WRITE_CALLS, 1); add(&LFS_WRITE_BYTES, bytes as u32); add(&LFS_WRITE_CYCLES, cycles); }
pub(crate) fn record_lfs_flush(cycles: u32) { add(&LFS_FLUSH_CYCLES, cycles); }
pub(crate) fn record_dhara_write(cycles: u32, pages: usize) { add(&DHARA_WRITE_CALLS, 1); add(&DHARA_WRITE_PAGES, pages as u32); add(&DHARA_WRITE_CYCLES, cycles); }
pub(crate) fn record_nand_program(cycles: u32, fifo_wait: u32, busy_wait: u32) { add(&NAND_PROGRAM_CALLS, 1); add(&NAND_PROGRAM_CYCLES, cycles); add(&NAND_FIFO_WAIT_CYCLES, fifo_wait); add(&NAND_BUSY_WAIT_CYCLES, busy_wait); }
pub(crate) fn record_nand_erase(cycles: u32) { add(&NAND_ERASE_CALLS, 1); add(&NAND_ERASE_CYCLES, cycles); }

#[derive(Clone, Copy)]
pub(crate) struct Snapshot {
    pub transfer_cycles: u32,
    pub zmodem_poll_calls: u32, pub zmodem_poll_cycles: u32,
    pub zmodem_submit_calls: u32, pub zmodem_submit_cycles: u32,
    pub lfs_open_cycles: u32, pub lfs_write_calls: u32, pub lfs_write_bytes: u32,
    pub lfs_write_cycles: u32, pub lfs_flush_cycles: u32,
    pub dhara_write_calls: u32, pub dhara_write_pages: u32, pub dhara_write_cycles: u32,
    pub nand_program_calls: u32, pub nand_program_cycles: u32,
    pub nand_fifo_wait_cycles: u32, pub nand_busy_wait_cycles: u32,
    pub nand_erase_calls: u32, pub nand_erase_cycles: u32,
}

pub(crate) fn snapshot() -> Snapshot {
    Snapshot {
        transfer_cycles: TRANSFER_CYCLES.load(Ordering::Relaxed),
        zmodem_poll_calls: ZMODEM_POLL_CALLS.load(Ordering::Relaxed), zmodem_poll_cycles: ZMODEM_POLL_CYCLES.load(Ordering::Relaxed),
        zmodem_submit_calls: ZMODEM_SUBMIT_CALLS.load(Ordering::Relaxed), zmodem_submit_cycles: ZMODEM_SUBMIT_CYCLES.load(Ordering::Relaxed),
        lfs_open_cycles: LFS_OPEN_CYCLES.load(Ordering::Relaxed), lfs_write_calls: LFS_WRITE_CALLS.load(Ordering::Relaxed), lfs_write_bytes: LFS_WRITE_BYTES.load(Ordering::Relaxed),
        lfs_write_cycles: LFS_WRITE_CYCLES.load(Ordering::Relaxed), lfs_flush_cycles: LFS_FLUSH_CYCLES.load(Ordering::Relaxed),
        dhara_write_calls: DHARA_WRITE_CALLS.load(Ordering::Relaxed), dhara_write_pages: DHARA_WRITE_PAGES.load(Ordering::Relaxed), dhara_write_cycles: DHARA_WRITE_CYCLES.load(Ordering::Relaxed),
        nand_program_calls: NAND_PROGRAM_CALLS.load(Ordering::Relaxed), nand_program_cycles: NAND_PROGRAM_CYCLES.load(Ordering::Relaxed),
        nand_fifo_wait_cycles: NAND_FIFO_WAIT_CYCLES.load(Ordering::Relaxed), nand_busy_wait_cycles: NAND_BUSY_WAIT_CYCLES.load(Ordering::Relaxed),
        nand_erase_calls: NAND_ERASE_CALLS.load(Ordering::Relaxed), nand_erase_cycles: NAND_ERASE_CYCLES.load(Ordering::Relaxed),
    }
}

pub(crate) fn cycles_to_us(cycles: u32) -> u32 {
    ((cycles as u64 * 1_000_000) / CORE_CLOCK_HZ as u64) as u32
}
