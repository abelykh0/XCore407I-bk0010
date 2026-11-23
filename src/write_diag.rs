//! Low-overhead timing counters for one ZMODEM receive transaction.
//! Enabled with `list-profile`, which also enables DWT CYCCNT.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

const CORE_CLOCK_HZ: u32 = crate::SYSTEM_CORE_CLOCK_HZ;
static ACTIVE: AtomicBool = AtomicBool::new(false);
static TRANSFER_US: AtomicU32 = AtomicU32::new(0);
static ZMODEM_POLL_CALLS: AtomicU32 = AtomicU32::new(0);
static ZMODEM_POLL_US: AtomicU32 = AtomicU32::new(0);
static ZMODEM_SUBMIT_CALLS: AtomicU32 = AtomicU32::new(0);
static ZMODEM_SUBMIT_US: AtomicU32 = AtomicU32::new(0);
static LFS_CREATE_US: AtomicU32 = AtomicU32::new(0);
static LFS_APPEND_US: AtomicU32 = AtomicU32::new(0);
static LFS_APPEND_CALLS: AtomicU32 = AtomicU32::new(0);
static LFS_OPEN_US: AtomicU32 = AtomicU32::new(0);
static LFS_CALLBACK_US: AtomicU32 = AtomicU32::new(0);
static LFS_CLOSE_US: AtomicU32 = AtomicU32::new(0);
static LFS_COMMIT_US: AtomicU32 = AtomicU32::new(0);
static LFS_WRITE_CALLS: AtomicU32 = AtomicU32::new(0);
static LFS_WRITE_BYTES: AtomicU32 = AtomicU32::new(0);
static LFS_WRITE_US: AtomicU32 = AtomicU32::new(0);
static DHARA_WRITE_CALLS: AtomicU32 = AtomicU32::new(0);
static DHARA_WRITE_PAGES: AtomicU32 = AtomicU32::new(0);
static DHARA_WRITE_US: AtomicU32 = AtomicU32::new(0);
static NAND_PROGRAM_CALLS: AtomicU32 = AtomicU32::new(0);
static NAND_PROGRAM_US: AtomicU32 = AtomicU32::new(0);
static NAND_FIFO_WAIT_US: AtomicU32 = AtomicU32::new(0);
static NAND_BUSY_WAIT_US: AtomicU32 = AtomicU32::new(0);
static NAND_ERASE_CALLS: AtomicU32 = AtomicU32::new(0);
static NAND_ERASE_US: AtomicU32 = AtomicU32::new(0);

#[inline]
pub(crate) fn cycle_count() -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
}

#[inline]
fn cycles_to_us(cycles: u32) -> u32 {
    ((cycles as u64 * 1_000_000) / CORE_CLOCK_HZ as u64) as u32
}

#[inline]
fn add(counter: &AtomicU32, value: u32) {
    if ACTIVE.load(Ordering::Relaxed) {
        counter.fetch_add(value, Ordering::Relaxed);
    }
}

pub(crate) fn begin() {
    ACTIVE.store(false, Ordering::Release);
    for counter in [
        &TRANSFER_US,
        &ZMODEM_POLL_CALLS, &ZMODEM_POLL_US,
        &ZMODEM_SUBMIT_CALLS, &ZMODEM_SUBMIT_US,
        &LFS_CREATE_US, &LFS_APPEND_US, &LFS_APPEND_CALLS,
        &LFS_OPEN_US, &LFS_CALLBACK_US, &LFS_CLOSE_US, &LFS_COMMIT_US,
        &LFS_WRITE_CALLS, &LFS_WRITE_BYTES, &LFS_WRITE_US,
        &DHARA_WRITE_CALLS, &DHARA_WRITE_PAGES, &DHARA_WRITE_US,
        &NAND_PROGRAM_CALLS, &NAND_PROGRAM_US, &NAND_FIFO_WAIT_US,
        &NAND_BUSY_WAIT_US, &NAND_ERASE_CALLS, &NAND_ERASE_US,
    ] {
        counter.store(0, Ordering::Relaxed);
    }
    ACTIVE.store(true, Ordering::Release);
}

/// Store the wall-clock duration from the RTC, avoiding DWT's 25.6 s wrap.
pub(crate) fn finish(elapsed_ms: u32) {
    TRANSFER_US.store(elapsed_ms.saturating_mul(1_000), Ordering::Relaxed);
    ACTIVE.store(false, Ordering::Release);
}

pub(crate) fn record_zmodem_poll(cycles: u32) {
    add(&ZMODEM_POLL_CALLS, 1);
    add(&ZMODEM_POLL_US, cycles_to_us(cycles));
}

pub(crate) fn record_zmodem_submit(cycles: u32) {
    add(&ZMODEM_SUBMIT_CALLS, 1);
    add(&ZMODEM_SUBMIT_US, cycles_to_us(cycles));
}

pub(crate) fn record_lfs_create(cycles: u32) { add(&LFS_CREATE_US, cycles_to_us(cycles)); }
pub(crate) fn record_lfs_append_open(cycles: u32) {
    add(&LFS_OPEN_US, cycles_to_us(cycles));
}
pub(crate) fn record_lfs_append_callback(cycles: u32) {
    add(&LFS_APPEND_CALLS, 1);
    add(&LFS_APPEND_US, cycles_to_us(cycles));
    add(&LFS_CALLBACK_US, cycles_to_us(cycles));
}
pub(crate) fn record_lfs_append_close(cycles: u32) {
    add(&LFS_CLOSE_US, cycles_to_us(cycles));
}
pub(crate) fn record_lfs_commit(cycles: u32) { add(&LFS_COMMIT_US, cycles_to_us(cycles)); }

pub(crate) fn record_lfs_write(cycles: u32, bytes: usize) {
    add(&LFS_WRITE_CALLS, 1);
    add(&LFS_WRITE_BYTES, bytes as u32);
    add(&LFS_WRITE_US, cycles_to_us(cycles));
}


pub(crate) fn record_dhara_write(cycles: u32, pages: usize) {
    add(&DHARA_WRITE_CALLS, 1);
    add(&DHARA_WRITE_PAGES, pages as u32);
    add(&DHARA_WRITE_US, cycles_to_us(cycles));
}

pub(crate) fn record_nand_program(cycles: u32, fifo_wait: u32, busy_wait: u32) {
    add(&NAND_PROGRAM_CALLS, 1);
    add(&NAND_PROGRAM_US, cycles_to_us(cycles));
    add(&NAND_FIFO_WAIT_US, cycles_to_us(fifo_wait));
    add(&NAND_BUSY_WAIT_US, cycles_to_us(busy_wait));
}

pub(crate) fn record_nand_erase(cycles: u32) {
    add(&NAND_ERASE_CALLS, 1);
    add(&NAND_ERASE_US, cycles_to_us(cycles));
}

#[derive(Clone, Copy)]
pub(crate) struct Snapshot {
    pub transfer_us: u32,
    pub zmodem_poll_calls: u32,
    pub zmodem_poll_us: u32,
    pub zmodem_submit_calls: u32,
    pub zmodem_submit_us: u32,
    pub lfs_create_us: u32,
    pub lfs_append_us: u32,
    pub lfs_append_calls: u32,
    pub lfs_open_us: u32,
    pub lfs_callback_us: u32,
    pub lfs_close_us: u32,
    pub lfs_commit_us: u32,
    pub lfs_write_calls: u32,
    pub lfs_write_bytes: u32,
    pub lfs_write_us: u32,
    pub dhara_write_calls: u32,
    pub dhara_write_pages: u32,
    pub dhara_write_us: u32,
    pub nand_program_calls: u32,
    pub nand_program_us: u32,
    pub nand_fifo_wait_us: u32,
    pub nand_busy_wait_us: u32,
    pub nand_erase_calls: u32,
    pub nand_erase_us: u32,
}

pub(crate) fn snapshot() -> Snapshot {
    Snapshot {
        transfer_us: TRANSFER_US.load(Ordering::Relaxed),
        zmodem_poll_calls: ZMODEM_POLL_CALLS.load(Ordering::Relaxed),
        zmodem_poll_us: ZMODEM_POLL_US.load(Ordering::Relaxed),
        zmodem_submit_calls: ZMODEM_SUBMIT_CALLS.load(Ordering::Relaxed),
        zmodem_submit_us: ZMODEM_SUBMIT_US.load(Ordering::Relaxed),
        lfs_create_us: LFS_CREATE_US.load(Ordering::Relaxed),
        lfs_append_us: LFS_APPEND_US.load(Ordering::Relaxed),
        lfs_append_calls: LFS_APPEND_CALLS.load(Ordering::Relaxed),
        lfs_open_us: LFS_OPEN_US.load(Ordering::Relaxed),
        lfs_callback_us: LFS_CALLBACK_US.load(Ordering::Relaxed),
        lfs_close_us: LFS_CLOSE_US.load(Ordering::Relaxed),
        lfs_commit_us: LFS_COMMIT_US.load(Ordering::Relaxed),
        lfs_write_calls: LFS_WRITE_CALLS.load(Ordering::Relaxed),
        lfs_write_bytes: LFS_WRITE_BYTES.load(Ordering::Relaxed),
        lfs_write_us: LFS_WRITE_US.load(Ordering::Relaxed),
        dhara_write_calls: DHARA_WRITE_CALLS.load(Ordering::Relaxed),
        dhara_write_pages: DHARA_WRITE_PAGES.load(Ordering::Relaxed),
        dhara_write_us: DHARA_WRITE_US.load(Ordering::Relaxed),
        nand_program_calls: NAND_PROGRAM_CALLS.load(Ordering::Relaxed),
        nand_program_us: NAND_PROGRAM_US.load(Ordering::Relaxed),
        nand_fifo_wait_us: NAND_FIFO_WAIT_US.load(Ordering::Relaxed),
        nand_busy_wait_us: NAND_BUSY_WAIT_US.load(Ordering::Relaxed),
        nand_erase_calls: NAND_ERASE_CALLS.load(Ordering::Relaxed),
        nand_erase_us: NAND_ERASE_US.load(Ordering::Relaxed),
    }
}
