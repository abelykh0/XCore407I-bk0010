//! Low-overhead timing counters for one ZMODEM receive transaction.
//! Enabled with `list-profile`, which also enables DWT CYCCNT.
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
const CORE_CLOCK_HZ: u32 = 168_000_000;
static ACTIVE: AtomicBool = AtomicBool::new(false);
static TRANSFER_US: AtomicU32 = AtomicU32::new(0);
static ZP_CALLS: AtomicU32 = AtomicU32::new(0); static ZP_US: AtomicU32 = AtomicU32::new(0);
static ZS_CALLS: AtomicU32 = AtomicU32::new(0); static ZS_US: AtomicU32 = AtomicU32::new(0);
static LFS_APPEND_US: AtomicU32 = AtomicU32::new(0);
static LW_CALLS: AtomicU32 = AtomicU32::new(0); static LW_BYTES: AtomicU32 = AtomicU32::new(0); static LW_US: AtomicU32 = AtomicU32::new(0); static LF_US: AtomicU32 = AtomicU32::new(0);
static DW_CALLS: AtomicU32 = AtomicU32::new(0); static DW_PAGES: AtomicU32 = AtomicU32::new(0); static DW_US: AtomicU32 = AtomicU32::new(0);
static NP_CALLS: AtomicU32 = AtomicU32::new(0); static NP_US: AtomicU32 = AtomicU32::new(0); static NF_US: AtomicU32 = AtomicU32::new(0); static NB_US: AtomicU32 = AtomicU32::new(0);
static NE_CALLS: AtomicU32 = AtomicU32::new(0); static NE_US: AtomicU32 = AtomicU32::new(0);
#[inline] pub(crate) fn cycle_count() -> u32 { cortex_m::peripheral::DWT::cycle_count() }
#[inline] fn us(cycles: u32) -> u32 { ((cycles as u64 * 1_000_000) / CORE_CLOCK_HZ as u64) as u32 }
#[inline] fn add(counter: &AtomicU32, value: u32) { if ACTIVE.load(Ordering::Relaxed) { counter.fetch_add(value, Ordering::Relaxed); } }
pub(crate) fn begin() {
 ACTIVE.store(false, Ordering::Release); TRANSFER_US.store(0, Ordering::Relaxed);
 for c in [&ZP_CALLS,&ZP_US,&ZS_CALLS,&ZS_US,&LFS_APPEND_US,&LW_CALLS,&LW_BYTES,&LW_US,&LF_US,&DW_CALLS,&DW_PAGES,&DW_US,&NP_CALLS,&NP_US,&NF_US,&NB_US,&NE_CALLS,&NE_US] { c.store(0, Ordering::Relaxed); }
 ACTIVE.store(true, Ordering::Release);
}
pub(crate) fn finish(elapsed_ms: u32) { TRANSFER_US.store(elapsed_ms.saturating_mul(1_000), Ordering::Relaxed); ACTIVE.store(false, Ordering::Release); }
pub(crate) fn record_zmodem_poll(c: u32) { add(&ZP_CALLS,1); add(&ZP_US,us(c)); }
pub(crate) fn record_zmodem_submit(c: u32) { add(&ZS_CALLS,1); add(&ZS_US,us(c)); }
pub(crate) fn record_lfs_append(c: u32) { add(&LFS_APPEND_US,us(c)); }
pub(crate) fn record_lfs_write(c: u32,b: usize) { add(&LW_CALLS,1); add(&LW_BYTES,b as u32); add(&LW_US,us(c)); }
pub(crate) fn record_lfs_flush(c: u32) { add(&LF_US,us(c)); }
pub(crate) fn record_dhara_write(c: u32,p: usize) { add(&DW_CALLS,1); add(&DW_PAGES,p as u32); add(&DW_US,us(c)); }
pub(crate) fn record_nand_program(c: u32,f: u32,b: u32) { add(&NP_CALLS,1); add(&NP_US,us(c)); add(&NF_US,us(f)); add(&NB_US,us(b)); }
pub(crate) fn record_nand_erase(c: u32) { add(&NE_CALLS,1); add(&NE_US,us(c)); }
#[derive(Clone,Copy)] pub(crate) struct Snapshot {
 pub transfer_us:u32, pub zmodem_poll_calls:u32,pub zmodem_poll_us:u32,pub zmodem_submit_calls:u32,pub zmodem_submit_us:u32,
 pub lfs_append_us:u32,pub lfs_write_calls:u32,pub lfs_write_bytes:u32,pub lfs_write_us:u32,pub lfs_flush_us:u32,
 pub dhara_write_calls:u32,pub dhara_write_pages:u32,pub dhara_write_us:u32,pub nand_program_calls:u32,pub nand_program_us:u32,
 pub nand_fifo_wait_us:u32,pub nand_busy_wait_us:u32,pub nand_erase_calls:u32,pub nand_erase_us:u32,
}
pub(crate) fn snapshot()->Snapshot { Snapshot {
 transfer_us:TRANSFER_US.load(Ordering::Relaxed),zmodem_poll_calls:ZP_CALLS.load(Ordering::Relaxed),zmodem_poll_us:ZP_US.load(Ordering::Relaxed),zmodem_submit_calls:ZS_CALLS.load(Ordering::Relaxed),zmodem_submit_us:ZS_US.load(Ordering::Relaxed),
 lfs_append_us:LFS_APPEND_US.load(Ordering::Relaxed),lfs_write_calls:LW_CALLS.load(Ordering::Relaxed),lfs_write_bytes:LW_BYTES.load(Ordering::Relaxed),lfs_write_us:LW_US.load(Ordering::Relaxed),lfs_flush_us:LF_US.load(Ordering::Relaxed),
 dhara_write_calls:DW_CALLS.load(Ordering::Relaxed),dhara_write_pages:DW_PAGES.load(Ordering::Relaxed),dhara_write_us:DW_US.load(Ordering::Relaxed),nand_program_calls:NP_CALLS.load(Ordering::Relaxed),nand_program_us:NP_US.load(Ordering::Relaxed),nand_fifo_wait_us:NF_US.load(Ordering::Relaxed),nand_busy_wait_us:NB_US.load(Ordering::Relaxed),nand_erase_calls:NE_CALLS.load(Ordering::Relaxed),nand_erase_us:NE_US.load(Ordering::Relaxed),
} }
