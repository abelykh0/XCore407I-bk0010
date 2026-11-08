//! BK-0010 speaker-edge capture and deferred PCM16 rendering for TinyUSB UAC2.
//!
//! The emulator publishes DWT-timestamped speaker transitions. The main-loop
//! worker samples that event stream at a fixed 48 kHz, independent of emulator
//! throughput. The USB completion ISR only copies six rendered samples.

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};

const SAMPLES_PER_MICROFRAME: usize = 6;
const PACKET_BYTES: usize = SAMPLES_PER_MICROFRAME * 2;
// 168 MHz core clock / 48 kHz USB audio = 3,500 DWT ticks per sample.
const AUDIO_SAMPLE_PERIOD_DWT_CYCLES: u32 = 3_500;
const AUDIO_STARTUP_LATENCY_DWT_CYCLES: u32 = 168_000; // 1 ms.
const AUDIO_PREFILL_DWT_CYCLES: u32 = 1_008_000; // 6 ms.
const AUDIO_AMPLITUDE: i16 = 12_000;
const PCM_RING_CAPACITY: u16 = 2048;
const PCM_RING_MASK: u16 = PCM_RING_CAPACITY - 1;
const PCM_MIN_TRACKING_START: u16 = 48; // Ignore startup occupancy until 1 ms is buffered.
const _: () = assert!(PCM_RING_CAPACITY.is_power_of_two());
const _: () = assert!(PACKET_BYTES <= crate::uvc::USB_HS_DMA_BUFFER_LIMIT);

static PCM_HEAD: AtomicU16 = AtomicU16::new(0);
static PCM_TAIL: AtomicU16 = AtomicU16::new(0);
static PCM_UNDERRUN_SAMPLES: AtomicU32 = AtomicU32::new(0);
static PCM_UNDERRUN_WINDOW: AtomicU32 = AtomicU32::new(0);
static PCM_RENDERED_WINDOW: AtomicU32 = AtomicU32::new(0);
static PCM_RENDER_CALLS_WINDOW: AtomicU32 = AtomicU32::new(0);
static PCM_MIN_OCCUPANCY: AtomicU16 = AtomicU16::new(PCM_RING_CAPACITY);
static PCM_MIN_TRACKING: AtomicBool = AtomicBool::new(false);
static PCM_MAX_OCCUPANCY: AtomicU16 = AtomicU16::new(0);
static mut PCM_RING: [i16; PCM_RING_CAPACITY as usize] = [0; PCM_RING_CAPACITY as usize];

unsafe extern "C" {
    fn bk_audio_start() -> u8;
    fn bk_audio_stop();
    fn bk_audio_pop_edge(cycles: *mut u32, level: *mut u8) -> u8;
    fn bk_audio_get_edge_overflows() -> u32;
}

pub struct BkEmulatorAudio {
    // DWT timestamps place speaker edges and PCM samples on wall time.
    next_sample_dwt: u32,
    sample_level: bool,
    pending_edge: Option<(u32, bool)>,
    active: bool,
}

impl BkEmulatorAudio {
    pub const fn new() -> Self {
        Self {
            next_sample_dwt: 0,
            sample_level: false,
            pending_edge: None,
            active: false,
        }
    }

    #[inline(always)]
    fn dwt_cycles() -> u32 {
        // ARM DWT_CYCCNT, enabled during early platform startup.
        unsafe { core::ptr::read_volatile(0xE000_1004 as *const u32) }
    }

    fn apply_edges_through(&mut self, sample_phase: u32) {
        loop {
            if self.pending_edge.is_none() {
                let mut cycles = 0u32;
                let mut level = 0u8;
                if unsafe { bk_audio_pop_edge(&mut cycles, &mut level) } == 0 {
                    break;
                }
                self.pending_edge = Some((cycles, level != 0));
            }

            let (edge_cycle, edge_level) = self.pending_edge.unwrap();
            if sample_phase.wrapping_sub(edge_cycle) as i32 >= 0 {
                self.sample_level = edge_level;
                self.pending_edge = None;
            } else {
                break;
            }
        }
    }

    /// Start/stop edge capture. Called from TinyUSB control/task context.
    pub fn tinyusb_set_active(&mut self, active: bool) {
        if active {
            if !self.active {
                self.sample_level = unsafe { bk_audio_start() != 0 };
                self.pending_edge = None;
                self.next_sample_dwt = Self::dwt_cycles()
                    .wrapping_sub(AUDIO_STARTUP_LATENCY_DWT_CYCLES + AUDIO_PREFILL_DWT_CYCLES);
                PCM_TAIL.store(0, Ordering::Relaxed);
                PCM_HEAD.store(0, Ordering::Release);
                PCM_UNDERRUN_SAMPLES.store(0, Ordering::Relaxed);
                PCM_UNDERRUN_WINDOW.store(0, Ordering::Relaxed);
                PCM_RENDERED_WINDOW.store(0, Ordering::Relaxed);
                PCM_RENDER_CALLS_WINDOW.store(0, Ordering::Relaxed);
                PCM_MIN_OCCUPANCY.store(PCM_RING_CAPACITY, Ordering::Relaxed);
                PCM_MIN_TRACKING.store(false, Ordering::Relaxed);
                PCM_MAX_OCCUPANCY.store(0, Ordering::Relaxed);
                self.active = true;
            }
        } else if self.active {
            self.active = false;
            self.pending_edge = None;
            unsafe { bk_audio_stop(); }
            PCM_TAIL.store(0, Ordering::Relaxed);
            PCM_HEAD.store(0, Ordering::Release);
        }
    }

    /// Render at most max_samples whose emulated timestamps are due.
    /// This runs in the main loop, never in the USB interrupt.
    pub fn render_available(&mut self, max_samples: usize) -> usize {
        if !self.active {
            return 0;
        }
        PCM_RENDER_CALLS_WINDOW.fetch_add(1, Ordering::Relaxed);

        let render_until = Self::dwt_cycles().wrapping_sub(AUDIO_STARTUP_LATENCY_DWT_CYCLES);
        let mut rendered = 0usize;
        while rendered < max_samples {
            let head = PCM_HEAD.load(Ordering::Relaxed);
            let tail = PCM_TAIL.load(Ordering::Acquire);
            if head.wrapping_sub(tail) >= PCM_RING_CAPACITY {
                break;
            }

            let sample_phase = self.next_sample_dwt;
            if (render_until.wrapping_sub(sample_phase) as i32) < 0 {
                break;
            }

            self.apply_edges_through(sample_phase);
            let sample = if self.sample_level { AUDIO_AMPLITUDE } else { -AUDIO_AMPLITUDE };
            unsafe {
                core::ptr::addr_of_mut!(PCM_RING)
                    .cast::<i16>()
                    .add((head & PCM_RING_MASK) as usize)
                    .write(sample);
            }
            let new_occupancy = head.wrapping_add(1).wrapping_sub(tail);
            PCM_MAX_OCCUPANCY.fetch_max(new_occupancy, Ordering::Relaxed);
            if new_occupancy >= PCM_MIN_TRACKING_START {
                PCM_MIN_TRACKING.store(true, Ordering::Release);
            }
            PCM_HEAD.store(head.wrapping_add(1), Ordering::Release);
            self.next_sample_dwt = self.next_sample_dwt
                .wrapping_add(AUDIO_SAMPLE_PERIOD_DWT_CYCLES);
            rendered += 1;
        }
        PCM_RENDERED_WINDOW.fetch_add(rendered as u32, Ordering::Relaxed);
        rendered
    }
}

/// Take one-second producer/consumer counters. The main loop calls this once
/// per performance interval; USB underruns are counted separately from lifetime
/// totals so the displayed rate directly exposes a sustained clock mismatch.
pub fn take_interval_diagnostics() -> (u32, u32, u32) {
    (
        PCM_RENDERED_WINDOW.swap(0, Ordering::AcqRel),
        PCM_RENDER_CALLS_WINDOW.swap(0, Ordering::AcqRel),
        PCM_UNDERRUN_WINDOW.swap(0, Ordering::AcqRel),
    )
}

/// Return cumulative edge-queue overflows, PCM zero-fill samples, and minimum
/// observed minimum and maximum PCM occupancy (in samples) since stream start.
pub fn diagnostics() -> (u32, u32, u16, u16) {
    (
        unsafe { bk_audio_get_edge_overflows() },
        PCM_UNDERRUN_SAMPLES.load(Ordering::Relaxed),
        PCM_MIN_OCCUPANCY.load(Ordering::Relaxed),
        PCM_MAX_OCCUPANCY.load(Ordering::Relaxed),
    )
}

/// Copy one USB microframe of pre-rendered PCM. Missing samples are zero-filled.
pub unsafe fn fill_usb_packet(buffer: *mut u8, capacity: u16) -> u16 {
    if buffer.is_null() || capacity < PACKET_BYTES as u16 {
        return 0;
    }

    let mut tail = PCM_TAIL.load(Ordering::Relaxed);
    let head = PCM_HEAD.load(Ordering::Acquire);
    let occupancy = head.wrapping_sub(tail);
    if PCM_MIN_TRACKING.load(Ordering::Acquire) {
        PCM_MIN_OCCUPANCY.fetch_min(occupancy, Ordering::Relaxed);
    }
    let mut missing = 0u32;
    for index in 0..SAMPLES_PER_MICROFRAME {
        let sample = if tail != head {
            let value = unsafe {
                core::ptr::addr_of!(PCM_RING)
                    .cast::<i16>()
                    .add((tail & PCM_RING_MASK) as usize)
                    .read()
            };
            tail = tail.wrapping_add(1);
            value
        } else {
            missing += 1;
            0
        };
        unsafe {
            buffer.add(index * 2)
                .cast::<i16>()
                .write_unaligned(sample.to_le());
        }
    }
    PCM_TAIL.store(tail, Ordering::Release);
    if missing != 0 {
        PCM_UNDERRUN_SAMPLES.fetch_add(missing, Ordering::Relaxed);
        PCM_UNDERRUN_WINDOW.fetch_add(missing, Ordering::Relaxed);
    }
    PACKET_BYTES as u16
}

impl Default for BkEmulatorAudio {
    fn default() -> Self { Self::new() }
}
