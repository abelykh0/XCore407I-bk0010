//! BK-0010 speaker-edge capture and deferred PCM16 rendering for TinyUSB UAC2.
//!
//! The emulator publishes timestamped speaker transitions. The main-loop audio
//! worker converts them to 24 kHz PCM in a ring buffer. The USB completion ISR
//! only copies three already-rendered samples from that ring.

use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

const PDP_CLOCK_HZ: u32 = 3_000_000;
const PHASE_UNITS_PER_CYCLE: u32 = 4;
const USB_MICROFRAMES_PER_SECOND: u32 = 8_000;
const SAMPLES_PER_MICROFRAME: usize =
    (crate::AUDIO_SAMPLE_RATE_HZ / USB_MICROFRAMES_PER_SECOND) as usize;
const PACKET_BYTES: usize = SAMPLES_PER_MICROFRAME * 2;
// Quarter-cycle phase units represent 125 PDP cycles/sample exactly at 24 kHz.
const AUDIO_SAMPLE_PERIOD_PHASE: u32 =
    PDP_CLOCK_HZ * PHASE_UNITS_PER_CYCLE / crate::AUDIO_SAMPLE_RATE_HZ;
const AUDIO_STARTUP_LATENCY_PHASE: u32 = PDP_CLOCK_HZ * PHASE_UNITS_PER_CYCLE / 1_000;
const AUDIO_PREFILL_PHASE: u32 = PDP_CLOCK_HZ * PHASE_UNITS_PER_CYCLE * 6 / 1_000;
const _: () = assert!(crate::AUDIO_SAMPLE_RATE_HZ % USB_MICROFRAMES_PER_SECOND == 0);
const _: () = assert!(PDP_CLOCK_HZ * PHASE_UNITS_PER_CYCLE % crate::AUDIO_SAMPLE_RATE_HZ == 0);
const AUDIO_AMPLITUDE: i16 = 12_000;
const PCM_RING_CAPACITY: u16 = 2048;
const PCM_RING_MASK: u16 = PCM_RING_CAPACITY - 1;
const _: () = assert!(PCM_RING_CAPACITY.is_power_of_two());
const _: () = assert!(PACKET_BYTES <= crate::uvc::USB_HS_DMA_BUFFER_LIMIT);

static PCM_HEAD: AtomicU16 = AtomicU16::new(0);
static PCM_TAIL: AtomicU16 = AtomicU16::new(0);

#[cfg(feature = "cpu-cycle-diag")]
static AUDIO_RENDERED_SAMPLES: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "cpu-cycle-diag")]
static AUDIO_UNDERRUN_SAMPLES: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "cpu-cycle-diag")]
static AUDIO_HIGH_WATER: AtomicU16 = AtomicU16::new(0);

#[cfg(feature = "cpu-cycle-diag")]
pub fn take_performance_stats() -> (u32, u32, u16) {
    (
        AUDIO_RENDERED_SAMPLES.swap(0, Ordering::Relaxed),
        AUDIO_UNDERRUN_SAMPLES.swap(0, Ordering::Relaxed),
        AUDIO_HIGH_WATER.swap(0, Ordering::Relaxed),
    )
}
static mut PCM_RING: [i16; PCM_RING_CAPACITY as usize] = [0; PCM_RING_CAPACITY as usize];

unsafe extern "C" {
    fn bk_audio_start() -> u8;
    fn bk_timing_cycles() -> u32;
    fn bk_audio_stop();
    fn bk_audio_pop_edge(cycles: *mut u32, level: *mut u8) -> u8;
}

pub struct BkEmulatorAudio {
    // Quarter-cycle units preserve exact sample timing; at 24 kHz each sample is 125 PDP cycles.
    next_sample_phase: u32,
    sample_level: bool,
    pending_edge: Option<(u32, bool)>,
    active: bool,
}

impl BkEmulatorAudio {
    pub const fn new() -> Self {
        Self {
            next_sample_phase: 0,
            sample_level: false,
            pending_edge: None,
            active: false,
        }
    }

    #[inline(always)]
    fn cycle_phase() -> u32 {
        unsafe { bk_timing_cycles() }.wrapping_mul(PHASE_UNITS_PER_CYCLE)
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
            if sample_phase.wrapping_sub(edge_cycle.wrapping_mul(PHASE_UNITS_PER_CYCLE)) as i32 >= 0 {
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
                self.next_sample_phase = Self::cycle_phase()
                    .wrapping_sub(AUDIO_STARTUP_LATENCY_PHASE + AUDIO_PREFILL_PHASE);
                PCM_TAIL.store(0, Ordering::Relaxed);
                PCM_HEAD.store(0, Ordering::Release);
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
        let render_until = Self::cycle_phase().wrapping_sub(AUDIO_STARTUP_LATENCY_PHASE);
        let mut rendered = 0usize;
        #[cfg(feature = "cpu-cycle-diag")]
        let mut high_water = 0u16;
        while rendered < max_samples {
            let head = PCM_HEAD.load(Ordering::Relaxed);
            let tail = PCM_TAIL.load(Ordering::Acquire);
            if head.wrapping_sub(tail) >= PCM_RING_CAPACITY {
                break;
            }

            let sample_phase = self.next_sample_phase;
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
            PCM_HEAD.store(head.wrapping_add(1), Ordering::Release);
            #[cfg(feature = "cpu-cycle-diag")]
            {
                let occupancy = head.wrapping_add(1).wrapping_sub(tail);
                high_water = high_water.max(occupancy);
            }
            self.next_sample_phase = self.next_sample_phase
                .wrapping_add(AUDIO_SAMPLE_PERIOD_PHASE);
            rendered += 1;
        }
        #[cfg(feature = "cpu-cycle-diag")]
        {
            AUDIO_RENDERED_SAMPLES.fetch_add(rendered as u32, Ordering::Relaxed);
            AUDIO_HIGH_WATER.fetch_max(high_water, Ordering::Relaxed);
        }
        rendered
    }
}

/// Copy one USB microframe of pre-rendered PCM. Missing samples are zero-filled.
pub unsafe fn fill_usb_packet(buffer: *mut u8, capacity: u16) -> u16 {
    if buffer.is_null() || capacity < PACKET_BYTES as u16 {
        return 0;
    }

    let mut tail = PCM_TAIL.load(Ordering::Relaxed);
    let head = PCM_HEAD.load(Ordering::Acquire);
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
            #[cfg(feature = "cpu-cycle-diag")]
            AUDIO_UNDERRUN_SAMPLES.fetch_add(1, Ordering::Relaxed);
            0
        };
        unsafe {
            buffer.add(index * 2)
                .cast::<i16>()
                .write_unaligned(sample.to_le());
        }
    }
    PCM_TAIL.store(tail, Ordering::Release);
    PACKET_BYTES as u16
}

impl Default for BkEmulatorAudio {
    fn default() -> Self { Self::new() }
}
