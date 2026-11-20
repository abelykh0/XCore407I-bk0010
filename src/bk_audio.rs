//! BK-0010 speaker-edge capture and deferred PCM16 rendering for TinyUSB UAC2.
//!
//! The emulator publishes timestamped speaker transitions. The main-loop audio
//! worker converts them to 24 kHz PCM in a ring buffer. The USB completion ISR
//! copies exactly three samples per 8 kHz microframe.

use core::sync::atomic::{AtomicI32, AtomicU16, Ordering};
#[cfg(feature = "audio-underrun-diag")]
use core::sync::atomic::{AtomicBool, AtomicU32};

const AUDIO_STARTUP_LATENCY_PHASE: u32 = PDP_CLOCK_HZ * PHASE_UNITS_PER_CYCLE / 1_000;
const AUDIO_PREFILL_PHASE: u32 = PDP_CLOCK_HZ * PHASE_UNITS_PER_CYCLE * 3 / 1_000;

const PDP_CLOCK_HZ: u32 = 3_000_000;
const PHASE_UNITS_PER_CYCLE: u32 = 4;
const USB_MICROFRAMES_PER_SECOND: u32 = 8_000;
const SAMPLES_PER_MICROFRAME: usize =
    (crate::AUDIO_SAMPLE_RATE_HZ / USB_MICROFRAMES_PER_SECOND) as usize;
const PACKET_BYTES: usize = SAMPLES_PER_MICROFRAME * 2;
// Quarter-cycle phase units represent 125 PDP cycles/sample exactly at 24 kHz.
const AUDIO_SAMPLE_PERIOD_PHASE: u32 =
    PDP_CLOCK_HZ * PHASE_UNITS_PER_CYCLE / crate::AUDIO_SAMPLE_RATE_HZ;
const _: () = assert!(crate::AUDIO_SAMPLE_RATE_HZ % USB_MICROFRAMES_PER_SECOND == 0);
const _: () = assert!(PDP_CLOCK_HZ * PHASE_UNITS_PER_CYCLE % crate::AUDIO_SAMPLE_RATE_HZ == 0);
const AUDIO_AMPLITUDE: i16 = 12_000;
const PCM_RING_CAPACITY: u16 = 2048;
const PCM_RING_MASK: u16 = PCM_RING_CAPACITY - 1;
const _: () = assert!(PCM_RING_CAPACITY.is_power_of_two());
const _: () = assert!(PACKET_BYTES <= crate::uvc::USB_HS_DMA_BUFFER_LIMIT);

static PCM_HEAD: AtomicU16 = AtomicU16::new(0);
static PCM_TAIL: AtomicU16 = AtomicU16::new(0);
static RESAMPLER_CORRECTION_PPM: AtomicI32 = AtomicI32::new(0);
#[cfg(feature = "audio-underrun-diag")]
static UNDERRUN_ACTIVE: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "audio-underrun-diag")]
static UNDERRUN_EVENTS: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "audio-underrun-diag")]
static UNDERRUN_SAMPLES: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "audio-underrun-diag")]
static PCM_SAMPLES_RENDERED: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "audio-underrun-diag")]
static PCM_SAMPLES_CONSUMED: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "audio-underrun-diag")]
static PCM_MIN_OCCUPANCY: AtomicU32 = AtomicU32::new(u32::MAX);
#[cfg(feature = "audio-underrun-diag")]
static LAST_RENDER_DWT: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "audio-underrun-diag")]
static RENDER_GAP_INITIALIZED: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "audio-underrun-diag")]
static MAX_RENDER_GAP_DWT: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "audio-underrun-diag")]
static FIFO_WRITE_FAILURES: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "audio-underrun-diag")]
static PCM_RING_FULL_EVENTS: AtomicU32 = AtomicU32::new(0);
static mut PCM_RING: [i16; PCM_RING_CAPACITY as usize] = [0; PCM_RING_CAPACITY as usize];

const RESAMPLER_ONE: u32 = 1 << 16;
const RESAMPLER_TARGET_OCCUPANCY: i32 = 72; // 3 ms at 24 kHz.
const RESAMPLER_CONTROL_PACKETS: u8 = 80; // 10 ms at 8 kHz.
const RESAMPLER_CORRECTION_LIMIT_PPM: i32 = 5_000;

/// USB-side source-position controller. The output cadence remains fixed;
/// only the fractional source position advances slightly faster or slower.
#[derive(Clone, Copy)]
struct PcmResampler {
    phase: u32,
    step: u32,
    control_packets: u8,
}

impl PcmResampler {
    const fn new() -> Self {
        Self {
            phase: 0,
            step: RESAMPLER_ONE,
            control_packets: 0,
        }
    }

    /// A fuller ring increases the source step, draining it faster.
    #[inline]
    fn update_rate(&mut self, occupancy: u16) {
        self.control_packets += 1;
        if self.control_packets < RESAMPLER_CONTROL_PACKETS {
            return;
        }
        self.control_packets = 0;

        let error = occupancy as i32 - RESAMPLER_TARGET_OCCUPANCY;
        let correction_ppm = (error * 4)
            .clamp(-RESAMPLER_CORRECTION_LIMIT_PPM,
                   RESAMPLER_CORRECTION_LIMIT_PPM);
        self.step = ((RESAMPLER_ONE as i64 * (1_000_000 + correction_ppm) as i64)
            / 1_000_000) as u32;
        let actual_correction_ppm = ((self.step as i64 - RESAMPLER_ONE as i64) * 1_000_000
            / RESAMPLER_ONE as i64) as i32;
        RESAMPLER_CORRECTION_PPM.store(actual_correction_ppm, Ordering::Release);
    }


}

static mut PCM_RESAMPLER: PcmResampler = PcmResampler::new();

unsafe extern "C" {
    safe fn bk_audio_start() -> u8;
    safe fn bk_timing_cycles() -> u32;
    safe fn bk_audio_stop();
    #[cfg(feature = "audio-underrun-diag")]
    safe fn bk_audio_take_edge_overflows() -> u32;
    fn bk_audio_pop_edge(cycles: *mut u32, level: *mut u8) -> u8;
}

#[derive(Clone, Copy)]
struct BlepEdge {
    phase: u32,
    level: bool,
}

const BLEP_EDGE_HISTORY_CAPACITY: usize = 128;

pub struct BkEmulatorAudio {
    // Quarter-cycle units preserve exact sample timing; at 24 kHz each sample is 125 PDP cycles.
    next_sample_phase: u32,
    sample_level: bool,
    pending_edge: Option<(u32, bool)>,
    // All transitions within the BLEP support window, in timestamp order.
    blep_edges: [BlepEdge; BLEP_EDGE_HISTORY_CAPACITY],
    blep_edge_count: usize,
    blep_applied_count: usize,
    edge_queue_empty: bool,
    active: bool,
}

impl BkEmulatorAudio {
    pub const fn new() -> Self {
        Self {
            next_sample_phase: 0,
            sample_level: false,
            pending_edge: None,
            blep_edges: [BlepEdge { phase: 0, level: false }; BLEP_EDGE_HISTORY_CAPACITY],
            blep_edge_count: 0,
            blep_applied_count: 0,
            edge_queue_empty: false,
            active: false,
        }
    }

    #[inline(always)]
    fn cycle_phase() -> u32 {
        bk_timing_cycles().wrapping_mul(PHASE_UNITS_PER_CYCLE)
    }

    /// Start/stop edge capture. Called from TinyUSB control/task context.
    pub fn tinyusb_set_active(&mut self, active: bool) {
        if active {
            if !self.active {
                self.sample_level = bk_audio_start() != 0;
                #[cfg(feature = "audio-underrun-diag")]
                reset_audio_diagnostics();
                self.pending_edge = None;
                self.blep_edge_count = 0;
                self.blep_applied_count = 0;
                unsafe {
                    core::ptr::addr_of_mut!(PCM_RESAMPLER).write(PcmResampler::new());
                }
                self.next_sample_phase = Self::cycle_phase()
                    .wrapping_sub(AUDIO_STARTUP_LATENCY_PHASE + AUDIO_PREFILL_PHASE);
                PCM_TAIL.store(0, Ordering::Relaxed);
                PCM_HEAD.store(0, Ordering::Release);
                self.active = true;
            }
        } else if self.active {
            self.active = false;
            self.pending_edge = None;
            self.blep_edge_count = 0;
            self.blep_applied_count = 0;
            unsafe {
                core::ptr::addr_of_mut!(PCM_RESAMPLER).write(PcmResampler::new());
            }
            RESAMPLER_CORRECTION_PPM.store(0, Ordering::Release);
            bk_audio_stop();
            PCM_TAIL.store(0, Ordering::Relaxed);
            PCM_HEAD.store(0, Ordering::Release);
        }
    }

    /// Reinitialize the emulator-side capture after BK reset without changing
    /// whether the USB audio alternate setting is active.
    pub fn reset_emulator_capture(&mut self) {
        if self.active {
            self.tinyusb_set_active(false);
            self.tinyusb_set_active(true);
        }
    }

    /// Render at most max_samples whose emulated timestamps are due.
    /// This runs in the main loop, never in the USB interrupt.
    pub fn render_available(&mut self, max_samples: usize) -> usize {
        if !self.active {
            return 0;
        }
        #[cfg(feature = "audio-underrun-diag")]
        record_render_interval();
        let render_until = Self::cycle_phase().wrapping_sub(AUDIO_STARTUP_LATENCY_PHASE);
        // Emulator execution and rendering are sequential in the main loop, so
        // an empty edge queue stays empty for the rest of this render batch.
        self.edge_queue_empty = false;
        // Keep producer indices local and publish the new head once per batch.
        // A stale tail is conservative: the USB consumer can only advance it.
        let mut head = PCM_HEAD.load(Ordering::Relaxed);
        let tail = PCM_TAIL.load(Ordering::Acquire);
        let mut rendered = 0usize;
        while rendered < max_samples {
            if head.wrapping_sub(tail) >= PCM_RING_CAPACITY {
                #[cfg(feature = "audio-underrun-diag")]
                PCM_RING_FULL_EVENTS.fetch_add(1, Ordering::Relaxed);
                break;
            }

            let sample_phase = self.next_sample_phase;
            if (render_until.wrapping_sub(sample_phase) as i32) < 0 {
                break;
            }

            self.prepare_blep_edges(sample_phase);
            let raw_sample = if self.sample_level { AUDIO_AMPLITUDE } else { -AUDIO_AMPLITUDE };
            let sample = (raw_sample as i32 + self.blep_edge_correction(sample_phase))
                .clamp(-(AUDIO_AMPLITUDE as i32), AUDIO_AMPLITUDE as i32) as i16;
            unsafe {
                core::ptr::addr_of_mut!(PCM_RING)
                    .cast::<i16>()
                    .add((head & PCM_RING_MASK) as usize)
                    .write(sample);
            }
            head = head.wrapping_add(1);
            self.next_sample_phase = self.next_sample_phase
                .wrapping_add(AUDIO_SAMPLE_PERIOD_PHASE);
            rendered += 1;
        }
        if rendered != 0 {
            PCM_HEAD.store(head, Ordering::Release);
            #[cfg(feature = "audio-underrun-diag")]
            PCM_SAMPLES_RENDERED.fetch_add(rendered as u32, Ordering::Relaxed);
        }
        rendered
    }
    // Sum the compact polynomial BLEP contribution from every transition
    // whose one-sample support overlaps this output sample.
    #[inline(always)]
    fn blep_edge_correction(&self, sample_phase: u32) -> i32 {
        let period = AUDIO_SAMPLE_PERIOD_PHASE;
        let mut correction = 0i32;

        for edge in self.blep_edges[..self.blep_edge_count].iter() {
            let age = sample_phase.wrapping_sub(edge.phase) as i32;
            let polarity = if edge.level { 1 } else { -1 };

            if age >= 0 && (age as u32) < period {
                let t = ((age as u32 * 32_768) / period) as i32;
                let q = 2 * t - ((t * t) >> 15) - 32_768;
                correction += polarity * AUDIO_AMPLITUDE as i32 * q / 32_768;
            } else {
                let ahead = edge.phase.wrapping_sub(sample_phase) as i32;
                if ahead > 0 && (ahead as u32) < period {
                    let t = (((period - ahead as u32) * 32_768) / period) as i32;
                    let q = (t * t) >> 15;
                    correction += polarity * AUDIO_AMPLITUDE as i32 * q / 32_768;
                }
            }
        }

        correction
    }

    // Retain every transition whose BLEP support overlaps this sample. Old
    // transitions update the held level; future transitions remain queued.
    fn prepare_blep_edges(&mut self, sample_phase: u32) {
        let period = AUDIO_SAMPLE_PERIOD_PHASE as i32;

        while self.blep_edge_count != 0 {
            let age = sample_phase.wrapping_sub(self.blep_edges[0].phase) as i32;
            if age < period {
                break;
            }
            for index in 1..self.blep_edge_count {
                self.blep_edges[index - 1] = self.blep_edges[index];
            }
            self.blep_edge_count -= 1;
            self.blep_applied_count = self.blep_applied_count.saturating_sub(1);
        }

        loop {
            if self.pending_edge.is_none() {
                if self.edge_queue_empty {
                    break;
                }
                let mut cycles = 0u32;
                let mut level = 0u8;
                if unsafe { bk_audio_pop_edge(&mut cycles, &mut level) } == 0 {
                    self.edge_queue_empty = true;
                    break;
                }
                self.pending_edge = Some((cycles, level != 0));
            }

            let (edge_cycle, edge_level) = self.pending_edge.unwrap();
            let edge_phase = edge_cycle.wrapping_mul(PHASE_UNITS_PER_CYCLE);
            let ahead = edge_phase.wrapping_sub(sample_phase) as i32;
            if ahead >= period {
                break;
            }

            if ahead <= -period {
                // Its BLEP support has ended, but its logic level still applies.
                self.sample_level = edge_level;
            } else if self.blep_edge_count < BLEP_EDGE_HISTORY_CAPACITY {
                self.blep_edges[self.blep_edge_count] = BlepEdge {
                    phase: edge_phase,
                    level: edge_level,
                };
                self.blep_edge_count += 1;
            } else {
                // Keep the edge pending; older entries will expire next sample.
                break;
            }
            self.pending_edge = None;
        }

        while self.blep_applied_count < self.blep_edge_count {
            let edge = self.blep_edges[self.blep_applied_count];
            if (sample_phase.wrapping_sub(edge.phase) as i32) < 0 {
                break;
            }
            self.sample_level = edge.level;
            self.blep_applied_count += 1;
        }
    }

}

/// Resample one USB microframe from the PCM ring at a slowly adjusted rate.
/// USB runs at 8 kHz and always receives exactly three 24 kHz output samples.
/// Linear interpolation changes source position smoothly without changing USB cadence.
///
/// # Safety
/// If non-null and sufficiently large, `buffer` must point to a valid,
/// writable allocation of at least `capacity` bytes for the duration of this call.
pub unsafe fn fill_usb_packet(buffer: *mut u8, capacity: u16) -> u16 {
    if buffer.is_null() || capacity < PACKET_BYTES as u16 {
        return 0;
    }

    let buffer = unsafe { core::slice::from_raw_parts_mut(buffer, capacity as usize) };
    let mut tail = PCM_TAIL.load(Ordering::Relaxed);
    let head = PCM_HEAD.load(Ordering::Acquire);
    let mut resampler = unsafe { core::ptr::addr_of!(PCM_RESAMPLER).read() };
    resampler.update_rate(head.wrapping_sub(tail));

    #[cfg(feature = "audio-underrun-diag")]
    let mut missing_samples = 0u32;
    #[cfg(feature = "audio-underrun-diag")]
    let mut underrun_active = UNDERRUN_ACTIVE.load(Ordering::Relaxed);

    for index in 0..SAMPLES_PER_MICROFRAME {
        let available = head.wrapping_sub(tail);
        #[cfg(feature = "audio-underrun-diag")]
        PCM_MIN_OCCUPANCY.fetch_min(available as u32, Ordering::Relaxed);

        let sample = if available >= 2 {
            let a = unsafe {
                core::ptr::addr_of!(PCM_RING)
                    .cast::<i16>()
                    .add((tail & PCM_RING_MASK) as usize)
                    .read()
            };
            let b = unsafe {
                core::ptr::addr_of!(PCM_RING)
                    .cast::<i16>()
                    .add((tail.wrapping_add(1) & PCM_RING_MASK) as usize)
                    .read()
            };
            let fraction = (resampler.phase & 0xFFFF) as u32;
            let interpolated = a as i32
                + (((b as i32 - a as i32) * fraction as i32) >> 16);
            resampler.phase = resampler.phase.wrapping_add(resampler.step);
            let advance = resampler.phase >> 16;
            resampler.phase &= 0xFFFF;
            tail = tail.wrapping_add(advance as u16);
            let sample = interpolated.clamp(i16::MIN as i32, i16::MAX as i32) as i16;

            #[cfg(feature = "audio-underrun-diag")]
            {
                underrun_active = false;
                PCM_SAMPLES_CONSUMED.fetch_add(advance, Ordering::Relaxed);
            }
            sample
        } else if available == 1 {
            // Hold the final available sample until its interpolation neighbor arrives.
            #[cfg(feature = "audio-underrun-diag")]
            {
                underrun_active = false;
            }
            unsafe {
                core::ptr::addr_of!(PCM_RING)
                    .cast::<i16>()
                    .add((tail & PCM_RING_MASK) as usize)
                    .read()
            }
        } else {
            resampler.phase = 0;
            #[cfg(feature = "audio-underrun-diag")]
            {
                if !underrun_active {
                    UNDERRUN_EVENTS.fetch_add(1, Ordering::Relaxed);
                    underrun_active = true;
                }
                missing_samples += 1;
            }
            0
        };

        let bytes = sample.to_le_bytes();
        buffer[index * 2] = bytes[0];
        buffer[index * 2 + 1] = bytes[1];
    }

    PCM_TAIL.store(tail, Ordering::Release);
    unsafe {
        core::ptr::addr_of_mut!(PCM_RESAMPLER).write(resampler);
    }
    #[cfg(feature = "audio-underrun-diag")]
    {
        UNDERRUN_ACTIVE.store(underrun_active, Ordering::Relaxed);
        UNDERRUN_SAMPLES.fetch_add(missing_samples, Ordering::Relaxed);
    }
    PACKET_BYTES as u16
}


/// Return and reset audio pipeline diagnostics for the previous interval.
#[cfg(feature = "audio-underrun-diag")]
pub fn take_audio_diagnostics() -> (u32, u32, u16, i32, u16, u32, u32, u32, u32, i32) {
    let underruns = UNDERRUN_EVENTS.swap(0, Ordering::AcqRel);
    let missing_samples = UNDERRUN_SAMPLES.swap(0, Ordering::AcqRel);
    let generated = PCM_SAMPLES_RENDERED.swap(0, Ordering::AcqRel);
    let consumed = PCM_SAMPLES_CONSUMED.swap(0, Ordering::AcqRel);
    let minimum_raw = PCM_MIN_OCCUPANCY.swap(u32::MAX, Ordering::AcqRel);
    let minimum = if minimum_raw == u32::MAX {
        0
    } else {
        minimum_raw.min(PCM_RING_CAPACITY as u32) as u16
    };
    let max_gap_dwt = MAX_RENDER_GAP_DWT.swap(0, Ordering::AcqRel);
    let max_gap_us = ((max_gap_dwt as u64 * 1_000_000) / 168_000_000) as u32;
    let edge_overflows = bk_audio_take_edge_overflows();
    let fifo_failures = FIFO_WRITE_FAILURES.swap(0, Ordering::AcqRel);
    let ring_full_events = PCM_RING_FULL_EVENTS.swap(0, Ordering::AcqRel);
    let head = PCM_HEAD.load(Ordering::Acquire);
    let tail = PCM_TAIL.load(Ordering::Acquire);
    (
        underruns,
        missing_samples,
        head.wrapping_sub(tail),
        generated as i32 - consumed as i32,
        minimum,
        max_gap_us,
        edge_overflows,
        fifo_failures,
        ring_full_events,
        RESAMPLER_CORRECTION_PPM.load(Ordering::Acquire),
    )
}

pub fn note_fifo_write_failure() {
    #[cfg(feature = "audio-underrun-diag")]
    FIFO_WRITE_FAILURES.fetch_add(1, Ordering::Relaxed);
}

#[cfg(feature = "audio-underrun-diag")]
fn reset_audio_diagnostics() {
    UNDERRUN_ACTIVE.store(false, Ordering::Relaxed);
    UNDERRUN_EVENTS.store(0, Ordering::Relaxed);
    UNDERRUN_SAMPLES.store(0, Ordering::Relaxed);
    PCM_SAMPLES_RENDERED.store(0, Ordering::Relaxed);
    PCM_SAMPLES_CONSUMED.store(0, Ordering::Relaxed);
    PCM_MIN_OCCUPANCY.store(u32::MAX, Ordering::Relaxed);
    RENDER_GAP_INITIALIZED.store(false, Ordering::Relaxed);
    MAX_RENDER_GAP_DWT.store(0, Ordering::Relaxed);
    FIFO_WRITE_FAILURES.store(0, Ordering::Relaxed);
    PCM_RING_FULL_EVENTS.store(0, Ordering::Relaxed);
}

#[cfg(feature = "audio-underrun-diag")]
fn record_render_interval() {
    const DWT_CYCCNT: *const u32 = 0xE000_1004 as *const u32;
    let now = unsafe { core::ptr::read_volatile(DWT_CYCCNT) };
    if RENDER_GAP_INITIALIZED.swap(true, Ordering::AcqRel) {
        let previous = LAST_RENDER_DWT.swap(now, Ordering::AcqRel);
        MAX_RENDER_GAP_DWT.fetch_max(now.wrapping_sub(previous), Ordering::Relaxed);
    } else {
        LAST_RENDER_DWT.store(now, Ordering::Release);
    }
}

impl Default for BkEmulatorAudio {
    fn default() -> Self { Self::new() }
}
