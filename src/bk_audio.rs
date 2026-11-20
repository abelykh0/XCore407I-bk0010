//! BK-0010 speaker-edge capture and deferred PCM16 rendering for TinyUSB UAC2.
//!
//! The emulator publishes timestamped speaker transitions. The main-loop audio
//! worker converts them to nominal 24 kHz PCM in a ring buffer. The USB
//! completion ISR dynamically resamples that ring to the fixed USB sample rate.

use core::sync::atomic::{AtomicU16, Ordering};
#[cfg(feature = "audio-underrun-diag")]
use core::sync::atomic::{AtomicBool, AtomicU32};

const AUDIO_STARTUP_LATENCY_PHASE: u32 = PDP_CLOCK_HZ * PHASE_UNITS_PER_CYCLE / 1_000;
const AUDIO_PREFILL_PHASE: u32 = PDP_CLOCK_HZ * PHASE_UNITS_PER_CYCLE * 2 / 1_000;

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

#[derive(Clone, Copy)]
struct PcmResampler {
    phase: u32,
    step: u32,
    // Emulator-cycle phase corresponding to PCM_RING[PCM_TAIL].
    source_phase: u32,
    pending_edge: Option<(u32, i32)>,
    next_blep_correction: i32,
}

impl PcmResampler {
    const fn new(source_phase: u32) -> Self {
        Self {
            phase: 0,
            step: RESAMPLER_ONE,
            source_phase,
            pending_edge: None,
            next_blep_correction: 0,
        }
    }

    // Sum every edge affecting this output sample. Edges in the interval
    // (sample_phase, next_phase] also contribute to the following sample;
    // carry that sum as a scalar instead of retaining an edge history array.
    #[inline(always)]
    fn blep_correction(&mut self, sample_phase: u32) -> i32 {
        let period = AUDIO_SAMPLE_PERIOD_PHASE;
        let mut correction = self.next_blep_correction;
        self.next_blep_correction = 0;

        loop {
            if self.pending_edge.is_none() {
                let mut cycles = 0u32;
                let mut level = 0u8;
                if unsafe { bk_audio_pop_blep_edge(&mut cycles, &mut level) } == 0 {
                    break;
                }
                self.pending_edge = Some((
                    cycles.wrapping_mul(PHASE_UNITS_PER_CYCLE),
                    if level != 0 { 1 } else { -1 },
                ));
            }

            let (edge_phase, polarity) = self.pending_edge.unwrap();
            let until_edge = edge_phase.wrapping_sub(sample_phase) as i32;
            if until_edge > period as i32 {
                break;
            }

            if until_edge > 0 {
                // The edge is ahead of this sample: apply the pre-edge
                // polynomial now and carry its post-edge correction forward.
                let t = (((period - until_edge as u32) * 32_768) / period) as i32;
                let q = (t * t) >> 15;
                correction += polarity * AUDIO_AMPLITUDE as i32 * q / 32_768;

                let age_next = period - until_edge as u32;
                let t_next = ((age_next * 32_768) / period) as i32;
                let q_next = 2 * t_next - ((t_next * t_next) >> 15) - 32_768;
                self.next_blep_correction +=
                    polarity * AUDIO_AMPLITUDE as i32 * q_next / 32_768;
            } else {
                // The edge is at or behind this sample.
                let age = sample_phase.wrapping_sub(edge_phase) as i32;
                if age >= 0 && (age as u32) < period {
                    let t = (((age as u32) * 32_768) / period) as i32;
                    let q = 2 * t - ((t * t) >> 15) - 32_768;
                    correction += polarity * AUDIO_AMPLITUDE as i32 * q / 32_768;
                }
            }

            self.pending_edge = None;
        }

        correction
    }
}

// Accessed by the single USB audio consumer. Stream start/stop resets it with
// USB interrupts masked, so the main-loop producer never races this state.
static mut PCM_RESAMPLER: PcmResampler = PcmResampler::new(0);

unsafe extern "C" {
    safe fn bk_audio_start() -> u8;
    safe fn bk_timing_cycles() -> u32;
    safe fn bk_audio_stop();
    #[cfg(feature = "audio-underrun-diag")]
    safe fn bk_audio_take_edge_overflows() -> u32;
    fn bk_audio_pop_edge(cycles: *mut u32, level: *mut u8) -> u8;
    fn bk_audio_pop_blep_edge(cycles: *mut u32, level: *mut u8) -> u8;
}

pub struct BkEmulatorAudio {
    // Quarter-cycle units preserve exact sample timing; at 24 kHz each sample is 125 PDP cycles.
    next_sample_phase: u32,
    sample_level: bool,
    pending_edge: Option<(u32, bool)>,
    edge_queue_empty: bool,
    active: bool,
}

impl BkEmulatorAudio {
    pub const fn new() -> Self {
        Self {
            next_sample_phase: 0,
            sample_level: false,
            pending_edge: None,
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
                self.next_sample_phase = Self::cycle_phase()
                    .wrapping_sub(AUDIO_STARTUP_LATENCY_PHASE + AUDIO_PREFILL_PHASE);
                PCM_TAIL.store(0, Ordering::Relaxed);
                PCM_HEAD.store(0, Ordering::Release);
                unsafe {
                    core::ptr::addr_of_mut!(PCM_RESAMPLER).write(PcmResampler::new(self.next_sample_phase));
                }
                self.active = true;
            }
        } else if self.active {
            self.active = false;
            self.pending_edge = None;
            bk_audio_stop();
            PCM_TAIL.store(0, Ordering::Relaxed);
            PCM_HEAD.store(0, Ordering::Release);
            unsafe {
                core::ptr::addr_of_mut!(PCM_RESAMPLER).write(PcmResampler::new(0));
            }
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

            self.apply_edges_through(sample_phase);
            // Store only the raw speaker level. BLEP is applied later by
            // the USB consumer, after the PCM sample position is resampled.
            let raw_sample = if self.sample_level { AUDIO_AMPLITUDE } else { -AUDIO_AMPLITUDE };
            unsafe {
                core::ptr::addr_of_mut!(PCM_RING)
                    .cast::<i16>()
                    .add((head & PCM_RING_MASK) as usize)
                    .write(raw_sample);
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
    fn apply_edges_through(&mut self, sample_phase: u32) {
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
            if sample_phase.wrapping_sub(edge_cycle.wrapping_mul(PHASE_UNITS_PER_CYCLE)) as i32 >= 0 {
                self.sample_level = edge_level;
                self.pending_edge = None;
            } else {
                break;
            }
        }
    }

}

/// Resample one USB microframe from the emulator PCM ring at a controlled
/// rate. USB always receives the fixed number of samples for this microframe.
///
/// The current test holds the source ratio at exactly 1.0. The phase
/// accumulator remains available for later rate-control experiments.
///
/// # Safety
/// If non-null and sufficiently large, `buffer` must point to a valid,
/// writable allocation of at least `capacity` bytes for the duration of this call.
pub unsafe fn fill_usb_packet(buffer: *mut u8, capacity: u16) -> u16 {
    if buffer.is_null() || capacity < PACKET_BYTES as u16 {
        return 0;
    }

    // The caller supplies the DMA packet buffer; validate its pointer and
    // declared extent once, then keep all packet writes within this slice.
    let buffer = unsafe { core::slice::from_raw_parts_mut(buffer, capacity as usize) };

    let mut tail = PCM_TAIL.load(Ordering::Relaxed);
    let head = PCM_HEAD.load(Ordering::Acquire);
    // Keep the ratio at exactly 1.0 while isolating post-resample BLEP.
    let mut resampler = unsafe { core::ptr::addr_of!(PCM_RESAMPLER).read() };
    resampler.step = RESAMPLER_ONE;

    #[cfg(feature = "audio-underrun-diag")]
    let mut missing_samples = 0u32;
    #[cfg(feature = "audio-underrun-diag")]
    let mut underrun_active = UNDERRUN_ACTIVE.load(Ordering::Relaxed);
    for index in 0..SAMPLES_PER_MICROFRAME {
        let available = head.wrapping_sub(tail);
        #[cfg(feature = "audio-underrun-diag")]
        PCM_MIN_OCCUPANCY.fetch_min(available as u32, Ordering::Relaxed);
        let sample = if available != 0 {
            #[cfg(feature = "audio-underrun-diag")]
            { underrun_active = false; }
            let s0 = unsafe {
                core::ptr::addr_of!(PCM_RING)
                    .cast::<i16>()
                    .add((tail & PCM_RING_MASK) as usize)
                    .read()
            };
            let s1 = if available > 1 {
                unsafe {
                    core::ptr::addr_of!(PCM_RING)
                        .cast::<i16>()
                        .add((tail.wrapping_add(1) & PCM_RING_MASK) as usize)
                        .read()
                }
            } else {
                s0
            };

            let fraction = (resampler.phase & 0xFFFF) as u32;
            let interpolated = s0 as i32
                + (((s1 as i32 - s0 as i32) * fraction as i32) >> 16);
            let sample_phase = resampler.source_phase.wrapping_add(
                (AUDIO_SAMPLE_PERIOD_PHASE.wrapping_mul(fraction)) >> 16,
            );

            // BLEP is evaluated at the actual resampled position, after raw
            // PCM interpolation rather than in the nominal-rate producer.
            let sample = (interpolated + resampler.blep_correction(sample_phase))
                .clamp(-(AUDIO_AMPLITUDE as i32), AUDIO_AMPLITUDE as i32) as i16;

            resampler.phase = resampler.phase.wrapping_add(resampler.step);
            let consumed = (resampler.phase >> 16) as u16;
            #[cfg(feature = "audio-underrun-diag")]
            PCM_SAMPLES_CONSUMED.fetch_add(consumed.min(available) as u32, Ordering::Relaxed);
            if consumed > available {
                // Keep the source-time cursor aligned with samples actually consumed.
                tail = tail.wrapping_add(available);
                resampler.source_phase = resampler.source_phase
                    .wrapping_add(available as u32 * AUDIO_SAMPLE_PERIOD_PHASE);
                resampler.phase = 0;
            } else {
                tail = tail.wrapping_add(consumed);
                resampler.source_phase = resampler.source_phase
                    .wrapping_add(consumed as u32 * AUDIO_SAMPLE_PERIOD_PHASE);
                resampler.phase &= 0xFFFF;
            }
            sample
        } else {
            // Do not advance the source cursor while empty; the next valid
            // sample starts a fresh interpolation interval.
            #[cfg(feature = "audio-underrun-diag")]
            {
                if !underrun_active {
                    UNDERRUN_EVENTS.fetch_add(1, Ordering::Relaxed);
                    underrun_active = true;
                }
                missing_samples += 1;
            }
            resampler.phase = 0;
            0
        };

        let bytes = sample.to_le_bytes();
        buffer[index * 2] = bytes[0];
        buffer[index * 2 + 1] = bytes[1];
    }

    unsafe {
        core::ptr::addr_of_mut!(PCM_RESAMPLER).write(resampler);
    }
    PCM_TAIL.store(tail, Ordering::Release);
    #[cfg(feature = "audio-underrun-diag")]
    {
        UNDERRUN_ACTIVE.store(underrun_active, Ordering::Relaxed);
        UNDERRUN_SAMPLES.fetch_add(missing_samples, Ordering::Relaxed);
    }
    PACKET_BYTES as u16
}


/// Return and reset audio pipeline diagnostics for the previous interval.
#[cfg(feature = "audio-underrun-diag")]
pub fn take_audio_diagnostics() -> (u32, u32, u16, i32, u16, u32, u32, u32, u32) {
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
    let edge_overflows = unsafe { bk_audio_take_edge_overflows() };
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
