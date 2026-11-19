//! BK-0010 speaker-edge capture and deferred PCM16 rendering for TinyUSB UAC2.
//!
//! The emulator publishes timestamped speaker transitions. The main-loop audio
//! worker converts them to 24 kHz PCM in a ring buffer. The USB completion ISR
//! only copies three already-rendered samples from that ring.

use core::sync::atomic::{AtomicI32, AtomicU16, AtomicU32, Ordering};
#[cfg(feature = "audio-underrun-diag")]
use core::sync::atomic::AtomicBool;

// Keep three milliseconds of PCM queued so normal main-loop jitter does not
// immediately starve the USB consumer. The controller targets this depth.
const AUDIO_STARTUP_LATENCY_PHASE: u32 =
    PDP_CLOCK_HZ * PHASE_UNITS_PER_CYCLE * 3 / 1_000;
const AUDIO_TARGET_BUFFER_SAMPLES: i32 =
    (crate::AUDIO_SAMPLE_RATE_HZ as i32 * 3) / 1_000;
const AUDIO_CLOCK_CONTROL_INTERVAL_CYCLES: u32 = PDP_CLOCK_HZ / 100;
const AUDIO_CLOCK_CORRECTION_LIMIT_PPM: i32 = 1_000;
const AUDIO_CLOCK_KP_PPM_PER_SAMPLE: i32 = 40;
const AUDIO_CLOCK_KI_PPM_PER_SAMPLE_PER_INTERVAL: i32 = 2;
const AUDIO_PREFILL_PHASE: u32 = PDP_CLOCK_HZ * PHASE_UNITS_PER_CYCLE * 2 / 1_000;

// Keep audio timestamps on the same clock as the emulator's cycle budget.
const PDP_CLOCK_HZ: u32 = 3_000_000;
// Quarter-cycle phase units preserve exact sample timing at 24 kHz.
const PHASE_UNITS_PER_CYCLE: u32 = 4;
const USB_MICROFRAMES_PER_SECOND: u32 = 8_000;
const SAMPLES_PER_MICROFRAME: usize =
    (crate::AUDIO_SAMPLE_RATE_HZ / USB_MICROFRAMES_PER_SECOND) as usize;
const PACKET_BYTES: usize = SAMPLES_PER_MICROFRAME * 2;
// At 3 MHz, each 24 kHz sample spans 125 PDP cycles (500 phase units).
const AUDIO_SAMPLE_PERIOD_PHASE: u32 =
    PDP_CLOCK_HZ * PHASE_UNITS_PER_CYCLE / crate::AUDIO_SAMPLE_RATE_HZ;
const _: () = assert!(crate::AUDIO_SAMPLE_RATE_HZ % USB_MICROFRAMES_PER_SECOND == 0);
const _: () = assert!(PDP_CLOCK_HZ * PHASE_UNITS_PER_CYCLE % crate::AUDIO_SAMPLE_RATE_HZ == 0);
const AUDIO_AMPLITUDE: i16 = 12_000;

// 2-pole Butterworth low-pass, 4.5 kHz cutoff at the 24 kHz PCM rate.
// Coefficients are normalized (a0 = 1) and calculated once, not per sample.
const SPEAKER_LPF_B0: f32 = 0.186_694_34;
const SPEAKER_LPF_B1: f32 = 0.373_388_68;
const SPEAKER_LPF_B2: f32 = 0.186_694_34;
const SPEAKER_LPF_A1: f32 = -0.462_938_04;
const SPEAKER_LPF_A2: f32 = 0.209_715_36;

struct SpeakerLowPass {
    // Transposed direct-form II delay elements.
    z1: f32,
    z2: f32,
}

impl SpeakerLowPass {
    const fn new() -> Self {
        Self { z1: 0.0, z2: 0.0 }
    }

    // Initialize to the steady-state output for a constant speaker level,
    // avoiding an artificial filter ramp when USB audio is enabled.
    fn reset(&mut self, level: bool) {
        let input = if level { AUDIO_AMPLITUDE as f32 } else { -(AUDIO_AMPLITUDE as f32) };
        self.z1 = (1.0 - SPEAKER_LPF_B0) * input;
        self.z2 = (SPEAKER_LPF_B2 - SPEAKER_LPF_A2) * input;
    }

    #[inline(always)]
    fn process(&mut self, input: i16) -> i16 {
        let input = input as f32;
        let output = SPEAKER_LPF_B0 * input + self.z1;
        self.z1 = SPEAKER_LPF_B1 * input - SPEAKER_LPF_A1 * output + self.z2;
        self.z2 = SPEAKER_LPF_B2 * input - SPEAKER_LPF_A2 * output;
        output as i16
    }
}
const PCM_RING_CAPACITY: u16 = 2048;
const PCM_RING_MASK: u16 = PCM_RING_CAPACITY - 1;
const _: () = assert!(PCM_RING_CAPACITY.is_power_of_two());
const _: () = assert!(PACKET_BYTES <= crate::uvc::USB_HS_DMA_BUFFER_LIMIT);

static PCM_HEAD: AtomicU16 = AtomicU16::new(0);
static PCM_TAIL: AtomicU16 = AtomicU16::new(0);

// USB consumer-side underrun smoothing. Gain ramps over 32 samples (~1.33 ms).
const UNDERRUN_GAIN_MAX: i32 = 32_768;
const UNDERRUN_GAIN_STEP: i32 = UNDERRUN_GAIN_MAX / 32;
static OUTPUT_GAIN_Q15: AtomicI32 = AtomicI32::new(0);
static LAST_PCM_SAMPLE: AtomicI32 = AtomicI32::new(0);
static LAST_OUTPUT_SAMPLE: AtomicI32 = AtomicI32::new(0);
static CROSSFADE_START_SAMPLE: AtomicI32 = AtomicI32::new(0);
static UNDERRUN_ACTIVE: AtomicU16 = AtomicU16::new(1);
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
static mut PCM_RING: [i16; PCM_RING_CAPACITY as usize] = [0; PCM_RING_CAPACITY as usize];

unsafe extern "C" {
    safe fn bk_audio_start() -> u8;
    safe fn bk_timing_cycles() -> u32;
    safe fn bk_audio_stop();
    #[cfg(feature = "audio-underrun-diag")]
    safe fn bk_audio_take_edge_overflows() -> u32;
    fn bk_audio_pop_edge(cycles: *mut u32, level: *mut u8) -> u8;
}

pub struct BkEmulatorAudio {
    // Quarter-cycle units preserve exact sample timing; at 24 kHz each sample is 125 PDP cycles.
    next_sample_phase: u32,
    // Fractional phase accumulator allows sub-phase-unit clock corrections.
    next_sample_fraction: u16,
    sample_step_q16: u32,
    last_clock_control_cycle: u32,
    clock_integral_ppm: i32,
    sample_level: bool,
    pending_edge: Option<(u32, bool)>,
    previous_edge: Option<(u32, i32)>,
    edge_queue_empty: bool,
    speaker_filter: SpeakerLowPass,
    active: bool,
}

impl BkEmulatorAudio {
    pub const fn new() -> Self {
        Self {
            next_sample_phase: 0,
            next_sample_fraction: 0,
            sample_step_q16: AUDIO_SAMPLE_PERIOD_PHASE << 16,
            last_clock_control_cycle: 0,
            clock_integral_ppm: 0,
            sample_level: false,
            pending_edge: None,
            previous_edge: None,
            edge_queue_empty: false,
            speaker_filter: SpeakerLowPass::new(),
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
                {
                    UNDERRUN_EVENTS.store(0, Ordering::Relaxed);
                    UNDERRUN_SAMPLES.store(0, Ordering::Relaxed);
                    PCM_SAMPLES_RENDERED.store(0, Ordering::Relaxed);
                    PCM_SAMPLES_CONSUMED.store(0, Ordering::Relaxed);
                    PCM_MIN_OCCUPANCY.store(u32::MAX, Ordering::Relaxed);
                    RENDER_GAP_INITIALIZED.store(false, Ordering::Relaxed);
                    MAX_RENDER_GAP_DWT.store(0, Ordering::Relaxed);
                    FIFO_WRITE_FAILURES.store(0, Ordering::Relaxed);
                }
                self.speaker_filter.reset(self.sample_level);
                self.pending_edge = None;
                self.previous_edge = None;
                let now_cycles = bk_timing_cycles();
                self.next_sample_phase = now_cycles
                    .wrapping_mul(PHASE_UNITS_PER_CYCLE)
                    .wrapping_sub(AUDIO_STARTUP_LATENCY_PHASE + AUDIO_PREFILL_PHASE);
                self.next_sample_fraction = 0;
                self.sample_step_q16 = AUDIO_SAMPLE_PERIOD_PHASE << 16;
                self.last_clock_control_cycle = now_cycles;
                self.clock_integral_ppm = 0;
                OUTPUT_GAIN_Q15.store(0, Ordering::Relaxed);
                LAST_PCM_SAMPLE.store(0, Ordering::Relaxed);
                LAST_OUTPUT_SAMPLE.store(0, Ordering::Relaxed);
                CROSSFADE_START_SAMPLE.store(0, Ordering::Relaxed);
                UNDERRUN_ACTIVE.store(1, Ordering::Relaxed);
                PCM_TAIL.store(0, Ordering::Relaxed);
                PCM_HEAD.store(0, Ordering::Release);
                self.active = true;
            }
        } else if self.active {
            self.active = false;
            self.pending_edge = None;
            self.previous_edge = None;
            self.speaker_filter.reset(false);
            OUTPUT_GAIN_Q15.store(0, Ordering::Relaxed);
            LAST_PCM_SAMPLE.store(0, Ordering::Relaxed);
            LAST_OUTPUT_SAMPLE.store(0, Ordering::Relaxed);
            CROSSFADE_START_SAMPLE.store(0, Ordering::Relaxed);
            UNDERRUN_ACTIVE.store(1, Ordering::Relaxed);
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
        let now_cycles = bk_timing_cycles();
        self.update_sample_rate_control(now_cycles);
        let render_until = now_cycles
            .wrapping_mul(PHASE_UNITS_PER_CYCLE)
            .wrapping_sub(AUDIO_STARTUP_LATENCY_PHASE);
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
                break;
            }

            let sample_phase = self.next_sample_phase;
            let phase_difference = render_until.wrapping_sub(sample_phase) as i32;
            if phase_difference < 0
                || (phase_difference == 0 && self.next_sample_fraction != 0)
            {
                break;
            }

            self.apply_edges_through(sample_phase);
            let raw_sample = if self.sample_level { AUDIO_AMPLITUDE } else { -AUDIO_AMPLITUDE };
            let corrected_sample = raw_sample as i32 + self.blep_edge_correction(sample_phase);
            let corrected_sample = corrected_sample.clamp(-(AUDIO_AMPLITUDE as i32), AUDIO_AMPLITUDE as i32) as i16;
            let sample = self.speaker_filter.process(corrected_sample);
            unsafe {
                core::ptr::addr_of_mut!(PCM_RING)
                    .cast::<i16>()
                    .add((head & PCM_RING_MASK) as usize)
                    .write(sample);
            }
            head = head.wrapping_add(1);
            // Apply a slowly varying Q16 sample interval. The integer phase
            // remains in quarter-cycle units; the fractional part accumulates
            // clock corrections much smaller than one phase unit per sample.
            let phase_step = self.sample_step_q16;
            let fraction = self.next_sample_fraction as u32 + (phase_step & 0xffff);
            self.next_sample_phase = self.next_sample_phase.wrapping_add(
                (phase_step >> 16).wrapping_add(fraction >> 16),
            );
            self.next_sample_fraction = fraction as u16;
            rendered += 1;
        }
        if rendered != 0 {
            PCM_HEAD.store(head, Ordering::Release);
            #[cfg(feature = "audio-underrun-diag")]
            PCM_SAMPLES_RENDERED.fetch_add(rendered as u32, Ordering::Relaxed);
        }
        rendered
    }
    // Occupancy feedback adjusts only the audio resampling clock. It never
    // changes PDP execution speed or the timestamps produced by the emulator.
    fn update_sample_rate_control(&mut self, now_cycles: u32) {
        let elapsed = now_cycles.wrapping_sub(self.last_clock_control_cycle);
        if elapsed < AUDIO_CLOCK_CONTROL_INTERVAL_CYCLES {
            return;
        }

        self.last_clock_control_cycle = now_cycles;
        let intervals = (elapsed / AUDIO_CLOCK_CONTROL_INTERVAL_CYCLES).min(10) as i32;
        let head = PCM_HEAD.load(Ordering::Acquire);
        let tail = PCM_TAIL.load(Ordering::Acquire);
        let occupancy = head.wrapping_sub(tail) as i32;
        let error = occupancy - AUDIO_TARGET_BUFFER_SAMPLES;

        // Positive error means the queue is too full: advance source time
        // slightly faster, producing fewer output samples. Negative error does
        // the opposite. The integral term removes persistent clock mismatch.
        self.clock_integral_ppm = (self.clock_integral_ppm
            + error * AUDIO_CLOCK_KI_PPM_PER_SAMPLE_PER_INTERVAL * intervals)
            .clamp(-AUDIO_CLOCK_CORRECTION_LIMIT_PPM, AUDIO_CLOCK_CORRECTION_LIMIT_PPM);
        let correction_ppm = (error * AUDIO_CLOCK_KP_PPM_PER_SAMPLE
            + self.clock_integral_ppm)
            .clamp(-AUDIO_CLOCK_CORRECTION_LIMIT_PPM, AUDIO_CLOCK_CORRECTION_LIMIT_PPM);
        let nominal_step_q16 = (AUDIO_SAMPLE_PERIOD_PHASE as i64) << 16;
        let corrected_step_q16 = nominal_step_q16
            + nominal_step_q16 * correction_ppm as i64 / 1_000_000;
        self.sample_step_q16 = corrected_step_q16 as u32;
    }

    // Apply a compact polynomial BLEP correction around each speaker edge.
    // The correction is calculated in Q15 to avoid floating-point work here.
    #[inline(always)]
    fn blep_edge_correction(&self, sample_phase: u32) -> i32 {
        let period = AUDIO_SAMPLE_PERIOD_PHASE;
        let mut correction = 0i32;

        if let Some((edge_phase, polarity)) = self.previous_edge {
            let age = sample_phase.wrapping_sub(edge_phase) as i32;
            if age >= 0 && (age as u32) < period {
                let t = ((age as u32 * 32_768) / period) as i32;
                let q = 2 * t - ((t * t) >> 15) - 32_768;
                correction += polarity * AUDIO_AMPLITUDE as i32 * q / 32_768;
            }
        }

        if let Some((edge_cycle, edge_level)) = self.pending_edge {
            let edge_phase = edge_cycle.wrapping_mul(PHASE_UNITS_PER_CYCLE);
            let ahead = edge_phase.wrapping_sub(sample_phase) as i32;
            if ahead > 0 && (ahead as u32) < period {
                let t = (((period - ahead as u32) * 32_768) / period) as i32;
                let q = (t * t) >> 15;
                let polarity = if edge_level { 1 } else { -1 };
                correction += polarity * AUDIO_AMPLITUDE as i32 * q / 32_768;
            }
        }

        correction
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
                self.previous_edge = Some((
                    edge_cycle.wrapping_mul(PHASE_UNITS_PER_CYCLE),
                    if edge_level { 1 } else { -1 },
                ));
                self.pending_edge = None;
            } else {
                break;
            }
        }
    }

}

/// Return and reset audio pipeline diagnostics for the previous interval.
#[cfg(feature = "audio-underrun-diag")]
pub fn take_audio_diagnostics() -> (u32, u32, u16, i32, u16, u32, u32, u32) {
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
    )
}

pub fn note_fifo_write_failure() {
    #[cfg(feature = "audio-underrun-diag")]
    FIFO_WRITE_FAILURES.fetch_add(1, Ordering::Relaxed);
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

/// Copy one USB microframe of pre-rendered PCM, smoothing buffer underruns with short fades.
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
    let mut gain = OUTPUT_GAIN_Q15.load(Ordering::Relaxed);
    let mut last_sample = LAST_PCM_SAMPLE.load(Ordering::Relaxed);
    let mut last_output = LAST_OUTPUT_SAMPLE.load(Ordering::Relaxed);
    let mut crossfade_start = CROSSFADE_START_SAMPLE.load(Ordering::Relaxed);
    let mut underrun_active = UNDERRUN_ACTIVE.load(Ordering::Relaxed) != 0;
    #[cfg(feature = "audio-underrun-diag")]
    PCM_MIN_OCCUPANCY.fetch_min(head.wrapping_sub(tail) as u32, Ordering::Relaxed);
    for index in 0..SAMPLES_PER_MICROFRAME {
        let sample = if tail != head {
            let value = unsafe {
                core::ptr::addr_of!(PCM_RING)
                    .cast::<i16>()
                    .add((tail & PCM_RING_MASK) as usize)
                    .read()
            };
            tail = tail.wrapping_add(1);
            #[cfg(feature = "audio-underrun-diag")]
            {
                PCM_SAMPLES_CONSUMED.fetch_add(1, Ordering::Relaxed);
                PCM_MIN_OCCUPANCY.fetch_min(head.wrapping_sub(tail) as u32, Ordering::Relaxed);
            }

            if underrun_active {
                // Start recovery from the exact last output value, not from zero.
                crossfade_start = last_output;
                gain = 0;
                underrun_active = false;
            }
            last_sample = value as i32;
            gain = (gain + UNDERRUN_GAIN_STEP).min(UNDERRUN_GAIN_MAX);
            // Smoothstep (3t² - 2t³) gives zero slope at both ends,
            // avoiding the slope discontinuity of a linear crossfade.
            let t2 = (gain * gain) >> 15;
            let blend = (t2 * (3 * 32_768 - 2 * gain)) >> 15;
            let blended = crossfade_start
                + (((last_sample - crossfade_start) * blend) >> 15);
            blended.clamp(i16::MIN as i32, i16::MAX as i32) as i16
        } else {
            // On the first missing sample, anchor the fade at the last emitted
            // value. Keep fading only while the producer remains behind.
            if !underrun_active {
                #[cfg(feature = "audio-underrun-diag")]
                UNDERRUN_EVENTS.fetch_add(1, Ordering::Relaxed);
                last_sample = last_output;
                underrun_active = true;
            }
            #[cfg(feature = "audio-underrun-diag")]
            UNDERRUN_SAMPLES.fetch_add(1, Ordering::Relaxed);
            gain = (gain - UNDERRUN_GAIN_STEP).max(0);
            ((last_sample * gain) >> 15) as i16
        };
        let bytes = sample.to_le_bytes();
        last_output = sample as i32;
        buffer[index * 2] = bytes[0];
        buffer[index * 2 + 1] = bytes[1];
    }
    OUTPUT_GAIN_Q15.store(gain, Ordering::Relaxed);
    LAST_PCM_SAMPLE.store(last_sample, Ordering::Relaxed);
    LAST_OUTPUT_SAMPLE.store(last_output, Ordering::Relaxed);
    CROSSFADE_START_SAMPLE.store(crossfade_start, Ordering::Relaxed);
    UNDERRUN_ACTIVE.store(underrun_active as u16, Ordering::Relaxed);
    PCM_TAIL.store(tail, Ordering::Release);
    PACKET_BYTES as u16
}

impl Default for BkEmulatorAudio {
    fn default() -> Self { Self::new() }
}
