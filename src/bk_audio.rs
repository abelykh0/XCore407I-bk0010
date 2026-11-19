//! BK-0010 speaker-edge capture and deferred PCM16 rendering for TinyUSB UAC2.
//!
//! The emulator publishes timestamped speaker transitions. The main-loop audio
//! worker converts them to 24 kHz PCM in a ring buffer. The USB completion ISR
//! only copies three already-rendered samples from that ring.

use core::sync::atomic::{AtomicI32, AtomicU16, Ordering};

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
static mut PCM_RING: [i16; PCM_RING_CAPACITY as usize] = [0; PCM_RING_CAPACITY as usize];

unsafe extern "C" {
    safe fn bk_audio_start() -> u8;
    safe fn bk_timing_cycles() -> u32;
    safe fn bk_audio_stop();
    fn bk_audio_pop_edge(cycles: *mut u32, level: *mut u8) -> u8;
}

pub struct BkEmulatorAudio {
    // Quarter-cycle units preserve exact sample timing; at 24 kHz each sample is 125 PDP cycles.
    next_sample_phase: u32,
    sample_level: bool,
    pending_edge: Option<(u32, bool)>,
    edge_queue_empty: bool,
    speaker_filter: SpeakerLowPass,
    active: bool,
}

impl BkEmulatorAudio {
    pub const fn new() -> Self {
        Self {
            next_sample_phase: 0,
            sample_level: false,
            pending_edge: None,
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
                self.speaker_filter.reset(self.sample_level);
                self.pending_edge = None;
                self.next_sample_phase = Self::cycle_phase()
                    .wrapping_sub(AUDIO_STARTUP_LATENCY_PHASE + AUDIO_PREFILL_PHASE);
                OUTPUT_GAIN_Q15.store(0, Ordering::Relaxed);
                LAST_PCM_SAMPLE.store(0, Ordering::Relaxed);
                PCM_TAIL.store(0, Ordering::Relaxed);
                PCM_HEAD.store(0, Ordering::Release);
                self.active = true;
            }
        } else if self.active {
            self.active = false;
            self.pending_edge = None;
            self.speaker_filter.reset(false);
            OUTPUT_GAIN_Q15.store(0, Ordering::Relaxed);
            LAST_PCM_SAMPLE.store(0, Ordering::Relaxed);
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
                break;
            }

            let sample_phase = self.next_sample_phase;
            if (render_until.wrapping_sub(sample_phase) as i32) < 0 {
                break;
            }

            self.apply_edges_through(sample_phase);
            let raw_sample = if self.sample_level { AUDIO_AMPLITUDE } else { -AUDIO_AMPLITUDE };
            let sample = self.speaker_filter.process(raw_sample);
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
    for index in 0..SAMPLES_PER_MICROFRAME {
        let sample = if tail != head {
            let value = unsafe {
                core::ptr::addr_of!(PCM_RING)
                    .cast::<i16>()
                    .add((tail & PCM_RING_MASK) as usize)
                    .read()
            };
            tail = tail.wrapping_add(1);
            last_sample = value as i32;
            gain = (gain + UNDERRUN_GAIN_STEP).min(UNDERRUN_GAIN_MAX);
            ((last_sample * gain) >> 15) as i16
        } else {
            // Hold the last sample briefly while fading its amplitude to zero.
            // This avoids an abrupt discontinuity when the PCM producer falls behind.
            gain = (gain - UNDERRUN_GAIN_STEP).max(0);
            ((last_sample * gain) >> 15) as i16
        };
        let bytes = sample.to_le_bytes();
        buffer[index * 2] = bytes[0];
        buffer[index * 2 + 1] = bytes[1];
    }
    OUTPUT_GAIN_Q15.store(gain, Ordering::Relaxed);
    LAST_PCM_SAMPLE.store(last_sample, Ordering::Relaxed);
    PCM_TAIL.store(tail, Ordering::Release);
    PACKET_BYTES as u16
}

impl Default for BkEmulatorAudio {
    fn default() -> Self { Self::new() }
}
