//! BK-0010 speaker-edge capture and deferred PCM16 rendering for TinyUSB UAC2.
//!
//! The emulator publishes timestamped speaker transitions. The main-loop audio
//! worker converts them to nominal 24 kHz PCM in a ring buffer. The USB
//! completion ISR dynamically resamples that ring to the fixed USB sample rate.

use core::sync::atomic::{AtomicU16, Ordering};

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
static mut PCM_RING: [i16; PCM_RING_CAPACITY as usize] = [0; PCM_RING_CAPACITY as usize];
// Keep BLEP separate so it is added after source samples are resampled.
static mut BLEP_RING: [i16; PCM_RING_CAPACITY as usize] = [0; PCM_RING_CAPACITY as usize];

const RESAMPLER_ONE: u32 = 1 << 16;
const RESAMPLER_MAX_CORRECTION: i32 = 328; // approximately +/-0.5%
const RESAMPLER_TARGET_LEVEL: i32 = 128; // about 5.3 ms at 24 kHz

#[derive(Clone, Copy)]
struct PcmResampler {
    phase: u32,
    step: u32,
}

impl PcmResampler {
    const fn new() -> Self {
        Self { phase: 0, step: RESAMPLER_ONE }
    }
}

// Accessed by the single USB audio consumer. Stream start/stop resets it with
// USB interrupts masked, so the main-loop producer never races this state.
static mut PCM_RESAMPLER: PcmResampler = PcmResampler::new();

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
    previous_edge: Option<(u32, i32)>,
    edge_queue_empty: bool,
    active: bool,
}

impl BkEmulatorAudio {
    pub const fn new() -> Self {
        Self {
            next_sample_phase: 0,
            sample_level: false,
            pending_edge: None,
            previous_edge: None,
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
                self.pending_edge = None;
                self.previous_edge = None;
                self.next_sample_phase = Self::cycle_phase()
                    .wrapping_sub(AUDIO_STARTUP_LATENCY_PHASE + AUDIO_PREFILL_PHASE);
                PCM_TAIL.store(0, Ordering::Relaxed);
                PCM_HEAD.store(0, Ordering::Release);
                unsafe {
                    core::ptr::addr_of_mut!(PCM_RESAMPLER).write(PcmResampler::new());
                }
                self.active = true;
            }
        } else if self.active {
            self.active = false;
            self.pending_edge = None;
            self.previous_edge = None;
            bk_audio_stop();
            PCM_TAIL.store(0, Ordering::Relaxed);
            PCM_HEAD.store(0, Ordering::Release);
            unsafe {
                core::ptr::addr_of_mut!(PCM_RESAMPLER).write(PcmResampler::new());
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
            let blep_sample = self.blep_edge_correction(sample_phase)
                .clamp(-(AUDIO_AMPLITUDE as i32), AUDIO_AMPLITUDE as i32) as i16;
            unsafe {
                core::ptr::addr_of_mut!(PCM_RING)
                    .cast::<i16>()
                    .add((head & PCM_RING_MASK) as usize)
                    .write(raw_sample);
                core::ptr::addr_of_mut!(BLEP_RING)
                    .cast::<i16>()
                    .add((head & PCM_RING_MASK) as usize)
                    .write(blep_sample);
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

/// Resample one USB microframe from the emulator PCM ring at a controlled
/// rate. USB always receives the fixed number of samples for this microframe.
///
/// Buffer occupancy adjusts the source read step: a high level speeds up
/// consumption slightly; a low level slows it down. Linear interpolation
/// smooths the fractional source positions.
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
    // Diagnostic isolation: keep the resampler at exactly 1.0.
    // This preserves fractional interpolation but disables occupancy feedback.
    let mut resampler = unsafe { core::ptr::addr_of!(PCM_RESAMPLER).read() };
    resampler.step = RESAMPLER_ONE;

    for index in 0..SAMPLES_PER_MICROFRAME {
        let available = head.wrapping_sub(tail);
        let sample = if available != 0 {
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
            let b0 = unsafe {
                core::ptr::addr_of!(BLEP_RING)
                    .cast::<i16>()
                    .add((tail & PCM_RING_MASK) as usize)
                    .read()
            };
            let b1 = if available > 1 {
                unsafe {
                    core::ptr::addr_of!(BLEP_RING)
                        .cast::<i16>()
                        .add((tail.wrapping_add(1) & PCM_RING_MASK) as usize)
                        .read()
                }
            } else {
                b0
            };

            let fraction = (resampler.phase & 0xFFFF) as i32;
            let resampled_raw = s0 as i32
                + (((s1 as i32 - s0 as i32) * fraction) >> 16);
            let resampled_blep = b0 as i32
                + (((b1 as i32 - b0 as i32) * fraction) >> 16);
            // Apply the band-limiting correction in the output sample domain.
            let output_sample = (resampled_raw + resampled_blep)
                .clamp(-(AUDIO_AMPLITUDE as i32), AUDIO_AMPLITUDE as i32);

            resampler.phase = resampler.phase.wrapping_add(resampler.step);
            let consumed = (resampler.phase >> 16) as u16;
            if consumed > available {
                // Never let the consumer cursor pass the published producer
                // head. This can occur at the upper rate limit when only one
                // source sample remains.
                tail = tail.wrapping_add(available);
                resampler.phase = 0;
            } else {
                tail = tail.wrapping_add(consumed);
                resampler.phase &= 0xFFFF;
            }
            output_sample as i16
        } else {
            // Do not advance the source cursor while empty; the next valid
            // sample starts a fresh interpolation interval.
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
    PACKET_BYTES as u16
}

impl Default for BkEmulatorAudio {
    fn default() -> Self { Self::new() }
}
