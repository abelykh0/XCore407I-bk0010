//! BK-0010 speaker-edge capture and deferred PCM16 rendering for TinyUSB UAC2.
//!
//! The emulator publishes timestamped speaker transitions. The main-loop audio
//! worker renders raw 24 kHz PCM into a ring buffer. The USB completion ISR
//! resamples that PCM and applies BLEP using the original edge timestamps.

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
const _: () = assert!(PACKET_BYTES <= crate::usb_device::uvc::USB_HS_DMA_BUFFER_LIMIT);

static PCM_HEAD: AtomicU16 = AtomicU16::new(0);
static PCM_TAIL: AtomicU16 = AtomicU16::new(0);
static mut PCM_RING: [i16; PCM_RING_CAPACITY as usize] = [0; PCM_RING_CAPACITY as usize];

#[inline(always)]
fn pcm_ring_sample(index: u16) -> i32 {
    unsafe {
        core::ptr::addr_of!(PCM_RING)
            .cast::<i16>()
            .add((index & PCM_RING_MASK) as usize)
            .read() as i32
    }
}

const RESAMPLER_ONE: u32 = 1 << 16;
const RESAMPLER_TARGET_OCCUPANCY: i32 = 72;
const RESAMPLER_CONTROL_PACKETS: u8 = 80;
const RESAMPLER_CORRECTION_LIMIT_PPM: i32 = 5_000;

#[derive(Clone, Copy)]
struct PcmResampler {
    phase: u32,
    step: u32,
    control_packets: u8,
    source_phase: u32,
    last_sample: i16,
    previous_edge: Option<(u32, i32)>,
    pending_edge: Option<(u32, bool)>,
}

impl PcmResampler {
    const fn new(source_phase: u32) -> Self {
        Self {
            phase: 0,
            step: RESAMPLER_ONE,
            control_packets: 0,
            source_phase,
            last_sample: 0,
            previous_edge: None,
            pending_edge: None,
        }
    }

    #[inline]
    fn update_rate(&mut self, occupancy: u16) {
        self.control_packets += 1;
        if self.control_packets < RESAMPLER_CONTROL_PACKETS {
            return;
        }
        self.control_packets = 0;
        let error = occupancy as i32 - RESAMPLER_TARGET_OCCUPANCY;
        let correction_ppm = (error * 4)
            .clamp(-RESAMPLER_CORRECTION_LIMIT_PPM, RESAMPLER_CORRECTION_LIMIT_PPM);
        self.step = ((RESAMPLER_ONE as i64 * (1_000_000 + correction_ppm) as i64)
            / 1_000_000) as u32;
    }

    #[inline(always)]
    fn update_blep_edges(&mut self, sample_phase: u32) {
        loop {
            if self.pending_edge.is_none() {
                let mut cycles = 0u32;
                let mut level = 0u8;
                if unsafe { bk_audio_pop_blep_edge(&mut cycles, &mut level) } == 0 {
                    break;
                }
                self.pending_edge = Some((cycles, level != 0));
            }

            let (edge_cycle, edge_level) = self.pending_edge.unwrap();
            let edge_phase = edge_cycle.wrapping_mul(PHASE_UNITS_PER_CYCLE);
            if sample_phase.wrapping_sub(edge_phase) as i32 >= 0 {
                self.previous_edge = Some((
                    edge_phase,
                    if edge_level { 1 } else { -1 },
                ));
                self.pending_edge = None;
            } else {
                break;
            }
        }
    }

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
}

static mut PCM_RESAMPLER: PcmResampler = PcmResampler::new(0);

unsafe extern "C" {
    safe fn bk_audio_start() -> u8;
    safe fn bk_timing_cycles() -> u32;
    safe fn bk_audio_stop();
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
                self.pending_edge = None;
                self.next_sample_phase = Self::cycle_phase()
                    .wrapping_sub(AUDIO_STARTUP_LATENCY_PHASE + AUDIO_PREFILL_PHASE);
                unsafe {
                    core::ptr::addr_of_mut!(PCM_RESAMPLER)
                        .write(PcmResampler::new(self.next_sample_phase));
                }
                PCM_TAIL.store(0, Ordering::Relaxed);
                PCM_HEAD.store(0, Ordering::Release);
                self.active = true;
            }
        } else if self.active {
            self.active = false;
            self.pending_edge = None;
            unsafe {
                core::ptr::addr_of_mut!(PCM_RESAMPLER).write(PcmResampler::new(0));
            }
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
            let sample = if self.sample_level { AUDIO_AMPLITUDE } else { -AUDIO_AMPLITUDE };
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

/// Resample the producer PCM and apply timestamped BLEP correction at the
/// final USB sample positions. The existing PCM and edge rings are reused.
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

    for index in 0..SAMPLES_PER_MICROFRAME {
        let available = head.wrapping_sub(tail);
        let mut output = resampler.last_sample;

        if available >= 2 {
            let sample_phase = resampler.source_phase
                .wrapping_add((resampler.phase * AUDIO_SAMPLE_PERIOD_PHASE) >> 16);
            resampler.update_blep_edges(sample_phase);
            // Use the rendered PCM as the signal; timestamped edges provide
            // only the BLEP correction at the resampled output time.
            let raw = pcm_ring_sample(tail);
            output = (raw + resampler.blep_edge_correction(sample_phase))
                .clamp(-(AUDIO_AMPLITUDE as i32), AUDIO_AMPLITUDE as i32) as i16;

            let phase_total = resampler.phase.wrapping_add(resampler.step);
            let advance = phase_total >> 16;
            if advance == 0 {
                resampler.phase = phase_total;
            } else if available > advance as u16 + 1 {
                resampler.phase = phase_total & 0xFFFF;
                tail = tail.wrapping_add(advance as u16);
                resampler.source_phase = resampler.source_phase
                    .wrapping_add(advance.wrapping_mul(AUDIO_SAMPLE_PERIOD_PHASE));
            }
        } else if available == 1 {
            let sample_phase = resampler.source_phase
                .wrapping_add((resampler.phase * AUDIO_SAMPLE_PERIOD_PHASE) >> 16);
            resampler.update_blep_edges(sample_phase);
            let raw = pcm_ring_sample(tail);
            output = (raw + resampler.blep_edge_correction(sample_phase))
                .clamp(-(AUDIO_AMPLITUDE as i32), AUDIO_AMPLITUDE as i32) as i16;
        }

        resampler.last_sample = output;
        let bytes = output.to_le_bytes();
        buffer[index * 2] = bytes[0];
        buffer[index * 2 + 1] = bytes[1];
    }

    PCM_TAIL.store(tail, Ordering::Release);
    unsafe {
        core::ptr::addr_of_mut!(PCM_RESAMPLER).write(resampler);
    }
    PACKET_BYTES as u16
}

impl Default for BkEmulatorAudio {
    fn default() -> Self { Self::new() }
}
