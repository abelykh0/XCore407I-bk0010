//! UAC2 capture source connected to the BK-0010 emulator speaker output.
//!
//! Speaker transitions are timestamped in integral BK PDP ticks. The main-loop
//! producer converts those transitions to 48 kHz PCM and queues samples in an
//! SPSC ring. TIM2 consumes six queued samples per 8 kHz USB microframe.

use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
use usb_device::bus::UsbBus;
use usb_device::endpoint::{Endpoint, In, Out};
use usb_device::UsbDirection;
use usbd_uac2::descriptors::ClockType;
use usbd_uac2::{
    AudioHandler, ClockSource, RangeEntry, UsbAudioClassError, UsbIsochronousFeedback,
};

const AUDIO_SAMPLE_RATE: u32 = 48_000;
const SAMPLES_PER_MICROFRAME: usize = 6;
const PACKET_BYTES: usize = SAMPLES_PER_MICROFRAME * 2;
const AUDIO_AMPLITUDE: i16 = 12_000;

// The PCM clock follows the measured BK tick rate. Twice-tick units
// preserve half-tick sample positions; a remainder accumulator distributes
// fractional intervals without long-term rounding drift.
const DEFAULT_BK_TICKS_PER_SECOND: u32 = 3_000_000;
const MIN_BK_TICKS_PER_SECOND: u32 = 1_000_000;
const MAX_BK_TICKS_PER_SECOND: u32 = 6_000_000;
const PCM_RATE_DIVISOR: u64 = AUDIO_SAMPLE_RATE as u64;
const PCM_RING_CAPACITY: u32 = 1_024;
const PCM_RING_MASK: u32 = PCM_RING_CAPACITY - 1;
const PCM_PREFILL_SAMPLES: u32 = 256;
const PCM_TARGET_QUEUED_SAMPLES: u32 = 480;
const PCM_RATE_SCALE: u64 = 1_000_000;
const PCM_RATE_ADJUST_PER_SAMPLE_PPM: i32 = 250;
const PCM_MAX_RATE_ADJUST_PPM: i32 = 150_000;
const MAX_PRODUCE_PER_CALL: u32 = 256;

unsafe extern "C" {
    fn bk_audio_start() -> u8;
    fn bk_audio_stop();
    fn bk_audio_pop_edge(edge_ticks: *mut u64, level: *mut u8) -> u8;
    fn bk_audio_get_edge_overflows() -> u32;
}

static PCM_HEAD: AtomicU32 = AtomicU32::new(0);
static PCM_TAIL: AtomicU32 = AtomicU32::new(0);
static PCM_EPOCH: AtomicU32 = AtomicU32::new(0);
static PCM_BK_TICKS_PER_SECOND: AtomicU32 = AtomicU32::new(DEFAULT_BK_TICKS_PER_SECOND);
static PCM_START_LEVEL: AtomicU8 = AtomicU8::new(0);
static PCM_ACTIVE: AtomicBool = AtomicBool::new(false);
static PCM_READY: AtomicBool = AtomicBool::new(false);
static PCM_UNDERRUNS: AtomicU32 = AtomicU32::new(0);
static PCM_OVERFLOWS: AtomicU32 = AtomicU32::new(0);
static PCM_USB_INCOMPLETE: AtomicU32 = AtomicU32::new(0);
static PCM_USB_SUBMIT_ERRORS: AtomicU32 = AtomicU32::new(0);

// Single producer: main loop. Single consumer: TIM2 USB ISR.
static mut PCM_RING: [i16; PCM_RING_CAPACITY as usize] =
    [0; PCM_RING_CAPACITY as usize];

/// Stateful PCM producer; keep this in main-loop context, never in the ISR.
pub struct AudioPcmProducer {
    epoch: u32,
    initialized: bool,
    next_sample_time2: u64,
    sample_remainder: u64,
    sample_level: bool,
    pending_edge: Option<(u64, bool)>,
    filtered_queue_q8: u32,
    last_control_ticks: u64,
    rate_adjust_ppm: i32,
}

impl AudioPcmProducer {
    pub const fn new() -> Self {
        Self {
            epoch: 0,
            initialized: false,
            next_sample_time2: 0,
            sample_remainder: 0,
            sample_level: false,
            pending_edge: None,
            filtered_queue_q8: PCM_TARGET_QUEUED_SAMPLES << 8,
            last_control_ticks: 0,
            rate_adjust_ppm: 0,
        }
    }

    fn reset_for_epoch(&mut self, epoch: u32, current_ticks: u64) {
        // The ISR clears PCM_READY before publishing a new epoch, so it will
        // not touch ring indices while they are being reinitialized here.
        PCM_HEAD.store(0, Ordering::Relaxed);
        PCM_TAIL.store(0, Ordering::Relaxed);
        PCM_UNDERRUNS.store(0, Ordering::Relaxed);
        PCM_OVERFLOWS.store(0, Ordering::Relaxed);
        PCM_USB_INCOMPLETE.store(0, Ordering::Relaxed);
        PCM_USB_SUBMIT_ERRORS.store(0, Ordering::Relaxed);
        self.epoch = epoch;
        self.initialized = true;
        self.sample_level = PCM_START_LEVEL.load(Ordering::Acquire) != 0;
        self.pending_edge = None;

        // Start 10 ms behind the current emulated time. This prefills the ring
        // with the current speaker level and gives the consumer jitter margin.
        let tick_rate = PCM_BK_TICKS_PER_SECOND.load(Ordering::Acquire) as u64;
        let startup_latency = tick_rate / 100;
        self.next_sample_time2 =
            current_ticks.saturating_sub(startup_latency).saturating_mul(2);
        self.sample_remainder = 0;
        self.filtered_queue_q8 = PCM_TARGET_QUEUED_SAMPLES << 8;
        self.last_control_ticks = current_ticks;
        self.rate_adjust_ppm = 0;
    }

    fn apply_edges_through(&mut self, sample_time2: u64) {
        loop {
            if self.pending_edge.is_none() {
                let mut edge_ticks = 0u64;
                let mut level = 0u8;
                if unsafe { bk_audio_pop_edge(&mut edge_ticks, &mut level) } == 0 {
                    break;
                }
                self.pending_edge = Some((edge_ticks.saturating_mul(2), level != 0));
            }

            let (edge_time2, edge_level) = self.pending_edge.unwrap();
            if edge_time2 <= sample_time2 {
                self.sample_level = edge_level;
                self.pending_edge = None;
            } else {
                break;
            }
        }
    }

    /// Update the PCM timebase from the once-per-second measured emulator rate.
    /// Values outside a plausible range are ignored to avoid corrupting timing
    /// if the diagnostic measurement is temporarily invalid.
    pub fn set_tick_rate(&mut self, ticks_per_second: u32) {
        if (MIN_BK_TICKS_PER_SECOND..=MAX_BK_TICKS_PER_SECOND)
            .contains(&ticks_per_second)
        {
            PCM_BK_TICKS_PER_SECOND.store(ticks_per_second, Ordering::Release);
        }
    }

    /// Current queue-controller correction, in parts per million.
    pub fn rate_adjust_ppm(&self) -> i32 {
        self.rate_adjust_ppm
    }

    /// Convert all currently elapsed BK time into PCM, bounded per call so
    /// video/USB servicing remains responsive.
    pub fn produce(&mut self, current_ticks: u64) {
        if !PCM_ACTIVE.load(Ordering::Acquire) {
            self.initialized = false;
            return;
        }

        let epoch = PCM_EPOCH.load(Ordering::Acquire);
        if !self.initialized || epoch != self.epoch {
            self.reset_for_epoch(epoch, current_ticks);
        }

        let current_time2 = current_ticks.saturating_mul(2);
        let base_tick_rate = PCM_BK_TICKS_PER_SECOND.load(Ordering::Acquire)
            .clamp(MIN_BK_TICKS_PER_SECOND, MAX_BK_TICKS_PER_SECOND) as u64;

        // Close the loop around the actual USB queue depth. A high queue means
        // the PCM producer is advancing through BK time too slowly, so increase
        // the effective ticks/sample. A low queue does the opposite. Update at
        // approximately 1 ms intervals and filter occupancy to avoid pitch jitter.
        let control_interval = (base_tick_rate / 1_000).max(1);
        if current_ticks.wrapping_sub(self.last_control_ticks) >= control_interval {
            self.last_control_ticks = current_ticks;
            let head = PCM_HEAD.load(Ordering::Acquire);
            let tail = PCM_TAIL.load(Ordering::Acquire);
            let queued = head.wrapping_sub(tail).min(PCM_RING_CAPACITY);
            let queued_q8 = queued << 8;
            let target_q8 = PCM_TARGET_QUEUED_SAMPLES << 8;
            if queued_q8 >= self.filtered_queue_q8 {
                self.filtered_queue_q8 += (queued_q8 - self.filtered_queue_q8) / 8;
            } else {
                self.filtered_queue_q8 -= (self.filtered_queue_q8 - queued_q8) / 8;
            }

            let error_samples =
                (self.filtered_queue_q8 as i64 - target_q8 as i64) / 256;
            let adjustment = (error_samples as i32)
                .saturating_mul(PCM_RATE_ADJUST_PER_SAMPLE_PPM);
            self.rate_adjust_ppm =
                adjustment.clamp(-PCM_MAX_RATE_ADJUST_PPM, PCM_MAX_RATE_ADJUST_PPM);
        }

        let rate_scale = (PCM_RATE_SCALE as i64 + self.rate_adjust_ppm as i64) as u64;
        let effective_tick_rate = base_tick_rate.saturating_mul(rate_scale) / PCM_RATE_SCALE;
        let sample_step_numerator = effective_tick_rate * 2;
        let mut produced = 0u32;

        while self.next_sample_time2 <= current_time2
            && produced < MAX_PRODUCE_PER_CALL
        {
            self.apply_edges_through(self.next_sample_time2);
            let sample = if self.sample_level {
                AUDIO_AMPLITUDE
            } else {
                -AUDIO_AMPLITUDE
            };

            let head = PCM_HEAD.load(Ordering::Relaxed);
            let tail = PCM_TAIL.load(Ordering::Acquire);
            if head.wrapping_sub(tail) < PCM_RING_CAPACITY {
                let slot = (head & PCM_RING_MASK) as usize;
                unsafe {
                    ptr::addr_of_mut!(PCM_RING).cast::<i16>().add(slot).write(sample);
                }
                PCM_HEAD.store(head.wrapping_add(1), Ordering::Release);
            } else {
                // Keep the emulated sample clock and edge queue moving even if
                // the emulator temporarily outruns the USB consumer.
                PCM_OVERFLOWS.fetch_add(1, Ordering::Relaxed);
            }

            // Exact average interval: (2 * BK ticks/sec) / 48,000.
            // Carry the remainder so fractional tick intervals do not drift.
            let interval_numerator = self.sample_remainder + sample_step_numerator;
            let interval_twice_ticks = interval_numerator / PCM_RATE_DIVISOR;
            self.sample_remainder = interval_numerator % PCM_RATE_DIVISOR;
            self.next_sample_time2 =
                self.next_sample_time2.saturating_add(interval_twice_ticks);
            produced += 1;
        }

        let head = PCM_HEAD.load(Ordering::Acquire);
        let tail = PCM_TAIL.load(Ordering::Acquire);
        if head.wrapping_sub(tail) >= PCM_PREFILL_SAMPLES {
            PCM_READY.store(true, Ordering::Release);
        }
    }
}

impl Default for AudioPcmProducer {
    fn default() -> Self {
        Self::new()
    }
}

#[repr(align(4))]
struct DmaPacket([u32; PACKET_BYTES / 4]);

pub struct BkEmulatorAudio {
    active: bool,
    dma_in_flight: bool,
    last_sample: i16,
    dma_packet: DmaPacket,
}

impl BkEmulatorAudio {
    const RATES: [RangeEntry<u32>; 1] = [RangeEntry::new_fixed(AUDIO_SAMPLE_RATE)];

    pub const fn new() -> Self {
        Self {
            active: false,
            dma_in_flight: false,
            last_sample: -AUDIO_AMPLITUDE,
            dma_packet: DmaPacket([0; PACKET_BYTES / 4]),
        }
    }

    /// (PCM underrun samples, PCM overflow samples, dropped speaker edges,
    /// USB incomplete transfers, USB submit errors, queued PCM samples).
    pub fn diagnostics() -> (u32, u32, u32, u32, u32, u32) {
        let head = PCM_HEAD.load(Ordering::Acquire);
        let tail = PCM_TAIL.load(Ordering::Acquire);
        (
            PCM_UNDERRUNS.load(Ordering::Relaxed),
            PCM_OVERFLOWS.load(Ordering::Relaxed),
            unsafe { bk_audio_get_edge_overflows() },
            PCM_USB_INCOMPLETE.load(Ordering::Relaxed),
            PCM_USB_SUBMIT_ERRORS.load(Ordering::Relaxed),
            head.wrapping_sub(tail),
        )
    }

fn fill_packet_from_ring(last_sample: &mut i16, packet: &mut [u8]) {
    if !PCM_READY.load(Ordering::Acquire) {
        // No valid PCM has been queued yet: transmit digital silence, not a
        // stale speaker level left over from a previous stream epoch.
        *last_sample = 0;
        packet.fill(0);
        return;
    }

    let tail = PCM_TAIL.load(Ordering::Relaxed);
    let head = PCM_HEAD.load(Ordering::Acquire);
    let available = head.wrapping_sub(tail).min(SAMPLES_PER_MICROFRAME as u32);
    for i in 0..available as usize {
        let slot = (tail.wrapping_add(i as u32) & PCM_RING_MASK) as usize;
        let sample = unsafe {
            ptr::addr_of!(PCM_RING).cast::<i16>().add(slot).read()
        };
        *last_sample = sample;
        let offset = i * 2;
        packet[offset..offset + 2].copy_from_slice(&sample.to_le_bytes());
    }

    for i in available as usize..SAMPLES_PER_MICROFRAME {
        let offset = i * 2;
        packet[offset..offset + 2].fill(0);
    }

    if available != 0 {
        PCM_TAIL.store(tail.wrapping_add(available), Ordering::Release);
    }
    let missing = SAMPLES_PER_MICROFRAME as u32 - available;
    if missing != 0 {
        // An empty ring is silence. Do not hold the previous square-wave level
        // indefinitely, which can leave a DC offset at the audio output.
        *last_sample = 0;
        PCM_UNDERRUNS.fetch_add(missing, Ordering::Relaxed);
    }
}
}

impl Default for BkEmulatorAudio {
    fn default() -> Self {
        Self::new()
    }
}

impl ClockSource for BkEmulatorAudio {
    const CLOCK_TYPE: ClockType = ClockType::InternalFixed;
    const SOF_SYNC: bool = false;

    fn sample_rate(&self) -> u32 {
        AUDIO_SAMPLE_RATE
    }

    fn sample_rates(&self) -> core::result::Result<&[RangeEntry<u32>], UsbAudioClassError> {
        Ok(&Self::RATES)
    }

    fn clock_validity(&self) -> core::result::Result<bool, UsbAudioClassError> {
        Ok(true)
    }
}

impl<'a, B: UsbBus> AudioHandler<'a, B> for BkEmulatorAudio {
    fn audio_data_rx(&mut self, _ep: &Endpoint<'a, B, Out>) {
        // Capture-only stream; there is no USB playback path.
    }

    fn audio_data_tx(&mut self, ep: &Endpoint<'a, B, In>) {
        if !self.active || self.dma_in_flight {
            return;
        }

        let packet = unsafe {
            core::slice::from_raw_parts_mut(
                self.dma_packet.0.as_mut_ptr().cast::<u8>(),
                PACKET_BYTES,
            )
        };
        Self::fill_packet_from_ring(&mut self.last_sample, packet);

        // Persistent DMA-visible SRAM; do not modify until completion callback.
        let dma_bytes = unsafe {
            core::slice::from_raw_parts(
                self.dma_packet.0.as_ptr().cast::<u8>(),
                PACKET_BYTES,
            )
        };
        match unsafe { ep.write_iso_batch_dma(dma_bytes, 1, 1) } {
            Ok(_) => self.dma_in_flight = true,
            Err(_) => {
                PCM_USB_SUBMIT_ERRORS.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    fn audio_data_tx_complete(&mut self) {
        self.dma_in_flight = false;
    }

    fn audio_data_tx_incomplete(&mut self) {
        self.dma_in_flight = false;
        PCM_USB_INCOMPLETE.fetch_add(1, Ordering::Relaxed);
    }

    fn feedback(&mut self, _nominal: UsbIsochronousFeedback) -> Option<UsbIsochronousFeedback> {
        None
    }

    fn alternate_setting_changed(&mut self, terminal: UsbDirection, alt_setting: u8) {
        if terminal != UsbDirection::In {
            return;
        }

        if alt_setting == 1 {
            let start_level = unsafe { bk_audio_start() != 0 };
            self.last_sample = if start_level {
                AUDIO_AMPLITUDE
            } else {
                -AUDIO_AMPLITUDE
            };
            self.dma_in_flight = false;
            PCM_READY.store(false, Ordering::Release);
            PCM_START_LEVEL.store(start_level as u8, Ordering::Relaxed);
            PCM_ACTIVE.store(true, Ordering::Release);
            PCM_EPOCH.fetch_add(1, Ordering::AcqRel);
            self.active = true;
        } else {
            self.active = false;
            PCM_ACTIVE.store(false, Ordering::Release);
            PCM_READY.store(false, Ordering::Release);
            self.dma_in_flight = false;
            unsafe {
                bk_audio_stop();
            }
        }
    }
}
