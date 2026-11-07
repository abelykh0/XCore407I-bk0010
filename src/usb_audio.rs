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

// 48 kHz samples are 62.5 BK ticks apart at the nominal 3 MHz BK clock.
// Twice-tick units keep this exact: each sample advances by 125 units.
const SAMPLE_STEP_TWICE_TICKS: u64 = 125;
const STARTUP_LATENCY_TICKS: u64 = 30_000; // 10 ms at 3 MHz
const PCM_RING_CAPACITY: u32 = 1_024;
const PCM_RING_MASK: u32 = PCM_RING_CAPACITY - 1;
const PCM_PREFILL_SAMPLES: u32 = 256;
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
    sample_level: bool,
    pending_edge: Option<(u64, bool)>,
}

impl AudioPcmProducer {
    pub const fn new() -> Self {
        Self {
            epoch: 0,
            initialized: false,
            next_sample_time2: 0,
            sample_level: false,
            pending_edge: None,
        }
    }

    fn reset_for_epoch(&mut self, epoch: u32, current_ticks: u64) {
        // The ISR clears PCM_READY before publishing a new epoch, so it will
        // not touch ring indices while they are being reinitialized here.
        PCM_HEAD.store(0, Ordering::Relaxed);
        PCM_TAIL.store(0, Ordering::Relaxed);
        PCM_UNDERRUNS.store(0, Ordering::Relaxed);
        PCM_OVERFLOWS.store(0, Ordering::Relaxed);
        self.epoch = epoch;
        self.initialized = true;
        self.sample_level = PCM_START_LEVEL.load(Ordering::Acquire) != 0;
        self.pending_edge = None;

        // Start 10 ms behind the current emulated time. This prefills the ring
        // with the current speaker level and gives the consumer jitter margin.
        self.next_sample_time2 =
            current_ticks.saturating_sub(STARTUP_LATENCY_TICKS).saturating_mul(2);
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

            self.next_sample_time2 =
                self.next_sample_time2.saturating_add(SAMPLE_STEP_TWICE_TICKS);
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
    sample_level: bool,
    active: bool,
    dma_in_flight: bool,
    last_sample: i16,
    dma_packet: DmaPacket,
}

impl BkEmulatorAudio {
    const RATES: [RangeEntry<u32>; 1] = [RangeEntry::new_fixed(AUDIO_SAMPLE_RATE)];

    pub const fn new() -> Self {
        Self {
            sample_level: false,
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
        for i in 0..SAMPLES_PER_MICROFRAME {
            let offset = i * 2;
            packet[offset..offset + 2].copy_from_slice(&last_sample.to_le_bytes());
        }
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
        packet[offset..offset + 2].copy_from_slice(&last_sample.to_le_bytes());
    }

    if available != 0 {
        PCM_TAIL.store(tail.wrapping_add(available), Ordering::Release);
    }
    let missing = SAMPLES_PER_MICROFRAME as u32 - available;
    if missing != 0 {
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
            self.sample_level = unsafe { bk_audio_start() != 0 };
            self.last_sample = if self.sample_level {
                AUDIO_AMPLITUDE
            } else {
                -AUDIO_AMPLITUDE
            };
            self.dma_in_flight = false;
            PCM_READY.store(false, Ordering::Release);
            PCM_START_LEVEL.store(self.sample_level as u8, Ordering::Relaxed);
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
