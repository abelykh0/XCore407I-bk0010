//! UAC2 capture test source: a precomputed 1 kHz tone, independent of the BK emulator.
//!
//! The main-loop producer copies samples from a fixed lookup table into an SPSC
//! ring. TIM2 consumes six prepared samples per 8 kHz USB microframe.

use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
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
const TONE_TABLE: [i16; 48] = [0, 1566, 3106, 4592, 6000, 7305, 8485, 9520, 10392, 11087, 11591, 11897, 12000, 11897, 11591, 11087, 10392, 9520, 8485, 7305, 6000, 4592, 3106, 1566, 0, -1566, -3106, -4592, -6000, -7305, -8485, -9520, -10392, -11087, -11591, -11897, -12000, -11897, -11591, -11087, -10392, -9520, -8485, -7305, -6000, -4592, -3106, -1566];
const PCM_RING_CAPACITY: u32 = 1_024;
const PCM_RING_MASK: u32 = PCM_RING_CAPACITY - 1;
const PCM_PREFILL_SAMPLES: u32 = 256;
const MAX_PRODUCE_PER_CALL: u32 = 256;

static PCM_HEAD: AtomicU32 = AtomicU32::new(0);
static PCM_TAIL: AtomicU32 = AtomicU32::new(0);
static PCM_EPOCH: AtomicU32 = AtomicU32::new(0);
static PCM_ACTIVE: AtomicBool = AtomicBool::new(false);
static PCM_READY: AtomicBool = AtomicBool::new(false);
static PCM_UNDERRUNS: AtomicU32 = AtomicU32::new(0);
static PCM_OVERFLOWS: AtomicU32 = AtomicU32::new(0);
static PCM_USB_INCOMPLETE: AtomicU32 = AtomicU32::new(0);
static PCM_USB_SUBMIT_ERRORS: AtomicU32 = AtomicU32::new(0);

// Single producer: main loop. Single consumer: TIM2 USB ISR.
static mut PCM_RING: [i16; PCM_RING_CAPACITY as usize] = [0; PCM_RING_CAPACITY as usize];

/// Stateful PCM producer; keep this in main-loop context, never in the ISR.
pub struct AudioPcmProducer {
    epoch: u32,
    initialized: bool,
    tone_index: usize,
}

impl AudioPcmProducer {
    pub const fn new() -> Self {
        Self { epoch: 0, initialized: false, tone_index: 0 }
    }

    /// Fill the PCM ring from the precomputed tone table without reading emulator state.
    pub fn produce_tone(&mut self) {
        if !PCM_ACTIVE.load(Ordering::Acquire) {
            self.initialized = false;
            return;
        }
        let epoch = PCM_EPOCH.load(Ordering::Acquire);
        if !self.initialized || epoch != self.epoch {
            PCM_HEAD.store(0, Ordering::Relaxed);
            PCM_TAIL.store(0, Ordering::Relaxed);
            PCM_UNDERRUNS.store(0, Ordering::Relaxed);
            PCM_OVERFLOWS.store(0, Ordering::Relaxed);
            PCM_USB_INCOMPLETE.store(0, Ordering::Relaxed);
            PCM_USB_SUBMIT_ERRORS.store(0, Ordering::Relaxed);
            self.epoch = epoch;
            self.initialized = true;
            self.tone_index = 0;
        }

        let mut produced = 0u32;
        while produced < MAX_PRODUCE_PER_CALL {
            let head = PCM_HEAD.load(Ordering::Relaxed);
            let tail = PCM_TAIL.load(Ordering::Acquire);
            if head.wrapping_sub(tail) >= PCM_RING_CAPACITY { break; }
            let sample = TONE_TABLE[self.tone_index];
            let slot = (head & PCM_RING_MASK) as usize;
            unsafe { ptr::addr_of_mut!(PCM_RING).cast::<i16>().add(slot).write(sample); }
            PCM_HEAD.store(head.wrapping_add(1), Ordering::Release);
            self.tone_index = (self.tone_index + 1) % TONE_TABLE.len();
            produced += 1;
        }

        let head = PCM_HEAD.load(Ordering::Acquire);
        let tail = PCM_TAIL.load(Ordering::Acquire);
        if head.wrapping_sub(tail) >= PCM_PREFILL_SAMPLES {
            PCM_READY.store(true, Ordering::Release);
        }
    }

    pub fn rate_adjust_ppm(&self) -> i32 { 0 }
}

impl Default for AudioPcmProducer {
    fn default() -> Self { Self::new() }
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
            last_sample: 0,
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
            0,
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
        match ep.write_iso_batch_fifo(packet, 1, 1) {
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
            self.last_sample = 0;
            self.dma_in_flight = false;
            PCM_READY.store(false, Ordering::Release);
            PCM_ACTIVE.store(true, Ordering::Release);
            PCM_EPOCH.fetch_add(1, Ordering::AcqRel);
            self.active = true;
        } else {
            self.active = false;
            PCM_ACTIVE.store(false, Ordering::Release);
            PCM_READY.store(false, Ordering::Release);
            self.dma_in_flight = false;
        }
    }
}
