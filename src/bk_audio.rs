//! BK-0010 speaker-edge capture and PCM16 packet generation for TinyUSB UAC2.
//!
//! The emulator timestamps transitions on port 0177714 bit 6 with the DWT
//! cycle counter. This module resamples that edge stream to 48 kHz mono PCM16.

const AUDIO_SAMPLE_RATE: u32 = 48_000;
const SAMPLES_PER_MICROFRAME: usize = 6;
const PACKET_BYTES: usize = SAMPLES_PER_MICROFRAME * 2;
const _: () = assert!(PACKET_BYTES <= crate::uvc::USB_HS_DMA_BUFFER_LIMIT);
const AUDIO_SAMPLE_PERIOD_CYCLES: u32 = 3_500; // 168 MHz / 48 kHz
const AUDIO_STARTUP_LATENCY_CYCLES: u32 = 168_000; // 1 ms edge-queue lead
const AUDIO_AMPLITUDE: i16 = 12_000;

// Word-aligned packet scratch storage. TinyUSB copies this packet into its
// audio FIFO before submitting it through the DWC2 endpoint DMA path.

unsafe extern "C" {
    fn bk_audio_start() -> u8;
    fn bk_audio_stop();
    fn bk_audio_pop_edge(cycles: *mut u32, level: *mut u8) -> u8;
}

#[repr(align(4))]
struct DmaPacket([u32; PACKET_BYTES / 4]);

pub struct BkEmulatorAudio {
    next_sample_cycle: u32,
    sample_level: bool,
    pending_edge: Option<(u32, bool)>,
    active: bool,
    dma_in_flight: bool,
    dma_packet: DmaPacket,
}

impl BkEmulatorAudio {
    pub const fn new() -> Self {
        Self {
            next_sample_cycle: 0,
            sample_level: false,
            pending_edge: None,
            active: false,
            dma_in_flight: false,
            dma_packet: DmaPacket([0; PACKET_BYTES / 4]),
        }
    }

    #[inline(always)]
    fn cycle_counter() -> u32 {
        cortex_m::peripheral::DWT::cycle_count()
    }

    fn apply_edges_through(&mut self, sample_cycle: u32) {
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
            if sample_cycle.wrapping_sub(edge_cycle) as i32 >= 0 {
                self.sample_level = edge_level;
                self.pending_edge = None;
            } else {
                break;
            }
        }
    }

    /// Start or stop capture from TinyUSB's UAC2 alternate-setting callback.
    pub fn tinyusb_set_active(&mut self, active: bool) {
        if active {
            if !self.active {
                self.sample_level = unsafe { bk_audio_start() != 0 };
                self.pending_edge = None;
                self.next_sample_cycle = Self::cycle_counter()
                    .wrapping_sub(AUDIO_STARTUP_LATENCY_CYCLES);
                self.dma_in_flight = false;
                self.active = true;
            }
        } else {
            self.active = false;
            self.dma_in_flight = false;
            self.pending_edge = None;
            unsafe { bk_audio_stop(); }
        }
    }

    /// Fill the six-sample packet scratch buffer.
    /// TinyUSB's callback copies it into the audio FIFO and immediately retires
    /// this scratch buffer; TinyUSB owns the subsequent DMA transfer.
    pub fn tinyusb_prepare_packet(&mut self) -> Option<(*mut u8, u16)> {
        if !self.active || self.dma_in_flight {
            return None;
        }
        let packet = self.dma_packet.0.as_mut_ptr().cast::<u8>();
        for sample_offset in 0..SAMPLES_PER_MICROFRAME {
            let sample_cycle = self.next_sample_cycle;
            self.apply_edges_through(sample_cycle);
            let sample = if self.sample_level { AUDIO_AMPLITUDE } else { -AUDIO_AMPLITUDE };
            unsafe {
                packet.add(sample_offset * 2)
                    .cast::<i16>()
                    .write_unaligned(sample.to_le());
            }
            self.next_sample_cycle = self.next_sample_cycle
                .wrapping_add(AUDIO_SAMPLE_PERIOD_CYCLES);
        }
        self.dma_in_flight = true;
        Some((packet, PACKET_BYTES as u16))
    }

    pub fn tinyusb_transfer_retired(&mut self) {
        self.dma_in_flight = false;
    }
}

impl Default for BkEmulatorAudio {
    fn default() -> Self { Self::new() }
}
