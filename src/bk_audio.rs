//! BK-0010 speaker-edge capture and deferred PCM16 rendering for TinyUSB UAC2.
//!
//! The emulator publishes timestamped speaker transitions. The main-loop audio
//! worker converts them to 48 kHz PCM in a ring buffer. The USB completion ISR
//! only copies six already-rendered samples from that ring.

use core::sync::atomic::{AtomicU16, Ordering};

const SAMPLES_PER_MICROFRAME: usize = 6;
const PACKET_BYTES: usize = SAMPLES_PER_MICROFRAME * 2;
const AUDIO_SAMPLE_PERIOD_CYCLES: u32 = 3_500; // 168 MHz / 48 kHz
const AUDIO_STARTUP_LATENCY_CYCLES: u32 = 168_000; // Keep a 1 ms render lead.
const AUDIO_AMPLITUDE: i16 = 12_000;
const PCM_RING_CAPACITY: u16 = 512;
const PCM_RING_MASK: u16 = PCM_RING_CAPACITY - 1;
const _: () = assert!(PCM_RING_CAPACITY.is_power_of_two());
const _: () = assert!(PACKET_BYTES <= crate::uvc::USB_HS_DMA_BUFFER_LIMIT);

static PCM_HEAD: AtomicU16 = AtomicU16::new(0);
static PCM_TAIL: AtomicU16 = AtomicU16::new(0);
static mut PCM_RING: [i16; PCM_RING_CAPACITY as usize] = [0; PCM_RING_CAPACITY as usize];

unsafe extern "C" {
    fn bk_audio_start() -> u8;
    fn bk_audio_stop();
    fn bk_audio_pop_edge(cycles: *mut u32, level: *mut u8) -> u8;
}

pub struct BkEmulatorAudio {
    next_sample_cycle: u32,
    sample_level: bool,
    pending_edge: Option<(u32, bool)>,
    active: bool,
}

impl BkEmulatorAudio {
    pub const fn new() -> Self {
        Self {
            next_sample_cycle: 0,
            sample_level: false,
            pending_edge: None,
            active: false,
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

    /// Start/stop edge capture. Called from TinyUSB control/task context.
    pub fn tinyusb_set_active(&mut self, active: bool) {
        if active {
            if !self.active {
                self.sample_level = unsafe { bk_audio_start() != 0 };
                self.pending_edge = None;
                self.next_sample_cycle = Self::cycle_counter()
                    .wrapping_sub(AUDIO_STARTUP_LATENCY_CYCLES);
                PCM_TAIL.store(0, Ordering::Relaxed);
                PCM_HEAD.store(0, Ordering::Release);
                self.active = true;
            }
        } else if self.active {
            self.active = false;
            self.pending_edge = None;
            unsafe { bk_audio_stop(); }
            PCM_TAIL.store(0, Ordering::Relaxed);
            PCM_HEAD.store(0, Ordering::Release);
        }
    }

    /// Render at most max_samples whose emulated timestamps are due.
    /// This runs in the main loop, never in the USB interrupt.
    pub fn render_available(&mut self, max_samples: usize) -> usize {
        if !self.active {
            return 0;
        }

        let render_until = Self::cycle_counter().wrapping_sub(AUDIO_STARTUP_LATENCY_CYCLES);
        let mut rendered = 0usize;
        while rendered < max_samples {
            let head = PCM_HEAD.load(Ordering::Relaxed);
            let tail = PCM_TAIL.load(Ordering::Acquire);
            if head.wrapping_sub(tail) >= PCM_RING_CAPACITY {
                break;
            }

            let sample_cycle = self.next_sample_cycle;
            if render_until.wrapping_sub(sample_cycle) as i32 < 0 {
                break;
            }

            self.apply_edges_through(sample_cycle);
            let sample = if self.sample_level { AUDIO_AMPLITUDE } else { -AUDIO_AMPLITUDE };
            unsafe {
                core::ptr::addr_of_mut!(PCM_RING)
                    .cast::<i16>()
                    .add((head & PCM_RING_MASK) as usize)
                    .write(sample);
            }
            PCM_HEAD.store(head.wrapping_add(1), Ordering::Release);
            self.next_sample_cycle = self.next_sample_cycle
                .wrapping_add(AUDIO_SAMPLE_PERIOD_CYCLES);
            rendered += 1;
        }
        rendered
    }
}

/// Copy one USB microframe of pre-rendered PCM. Missing samples are zero-filled.
pub unsafe fn fill_usb_packet(buffer: *mut u8, capacity: u16) -> u16 {
    if buffer.is_null() || capacity < PACKET_BYTES as u16 {
        return 0;
    }

    let mut tail = PCM_TAIL.load(Ordering::Relaxed);
    let head = PCM_HEAD.load(Ordering::Acquire);
    for index in 0..SAMPLES_PER_MICROFRAME {
        let sample = if tail != head {
            let value = unsafe {
                core::ptr::addr_of!(PCM_RING)
                    .cast::<i16>()
                    .add((tail & PCM_RING_MASK) as usize)
                    .read()
            };
            tail = tail.wrapping_add(1);
            value
        } else {
            0
        };
        unsafe {
            buffer.add(index * 2)
                .cast::<i16>()
                .write_unaligned(sample.to_le());
        }
    }
    PCM_TAIL.store(tail, Ordering::Release);
    PACKET_BYTES as u16
}

impl Default for BkEmulatorAudio {
    fn default() -> Self { Self::new() }
}
