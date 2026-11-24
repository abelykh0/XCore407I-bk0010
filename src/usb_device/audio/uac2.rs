//! TinyUSB UAC2 audio class callbacks and BK emulator audio packet handling.

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::emulator::bk_audio::BkEmulatorAudio;
const UAC_STREAM_INTERFACE: u8 = 3;

static AUDIO_STATE_READY: AtomicBool = AtomicBool::new(false);
static mut AUDIO_STATE: MaybeUninit<BkEmulatorAudio> = MaybeUninit::uninit();

fn audio_state_ptr() -> Option<*mut BkEmulatorAudio> {
    if !AUDIO_STATE_READY.load(Ordering::Acquire) {
        return None;
    }
    Some(core::ptr::addr_of_mut!(AUDIO_STATE).cast::<BkEmulatorAudio>())
}

fn initialize_audio_state() -> bool {
    if AUDIO_STATE_READY.load(Ordering::Acquire) {
        return true;
    }
    unsafe {
        core::ptr::addr_of_mut!(AUDIO_STATE)
            .cast::<BkEmulatorAudio>()
            .write(BkEmulatorAudio::new());
    }
    AUDIO_STATE_READY.store(true, Ordering::Release);
    true
}

/// Called by TinyUSB after it has applied SET_INTERFACE to the audio stream.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_audio_set_interface(interface: u8, alternate: u8) -> bool {
    if interface != UAC_STREAM_INTERFACE {
        return true;
    }
    if alternate > 1 {
        return false;
    }
    cortex_m::interrupt::free(|_| {
        if !initialize_audio_state() {
            return false;
        }
        let Some(state) = audio_state_ptr() else {
            return false;
        };
        unsafe { (&mut *state).tinyusb_set_active(alternate == 1); }
        true
    })
}

/// Publish elapsed source-time samples for adaptive resampling.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_audio_advance_source_time() {
    if let Some(state) = audio_state_ptr() {
        unsafe { (&mut *state).advance_source_time(448); }
    }
}

/// USB completion callback: reconstruct samples from timestamped speaker edges.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn xcore_tud_audio_fill_packet(buffer: *mut u8, capacity: u16) -> u16 {
    unsafe { crate::emulator::bk_audio::fill_usb_packet(buffer, capacity) }
}

/// TinyUSB calls this on unmount/reset; the next SET_INTERFACE will reinitialize
/// capture state and start a fresh audio stream.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_audio_stop() {
    cortex_m::interrupt::free(|_| {
        if let Some(state) = audio_state_ptr() {
            unsafe { (&mut *state).tinyusb_set_active(false); }
        }
    })
}

/// Reinitialize BK speaker capture after an emulator reset. Preserve the
/// current USB streaming state; do not wait for another SET_INTERFACE request.
pub fn reset_emulator_capture() {
    cortex_m::interrupt::free(|_| {
        if let Some(state) = audio_state_ptr() {
            unsafe { (&mut *state).reset_emulator_capture(); }
        }
    })
}
