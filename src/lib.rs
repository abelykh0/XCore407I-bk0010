#![no_std]
#![allow(suspicious_runtime_symbol_definitions)]

pub mod camera;
pub mod emulator;
pub mod format;
pub mod nand_fs;
pub mod dhara_fs;
pub mod ethernet;
pub mod libc_shims;
pub mod rtc;
pub mod tusb;
pub mod usb_device;

mod nand_ecc;
mod copy_words;
mod palette;

#[path = "../resources/font8x8.rs"]
mod font8x8;
#[path = "../resources/rom.rs"]
pub mod rom;

#[cfg(feature = "list-profile")]
mod list_diag;
#[cfg(feature = "list-profile")]
mod write_diag;

/// Single source of truth for the USB audio and BK emulator sample rate.
pub const AUDIO_KHZ: u32 = 24;
pub const AUDIO_SAMPLE_RATE_HZ: u32 = AUDIO_KHZ * 1_000;

#[unsafe(no_mangle)]
pub extern "C" fn xcore_audio_sample_rate_hz() -> u32 {
    AUDIO_SAMPLE_RATE_HZ
}
