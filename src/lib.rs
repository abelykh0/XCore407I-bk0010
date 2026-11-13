#![no_std]
#![allow(suspicious_runtime_symbol_definitions)]

pub mod bk_screen;
pub mod camera;
pub mod nand_fs;
pub mod mtp_storage;
pub mod dhara_fs;
pub mod libc_shims;
mod copy_words;
mod font8x8;
mod palette;
pub mod rom;
pub mod rtc;
pub mod bk_audio;
pub mod tusb;
pub mod tusb_class_drivers;
pub mod tusb_descriptors;
pub mod uvc;

/// Single source of truth for the USB audio and BK emulator sample rate.
pub const AUDIO_KHZ: u32 = 24;
pub const AUDIO_SAMPLE_RATE_HZ: u32 = AUDIO_KHZ * 1_000;

#[unsafe(no_mangle)]
pub extern "C" fn xcore_audio_sample_rate_hz() -> u32 {
    AUDIO_SAMPLE_RATE_HZ
}
