#![no_std]
#![allow(suspicious_runtime_symbol_definitions)]

pub mod bk_screen;
pub mod cdc_console;
pub mod cdc_zmodem;
pub mod camera;
pub mod nand_fs;
mod nand_ecc;
#[cfg(feature = "list-profile")]
mod list_diag;
#[cfg(feature = "list-profile")]
mod write_diag;
pub mod dhara_fs;
pub mod libc_shims;
mod copy_words;
#[path = "../resources/font8x8.rs"]
mod font8x8;
mod palette;
#[path = "../resources/rom.rs"]
pub mod rom;
pub mod rtc;
pub mod bk_audio;
pub mod tusb;
pub mod audio_uac2;
pub mod uvc_nv12;
pub mod tusb_descriptors;
pub mod uvc;

/// Single source of truth for the USB audio and BK emulator sample rate.
pub const AUDIO_KHZ: u32 = 24;
pub const AUDIO_SAMPLE_RATE_HZ: u32 = AUDIO_KHZ * 1_000;

#[unsafe(no_mangle)]
pub extern "C" fn xcore_audio_sample_rate_hz() -> u32 {
    AUDIO_SAMPLE_RATE_HZ
}
