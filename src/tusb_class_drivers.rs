//! Rust side of the TinyUSB custom-class callback boundary.
//!
//! The class-driver registration structure stays in C because it is private
//! to TinyUSB. These inert callbacks intentionally decline every interface;
//! the existing built-in/legacy device path remains in control until the
//! UVC and UAC2 implementations are complete.

use core::ffi::c_void;

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_open(
    _rhport: u8,
    _interface_desc: *const c_void,
    _max_len: u16,
) -> u16 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_control_xfer(
    _rhport: u8,
    _stage: u8,
    _request: *const c_void,
) -> bool {
    false
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_xfer(
    _rhport: u8,
    _ep_addr: u8,
    _result: i32,
    _xferred_bytes: u32,
) -> bool {
    false
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_audio_open(
    _rhport: u8,
    _interface_desc: *const c_void,
    _max_len: u16,
) -> u16 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_audio_control_xfer(
    _rhport: u8,
    _stage: u8,
    _request: *const c_void,
) -> bool {
    false
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_audio_xfer(
    _rhport: u8,
    _ep_addr: u8,
    _result: i32,
    _xferred_bytes: u32,
) -> bool {
    false
}
