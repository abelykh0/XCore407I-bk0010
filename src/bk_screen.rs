//! BK-0010 screen adapter translated from the original BkScreen.cpp.
//!
//! It writes directly to the existing camera canvas; no second framebuffer
//! or shadow video-memory buffer is allocated.

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

const BK_UNSCROLLED_VALUE: u8 = 216;
static BK_VERTICAL_SHIFT: AtomicU8 = AtomicU8::new(0);
static BK_EXTENDED_MEMORY: AtomicBool = AtomicBool::new(false);

/// Whether the BK address map currently exposes the reclaimed 12 KiB as program RAM.
pub fn extended_memory_enabled() -> bool {
    BK_EXTENDED_MEMORY.load(Ordering::Acquire)
}

// Extended-mode program RAM is backed by the hidden portion of the camera canvas.

fn shifted_row(vram_y: usize) -> usize {
    (vram_y + 256 - BK_VERTICAL_SHIFT.load(Ordering::Acquire) as usize) & 0xff
}

pub const VIDEO_START: u32 = 0x4000;
pub const VIDEO_END: u32 = 0x7fff;
pub const BYTES_PER_ROW: usize = 64;

#[derive(Clone, Copy, Debug, Default)]
pub struct BkScreen;

impl BkScreen {
    /// Read one packed video byte. mode_register is RAM[0x0020].
    pub fn ll_byte(&self, addr: u32, mode_register: u8) -> Option<u8> {
        if BK_EXTENDED_MEMORY.load(Ordering::Acquire) && (0x4000..0x7000).contains(&addr) {
            let offset = (addr - 0x4000) as usize;
            return Some(crate::camera::bk_extended_ram_read(offset));
        }
        let video_start = if BK_EXTENDED_MEMORY.load(Ordering::Acquire) { 0x7000 } else { VIDEO_START };
        if !(video_start..=VIDEO_END).contains(&addr) { return None; }
        let offset = (addr - video_start) as usize;
        let y = if BK_EXTENDED_MEMORY.load(Ordering::Acquire) {
            offset / BYTES_PER_ROW
        } else {
            shifted_row(offset / BYTES_PER_ROW)
        };
        let byte_x = offset % BYTES_PER_ROW;
        let mut result = 0u8;
        if mode_register == 0 {
            let x = byte_x * 8;
            for i in 0..8 {
                result >>= 1;
                if crate::camera::get_bk_bw_pixel(x + i, y) != 0 { result |= 0x80; }
            }
        } else {
            let x = byte_x * 4;
            for i in 0..4 {
                let code = match crate::camera::get_bk_color_pixel(x + i, y) {
                    0x00 => 0, 0x30 => 1, 0x0c => 2, 0x03 => 3, _ => 0,
                };
                result = (result >> 2) | (code << 6);
            }
        }
        Some(result)
    }

    /// Read a little-endian word, matching the original two-byte operation.
    pub fn ll_word(&self, addr: u32, mode_register: u8) -> Option<u16> {
        let lo = self.ll_byte(addr, mode_register)?;
        let hi = self.ll_byte(addr + 1, mode_register).unwrap_or(0);
        Some(u16::from(lo) | (u16::from(hi) << 8))
    }

    /// Write one packed video byte directly to the camera canvas.
    /// mode_register is zero for monochrome and nonzero for color.
    pub fn sl_byte(&self, addr: u32, byte: u8, mode_register: u8) -> bool {
        if BK_EXTENDED_MEMORY.load(Ordering::Acquire) && (0x4000..0x7000).contains(&addr) {
            let offset = (addr - 0x4000) as usize;
            crate::camera::bk_extended_ram_write(offset, byte);
            return true;
        }
        let video_start = if BK_EXTENDED_MEMORY.load(Ordering::Acquire) { 0x7000 } else { VIDEO_START };
        if !(video_start..=VIDEO_END).contains(&addr) { return false; }
        let offset = (addr - video_start) as usize;
        let y = if BK_EXTENDED_MEMORY.load(Ordering::Acquire) {
            offset / BYTES_PER_ROW
        } else {
            shifted_row(offset / BYTES_PER_ROW)
        };
        let byte_x = offset % BYTES_PER_ROW;
        crate::camera::set_bk_video_byte(byte_x, y, byte, mode_register == 0);
        true
    }

    /// Write a little-endian word as two consecutive video-byte writes.
    pub fn sl_word(&self, addr: u32, word: u16, mode_register: u8) -> bool {
        if !self.sl_byte(addr, word as u8, mode_register) { return false; }
        self.sl_byte(addr + 1, (word >> 8) as u8, mode_register)
    }
}

/// C platform-layer callback for BK video-memory reads.
#[unsafe(no_mangle)]
pub extern "C" fn bk_video_read_byte(addr: u32, mode: u8, value: *mut u8) -> i32 {
    if value.is_null() {
        return 2; // BUS_ERROR
    }
    match BkScreen.ll_byte(addr, mode) {
        Some(byte) => {
            unsafe { value.write(byte) };
            0
        }
        None => 2,
    }
}

/// C platform-layer callback for BK video-memory writes.
#[unsafe(no_mangle)]
pub extern "C" fn bk_video_write_byte(addr: u32, mode: u8, value: u8) -> i32 {
    if BkScreen.sl_byte(addr, value, mode) { 0 } else { 2 }
}

/// Apply the BK hardware vertical-roll register (port 0177664).
#[unsafe(no_mangle)]
pub extern "C" fn bk_video_shift_changed(register: u16) {
    // Bit 9 selects the BK extended-memory layout: 1 = normal, 0 = extended.
    let extended = (register & 0x0200) == 0;
    BK_EXTENDED_MEMORY.store(extended, Ordering::Release);
    crate::camera::set_bk_extended_mode(extended);

    if extended {
        // Extended mode exposes only 070000-077777 (64 rows); scrolling is fixed.
        BK_VERTICAL_SHIFT.store(0, Ordering::Release);
    } else {
        // Normal mode's neutral hardware value is 216 (decimal).
        let normalized = (register as u8).wrapping_sub(BK_UNSCROLLED_VALUE);
        let previous = BK_VERTICAL_SHIFT.swap(normalized, Ordering::AcqRel);
        crate::camera::rotate_bk_canvas_rows(normalized.wrapping_sub(previous) as usize);
    }
}

/// Keep the camera canvas interpretation aligned with BK RAM[0x0020].
#[unsafe(no_mangle)]
pub extern "C" fn bk_video_mode_changed(mode: u8) {
    let camera_mode = if mode == 0 {
        crate::camera::CameraMode::Monochrome
    } else {
        crate::camera::CameraMode::Color
    };
    crate::camera::request_camera_mode(camera_mode);
}
