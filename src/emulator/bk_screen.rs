//! BK-0010 screen adapter translated from the original BkScreen.cpp.
//!
//! It writes directly to the existing camera canvas; no second framebuffer
//! or shadow video-memory buffer is allocated.

use super::bk_video;

pub const VIDEO_START: u32 = 0x4000;
pub const VIDEO_END: u32 = 0x7fff;
pub const BYTES_PER_ROW: usize = 64;

#[derive(Clone, Copy, Debug, Default)]
pub struct BkScreen;

impl BkScreen {
    /// Read one packed video byte. mode_register is RAM[0x0020].
    pub fn ll_byte(&self, addr: u32, mode_register: u8) -> Option<u8> {
        if bk_video::extended_memory_enabled() && (0x4000..0x7000).contains(&addr) {
            let offset = (addr - 0x4000) as usize;
            return Some(bk_video::extended_ram_read(offset));
        }
        let extended = bk_video::extended_memory_enabled();
        let video_start = if extended { 0x7000 } else { VIDEO_START };
        if !(video_start..=VIDEO_END).contains(&addr) {
            return None;
        }
        let offset = (addr - video_start) as usize;
        let y = if extended {
            offset / BYTES_PER_ROW
        } else {
            bk_video::shifted_row(offset / BYTES_PER_ROW)
        };
        let byte_x = offset % BYTES_PER_ROW;
        bk_video::read_video_byte(byte_x, y, mode_register == 0)
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
        if bk_video::extended_memory_enabled() && (0x4000..0x7000).contains(&addr) {
            let offset = (addr - 0x4000) as usize;
            bk_video::extended_ram_write(offset, byte);
            return true;
        }
        let extended = bk_video::extended_memory_enabled();
        let video_start = if extended { 0x7000 } else { VIDEO_START };
        if !(video_start..=VIDEO_END).contains(&addr) {
            return false;
        }
        let offset = (addr - video_start) as usize;
        let y = if extended {
            offset / BYTES_PER_ROW
        } else {
            bk_video::shifted_row(offset / BYTES_PER_ROW)
        };
        let byte_x = offset % BYTES_PER_ROW;
        bk_video::write_video_byte(byte_x, y, byte, mode_register == 0)
    }

    /// Write a little-endian word as two consecutive video-byte writes.
    pub fn sl_word(&self, addr: u32, word: u16, mode_register: u8) -> bool {
        if !self.sl_byte(addr, word as u8, mode_register) {
            return false;
        }
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
    if BkScreen.sl_byte(addr, value, mode) {
        0
    } else {
        2
    }
}
