//! BK-0010 video memory format, mode, extended-memory, and scrolling behavior.
//!
//! The camera module owns the split SRAM/CCM canvas. This module accesses it
//! through bounded byte-range operations and translates BK pixels to/from the
//! active canvas representation.

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use crate::camera::{self, CameraMode};
use crate::palette::{GRAYSCALE, UV_PALETTE};

const WIDTH: usize = camera::WIDTH;
const CANVAS_HEIGHT: usize = camera::CANVAS_HEIGHT;
const EXTENDED_VIDEO_ROWS: usize = 64;
const EXTENDED_RAM_OFFSET: usize = EXTENDED_VIDEO_ROWS * WIDTH;
const EXTENDED_RAM_BYTES: usize = 0x3000;
const BK_UNSCROLLED_VALUE: u8 = 216;

static EXTENDED_MEMORY: AtomicBool = AtomicBool::new(false);
static VERTICAL_SHIFT: AtomicU8 = AtomicU8::new(0);

/// Whether the BK address map exposes the reclaimed 12 KiB as program RAM.
pub fn extended_memory_enabled() -> bool {
    EXTENDED_MEMORY.load(Ordering::Acquire)
}

/// Read a byte from the 12 KiB program-RAM window backed by hidden canvas rows.
pub fn extended_ram_read(offset: usize) -> u8 {
    if offset >= EXTENDED_RAM_BYTES {
        return 0;
    }
    let mut byte = [0];
    let read_ok = camera::read_canvas(EXTENDED_RAM_OFFSET + offset, &mut byte);
    debug_assert!(read_ok);
    byte[0]
}

/// Write a byte to the 12 KiB program-RAM window backed by hidden canvas rows.
pub fn extended_ram_write(offset: usize, value: u8) {
    if offset >= EXTENDED_RAM_BYTES {
        return;
    }
    let written = camera::write_canvas(EXTENDED_RAM_OFFSET + offset, &[value]);
    debug_assert!(written);
}

pub(super) fn shifted_row(vram_y: usize) -> usize {
    (vram_y + CANVAS_HEIGHT - VERTICAL_SHIFT.load(Ordering::Acquire) as usize) % CANVAS_HEIGHT
}

/// Read a packed BK video byte from the camera canvas.
pub(super) fn read_video_byte(byte_x: usize, y: usize, monochrome: bool) -> Option<u8> {
    if y >= CANVAS_HEIGHT || (extended_memory_enabled() && y >= EXTENDED_VIDEO_ROWS) {
        return None;
    }

    let mut result = 0u8;
    if monochrome {
        if byte_x >= WIDTH / 8 {
            return None;
        }
        let mut pixels = [0; 8];
        let offset = y * WIDTH + byte_x * 8;
        if !camera::read_canvas(offset, &mut pixels) {
            return None;
        }
        for pixel in pixels {
            result = (result >> 1) | if pixel != 0 { 0x80 } else { 0 };
        }
    } else {
        if byte_x >= (WIDTH / 2) / 4 {
            return None;
        }
        let mut pixels = [0; 8];
        let offset = (y * (WIDTH / 2) + byte_x * 4) * 2;
        if !camera::read_canvas(offset, &mut pixels) {
            return None;
        }
        for pixel in 0..4 {
            let uv = [pixels[pixel * 2], pixels[pixel * 2 + 1]];
            let color = color_from_uv(uv);
            let code = match color {
                0x00 => 0,
                0x30 => 1,
                0x0c => 2,
                0x03 => 3,
                _ => 0,
            };
            result = (result >> 2) | (code << 6);
        }
    }
    Some(result)
}

/// Write a packed BK video byte into the camera canvas.
pub(super) fn write_video_byte(byte_x: usize, y: usize, byte: u8, monochrome: bool) -> bool {
    if y >= CANVAS_HEIGHT || (extended_memory_enabled() && y >= EXTENDED_VIDEO_ROWS) {
        return false;
    }

    // The camera-mode change is applied later in the foreground loop. Until
    // then, preserve the visible image by drawing each BK pixel in the active
    // camera representation.
    if (camera::camera_mode() == CameraMode::Monochrome) != monochrome {
        if monochrome {
            let x = byte_x * 8;
            if x >= camera::canvas_width_for(camera::camera_mode()) {
                return false;
            }
            for i in 0..8 {
                camera::set_pixel(x + i, y, if byte & (1 << i) != 0 { 0x3f } else { 0 });
            }
        } else {
            let x = byte_x * 4;
            if x >= camera::canvas_width_for(camera::camera_mode()) {
                return false;
            }
            for i in 0..4 {
                camera::set_pixel(x + i, y, color_for_code((byte >> (i * 2)) & 3));
            }
        }
        return true;
    }

    let offset = if monochrome {
        if byte_x >= WIDTH / 8 {
            return false;
        }
        y * WIDTH + byte_x * 8
    } else {
        if byte_x >= (WIDTH / 2) / 4 {
            return false;
        }
        (y * (WIDTH / 2) + byte_x * 4) * 2
    };

    let mut pixels = [0; 8];
    if monochrome {
        for (i, pixel) in pixels.iter_mut().enumerate() {
            *pixel = GRAYSCALE[if byte & (1 << i) != 0 { 0x3f } else { 0 }];
        }
    } else {
        for i in 0..4 {
            let [u, v] = UV_PALETTE[color_for_code((byte >> (i * 2)) & 3) as usize];
            pixels[i * 2] = u;
            pixels[i * 2 + 1] = v;
        }
    }
    camera::write_canvas(offset, &pixels)
}

fn color_for_code(code: u8) -> u8 {
    match code {
        0 => 0x00,
        1 => 0x30,
        2 => 0x0c,
        _ => 0x03,
    }
}

fn color_from_uv(uv: [u8; 2]) -> u8 {
    if uv == UV_PALETTE[0] {
        0
    } else if uv == UV_PALETTE[48] {
        0x30
    } else if uv == UV_PALETTE[12] {
        0x0c
    } else if uv == UV_PALETTE[3] {
        0x03
    } else {
        0
    }
}

/// Re-encode visible BK pixels when the camera canvas changes mode.
pub(crate) fn convert_canvas_mode(old_mode: CameraMode, new_mode: CameraMode) {
    if old_mode == new_mode {
        return;
    }
    let rows = if extended_memory_enabled() {
        EXTENDED_VIDEO_ROWS
    } else {
        CANVAS_HEIGHT
    };

    // Each eight-byte group maps to eight monochrome pixels or four color
    // pixels. Read the entire group before writing because conversion is
    // in-place and both representations share the same row footprint.
    for row in 0..rows {
        for group in (0..WIDTH).step_by(8) {
            let offset = row * WIDTH + group;
            let mut source = [0; 8];
            let mut destination = [0; 8];
            if !camera::read_canvas(offset, &mut source) {
                return;
            }
            if old_mode == CameraMode::Color {
                for i in (0..8).step_by(2) {
                    let hash = (source[i] ^ source[i + 1]) as usize;
                    let code = match hash {
                        h if h == (UV_PALETTE[48][0] ^ UV_PALETTE[48][1]) as usize => 1,
                        h if h == (UV_PALETTE[12][0] ^ UV_PALETTE[12][1]) as usize => 2,
                        h if h == (UV_PALETTE[3][0] ^ UV_PALETTE[3][1]) as usize => 3,
                        _ => 0,
                    };
                    destination[i] = GRAYSCALE[if code & 2 != 0 { 0x3f } else { 0 }];
                    destination[i + 1] = GRAYSCALE[if code & 1 != 0 { 0x3f } else { 0 }];
                }
            } else {
                for i in (0..8).step_by(2) {
                    let code = (u8::from(source[i] != 0) << 1) | u8::from(source[i + 1] != 0);
                    let [u, v] = UV_PALETTE[color_for_code(code) as usize];
                    destination[i] = u;
                    destination[i + 1] = v;
                }
            }
            if !camera::write_canvas(offset, &destination) {
                return;
            }
        }
    }
}

fn copy_canvas_row_to(row: usize, destination: &mut [u8; WIDTH]) -> bool {
    camera::read_canvas(row * WIDTH, destination)
}

fn copy_canvas_row(src_row: usize, dst_row: usize) -> bool {
    camera::copy_canvas(src_row * WIDTH, dst_row * WIDTH, WIDTH)
}

fn copy_row_to_canvas(source: &[u8; WIDTH], row: usize) -> bool {
    camera::write_canvas(row * WIDTH, source)
}

fn reverse_canvas_rows(first: usize, end: usize, temporary: &mut [u8; WIDTH]) -> bool {
    let mut left = first;
    let mut right = end;
    while left < right {
        right -= 1;
        if left >= right {
            break;
        }
        if !copy_canvas_row_to(left, temporary)
            || !copy_canvas_row(right, left)
            || !copy_row_to_canvas(temporary, right)
        {
            return false;
        }
        left += 1;
    }
    true
}

/// Rotate the BK canvas upward by the requested number of rows.
pub(crate) fn rotate_canvas_rows(rows: usize) {
    let rows = rows % CANVAS_HEIGHT;
    if rows == 0 {
        return;
    }
    let mut temporary = [0; WIDTH];
    let _ = reverse_canvas_rows(0, rows, &mut temporary);
    let _ = reverse_canvas_rows(rows, CANVAS_HEIGHT, &mut temporary);
    let _ = reverse_canvas_rows(0, CANVAS_HEIGHT, &mut temporary);
}

/// Handle the BK vertical-roll register (port 0177664).
#[unsafe(no_mangle)]
pub extern "C" fn bk_video_shift_changed(register: u16) {
    let extended = (register & 0x0200) == 0;
    EXTENDED_MEMORY.store(extended, Ordering::Release);
    camera::set_visible_canvas_rows(if extended {
        EXTENDED_VIDEO_ROWS
    } else {
        CANVAS_HEIGHT
    });
    if extended {
        VERTICAL_SHIFT.store(0, Ordering::Release);
    } else {
        let normalized = (register as u8).wrapping_sub(BK_UNSCROLLED_VALUE);
        let previous = VERTICAL_SHIFT.swap(normalized, Ordering::AcqRel);
        rotate_canvas_rows(normalized.wrapping_sub(previous) as usize);
    }
}

/// Keep the camera canvas interpretation aligned with BK RAM[0x0020].
#[unsafe(no_mangle)]
pub extern "C" fn bk_video_mode_changed(mode: u8) {
    camera::request_camera_mode(if mode == 0 {
        CameraMode::Monochrome
    } else {
        CameraMode::Color
    });
}
