//! C-style compact color canvas and NV12 UVC packetizer.
//!
//! This mirrors the original C canvas: 128 KiB of source data split between
//! ordinary SRAM and CCM, with two 768-byte row caches in CCM. Color stores
//! packed Cb/Cr pairs at 256x256; BW stores Y at 512x256.

// PACKET_SIZE_NO_HEADER is 1.5 * UVC_WIDTH
// Every packet we fill next 2/3 of PLANE_WIDTH
// One packet is either on one or two UV rows
// ╔════════╦════════════╦═════════╦═════════════════════════════════════════════════════════════════════╦═══════════════════════╗
// ║ Packet ║   Y Row    ║  UV Row ║                      Action Sequence                                ║ Modified buffers      ║
// ╠════════╬════════════╬═════════╬═════════════════════════════════════════════════════════════════════╬═══════════════════════╣
// ║  N/A   ║            ║    0    ║ while we are processing the UV plane, prepare currentRow (0)        ║ currentRow (0) ██████ ║
// ╠════════╬════════════╬═════════╬═════════════════════════════════════════════════════════════════════╬═══════════════════════╣
// ║    0   ║ 0  ██████  ║    0    ║ 1. use currentRow (0) to fill "out"                                 ║                       ║
// ║        ║ 1  ███░░░  ║         ║ 2. prepare nextRow (1)                                              ║ nextRow    (1) ██████ ║
// ╠════════╬════════════╬═════════╬═════════════════════════════════════════════════════════════════════╬═══════════════════════╣
// ║    1   ║ 1  ░░░███  ║    0    ║ 1. use second half of currentRow (0) and nextRow (1) to fill "out"  ║                       ║
// ║        ║ 2  ██████  ║    1    ║ 2. === swap buffers === currentRow (1), nextRow (2)                 ║                       ║
// ║        ║            ║         ║ 3. prepare nextRow (2)                                              ║ nextRow    (2) ██████ ║
// ╠════════╬════════════╬═════════╬═════════════════════════════════════════════════════════════════════╬═══════════════════════╣
// ║    2   ║ 3  ██████  ║    1    ║ 1. use currentRow (1) and first half of nextRow (2) to fill "out"   ║                       ║
// ║        ║ 4  ███░░░  ║    2    ║ 2. === swap buffers === currentRow (2), nextRow (3)                 ║                       ║
// ║        ║            ║         ║ 3. prepare nextRow (3)                                              ║ nextRow    (3) ██████ ║
// ╠════════╬════════════╬═════════╬═════════════════════════════════════════════════════════════════════╬═══════════════════════╣
// ║    3   ║ 4  ░░░███  ║    2    ║ 1. use currentRow (2) to fill "out"                                 ║                       ║
// ║        ║ 5  ██████  ║         ║ 2. === swap buffers === currentRow (3), nextRow (4)                 ║ nextRow    (4) ░░░░░░ ║
// ╠════════╬════════════╬═════════╬═════════════════════════════════════════════════════════════════════╬═══════════════════════╣
// ║    4   ║ 6  ██████  ║    3    ║ Same as packet 0                                                    ║                       ║
// ║        ║ 7  ███░░░  ║         ║                                                                     ║ nextRow    (4) ██████ ║
// ╚════════╩════════════╩═════════╩═════════════════════════════════════════════════════════════════════╩═══════════════════════╝

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::palette::{UV_PALETTE, Y_HASH_TABLE};

const fn make_duplicated_y_table() -> [u16; 256] {
    let mut table = [0; 256];
    let mut i = 0;
    while i < table.len() {
        let y = Y_HASH_TABLE[i] as u16;
        table[i] = y | (y << 8);
        i += 1;
    }
    table
}
// Mutable initialized data is placed in SRAM by the startup .data copy.
static mut DUPLICATED_Y_TABLE: [u16; 256] = make_duplicated_y_table();

#[inline(always)]
fn duplicated_y_lookup(index: usize) -> u16 {
    unsafe {
        core::ptr::addr_of!(DUPLICATED_Y_TABLE)
            .cast::<u16>()
            .add(index)
            .read()
    }
}


#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum CameraMode {
    Color = 0,
    Monochrome = 1,
}

static ACTIVE_CAMERA_MODE: AtomicU32 = AtomicU32::new(CameraMode::Color as u32);
static PENDING_CAMERA_MODE: AtomicU32 = AtomicU32::new(u32::MAX);
#[inline]
pub fn camera_mode() -> CameraMode {
    if ACTIVE_CAMERA_MODE.load(Ordering::Acquire) == CameraMode::Monochrome as u32 {
        CameraMode::Monochrome
    } else {
        CameraMode::Color
    }
}

pub fn request_camera_mode(mode: CameraMode) {
    PENDING_CAMERA_MODE.store(mode as u32, Ordering::Release);
}

pub fn request_camera_mode_toggle() {
    let current = PENDING_CAMERA_MODE.load(Ordering::Acquire);
    let base = if current != u32::MAX { current } else { ACTIVE_CAMERA_MODE.load(Ordering::Acquire) };
    let next = if base == CameraMode::Color as u32 {
        CameraMode::Monochrome
    } else {
        CameraMode::Color
    };
    request_camera_mode(next);
}

fn apply_pending_camera_mode() -> bool {
    let requested = PENDING_CAMERA_MODE.swap(u32::MAX, Ordering::AcqRel);
    if requested == u32::MAX || requested == ACTIVE_CAMERA_MODE.load(Ordering::Acquire) {
        return false;
    }
    ACTIVE_CAMERA_MODE.store(requested, Ordering::Release);
    USB_STATUS_ROW.store(0, Ordering::Relaxed);
    KEYBOARD_TEXT_COLUMN.store(0, Ordering::Relaxed);
    CLOCK_VERSION.fetch_add(1, Ordering::Relaxed);
    true
}

#[inline]
pub const fn canvas_width_for(mode: CameraMode) -> usize {
    match mode { CameraMode::Color => WIDTH / 2, CameraMode::Monochrome => WIDTH }
}
#[inline]
pub fn canvas_width() -> usize { canvas_width_for(camera_mode()) }
#[inline]
pub const fn canvas_height() -> usize { HEIGHT / 2 }

pub const WIDTH: usize = 512;
pub const HEIGHT: usize = 512;
pub const Y_PLANE_BYTES: usize = WIDTH * HEIGHT;
pub const FRAME_BYTES: usize = Y_PLANE_BYTES + (WIDTH * HEIGHT / 2);
pub const UVC_HEADER_BYTES: usize = 4;
// Match the original C packetizer: one 512-pixel NV12 row's worth of payload
// (512 Y bytes + 256 UV bytes) plus the four-byte UVC header.
pub const PAYLOAD_BYTES: usize = WIDTH * 3 / 2;
pub const TRANSFER_BYTES: usize = UVC_HEADER_BYTES + PAYLOAD_BYTES;

pub const CANVAS_HEIGHT: usize = HEIGHT / 2;
pub const CANVAS_BYTES: usize = WIDTH * CANVAS_HEIGHT;
pub const CANVAS_TAIL_BYTES: usize = 46_848;
pub const CANVAS_MAIN_BYTES: usize = CANVAS_BYTES - CANVAS_TAIL_BYTES;

#[repr(C, align(4))]
struct AlignedCanvas<const N: usize>([u8; N]);

static mut DEMO_CANVAS: AlignedCanvas<CANVAS_MAIN_BYTES> = AlignedCanvas([0; CANVAS_MAIN_BYTES]);
#[unsafe(link_section = ".ccmram.canvas")]
static mut DEMO_CANVAS_TAIL: AlignedCanvas<CANVAS_TAIL_BYTES> =
    AlignedCanvas([0; CANVAS_TAIL_BYTES]);

#[repr(C, align(4))]
struct RowBuffers([u8; WIDTH * 2]);
#[unsafe(link_section = ".ccmram.rows")]
static mut ROW_BUFFERS: RowBuffers = RowBuffers([0; WIDTH * 2]);
static CAMERA_IN_USE: AtomicBool = AtomicBool::new(false);
static CLOCK_VERSION: AtomicU32 = AtomicU32::new(0);

#[cfg(feature = "cpu-cycle-diag")]
const VIDEO_PHASE_SAMPLE_INTERVAL: u8 = 61;
#[cfg(feature = "cpu-cycle-diag")]
static VIDEO_Y_PHASE_CYCLES: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "cpu-cycle-diag")]
static VIDEO_UV_PHASE_CYCLES: AtomicU32 = AtomicU32::new(0);

#[cfg(feature = "cpu-cycle-diag")]
pub fn take_video_phase_stats() -> (u32, u32) {
    (
        VIDEO_Y_PHASE_CYCLES.swap(0, Ordering::Relaxed)
            .saturating_mul(VIDEO_PHASE_SAMPLE_INTERVAL as u32),
        VIDEO_UV_PHASE_CYCLES.swap(0, Ordering::Relaxed)
            .saturating_mul(VIDEO_PHASE_SAMPLE_INTERVAL as u32),
    )
}

fn write_canvas(index: usize, value: u8) {
    unsafe {
        if index < CANVAS_MAIN_BYTES {
            core::ptr::addr_of_mut!(DEMO_CANVAS.0).cast::<u8>().add(index).write(value);
        } else {
            core::ptr::addr_of_mut!(DEMO_CANVAS_TAIL.0).cast::<u8>()
                .add(index - CANVAS_MAIN_BYTES).write(value);
        }
    }
}

fn canvas_read_chunk(offset: usize) -> (*const u8, usize) {
    unsafe {
        if offset < CANVAS_MAIN_BYTES {
            (
                core::ptr::addr_of!(DEMO_CANVAS.0).cast::<u8>().add(offset),
                CANVAS_MAIN_BYTES - offset,
            )
        } else {
            let tail_offset = offset - CANVAS_MAIN_BYTES;
            (
                core::ptr::addr_of!(DEMO_CANVAS_TAIL.0)
                    .cast::<u8>()
                    .add(tail_offset),
                CANVAS_TAIL_BYTES - tail_offset,
            )
        }
    }
}

/// Fill the entire canvas with one uniform palette color.
///
/// `color` is a palette index (0..63), matching `set_pixel()`.
/// In color mode the corresponding Cb/Cr pair is written for every pixel;
/// in B/W mode the corresponding luma value is written for every pixel.
pub fn clear_demo_canvas(color: u8) {
    let mode = camera_mode();
    let value = color & 0x3f;
    if mode == CameraMode::Monochrome {
        let luma = GRAYSCALE[value as usize];
        unsafe {
            core::ptr::addr_of_mut!(DEMO_CANVAS.0).cast::<u8>().write_bytes(luma, CANVAS_MAIN_BYTES);
            core::ptr::addr_of_mut!(DEMO_CANVAS_TAIL.0).cast::<u8>().write_bytes(luma, CANVAS_TAIL_BYTES);
        }
    } else {
        let [u, v] = UV_PALETTE[value as usize];
        let main_pixels = CANVAS_MAIN_BYTES / 2;
        let tail_pixels = CANVAS_TAIL_BYTES / 2;
        unsafe {
            let main = core::ptr::addr_of_mut!(DEMO_CANVAS.0).cast::<u8>();
            let tail = core::ptr::addr_of_mut!(DEMO_CANVAS_TAIL.0).cast::<u8>();
            for i in 0..main_pixels { main.add(i * 2).write(u); main.add(i * 2 + 1).write(v); }
            for i in 0..tail_pixels { tail.add(i * 2).write(u); tail.add(i * 2 + 1).write(v); }
        }
    }
}

/// Initialize the canvas to black in the currently selected runtime mode.
pub fn initialize_demo_canvas() {
    clear_demo_canvas(0);
}


/// Copy a row from the split SRAM/CCM canvas into an aligned temporary row.
#[inline]
unsafe fn copy_canvas_row_to(row: usize, destination: *mut u8) {
    unsafe {
    copy_canvas_range(row * WIDTH, destination, WIDTH);

    }
}

/// Copy a temporary row into the split SRAM/CCM canvas.
#[inline]
unsafe fn copy_row_to_canvas(source: *const u8, row: usize) {
    unsafe {
    let mut offset = row * WIDTH;
    let mut source = source;
    let mut remaining = WIDTH;
    while remaining != 0 {
        let (destination, available) = if offset < CANVAS_MAIN_BYTES {
            (
                core::ptr::addr_of_mut!(DEMO_CANVAS.0).cast::<u8>().add(offset),
                CANVAS_MAIN_BYTES - offset,
            )
        } else {
            let tail_offset = offset - CANVAS_MAIN_BYTES;
            (
                core::ptr::addr_of_mut!(DEMO_CANVAS_TAIL.0).cast::<u8>().add(tail_offset),
                CANVAS_TAIL_BYTES - tail_offset,
            )
        };
        let run = remaining.min(available);
        crate::copy_words::copy_words(source, destination, run);
        source = source.add(run);
        offset += run;
        remaining -= run;
    }

    }
}

/// Copy one canvas row directly to another, splitting at the SRAM/CCM
/// boundary independently for source and destination.
unsafe fn copy_canvas_row(src_row: usize, dst_row: usize) {
    unsafe {
    let mut src_offset = src_row * WIDTH;
    let mut dst_offset = dst_row * WIDTH;
    let mut remaining = WIDTH;
    while remaining != 0 {
        let (source, src_available) = if src_offset < CANVAS_MAIN_BYTES {
            (
                core::ptr::addr_of!(DEMO_CANVAS.0).cast::<u8>().add(src_offset),
                CANVAS_MAIN_BYTES - src_offset,
            )
        } else {
            let offset = src_offset - CANVAS_MAIN_BYTES;
            (
                core::ptr::addr_of!(DEMO_CANVAS_TAIL.0).cast::<u8>().add(offset),
                CANVAS_TAIL_BYTES - offset,
            )
        };
        let (destination, dst_available) = if dst_offset < CANVAS_MAIN_BYTES {
            (
                core::ptr::addr_of_mut!(DEMO_CANVAS.0).cast::<u8>().add(dst_offset),
                CANVAS_MAIN_BYTES - dst_offset,
            )
        } else {
            let offset = dst_offset - CANVAS_MAIN_BYTES;
            (
                core::ptr::addr_of_mut!(DEMO_CANVAS_TAIL.0).cast::<u8>().add(offset),
                CANVAS_TAIL_BYTES - offset,
            )
        };
        let run = remaining.min(src_available).min(dst_available);
        crate::copy_words::copy_words(source, destination, run);
        src_offset += run;
        dst_offset += run;
        remaining -= run;
    }

    }
}

/// Reverse a range of canvas rows in place, using one temporary row.
/// A row may straddle the SRAM/CCM boundary, so copies are split as needed.
unsafe fn reverse_canvas_rows(first: usize, end: usize, temporary: *mut u8) {
    unsafe {
    let mut left = first;
    let mut right = end;
    while left < right {
        right -= 1;
        if left >= right {
            break;
        }
        copy_canvas_row_to(left, temporary);
        // The rows are disjoint, so the optimized copier can move them directly.
        copy_canvas_row(right, left);
        copy_row_to_canvas(temporary, right);
        left += 1;
    }

    }
}

/// Rotate the shared BK canvas upward by whole rows. Three reversals perform
/// the rotation in O(height) row swaps, independent of the shift distance.
pub(crate) fn rotate_bk_canvas_rows(rows: usize) {
    let rows = rows % CANVAS_HEIGHT;
    if rows == 0 {
        return;
    }

    let mut temporary = [0u8; WIDTH];
    unsafe {
        reverse_canvas_rows(0, rows, temporary.as_mut_ptr());
        reverse_canvas_rows(rows, CANVAS_HEIGHT, temporary.as_mut_ptr());
        reverse_canvas_rows(0, CANVAS_HEIGHT, temporary.as_mut_ptr());
    }
}

/// Change one logical color pixel. Every source pixel expands to a 2x2 block
/// in the 512x512 NV12 stream, as in the original C color mode.
#[inline]
pub fn set_pixel(x: usize, y: usize, color: u8) {
    let mode = camera_mode();
    let width = if mode == CameraMode::Monochrome { WIDTH } else { WIDTH / 2 };
    if x >= width || y >= CANVAS_HEIGHT {
        return;
    }

    let palette_index = (color & 0x3f) as usize;
    unsafe {
        if mode == CameraMode::Monochrome {
            let offset = y * WIDTH + x;
            let value = GRAYSCALE[palette_index];
            if offset < CANVAS_MAIN_BYTES {
                core::ptr::addr_of_mut!(DEMO_CANVAS.0).cast::<u8>().add(offset).write(value);
            } else {
                core::ptr::addr_of_mut!(DEMO_CANVAS_TAIL.0)
                    .cast::<u8>().add(offset - CANVAS_MAIN_BYTES).write(value);
            }
        } else {
            let offset = (y * (WIDTH / 2) + x) * 2;
            let [u, v] = UV_PALETTE[palette_index];
            let destination = if offset < CANVAS_MAIN_BYTES {
                core::ptr::addr_of_mut!(DEMO_CANVAS.0).cast::<u8>().add(offset)
            } else {
                core::ptr::addr_of_mut!(DEMO_CANVAS_TAIL.0)
                    .cast::<u8>().add(offset - CANVAS_MAIN_BYTES)
            };
            destination.write(u);
            destination.add(1).write(v);
        }
    }
}

/// Write one packed BK video byte directly to the compact canvas.
///
/// The byte expands to eight luma pixels in monochrome mode or four Cb/Cr
/// pairs in color mode. The packed group is eight bytes in either mode, and
/// its aligned offset cannot straddle the SRAM/CCM canvas boundary.
#[inline]
pub(crate) fn set_bk_video_byte(byte_x: usize, y: usize, byte: u8, monochrome: bool) -> bool {
    let mode = camera_mode();
    if y >= CANVAS_HEIGHT {
        return false;
    }

    // During a deferred camera-mode transition, preserve set_pixel()'s
    // original behavior until the new mode becomes active.
    if (mode == CameraMode::Monochrome) != monochrome {
        if monochrome {
            let x = byte_x * 8;
            if x >= canvas_width_for(mode) { return false; }
            for i in 0..8 {
                let color = if byte & (1 << i) != 0 { 0x3f } else { 0x00 };
                set_pixel(x + i, y, color);
            }
        } else {
            let x = byte_x * 4;
            if x >= canvas_width_for(mode) { return false; }
            for i in 0..4 {
                let color = match (byte >> (i * 2)) & 0x03 {
                    0 => 0x00, 1 => 0x30, 2 => 0x0c, _ => 0x03,
                };
                set_pixel(x + i, y, color);
            }
        }
        return true;
    }

    let offset = if monochrome {
        if byte_x >= WIDTH / 8 { return false; }
        y * WIDTH + byte_x * 8
    } else {
        if byte_x >= (WIDTH / 2) / 4 { return false; }
        (y * (WIDTH / 2) + byte_x * 4) * 2
    };

    // CANVAS_MAIN_BYTES is divisible by eight, as are all packed-byte
    // offsets, so this eight-byte span is wholly within one backing region.
    unsafe {
        let destination = if offset < CANVAS_MAIN_BYTES {
            core::ptr::addr_of_mut!(DEMO_CANVAS.0).cast::<u8>().add(offset)
        } else {
            core::ptr::addr_of_mut!(DEMO_CANVAS_TAIL.0)
                .cast::<u8>().add(offset - CANVAS_MAIN_BYTES)
        };

        if monochrome {
            let set = GRAYSCALE[0x3f];
            let clear = GRAYSCALE[0];
            for i in 0..8 {
                destination.add(i).write(if byte & (1 << i) != 0 { set } else { clear });
            }
        } else {
            for i in 0..4 {
                let palette_index = match (byte >> (i * 2)) & 0x03 {
                    0 => 0,
                    1 => 0x30,
                    2 => 0x0c,
                    _ => 0x03,
                };
                let [u, v] = UV_PALETTE[palette_index];
                destination.add(i * 2).write(u);
                destination.add(i * 2 + 1).write(v);
            }
        }
    }
    true
}

/// Read a BK color pixel back from the NV12 source canvas.
pub(crate) fn get_bk_color_pixel(x: usize, y: usize) -> u8 {
    if x >= 256 || y >= CANVAS_HEIGHT {
        return 0;
    }
    let offset = (y * 256 + x) * 2;
    unsafe {
        let base = core::ptr::addr_of!(DEMO_CANVAS.0).cast::<u8>();
        let tail = core::ptr::addr_of!(DEMO_CANVAS_TAIL.0).cast::<u8>();
        let (cb, cr) = if offset < CANVAS_MAIN_BYTES {
            (base.add(offset).read(), base.add(offset + 1).read())
        } else {
            let o = offset - CANVAS_MAIN_BYTES;
            (tail.add(o).read(), tail.add(o + 1).read())
        };
        let uv = [cb, cr];
        if uv == UV_PALETTE[0] { 0x00 }
        else if uv == UV_PALETTE[48] { 0x30 }
        else if uv == UV_PALETTE[12] { 0x0c }
        else if uv == UV_PALETTE[3] { 0x03 }
        else { 0x00 }
    }
}

/// Read a BK monochrome pixel as zero (black) or nonzero (white).
pub(crate) fn get_bk_bw_pixel(x: usize, y: usize) -> u8 {
    if x >= WIDTH || y >= CANVAS_HEIGHT {
        return 0;
    }
    let offset = y * WIDTH + x;
    unsafe {
        let value = if offset < CANVAS_MAIN_BYTES {
            core::ptr::addr_of!(DEMO_CANVAS.0).cast::<u8>().add(offset).read()
        } else {
            core::ptr::addr_of!(DEMO_CANVAS_TAIL.0).cast::<u8>()
                .add(offset - CANVAS_MAIN_BYTES).read()
        };
        if value == 0 { 0 } else { 1 }
    }
}

/// Draw a USB diagnostic message at the next text row, starting at the
/// top-left corner (X=0, Y=0). Successive messages advance down the canvas
/// and wrap to row zero after the last row. Call from the main loop, not an ISR.
static USB_STATUS_ROW: AtomicU32 = AtomicU32::new(0);
// Keyboard input is appended on a dedicated line; ordinary status messages
// continue to consume their own rows.
static KEYBOARD_TEXT_COLUMN: AtomicU32 = AtomicU32::new(0);

pub fn show_usb_status(message: &[u8]) {
    use crate::font8x8::FONT8X8;

    let max_chars = canvas_width() / 8;
    const FOREGROUND: u8 = 63;
    const BACKGROUND: u8 = 0;
    const TEXT_ROWS: usize = CANVAS_HEIGHT / 8;

    let text_row = USB_STATUS_ROW.fetch_add(1, Ordering::Relaxed) as usize % TEXT_ROWS;

    for character_index in 0..max_chars {
        let character = message.get(character_index).copied().unwrap_or(b' ');
        // The font table uses one-based character indices. Restrict input to
        // printable ASCII to keep arbitrary diagnostic strings in bounds.
        let character = if (1..=128).contains(&character) { character } else { b'?' };
        let glyph = (character as usize - 1) * 8;

        for row in 0..8 {
            let bits = FONT8X8[glyph + row];
            for column in 0..8 {
                let color = if bits & (0x80 >> column) != 0 {
                    FOREGROUND
                } else {
                    BACKGROUND
                };
                set_pixel(
                    character_index * 8 + column,
                    text_row * 8 + row,
                    color,
                );
            }
        }
    }
}

/// Draw one fixed 8-pixel text row for the NAND file browser.
pub fn show_file_browser_line(row: usize, message: &[u8], selected: bool) {
    use crate::font8x8::FONT8X8;

    if row >= CANVAS_HEIGHT / 8 {
        return;
    }
    let columns = canvas_width() / 8;
    // Palette indices use BBGGRR: all channels set gives white; blue-only is 00010000.
    const FOREGROUND_WHITE: u8 = 0b00111111;
    const BACKGROUND_BLUE: u8 = 0b00010000;
    let foreground = FOREGROUND_WHITE;
    let background = BACKGROUND_BLUE;

    for column in 0..columns {
        let raw = message.get(column).copied().unwrap_or(b' ');
        let character = if (32..=126).contains(&raw) { raw } else { b'?' };
        let glyph = (character as usize - 1) * 8;
        for y in 0..8 {
            let bits = FONT8X8[glyph + y];
            for x in 0..8 {
                let color = if bits & (0x80 >> x) != 0 {
                    foreground
                } else {
                    background
                };
                set_pixel(column * 8 + x, row * 8 + y, color);
            }
        }
    }
}

/// Update the emulator throughput row on the camera image.
#[cfg(feature = "cpu-cycle-diag")]
pub fn show_performance_status(cycles_per_second: u32) {
    let mut line = [b' '; 32];
    let mut pos = 0usize;
    append_bytes(&mut line, &mut pos, b"C");
    append_decimal(&mut line, &mut pos, cycles_per_second);
    show_file_browser_line(28, &line[..pos], false);
}

/// Display one-second audio and UVC buffer-flow counters.
#[cfg(feature = "cpu-cycle-diag")]
pub fn show_performance_details(
    audio_rendered: u32,
    audio_underruns: u32,
    audio_high_water: u16,
    video_batches: u32,
    video_repeats: u32,
    video_cpu_percent: u32,
    video_max_us: u32,
) {
    let mut audio = [b' '; 32];
    let mut pos = 0usize;
    append_bytes(&mut audio, &mut pos, b"A R:");
    append_decimal(&mut audio, &mut pos, audio_rendered);
    append_bytes(&mut audio, &mut pos, b" U:");
    append_decimal(&mut audio, &mut pos, audio_underruns);
    append_bytes(&mut audio, &mut pos, b" H:");
    append_decimal(&mut audio, &mut pos, audio_high_water as u32);
    show_file_browser_line(29, &audio[..pos], false);

    let mut video = [b' '; 32];
    let mut pos = 0usize;
    append_bytes(&mut video, &mut pos, b"V B:");
    append_decimal(&mut video, &mut pos, video_batches);
    append_bytes(&mut video, &mut pos, b" R:");
    append_decimal(&mut video, &mut pos, video_repeats);
    append_bytes(&mut video, &mut pos, b" C:");
    append_decimal(&mut video, &mut pos, video_cpu_percent);
    append_bytes(&mut video, &mut pos, b"% M:");
    append_decimal(&mut video, &mut pos, video_max_us);
    append_bytes(&mut video, &mut pos, b"us");
    show_file_browser_line(30, &video[..pos], false);
}

#[cfg(feature = "cpu-cycle-diag")]
pub fn show_video_phase_status(y_cpu_percent: u32, uv_cpu_percent: u32) {
    let mut line = [b' '; 32];
    let mut pos = 0usize;
    append_bytes(&mut line, &mut pos, b"Y:");
    append_decimal(&mut line, &mut pos, y_cpu_percent);
    append_bytes(&mut line, &mut pos, b"% UV:");
    append_decimal(&mut line, &mut pos, uv_cpu_percent);
    append_bytes(&mut line, &mut pos, b"%");
    show_file_browser_line(31, &line[..pos], false);
}

fn append_bytes(out: &mut [u8; 32], pos: &mut usize, bytes: &[u8]) {
    for &byte in bytes {
        if *pos == out.len() { break; }
        out[*pos] = byte;
        *pos += 1;
    }
}

fn append_decimal(out: &mut [u8; 32], pos: &mut usize, mut value: u32) {
    let mut digits = [0u8; 10];
    let mut count = 0usize;
    loop {
        digits[count] = b'0' + (value % 10) as u8;
        count += 1;
        value /= 10;
        if value == 0 { break; }
    }
    while count != 0 && *pos < out.len() {
        count -= 1;
        out[*pos] = digits[count];
        *pos += 1;
    }
}

/// Append keyboard input to one persistent canvas row instead of allocating a
/// new status row for every key report. When the line fills, overwrite it from
/// the beginning; a single key press never advances the row.
pub fn show_keyboard_input(message: &[u8]) {
    use crate::font8x8::FONT8X8;

    let max_chars = canvas_width() / 8;
    const FOREGROUND: u8 = 63;
    const BACKGROUND: u8 = 0;
    const TEXT_ROW: usize = CANVAS_HEIGHT / 8 - 2;

    for &character in message {
        let column = KEYBOARD_TEXT_COLUMN.fetch_add(1, Ordering::Relaxed) as usize % max_chars;
        let character = if (1..=128).contains(&character) { character } else { b'?' };
        let glyph = (character as usize - 1) * 8;

        for row in 0..8 {
            let bits = FONT8X8[glyph + row];
            for x in 0..8 {
                let color = if bits & (0x80 >> x) != 0 { FOREGROUND } else { BACKGROUND };
                set_pixel(column * 8 + x, TEXT_ROW * 8 + row, color);
            }
        }
    }
}

/// Draw a persistent status message after a USB keyboard is enumerated.
pub fn show_keyboard_connected() {
    show_usb_status(b"USB KEYBOARD CONNECTED");
}

/// Update the top-center clock text directly in the source canvas.
pub fn update_clock_from_rtc() {
    let text = crate::rtc::display_time();
    const CLOCK_ATTRIBUTE: u16 = 0x2a10;

    let width = canvas_width();
    let left = (width - text.len() * 8) / 2;
    let top = CANVAS_HEIGHT - 8;
    let foreground = (CLOCK_ATTRIBUTE >> 8) as u8 & 0x3f;
    let background = CLOCK_ATTRIBUTE as u8 & 0x3f;
    draw_text_pixels(&text, left, top, foreground, background);

    CLOCK_VERSION.fetch_add(1, Ordering::Relaxed);
}

fn draw_text_pixels(text: &[u8; 10], left: usize, top: usize, foreground: u8, background: u8) {
    use crate::font8x8::FONT8X8;

    // B/W mode has a 512x256 one-byte-per-pixel canvas. Draw native 8x8
    // glyphs centered horizontally in the final (31st) text row.
    for (character_index, character) in text.iter().enumerate() {
        let glyph = (*character as usize - 1) * 8;
        for row in 0..8 {
            let bits = FONT8X8[glyph + row];
            for column in 0..8 {
                let color = if bits & (0x80 >> column) != 0 {
                    foreground
                } else {
                    background
                };
                set_pixel(left + character_index * 8 + column, top + row, color);
            }
        }
    }
}

const GRAYSCALE: [u8; 64] = [
    0,6,12,18,61,67,73,79,122,128,134,140,183,189,195,201,
    18,24,30,36,79,85,91,97,140,146,152,158,201,207,213,219,
    36,42,48,54,97,103,109,115,158,164,170,176,219,225,231,237,
    54,60,66,72,115,121,127,133,176,182,188,194,237,243,249,255,
];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Packet {
    pub len: usize,
    pub frame_complete: bool,
}

/// Emits the same compact demo image repeatedly as UVC payloads.
pub struct Nv12DemoImage {
    offset: usize,
    frame_id: bool,
    cpu_available_percent: u8,
    cpu_generate_percent: u8,
    cpu_submit_percent: u8,
    cached_y_source_row: u16,
    cached_uv_source_row: u16,
    cached_clock_version: u32,
    #[cfg(feature = "cpu-cycle-diag")]
    profile_countdown: u8,
}

impl Nv12DemoImage {
    /// Claim the shared row buffers for the single live camera instance.
    /// Returns `None` while another camera instance owns them.
    pub fn try_new() -> Option<Self> {
        CAMERA_IN_USE
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .ok()?;
        Some(Self {
            offset: 0,
            frame_id: false,
            cpu_available_percent: 100,
            cpu_generate_percent: 0,
            cpu_submit_percent: 0,
            cached_y_source_row: u16::MAX,
            cached_uv_source_row: u16::MAX,
            cached_clock_version: CLOCK_VERSION.load(Ordering::Relaxed),
            #[cfg(feature = "cpu-cycle-diag")]
            profile_countdown: 0,
        })
    }

    pub(crate) fn reset(&mut self) {
        self.offset = 0;
        self.frame_id = false;
        self.cpu_available_percent = 100;
        self.cpu_generate_percent = 0;
        self.cpu_submit_percent = 0;
        self.cached_y_source_row = u16::MAX;
        self.cached_uv_source_row = u16::MAX;
        self.cached_clock_version = CLOCK_VERSION.load(Ordering::Relaxed);
        #[cfg(feature = "cpu-cycle-diag")]
        {
            self.profile_countdown = 0;
        }
    }

    /// Set the estimated CPU percentage available outside measured UVC work.
    pub fn set_cpu_metrics(&mut self, available: u8, generate: u8, submit: u8) {
        self.cpu_available_percent = available.min(100);
        self.cpu_generate_percent = generate.min(100);
        self.cpu_submit_percent = submit.min(100);
    }

    /// Fill one UVC payload transfer, which may span multiple USB packets in
    /// a high-bandwidth HS microframe. UVC places a single payload header at
    /// the start of the transfer; continuation transactions contain video
    /// bytes only.
    pub fn next_payload_transfer(
        &mut self,
        out: &mut [u8],
        max_transfer_bytes: usize,
    ) -> Option<Packet> {
        if self.offset == 0 {
            if apply_pending_camera_mode() {
                self.cached_y_source_row = u16::MAX;
                self.cached_uv_source_row = u16::MAX;
                self.cached_clock_version = CLOCK_VERSION.load(Ordering::Relaxed);
            }
        }
        if out.len() < UVC_HEADER_BYTES {
            return None;
        }

        let transfer_capacity = max_transfer_bytes.min(out.len());
        if transfer_capacity < UVC_HEADER_BYTES {
            return None;
        }
        let remaining = FRAME_BYTES - self.offset;
        let data_len = remaining.min(transfer_capacity - UVC_HEADER_BYTES);
        if data_len == 0 {
            return None;
        }

        let complete = data_len == remaining;
        out[0] = UVC_HEADER_BYTES as u8;
        // EOH terminates this four-byte header; FID toggles per frame and
        // EOF marks the last payload of each frame.
        out[1] = 0x80 | u8::from(self.frame_id) | if complete { 0x02 } else { 0 };
        out[2] = 0; // PTS not present
        out[3] = 0; // SCR not present
        // Render the live NV12 image from the compact camera canvas.
        self.fill_nv12(
            self.offset,
            &mut out[UVC_HEADER_BYTES..UVC_HEADER_BYTES + data_len],
        );

        self.offset += data_len;
        if complete {
            self.offset = 0;
            self.frame_id = !self.frame_id;
        }

        Some(Packet {
            len: UVC_HEADER_BYTES + data_len,
            frame_complete: complete,
        })
    }
}

impl Drop for Nv12DemoImage {
    fn drop(&mut self) {
        CAMERA_IN_USE.store(false, Ordering::Release);
    }
}


impl Nv12DemoImage {
    /// Render sequential NV12 bytes from the mode-specific compact canvas.
    /// Color doubles both axes from 256x256 UV pairs; BW doubles the 512x256
    /// luma canvas vertically and uses neutral chroma.
    fn fill_nv12(&mut self, offset: usize, out: &mut [u8]) {
        let mode = camera_mode();
        let mut pos = offset;
        let mut written = 0;
        #[cfg(feature = "cpu-cycle-diag")]
        let profile_sample = {
            self.profile_countdown += 1;
            if self.profile_countdown >= VIDEO_PHASE_SAMPLE_INTERVAL {
                self.profile_countdown = 0;
                true
            } else {
                false
            }
        };
        #[cfg(feature = "cpu-cycle-diag")]
        let (mut y_phase_cycles, mut uv_phase_cycles) = (0u32, 0u32);
        while written < out.len() && pos < FRAME_BYTES {
            if pos < Y_PLANE_BYTES {
                let output_row = pos / WIDTH;
                let x = pos % WIDTH;
                let source_row = (output_row / 2) as u16;
                #[cfg(feature = "cpu-cycle-diag")]
                let phase_start = if profile_sample {
                    Some(cortex_m::peripheral::DWT::cycle_count())
                } else {
                    None
                };
                self.prepare_y_row(source_row);
                let run = (WIDTH - x)
                    .min(Y_PLANE_BYTES - pos)
                    .min(out.len() - written);
                unsafe {
                    crate::copy_words::copy_words(
                        y_row_ptr().add(x), out.as_mut_ptr().add(written), run,
                    );
                }
                #[cfg(feature = "cpu-cycle-diag")]
                if let Some(start) = phase_start {
                    y_phase_cycles = y_phase_cycles.wrapping_add(
                        cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start),
                    );
                }
                pos += run;
                written += run;
            } else {
                let chroma_pos = pos - Y_PLANE_BYTES;
                let x = chroma_pos % WIDTH;
                // In color mode the UV plane is a contiguous copy of the
                // source canvas. Let the copy cross source-row boundaries;
                // copy_canvas_range() already handles the SRAM/CCM split.
                // Monochrome still needs row-sized chunks to prepare neutral UV.
                let run = if mode == CameraMode::Monochrome {
                    (WIDTH - x).min(FRAME_BYTES - pos).min(out.len() - written)
                } else {
                    (FRAME_BYTES - pos).min(out.len() - written)
                };
                #[cfg(feature = "cpu-cycle-diag")]
                let phase_start = if profile_sample {
                    Some(cortex_m::peripheral::DWT::cycle_count())
                } else {
                    None
                };
                if mode == CameraMode::Monochrome {
                    let source_row = (chroma_pos / WIDTH) as u16;
                    self.prepare_uv_row(source_row);
                    unsafe {
                        crate::copy_words::copy_words(uv_row_ptr().add(x), out.as_mut_ptr().add(written), run);
                    }
                } else {
                    unsafe {
                        copy_canvas_range(chroma_pos, out.as_mut_ptr().add(written), run);
                    }
                }
                #[cfg(feature = "cpu-cycle-diag")]
                if let Some(start) = phase_start {
                    uv_phase_cycles = uv_phase_cycles.wrapping_add(
                        cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start),
                    );
                }
                pos += run;
                written += run;
            }
        }
        out[written..].fill(0);
        #[cfg(feature = "cpu-cycle-diag")]
        if profile_sample {
            VIDEO_Y_PHASE_CYCLES.fetch_add(y_phase_cycles, Ordering::Relaxed);
            VIDEO_UV_PHASE_CYCLES.fetch_add(uv_phase_cycles, Ordering::Relaxed);
        }
    }

    fn prepare_y_row(&mut self, source_row: u16) {
        if source_row == 0 {
            let clock_version = CLOCK_VERSION.load(Ordering::Relaxed);
            if self.cached_clock_version != clock_version {
                self.cached_clock_version = clock_version;
                self.cached_y_source_row = u16::MAX;
            }
        }
        if self.cached_y_source_row == source_row {
            return;
        }
        let mode = camera_mode();
        let source_bytes_per_pixel = if mode == CameraMode::Monochrome { 1 } else { 2 };
        let source_width = canvas_width_for(mode);
        let row_offset = source_row as usize * source_width * source_bytes_per_pixel;
        let mut pixel_x = 0;
        while pixel_x < source_width {
            let byte_offset = row_offset + pixel_x * source_bytes_per_pixel;
            let (source, available_bytes) = canvas_read_chunk(byte_offset);
            let chunk_pixels = (source_width - pixel_x).min(available_bytes / source_bytes_per_pixel);
            let mut i = 0;
            if mode == CameraMode::Monochrome {
                let destination = unsafe { y_row_ptr().add(pixel_x) };
                // Process eight luma pixels per iteration to reduce loop
                // control overhead while keeping the byte-for-byte copy.
                while i + 8 <= chunk_pixels {
                    unsafe {
                        destination.add(i).write(source.add(i).read());
                        destination.add(i + 1).write(source.add(i + 1).read());
                        destination.add(i + 2).write(source.add(i + 2).read());
                        destination.add(i + 3).write(source.add(i + 3).read());
                        destination.add(i + 4).write(source.add(i + 4).read());
                        destination.add(i + 5).write(source.add(i + 5).read());
                        destination.add(i + 6).write(source.add(i + 6).read());
                        destination.add(i + 7).write(source.add(i + 7).read());
                    }
                    i += 8;
                }
                while i < chunk_pixels {
                    unsafe { destination.add(i).write(source.add(i).read()) };
                    i += 1;
                }
            } else {
                let source_pairs = source.cast::<u16>();
                let destination = unsafe { y_row_ptr().cast::<u16>().add(pixel_x) };

                // Process eight Cb/Cr pairs per iteration. Keep the
                // original eight-pixel unrolling, now reading the lookup
                // table from SRAM rather than Flash.
                while i + 8 <= chunk_pixels {
                    let p0 = unsafe { source_pairs.add(i).read() };
                    let p1 = unsafe { source_pairs.add(i + 1).read() };
                    let p2 = unsafe { source_pairs.add(i + 2).read() };
                    let p3 = unsafe { source_pairs.add(i + 3).read() };
                    let p4 = unsafe { source_pairs.add(i + 4).read() };
                    let p5 = unsafe { source_pairs.add(i + 5).read() };
                    let p6 = unsafe { source_pairs.add(i + 6).read() };
                    let p7 = unsafe { source_pairs.add(i + 7).read() };
                    let h0 = (p0 as u8 ^ (p0 >> 8) as u8) as usize;
                    let h1 = (p1 as u8 ^ (p1 >> 8) as u8) as usize;
                    let h2 = (p2 as u8 ^ (p2 >> 8) as u8) as usize;
                    let h3 = (p3 as u8 ^ (p3 >> 8) as u8) as usize;
                    let h4 = (p4 as u8 ^ (p4 >> 8) as u8) as usize;
                    let h5 = (p5 as u8 ^ (p5 >> 8) as u8) as usize;
                    let h6 = (p6 as u8 ^ (p6 >> 8) as u8) as usize;
                    let h7 = (p7 as u8 ^ (p7 >> 8) as u8) as usize;
                    unsafe {
                        destination.add(i).write(duplicated_y_lookup(h0));
                        destination.add(i + 1).write(duplicated_y_lookup(h1));
                        destination.add(i + 2).write(duplicated_y_lookup(h2));
                        destination.add(i + 3).write(duplicated_y_lookup(h3));
                        destination.add(i + 4).write(duplicated_y_lookup(h4));
                        destination.add(i + 5).write(duplicated_y_lookup(h5));
                        destination.add(i + 6).write(duplicated_y_lookup(h6));
                        destination.add(i + 7).write(duplicated_y_lookup(h7));
                    }
                    i += 8;
                }
                while i < chunk_pixels {
                    let pair = unsafe { source_pairs.add(i).read() };
                    let hash = pair as u8 ^ (pair >> 8) as u8;
                    unsafe { destination.add(i).write(duplicated_y_lookup(hash as usize)) };
                    i += 1;
                }
            }
            pixel_x += chunk_pixels;
        }
        self.cached_y_source_row = source_row;
    }

    fn prepare_uv_row(&mut self, source_row: u16) {
        if self.cached_uv_source_row == source_row {
            return;
        }
        if camera_mode() == CameraMode::Monochrome {
            unsafe { uv_row_ptr().write_bytes(128, WIDTH) };
        } else {
            unsafe { copy_canvas_range(source_row as usize * WIDTH, uv_row_ptr(), WIDTH) };
        }
        self.cached_uv_source_row = source_row;
    }
}

#[inline(always)]
unsafe fn y_row_ptr() -> *mut u8 {
    unsafe {
    core::ptr::addr_of_mut!(ROW_BUFFERS.0).cast::<u8>()

    }
}

#[inline(always)]
unsafe fn uv_row_ptr() -> *mut u8 {
    unsafe {
    core::ptr::addr_of_mut!(ROW_BUFFERS.0).cast::<u8>().add(WIDTH)

    }
}

/// Copy an aligned region from the C-style main SRAM + CCM canvas split.
unsafe fn copy_canvas_range(mut offset: usize, mut destination: *mut u8, mut bytes: usize) {
    unsafe {
    while bytes != 0 {
        let (source, available) = if offset < CANVAS_MAIN_BYTES {
            (
                core::ptr::addr_of!(DEMO_CANVAS.0).cast::<u8>().add(offset),
                CANVAS_MAIN_BYTES - offset,
            )
        } else {
            let tail_offset = offset - CANVAS_MAIN_BYTES;
            (
                core::ptr::addr_of!(DEMO_CANVAS_TAIL.0).cast::<u8>().add(tail_offset),
                CANVAS_TAIL_BYTES - tail_offset,
            )
        };
        let run = bytes.min(available);
        crate::copy_words::copy_words(source, destination, run);
        offset += run;
        destination = destination.add(run);
        bytes -= run;
    }

    }
}

