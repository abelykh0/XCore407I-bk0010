//! Compact color canvas and NV12 UVC packetizer.
//!
//! 128 KiB of source data split between ordinary SRAM and CCM, with one 512-byte luma row cache in SRAM. Color stores
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

use crate::palette::{UV_PALETTE, Y_HASH_TABLE, GRAYSCALE};

const fn make_duplicated_y_table() -> [u16; 256] {
    let mut table = [0; 256];
    let mut i = 0;
    while i < table.len() {
        let y = Y_HASH_TABLE[i] as u16;
        table[i] = y | (y << 8);
        i += 1;
    }
    // Palette color 0x15 is neutral gray [128, 127]. Force its luma
    // to match the extended-area monochrome background exactly.
    let gray_hash = (UV_PALETTE[0x15][0] ^ UV_PALETTE[0x15][1]) as usize;
    table[gray_hash] = 0xAAAA;
    table
}
const DUPLICATED_Y_TABLE: [u16; 256] = make_duplicated_y_table();
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum CameraMode {
    Color = 0,
    Monochrome = 1,
}

static ACTIVE_CAMERA_MODE: AtomicU32 = AtomicU32::new(CameraMode::Color as u32);
static PENDING_CAMERA_MODE: AtomicU32 = AtomicU32::new(u32::MAX);
static VISIBLE_CANVAS_ROWS: AtomicU32 = AtomicU32::new(CANVAS_HEIGHT as u32);

pub(crate) fn set_visible_canvas_rows(rows: usize) {
    VISIBLE_CANVAS_ROWS.store(rows.min(CANVAS_HEIGHT) as u32, Ordering::Release);
}

#[inline]
fn visible_canvas_rows() -> usize {
    VISIBLE_CANVAS_ROWS.load(Ordering::Acquire) as usize
}
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

/// True while the renderer still needs to apply a requested BK display mode.
#[inline]
pub fn camera_mode_change_pending() -> bool {
    let requested = PENDING_CAMERA_MODE.load(Ordering::Acquire);
    requested != u32::MAX && requested != ACTIVE_CAMERA_MODE.load(Ordering::Acquire)
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

    let old_mode = camera_mode();
    let new_mode = if requested == CameraMode::Monochrome as u32 {
        CameraMode::Monochrome
    } else {
        CameraMode::Color
    };
    crate::emulator::bk_video::convert_canvas_mode(old_mode, new_mode);

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

pub const WIDTH: usize = 512;
pub const HEIGHT: usize = 512;
pub const Y_PLANE_BYTES: usize = WIDTH * HEIGHT;
pub const FRAME_BYTES: usize = Y_PLANE_BYTES + (WIDTH * HEIGHT / 2);
pub const UVC_HEADER_BYTES: usize = 4;

pub const CANVAS_HEIGHT: usize = HEIGHT / 2;

const TEXT_FOREGROUND_WHITE: u8 = 0b0011_1111;
const TEXT_BACKGROUND_BLACK: u8 = 0;
const TEXT_BACKGROUND_BLUE: u8 = 0b0001_0000;
pub const CANVAS_BYTES: usize = WIDTH * CANVAS_HEIGHT;
pub const CANVAS_TAIL_BYTES: usize = 46_848;
pub const CANVAS_MAIN_BYTES: usize = CANVAS_BYTES - CANVAS_TAIL_BYTES;

#[derive(Clone, Copy)]
#[repr(C, align(4))]
struct AlignedCanvas<const N: usize>([u8; N]);
#[repr(C, align(8))]
struct DmaAlignedCanvas<const N: usize>([u8; N]);

static mut DEMO_CANVAS: DmaAlignedCanvas<CANVAS_MAIN_BYTES> = DmaAlignedCanvas([0; CANVAS_MAIN_BYTES]);
#[repr(C, align(8))]
union CanvasTailStorage {
    canvas: AlignedCanvas<CANVAS_TAIL_BYTES>,
    cpu: [u8; CANVAS_TAIL_BYTES],
}

// DEMO_CANVAS and DEMO_CANVAS_TAIL are two physical segments of one logical
// 128 KiB BK canvas. In CDC mode, the main segment is reused for ZMODEM
// receiver/send staging, Ethernet DMA and HTTP RX/body storage plus the alternate stack; the CCM
// tail holds CPU-only smoltcp socket metadata and the TCP transmit buffer.
#[unsafe(link_section = ".ccmram.canvas")]
static mut DEMO_CANVAS_TAIL: CanvasTailStorage =
    CanvasTailStorage { canvas: AlignedCanvas([0; CANVAS_TAIL_BYTES]) };

pub(crate) const CDC_ZMODEM_STORAGE_BYTES: usize = 4096;
pub(crate) const CDC_ETHERNET_DMA_STORAGE_BYTES: usize = 2 * 16 + 2 * 1536 + 16 + 1536;
pub(crate) const CDC_HTTP_RX_BYTES: usize = 4096;
pub(crate) const CDC_HTTP_BODY_BYTES: usize = 62_416;
pub(crate) const CDC_HTTP_STORAGE_BYTES: usize =
    CDC_HTTP_RX_BYTES + CDC_HTTP_BODY_BYTES;
const CDC_ALT_STACK_BYTES: usize = 8 * 1024 + 768;
const CDC_ALT_STACK_BOTTOM: usize = CANVAS_MAIN_BYTES - CDC_ALT_STACK_BYTES;

// ZMODEM storage, DMA ring and HTTP buffers occupy the low end; the enlarged
// filesystem alternate stack grows down from the high end.
const _: () = assert!(
    CDC_ZMODEM_STORAGE_BYTES
        + CDC_ETHERNET_DMA_STORAGE_BYTES
        + CDC_HTTP_STORAGE_BYTES
        <= CDC_ALT_STACK_BOTTOM
);
const _: () = assert!(
    CDC_ZMODEM_STORAGE_BYTES
        + CDC_ETHERNET_DMA_STORAGE_BYTES
        + CDC_HTTP_STORAGE_BYTES
        == CDC_ALT_STACK_BOTTOM
);

pub(crate) fn cdc_alternate_stack_top() -> *mut u8 {
    unsafe {
        core::ptr::addr_of_mut!(DEMO_CANVAS.0)
            .cast::<u8>()
            .add(CANVAS_MAIN_BYTES)
    }
}

pub(crate) fn cdc_ethernet_dma_storage_ptr() -> *mut u8 {
    unsafe {
        core::ptr::addr_of_mut!(DEMO_CANVAS.0)
            .cast::<u8>()
            .add(CDC_ZMODEM_STORAGE_BYTES)
    }
}

pub(crate) fn cdc_ethernet_http_storage_ptr() -> *mut u8 {
    unsafe {
        core::ptr::addr_of_mut!(DEMO_CANVAS.0)
            .cast::<u8>()
            .add(CDC_ZMODEM_STORAGE_BYTES + CDC_ETHERNET_DMA_STORAGE_BYTES)
    }
}

pub(crate) fn cdc_ethernet_socket_storage_ptr() -> *mut u8 {
    unsafe {
        core::ptr::addr_of_mut!(DEMO_CANVAS_TAIL.cpu).cast::<u8>()
    }
}

#[repr(C, align(4))]
struct RowBuffer([u8; WIDTH]);
// CPU-only luma conversion scratch; keep it in SRAM to conserve CCM.
static mut ROW_BUFFER: RowBuffer = RowBuffer([0; WIDTH]);
static CAMERA_IN_USE: AtomicBool = AtomicBool::new(false);
// CDC mode uses the PC CONNECT renderer and does not consume the demo canvas.
// Its main-SRAM range holds shared ZMODEM receiver/send storage, the
// Ethernet DMA ring, HTTP buffers and alternate stack. The CCM tail holds smoltcp socket storage.
static CDC_ZMODEM_STORAGE_IN_USE: AtomicBool = AtomicBool::new(false);

pub(crate) fn claim_cdc_zmodem_storage() -> Option<(*mut u8, usize)> {
    CDC_ZMODEM_STORAGE_IN_USE
        .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
        .ok()?;
    let ptr = unsafe { core::ptr::addr_of_mut!(DEMO_CANVAS.0).cast::<u8>() };
    Some((ptr, CDC_ZMODEM_STORAGE_BYTES))
}

pub(crate) fn release_cdc_zmodem_storage() {
    CDC_ZMODEM_STORAGE_IN_USE.store(false, Ordering::Release);
}

static CLOCK_VERSION: AtomicU32 = AtomicU32::new(0);

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
                core::ptr::addr_of!(DEMO_CANVAS_TAIL.canvas.0)
                    .cast::<u8>()
                    .add(tail_offset),
                CANVAS_TAIL_BYTES - tail_offset,
            )
        }
    }
}


fn canvas_write_chunk(offset: usize) -> (*mut u8, usize) {
    unsafe {
        if offset < CANVAS_MAIN_BYTES {
            (
                core::ptr::addr_of_mut!(DEMO_CANVAS.0).cast::<u8>().add(offset),
                CANVAS_MAIN_BYTES - offset,
            )
        } else {
            let tail_offset = offset - CANVAS_MAIN_BYTES;
            (
                core::ptr::addr_of_mut!(DEMO_CANVAS_TAIL.canvas.0).cast::<u8>().add(tail_offset),
                CANVAS_TAIL_BYTES - tail_offset,
            )
        }
    }
}

/// Read a byte range from the logical canvas, which spans SRAM and CCM.
pub(crate) fn read_canvas(offset: usize, output: &mut [u8]) -> bool {
    let Some(end) = offset.checked_add(output.len()) else { return false; };
    if end > CANVAS_BYTES { return false; }
    let mut offset = offset;
    let mut copied = 0;
    while copied < output.len() {
        let (source, available) = canvas_read_chunk(offset);
        let count = available.min(output.len() - copied);
        unsafe {
            crate::copy_words::copy_words(source, output.as_mut_ptr().add(copied), count);
        }
        offset += count;
        copied += count;
    }
    true
}

/// Write a byte range into the logical canvas, which spans SRAM and CCM.
pub(crate) fn write_canvas(offset: usize, input: &[u8]) -> bool {
    let Some(end) = offset.checked_add(input.len()) else { return false; };
    if end > CANVAS_BYTES { return false; }
    let mut offset = offset;
    let mut copied = 0;
    while copied < input.len() {
        let (destination, available) = canvas_write_chunk(offset);
        let count = available.min(input.len() - copied);
        unsafe {
            crate::copy_words::copy_words(input.as_ptr().add(copied), destination, count);
        }
        offset += count;
        copied += count;
    }
    true
}

/// Copy between non-overlapping ranges of the split logical canvas.
pub(crate) fn copy_canvas(source_offset: usize, destination_offset: usize, bytes: usize) -> bool {
    let Some(source_end) = source_offset.checked_add(bytes) else { return false; };
    let Some(destination_end) = destination_offset.checked_add(bytes) else { return false; };
    if source_end > CANVAS_BYTES
        || destination_end > CANVAS_BYTES
        || (source_offset < destination_end && destination_offset < source_end)
    {
        return false;
    }
    let mut source_offset = source_offset;
    let mut destination_offset = destination_offset;
    let mut copied = 0;
    while copied < bytes {
        let (source, source_available) = canvas_read_chunk(source_offset);
        let (destination, destination_available) = canvas_write_chunk(destination_offset);
        let count = (bytes - copied).min(source_available).min(destination_available);
        unsafe { crate::copy_words::copy_words(source, destination, count); }
        source_offset += count;
        destination_offset += count;
        copied += count;
    }
    true
}

/// Fill the entire canvas with one uniform palette color.
///
/// `color` is a palette index (0..63), matching `set_pixel()`.
/// In color mode the corresponding Cb/Cr pair is written for every pixel;
/// in B/W mode the corresponding luma value is written for every pixel.
pub fn clear_demo_canvas(color: u8) {
    let mode = camera_mode();
    let value = color & 0x3f;
    if visible_canvas_rows() < CANVAS_HEIGHT {
        // Only the visible 64 rows may be cleared; the following 12 KiB
        // are live BK program RAM in extended mode.
        let visible_bytes = visible_canvas_rows() * WIDTH;
        unsafe {
            let main = core::ptr::addr_of_mut!(DEMO_CANVAS.0).cast::<u8>();
            if mode == CameraMode::Monochrome {
                main.write_bytes(GRAYSCALE[value as usize], visible_bytes);
            } else {
                let [u, v] = UV_PALETTE[value as usize];
                for i in 0..(visible_bytes / 2) {
                    main.add(i * 2).write(u);
                    main.add(i * 2 + 1).write(v);
                }
            }
        }
        return;
    }

    if mode == CameraMode::Monochrome {
        let luma = GRAYSCALE[value as usize];
        unsafe {
            core::ptr::addr_of_mut!(DEMO_CANVAS.0).cast::<u8>().write_bytes(luma, CANVAS_MAIN_BYTES);
            core::ptr::addr_of_mut!(DEMO_CANVAS_TAIL.canvas.0).cast::<u8>().write_bytes(luma, CANVAS_TAIL_BYTES);
        }
    } else {
        let [u, v] = UV_PALETTE[value as usize];
        let main_pixels = CANVAS_MAIN_BYTES / 2;
        let tail_pixels = CANVAS_TAIL_BYTES / 2;
        unsafe {
            let main = core::ptr::addr_of_mut!(DEMO_CANVAS.0).cast::<u8>();
            let tail = core::ptr::addr_of_mut!(DEMO_CANVAS_TAIL.canvas.0).cast::<u8>();
            for i in 0..main_pixels { main.add(i * 2).write(u); main.add(i * 2 + 1).write(v); }
            for i in 0..tail_pixels { tail.add(i * 2).write(u); tail.add(i * 2 + 1).write(v); }
        }
    }
}

/// Change one logical color pixel. Every source pixel expands to a 2x2 block
/// in the 512x512 NV12 stream, as in the original C color mode.
#[inline]
pub fn set_pixel(x: usize, y: usize, color: u8) {
    let mode = camera_mode();
    let width = if mode == CameraMode::Monochrome { WIDTH } else { WIDTH / 2 };
    if x >= width || y >= visible_canvas_rows() {
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
                core::ptr::addr_of_mut!(DEMO_CANVAS_TAIL.canvas.0)
                    .cast::<u8>().add(offset - CANVAS_MAIN_BYTES).write(value);
            }
        } else {
            let offset = (y * (WIDTH / 2) + x) * 2;
            let [u, v] = UV_PALETTE[palette_index];
            let destination = if offset < CANVAS_MAIN_BYTES {
                core::ptr::addr_of_mut!(DEMO_CANVAS.0).cast::<u8>().add(offset)
            } else {
                core::ptr::addr_of_mut!(DEMO_CANVAS_TAIL.canvas.0)
                    .cast::<u8>().add(offset - CANVAS_MAIN_BYTES)
            };
            destination.write(u);
            destination.add(1).write(v);
        }
    }
}

/// Draw a USB diagnostic message at the next text row, starting at the
/// top-left corner (X=0, Y=0). Successive messages advance down the canvas
/// and wrap to row zero after the last row. Call from the main loop, not an ISR.
static USB_STATUS_ROW: AtomicU32 = AtomicU32::new(0);
// Keyboard input is appended on a dedicated line; ordinary status messages
// continue to consume their own rows.
static KEYBOARD_TEXT_COLUMN: AtomicU32 = AtomicU32::new(0);

/// Reset text-overlay cursors after a BK machine reset without changing the
/// selected USB camera format or its transport state.
pub fn reset_demo_canvas_state() {
    USB_STATUS_ROW.store(0, Ordering::Release);
    KEYBOARD_TEXT_COLUMN.store(0, Ordering::Release);
    CLOCK_VERSION.fetch_add(1, Ordering::Relaxed);
}

pub fn show_usb_status(message: &[u8]) {
    use crate::font8x8::FONT8X8;

    let max_chars = canvas_width() / 8;
    const TEXT_ROWS: usize = CANVAS_HEIGHT / 8;

    let text_row = USB_STATUS_ROW.fetch_add(1, Ordering::Relaxed) as usize % TEXT_ROWS;

    for character_index in 0..max_chars {
        let character = message.get(character_index).copied().unwrap_or(b' ');
        // Font glyphs are indexed by character code minus one: all nonzero
        // byte values (1..=255) are valid; reserve zero as the missing glyph.
        let character = if character == 0 { b'?' } else { character };
        let glyph = (character as usize - 1) * 8;

        for row in 0..8 {
            let bits = FONT8X8[glyph + row];
            for column in 0..8 {
                let color = if bits & (0x80 >> column) != 0 {
                    TEXT_FOREGROUND_WHITE
                } else {
                    TEXT_BACKGROUND_BLACK
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
pub fn show_file_browser_line(row: usize, message: &[u8], _selected: bool) {
    use crate::font8x8::FONT8X8;

    if row >= CANVAS_HEIGHT / 8 {
        return;
    }
    let columns = canvas_width() / 8;
    // Palette indices use BBGGRR: all channels set gives white; blue-only is 00010000.
    let foreground = TEXT_FOREGROUND_WHITE;
    let background = TEXT_BACKGROUND_BLUE;

    for column in 0..columns {
        let raw = message.get(column).copied().unwrap_or(b' ');
        // The font has 255 glyphs, indexed by byte value minus one.
        // Only NUL (0) is reserved; every other byte is a valid glyph.
        let character = if raw == 0 { b'?' } else { raw };
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

/// Show the measured PDP clock rate over a one-second DWT interval.
#[cfg(feature = "cpu-cycle-diag")]
pub fn show_pdp_cycles_per_second(pdp_cycles_per_second: u32) {
    let mut line = [b' '; 32];
    let mut pos = 0usize;
    append_bytes(&mut line, &mut pos, b"PDP ");
    append_decimal(&mut line, &mut pos, pdp_cycles_per_second);
    show_file_browser_line(30, &line[..pos], false);
}

#[cfg(feature = "cpu-cycle-diag")]
fn append_bytes(out: &mut [u8; 32], pos: &mut usize, bytes: &[u8]) {
    for &byte in bytes {
        if *pos == out.len() { break; }
        out[*pos] = byte;
        *pos += 1;
    }
}

#[cfg(feature = "cpu-cycle-diag")]
fn append_decimal(out: &mut [u8; 32], pos: &mut usize, value: u32) {
    let mut digits = [0u8; 10];
    let digits = crate::format::u32_to_decimal(value, &mut digits);
    append_bytes(out, pos, digits);
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Packet {
    pub len: usize,
    pub frame_complete: bool,
}

/// Emits the same compact demo image repeatedly as UVC payloads.
pub struct Nv12DemoImage {
    offset: usize,
    frame_id: bool,
    cached_y_source_row: u16,
    cached_clock_version: u32,
    pc_connect_screen: bool,
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
            cached_y_source_row: u16::MAX,
            cached_clock_version: CLOCK_VERSION.load(Ordering::Relaxed),
            pc_connect_screen: false,
        })
    }

    /// Use a generated PC CONNECT frame without allocating or reading the demo canvas.
    pub fn set_pc_connect_screen(&mut self) {
        self.offset = 0;
        self.frame_id = false;
        self.pc_connect_screen = true;
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
        if self.pc_connect_screen {
            Self::fill_pc_connect_nv12(
                self.offset,
                &mut out[UVC_HEADER_BYTES..UVC_HEADER_BYTES + data_len],
            );
        } else {
            self.fill_nv12(
                self.offset,
                &mut out[UVC_HEADER_BYTES..UVC_HEADER_BYTES + data_len],
            );
        }

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
    /// Generate a fixed NV12 "PC CONNECT" frame without reading the emulator canvas.
    fn fill_pc_connect_nv12(offset: usize, out: &mut [u8]) {
        const TEXT: &[u8] = b"PC CONNECT";
        const TEXT_X: usize = (WIDTH - TEXT.len() * 16) / 2;
        const TEXT_Y: usize = (HEIGHT - 16) / 2;

        let mut position = offset;
        let mut written = 0usize;
        while written < out.len() && position < FRAME_BYTES {
            if position >= Y_PLANE_BYTES {
                let count = (FRAME_BYTES - position).min(out.len() - written);
                out[written..written + count].fill(128);
                position += count;
                written += count;
                continue;
            }

            let row = position / WIDTH;
            let x = position % WIDTH;
            let count = (WIDTH - x)
                .min(Y_PLANE_BYTES - position)
                .min(out.len() - written);
            let row_out = &mut out[written..written + count];
            row_out.fill(16);
            if (TEXT_Y..TEXT_Y + 16).contains(&row) {
                let glyph_row = (row - TEXT_Y) / 2;
                for (character_index, &character) in TEXT.iter().enumerate() {
                    let glyph = crate::font8x8::FONT8X8
                        [(character as usize - 1) * 8 + glyph_row];
                    for glyph_x in 0..8 {
                        if glyph & (0x80 >> glyph_x) == 0 { continue; }
                        let pixel_x = TEXT_X + character_index * 16 + glyph_x * 2;
                        if pixel_x >= x && pixel_x + 1 < x + count {
                            let local = pixel_x - x;
                            row_out[local] = 235;
                            row_out[local + 1] = 235;
                        }
                    }
                }
            }
            position += count;
            written += count;
        }
        out[written..].fill(128);
    }

    /// Render sequential NV12 bytes from the mode-specific compact canvas.
    /// Color doubles both axes from 256x256 UV pairs; BW doubles the 512x256
    /// luma canvas vertically and uses neutral chroma.
    fn fill_nv12(&mut self, offset: usize, out: &mut [u8]) {
        let mode = camera_mode();
        let mut pos = offset;
        let mut written = 0;
        while written < out.len() && pos < FRAME_BYTES {
            if pos < Y_PLANE_BYTES {
                let output_row = pos / WIDTH;
                let x = pos % WIDTH;
                let source_row = (output_row / 2) as u16;
                let run = (WIDTH - x)
                    .min(Y_PLANE_BYTES - pos)
                    .min(out.len() - written);
                if source_row >= visible_canvas_rows() as u16 {
                    // Keep the extended-memory area neutral light gray in
                    // both display modes; it is not part of the BK video RAM.
                    let background = GRAYSCALE[0x2a];
                    out[written..written + run].fill(background);
                } else {
                    self.prepare_y_row(source_row);
                    unsafe {
                        crate::copy_words::copy_words(
                            y_row_ptr().add(x), out.as_mut_ptr().add(written), run,
                        );
                    }
                }
                pos += run;
                written += run;
            } else {
                let chroma_pos = pos - Y_PLANE_BYTES;
                let x = chroma_pos % WIDTH;
                // In color mode the UV plane is a contiguous copy of the
                // source canvas. Let the copy cross source-row boundaries;
                // copy_canvas_range() already handles the SRAM/CCM split.
                let source_row = chroma_pos / WIDTH;
                let run = (WIDTH - x).min(FRAME_BYTES - pos).min(out.len() - written);
                if mode == CameraMode::Monochrome {
                    out[written..written + run].fill(128);
                } else if source_row >= visible_canvas_rows() {
                    let [u, v] = UV_PALETTE[0x15];
                    for (i, byte) in out[written..written + run].iter_mut().enumerate() {
                        *byte = if (x + i) & 1 == 0 { u } else { v };
                    }
                } else {
                    unsafe {
                        copy_canvas_range(chroma_pos, out.as_mut_ptr().add(written), run);
                    }
                }
                pos += run;
                written += run;
            }
        }
        out[written..].fill(0);
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

                // Process eight Cb/Cr pairs per iteration. The transform and
                // lookup are unchanged; wider unrolling reduces loop overhead
                // in this per-row hot path.
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
                    let y0 = DUPLICATED_Y_TABLE[h0];
                    let y1 = DUPLICATED_Y_TABLE[h1];
                    let y2 = DUPLICATED_Y_TABLE[h2];
                    let y3 = DUPLICATED_Y_TABLE[h3];
                    let y4 = DUPLICATED_Y_TABLE[h4];
                    let y5 = DUPLICATED_Y_TABLE[h5];
                    let y6 = DUPLICATED_Y_TABLE[h6];
                    let y7 = DUPLICATED_Y_TABLE[h7];

                    // Pack adjacent luma pixels to replace eight halfword
                    // stores with four aligned word stores.
                    unsafe {
                        let packed01: u32;
                        core::arch::asm!(
                            "pkhbt {out}, {low}, {high}, lsl #16",
                            out = lateout(reg) packed01,
                            low = in(reg) y0 as u32,
                            high = in(reg) y1 as u32,
                            options(pure, nomem, nostack),
                        );
                        let packed23: u32;
                        core::arch::asm!(
                            "pkhbt {out}, {low}, {high}, lsl #16",
                            out = lateout(reg) packed23,
                            low = in(reg) y2 as u32,
                            high = in(reg) y3 as u32,
                            options(pure, nomem, nostack),
                        );
                        let packed45: u32;
                        core::arch::asm!(
                            "pkhbt {out}, {low}, {high}, lsl #16",
                            out = lateout(reg) packed45,
                            low = in(reg) y4 as u32,
                            high = in(reg) y5 as u32,
                            options(pure, nomem, nostack),
                        );
                        let packed67: u32;
                        core::arch::asm!(
                            "pkhbt {out}, {low}, {high}, lsl #16",
                            out = lateout(reg) packed67,
                            low = in(reg) y6 as u32,
                            high = in(reg) y7 as u32,
                            options(pure, nomem, nostack),
                        );

                        let output = destination.add(i).cast::<u32>();
                        output.write(packed01);
                        output.add(1).write(packed23);
                        output.add(2).write(packed45);
                        output.add(3).write(packed67);
                    }
                    i += 8;
                }
                while i < chunk_pixels {
                    let pair = unsafe { source_pairs.add(i).read() };
                    let hash = pair as u8 ^ (pair >> 8) as u8;
                    unsafe { destination.add(i).write(DUPLICATED_Y_TABLE[hash as usize]) };
                    i += 1;
                }
            }
            pixel_x += chunk_pixels;
        }
        self.cached_y_source_row = source_row;
    }

}

#[inline(always)]
unsafe fn y_row_ptr() -> *mut u8 {
    unsafe {
    core::ptr::addr_of_mut!(ROW_BUFFER.0).cast::<u8>()

    }
}

/// Copy an aligned region from the SRAM + CCM canvas split.
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
                core::ptr::addr_of!(DEMO_CANVAS_TAIL.canvas.0).cast::<u8>().add(tail_offset),
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

