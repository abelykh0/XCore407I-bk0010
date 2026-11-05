//! BK-0010 video-memory adapter.
//!
//! The PDP-11 CPU continues to use its original C/C++ memory interface. This
//! module models only the BK video-memory window (0x4000..=0x7fff), translating
//! packed BK pixels to and from the existing Rust camera canvas.
//!
//! The canvas implementation is supplied through `BkCanvas`; this keeps the
//! video-memory format independent of the UVC renderer and avoids a second
//! framebuffer.

pub const VIDEO_RAM_START: u16 = 0x4000;
pub const VIDEO_RAM_END: u16 = 0x7fff;
pub const VIDEO_RAM_BYTES: usize = 0x4000;
pub const BYTES_PER_ROW: usize = 64;
pub const ROWS: usize = 256;

/// Pixel operations required from the camera canvas.
pub trait BkCanvas {
    /// Read or write a 6-bit RGB palette value (BBGGRR).
    fn get_color_pixel(&self, x: usize, y: usize) -> u8;
    fn set_color_pixel(&mut self, x: usize, y: usize, color: u8);

    /// Read or write a 6-bit grayscale palette value.
    fn get_bw_pixel(&self, x: usize, y: usize) -> u8;
    fn set_bw_pixel(&mut self, x: usize, y: usize, color: u8);
}

/// BK screen mode selected by RAM byte 0x0020.
///
/// A zero value selects 512x256 monochrome; any nonzero value selects
/// 256x256 four-color mode, matching the original `BkScreen` implementation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BkVideoMode {
    Monochrome,
    Color,
}

impl BkVideoMode {
    pub const fn from_mode_register(value: u8) -> Self {
        if value == 0 { Self::Monochrome } else { Self::Color }
    }
}

/// Adapter for the BK video-memory address range.
pub struct BkVideoMemory<C> {
    canvas: C,
    mode: BkVideoMode,
}

impl<C: BkCanvas> BkVideoMemory<C> {
    pub const fn new(canvas: C, mode_register: u8) -> Self {
        Self {
            canvas,
            mode: BkVideoMode::from_mode_register(mode_register),
        }
    }

    pub const fn mode(&self) -> BkVideoMode {
        self.mode
    }

    /// Apply a write to RAM[0x0020], changing the interpretation of video RAM.
    pub fn set_mode_register(&mut self, value: u8) {
        self.mode = BkVideoMode::from_mode_register(value);
    }

    pub fn canvas(&self) -> &C {
        &self.canvas
    }

    pub fn canvas_mut(&mut self) -> &mut C {
        &mut self.canvas
    }

    /// Read one byte from BK video RAM. Returns `None` outside 0x4000..=0x7fff.
    ///
    /// Pixels are packed least-significant first: eight 1-bit pixels in
    /// monochrome mode, or four 2-bit pixels in color mode.
    pub fn read_byte(&self, address: u16) -> Option<u8> {
        let offset = address.checked_sub(VIDEO_RAM_START)? as usize;
        if offset >= VIDEO_RAM_BYTES {
            return None;
        }

        let y = offset / BYTES_PER_ROW;
        let byte_x = offset % BYTES_PER_ROW;
        let mut result = 0u8;

        match self.mode {
            BkVideoMode::Monochrome => {
                let x = byte_x * 8;
                for pixel in 0..8 {
                    result >>= 1;
                    if self.canvas.get_bw_pixel(x + pixel, y) != 0 {
                        result |= 0x80;
                    }
                }
            }
            BkVideoMode::Color => {
                let x = byte_x * 4;
                for pixel in 0..4 {
                    let code = match self.canvas.get_color_pixel(x + pixel, y) & 0x3f {
                        0x00 => 0,
                        0x30 => 1, // blue
                        0x0c => 2, // green
                        0x03 => 3, // red
                        _ => 0,
                    };
                    result = (result >> 2) | (code << 6);
                }
            }
        }

        Some(result)
    }

    /// Write one byte to BK video RAM. Returns `false` outside the video window.
    pub fn write_byte(&mut self, address: u16, value: u8) -> bool {
        let Some(offset) = address.checked_sub(VIDEO_RAM_START).map(usize::from) else {
            return false;
        };
        if offset >= VIDEO_RAM_BYTES {
            return false;
        }

        let y = offset / BYTES_PER_ROW;
        let byte_x = offset % BYTES_PER_ROW;

        match self.mode {
            BkVideoMode::Monochrome => {
                let x = byte_x * 8;
                for pixel in 0..8 {
                    let color = if value & (1 << pixel) == 0 { 0 } else { 0x3f };
                    self.canvas.set_bw_pixel(x + pixel, y, color);
                }
            }
            BkVideoMode::Color => {
                let x = byte_x * 4;
                for pixel in 0..4 {
                    let color = match (value >> (pixel * 2)) & 0x03 {
                        0 => 0x00, // black
                        1 => 0x30, // blue
                        2 => 0x0c, // green
                        _ => 0x03, // red
                    };
                    self.canvas.set_color_pixel(x + pixel, y, color);
                }
            }
        }

        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone)]
    struct TestCanvas {
        color: [[u8; 256]; ROWS],
        bw: [[u8; 512]; ROWS],
    }

    impl TestCanvas {
        fn new() -> Self {
            Self { color: [[0; 256]; ROWS], bw: [[0; 512]; ROWS] }
        }
    }

    impl BkCanvas for TestCanvas {
        fn get_color_pixel(&self, x: usize, y: usize) -> u8 { self.color[y][x] }
        fn set_color_pixel(&mut self, x: usize, y: usize, color: u8) {
            self.color[y][x] = color;
        }
        fn get_bw_pixel(&self, x: usize, y: usize) -> u8 { self.bw[y][x] }
        fn set_bw_pixel(&mut self, x: usize, y: usize, color: u8) {
            self.bw[y][x] = color;
        }
    }

    #[test]
    fn color_byte_uses_two_bit_lsb_first_pixels() {
        let mut video = BkVideoMemory::new(TestCanvas::new(), 0xff);
        assert!(video.write_byte(VIDEO_RAM_START, 0b11_10_01_00));
        assert_eq!(video.canvas().color[0][0..4], [0x00, 0x30, 0x0c, 0x03]);
        assert_eq!(video.read_byte(VIDEO_RAM_START), Some(0b11_10_01_00));
    }

    #[test]
    fn monochrome_byte_uses_one_bit_lsb_first_pixels() {
        let mut video = BkVideoMemory::new(TestCanvas::new(), 0);
        assert!(video.write_byte(VIDEO_RAM_START, 0b1010_0101));
        assert_eq!(video.canvas().bw[0][0..8], [0x3f, 0, 0x3f, 0, 0, 0x3f, 0, 0x3f]);
        assert_eq!(video.read_byte(VIDEO_RAM_START), Some(0b1010_0101));
    }

    #[test]
    fn mode_register_selects_video_format() {
        let mut video = BkVideoMemory::new(TestCanvas::new(), 0);
        assert_eq!(video.mode(), BkVideoMode::Monochrome);
        video.set_mode_register(0xff);
        assert_eq!(video.mode(), BkVideoMode::Color);
        video.set_mode_register(1);
        assert_eq!(video.mode(), BkVideoMode::Color);
        video.set_mode_register(0);
        assert_eq!(video.mode(), BkVideoMode::Monochrome);
    }

    #[test]
    fn rejects_addresses_outside_video_window() {
        let mut video = BkVideoMemory::new(TestCanvas::new(), 0);
        assert_eq!(video.read_byte(VIDEO_RAM_START - 1), None);
        assert_eq!(video.read_byte(VIDEO_RAM_END + 1), None);
        assert!(!video.write_byte(VIDEO_RAM_START - 1, 0xff));
        assert!(!video.write_byte(VIDEO_RAM_END + 1, 0xff));
        assert!(video.write_byte(VIDEO_RAM_END, 0xff));
    }
}
