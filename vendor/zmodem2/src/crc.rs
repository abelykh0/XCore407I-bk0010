// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2023-2025 Jarkko Sakkinen <jarkko.sakkinen@iki.fi>

/// Computes the CRC-16-XMODEM checksum.
#[must_use]
pub const fn crc16_xmodem(data: &[u8]) -> u16 {
    let mut crc: u16 = 0x0000;
    let mut i = 0;
    while i < data.len() {
        crc = crc16_update(crc, data[i]);
        i += 1;
    }
    crc
}

/// Computes the CRC-32-ISO-HDLC checksum.
#[must_use]
pub fn crc32_iso_hdlc(data: &[u8]) -> u32 {
    stm32f4_crc32::checksum(data)
}

/// A stateful, iterative CRC-16-XMODEM calculator.
pub struct Crc16 {
    crc: u16,
}

impl Crc16 {
    /// Creates a new CRC-16 calculator.
    #[must_use]
    pub const fn new() -> Self {
        Self { crc: 0x0000 }
    }

    /// Updates the CRC state with a slice of bytes.
    pub fn update(&mut self, data: &[u8]) {
        for &byte in data {
            self.update_byte(byte);
        }
    }

    /// Updates the CRC state with a single byte.
    pub fn update_byte(&mut self, byte: u8) {
        self.crc = crc16_update(self.crc, byte);
    }

    /// Finalizes the CRC calculation and returns the checksum.
    #[must_use]
    pub const fn finalize(&self) -> u16 {
        self.crc
    }
}

/// A stateful, iterative CRC-32-ISO-HDLC calculator.
pub struct Crc32 {
    crc: u32,
}

impl Crc32 {
    /// Creates a new CRC-32 calculator.
    #[must_use]
    pub const fn new() -> Self {
        Self { crc: 0xFFFF_FFFF }
    }

    /// Updates the CRC state with a slice of bytes.
    pub fn update(&mut self, data: &[u8]) {
        self.crc = stm32f4_crc32::update(self.crc, data);
    }

    /// Update the CRC and return (hardware words, software fallback bytes).
    pub fn update_with_stats(&mut self, data: &[u8]) -> (u32, u32) {
        let (crc, hardware_words, fallback_bytes) =
            stm32f4_crc32::update_with_stats(self.crc, data);
        self.crc = crc;
        (hardware_words, fallback_bytes)
    }

    /// Updates the CRC state with a single byte.
    pub fn update_byte(&mut self, byte: u8) {
        self.crc = stm32f4_crc32::update(self.crc, &[byte]);
    }

    /// Finalizes the CRC calculation and returns the checksum.
    #[must_use]
    pub const fn finalize(&self) -> u32 {
        !self.crc
    }
}

/// Performs a single byte update for CRC-16-XMODEM.
const fn crc16_update(mut crc: u16, byte: u8) -> u16 {
    crc ^= (byte as u16) << 8;
    let mut i = 0;
    while i < 8 {
        if (crc & 0x8000) != 0 {
            crc = (crc << 1) ^ 0x1021;
        } else {
            crc <<= 1;
        }
        i += 1;
    }
    crc
}
