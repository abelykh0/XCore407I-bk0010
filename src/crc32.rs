//! STM32F407 hardware CRC-32/ISO-HDLC test implementation.
//!
//! This is deliberately separate from the ZMODEM and LittleFS implementations
//! so CRC32TEST can compare all three before either library is switched over.

/// Compute the finalized reflected CRC-32/ISO-HDLC checksum using the STM32F4
/// CRC peripheral. The peripheral accepts 32-bit words, so a 1-3 byte tail is
/// completed with the same reflected software polynomial.
#[cfg(target_arch = "arm")]
pub fn hardware_checksum(data: &[u8]) -> u32 {
    use core::ptr::{read_volatile, write_volatile};

    const RCC_AHB1ENR: *mut u32 = 0x4002_3830 as *mut u32;
    const CRC_DR: *mut u32 = 0x4002_3000 as *mut u32;
    const CRC_CR: *mut u32 = 0x4002_3008 as *mut u32;
    const CRC_ENABLE: u32 = 1 << 12;

    cortex_m::interrupt::free(|_| unsafe {
        let clock = read_volatile(RCC_AHB1ENR);
        write_volatile(RCC_AHB1ENR, clock | CRC_ENABLE);
        let _ = read_volatile(RCC_AHB1ENR);
        write_volatile(CRC_CR, 1);

        let full_len = data.len() & !3;
        for bytes in data[..full_len].chunks_exact(4) {
            let word = (bytes[0].reverse_bits() as u32) << 24
                | (bytes[1].reverse_bits() as u32) << 16
                | (bytes[2].reverse_bits() as u32) << 8
                | bytes[3].reverse_bits() as u32;
            write_volatile(CRC_DR, word);
        }

        let mut crc = read_volatile(CRC_DR).reverse_bits();
        for &byte in &data[full_len..] {
            crc ^= byte as u32;
            for _ in 0..8 {
                crc = if crc & 1 != 0 {
                    (crc >> 1) ^ 0xEDB8_8320
                } else {
                    crc >> 1
                };
            }
        }
        !crc
    })
}
