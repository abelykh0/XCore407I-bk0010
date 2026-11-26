#![no_std]

//! Shared STM32F4 hardware implementation of reflected CRC-32/ISO-HDLC.
//!
//! The STM32F407 CRC unit implements the non-reflected 0x04C11DB7
//! polynomial, accepts 32-bit input words, and resets to 0xFFFF_FFFF.
//! Reverse each input byte's bits and reverse the result to obtain the
//! reflected CRC-32/ISO-HDLC convention used by littlefs and ZMODEM.

use core::sync::atomic::{AtomicU32, Ordering};

const INITIAL: u32 = 0xFFFF_FFFF;

static HARDWARE_WORDS: AtomicU32 = AtomicU32::new(0);
static HARDWARE_FALLBACK_BYTES: AtomicU32 = AtomicU32::new(0);

/// Reset counters for actual CRC peripheral feeds and hardware-path fallbacks.
pub fn reset_diagnostics() {
    HARDWARE_WORDS.store(0, Ordering::Relaxed);
    HARDWARE_FALLBACK_BYTES.store(0, Ordering::Relaxed);
}

/// Return (32-bit words written to CRC_DR, bytes processed by software
/// because the hardware continuation state was unavailable or a tail remained).
pub fn diagnostics() -> (u32, u32) {
    (
        HARDWARE_WORDS.load(Ordering::Relaxed),
        HARDWARE_FALLBACK_BYTES.load(Ordering::Relaxed),
    )
}

// Same 16-entry (64-byte) nibble table used by littlefs. This replaces a
// 256-entry (1 KiB) byte table while remaining substantially faster than
// the previous bit-at-a-time fallback.
const CRC32_NIBBLE_TABLE: [u32; 16] = [
    0x0000_0000, 0x1db7_1064, 0x3b6e_20c8, 0x26d9_30ac,
    0x76dc_4190, 0x6b6b_51f4, 0x4db2_6158, 0x5005_713c,
    0xedb8_8320, 0xf00f_9344, 0xd6d6_a3e8, 0xcb61_b38c,
    0x9b64_c2b0, 0x86d3_d2d4, 0xa00a_e278, 0xbdbd_f21c,
];

#[inline]
fn software_update(mut crc: u32, data: &[u8]) -> u32 {
    for &byte in data {
        crc = (crc >> 4) ^ CRC32_NIBBLE_TABLE[((crc ^ byte as u32) & 0x0f) as usize];
        crc = (crc >> 4) ^ CRC32_NIBBLE_TABLE[((crc ^ (byte as u32 >> 4)) & 0x0f) as usize];
    }
    crc
}

/// Update a reflected CRC-32/ISO-HDLC state without applying the final XOR.
///
/// The returned state can be passed to a later call as the seed. The hardware
/// peripheral is used when it can continue from the supplied state; otherwise
/// the compact littlefs-compatible software implementation is used.
pub fn update(seed: u32, data: &[u8]) -> u32 {
    #[cfg(target_arch = "arm")]
    {
        hardware_update(seed, data)
    }

    #[cfg(not(target_arch = "arm"))]
    {
        software_update(seed, data)
    }
}

#[cfg(target_arch = "arm")]
fn hardware_update(seed: u32, data: &[u8]) -> u32 {
    use core::ptr::{read_volatile, write_volatile};

    const RCC_AHB1ENR: *mut u32 = 0x4002_3830 as *mut u32;
    const CRC_DR: *mut u32 = 0x4002_3000 as *mut u32;
    const CRC_CR: *mut u32 = 0x4002_3008 as *mut u32;
    const CRC_ENABLE: u32 = 1 << 12;

    // The CRC peripheral is a single shared state machine. Exclude interrupts
    // while checking its state and feeding the next words.
    cortex_m::interrupt::free(|_| unsafe {
        let clock = read_volatile(RCC_AHB1ENR);
        write_volatile(RCC_AHB1ENR, clock | CRC_ENABLE);
        let _ = read_volatile(RCC_AHB1ENR);

        static mut LAST_STATE: u32 = INITIAL;
        static mut LAST_VALID: bool = false;

        if seed == INITIAL {
            write_volatile(CRC_CR, 1);
            LAST_STATE = INITIAL;
            LAST_VALID = true;
        } else if !LAST_VALID || LAST_STATE != seed {
            HARDWARE_FALLBACK_BYTES.fetch_add(data.len() as u32, Ordering::Relaxed);
            return software_update(seed, data);
        }

        let full_len = data.len() & !3;
        for bytes in data[..full_len].chunks_exact(4) {
            // The F407 CRC unit consumes each word most-significant bit first.
            // Bit-reversing each byte preserves the reflected CRC byte order.
            let word = (bytes[0].reverse_bits() as u32) << 24
                | (bytes[1].reverse_bits() as u32) << 16
                | (bytes[2].reverse_bits() as u32) << 8
                | bytes[3].reverse_bits() as u32;
            write_volatile(CRC_DR, word);
        }
        HARDWARE_WORDS.fetch_add((full_len / 4) as u32, Ordering::Relaxed);

        let mut result = read_volatile(CRC_DR).reverse_bits();
        if full_len != data.len() {
            // CRC_DR only supports word accesses. Finish a 1–3 byte tail in
            // software and invalidate the peripheral continuation state.
            HARDWARE_FALLBACK_BYTES.fetch_add((data.len() - full_len) as u32, Ordering::Relaxed);
            result = software_update(result, &data[full_len..]);
            LAST_VALID = false;
        } else {
            LAST_STATE = result;
            LAST_VALID = true;
        }
        result
    })
}

/// Compute a complete CRC-32/ISO-HDLC checksum.
#[inline]
pub fn checksum(data: &[u8]) -> u32 {
    !update(INITIAL, data)
}

#[cfg(test)]
mod tests {
    use super::{checksum, update};

    fn emulate_f407_update(seed: u32, data: &[u8]) -> u32 {
        const POLY: u32 = 0x04C1_1DB7;
        let mut state = seed.reverse_bits();
        let full_len = data.len() & !3;
        for bytes in data[..full_len].chunks_exact(4) {
            let word = (bytes[0].reverse_bits() as u32) << 24
                | (bytes[1].reverse_bits() as u32) << 16
                | (bytes[2].reverse_bits() as u32) << 8
                | bytes[3].reverse_bits() as u32;
            for bit in (0..32).rev() {
                let mix = ((state >> 31) ^ (word >> bit)) & 1;
                state = (state << 1) ^ if mix != 0 { POLY } else { 0 };
            }
        }
        let reflected = state.reverse_bits();
        super::software_update(reflected, &data[full_len..])
    }

    #[test]
    fn f407_word_order_matches_reflected_crc32() {
        for data in [
            &b"1234"[..],
            &b"12345678"[..],
            &b"123456789"[..],
            &b"unaligned tail"[..],
            &b"littlefs and zmodem share the STM32 CRC unit"[..],
        ] {
            assert_eq!(
                !emulate_f407_update(0xFFFF_FFFF, data),
                checksum(data)
            );
        }
    }

    #[test]
    fn f407_incremental_word_updates_match() {
        let data = b"four-byte chunks can continue across calls";
        let split = 8;
        let state = emulate_f407_update(0xFFFF_FFFF, &data[..split]);
        assert_eq!(
            !emulate_f407_update(state, &data[split..]),
            checksum(data)
        );
    }

    #[test]
    fn standard_crc32_check_value() {
        assert_eq!(checksum(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn incremental_updates_match_one_shot() {
        let data = b"littlefs and zmodem share the STM32 CRC unit";
        let split = 20;
        let state = update(0xFFFF_FFFF, &data[..split]);
        assert_eq!(!update(state, &data[split..]), checksum(data));
    }

    #[test]
    fn empty_input_preserves_seed() {
        assert_eq!(update(0x1234_5678, &[]), 0x1234_5678);
        assert_eq!(checksum(&[]), 0);
    }
}
