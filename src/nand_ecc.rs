//! NAND 512-byte Hamming ECC calculation and data correction.
//!
//! The STM32 FSMC hardware calculates ECC during normal NAND page writes and
//! reads. The normal read path uses the hardware-generated ECC values with
//! `correct_ecc_chunk()` to detect and correct single-bit data errors.
//!
//! Software ECC calculation and comparison helpers are diagnostic-only and
//! are enabled by the `nand-diagnostics` feature. They are not used to
//! generate ECC during normal filesystem operations.
//! The NAND controller generates and stores ECC during normal page writes.

const ECC_CHUNK_SIZE: usize = 512;

/// Decode a 24-bit STM32 FSMC syndrome using Linux's SM-order Hamming
/// decoder. FSMC ECC is Linux ECC rotated left by six bits and complemented,
/// so the syndrome is rotated right by six before decoding.
fn ecc_error_location(syndrome: u32) -> Option<(usize, usize)> {
    let linux = ((syndrome >> 6) | (syndrome << 18)) & 0x00FF_FFFF;
    let b0 = linux as u8;
    let b1 = (linux >> 8) as u8;
    let b2 = (linux >> 16) as u8;

    // A 512-byte data-bit syndrome has alternating parity in every
    // address pair, including both high byte-address bits in b2.
    if ((b0 ^ (b0 >> 1)) & 0x55) != 0x55
        || ((b1 ^ (b1 >> 1)) & 0x55) != 0x55
        || ((b2 ^ (b2 >> 1)) & 0x55) != 0x55
    {
        return None;
    }

    // Linux addressbits[] extracts the odd-positioned bits from each
    // interleaved parity byte: input bits 1,3,5,7 become output bits 0..3.
    #[inline]
    fn address_bits(value: u8) -> usize {
        (((value >> 1) & 1)
            | ((value >> 2) & 2)
            | ((value >> 3) & 4)
            | ((value >> 4) & 8)) as usize
    }

    let byte_index = (address_bits(b2 & 0x03) << 8)
        | (address_bits(b1) << 4)
        | address_bits(b0);
    let bit_index = address_bits(b2 >> 2);

    if byte_index >= ECC_CHUNK_SIZE || bit_index >= 8 {
        return None;
    }
    Some((byte_index, bit_index))
}

pub(crate) fn correct_ecc_chunk(data: &mut [u8], stored: u32, calculated: u32) -> bool {
    if data.len() != ECC_CHUNK_SIZE {
        return false;
    }

    let syndrome = (stored ^ calculated) & 0x00FF_FFFF;
    if syndrome == 0 {
        return true;
    }

    // A single changed ECC bit is a parity-bit error, not a data-bit error.
    if syndrome.count_ones() == 1 {
        return true;
    }

    let Some((byte_index, bit_index)) = ecc_error_location(syndrome) else {
        return false;
    };
    data[byte_index] ^= 1 << bit_index;
    true
}

/// Linux FSMC driver's 512-byte NAND_ECC_SOFT_HAMMING_SM_ORDER
/// reference algorithm, expressed without alignment-dependent word loads.
#[cfg(feature = "nand-diagnostics")]
pub(crate) fn ecc_software_512(data: &[u8]) -> u32 {
    if data.len() != ECC_CHUNK_SIZE {
        return 0;
    }

    #[inline]
    fn inv_parity(value: u8) -> u8 {
        ((!value.count_ones()) & 1) as u8
    }

    let mut rp4 = 0u32;
    let mut rp6 = 0u32;
    let mut rp8 = 0u32;
    let mut rp10 = 0u32;
    let mut rp12 = 0u32;
    let mut rp14 = 0u32;
    let mut rp16 = 0u32;
    let mut par = 0u32;

    // Linux ecc_sw_hamming_calculate(): 8 groups x 16 little-endian words.
    let words: [u32; 128] = core::array::from_fn(|i| {
        u32::from_le_bytes([
            data[i * 4],
            data[i * 4 + 1],
            data[i * 4 + 2],
            data[i * 4 + 3],
        ])
    });

    for group in 0..8 {
        let base = group * 16;
        let mut t = words[base];
        rp4 ^= words[base];
        t ^= words[base + 1];
        rp6 ^= t;
        t ^= words[base + 2];
        rp4 ^= words[base + 2];
        t ^= words[base + 3];
        rp8 ^= t;

        t ^= words[base + 4];
        rp4 ^= words[base + 4];
        rp6 ^= words[base + 4];
        t ^= words[base + 5];
        rp6 ^= words[base + 5];
        t ^= words[base + 6];
        rp4 ^= words[base + 6];
        t ^= words[base + 7];
        rp10 ^= t;

        t ^= words[base + 8];
        rp4 ^= words[base + 8];
        rp6 ^= words[base + 8];
        rp8 ^= words[base + 8];
        t ^= words[base + 9];
        rp6 ^= words[base + 9];
        rp8 ^= words[base + 9];
        t ^= words[base + 10];
        rp4 ^= words[base + 10];
        rp8 ^= words[base + 10];
        t ^= words[base + 11];
        rp8 ^= words[base + 11];

        t ^= words[base + 12];
        rp4 ^= words[base + 12];
        rp6 ^= words[base + 12];
        t ^= words[base + 13];
        rp6 ^= words[base + 13];
        t ^= words[base + 14];
        rp4 ^= words[base + 14];
        t ^= words[base + 15];

        par ^= t;
        if group & 1 == 0 { rp12 ^= t; }
        if group & 2 == 0 { rp14 ^= t; }
        if group & 4 == 0 { rp16 ^= t; }
    }

    let fold = |v: u32| -> u8 {
        let v = v ^ (v >> 16);
        let v = v ^ (v >> 8);
        v as u8
    };
    let rp4 = fold(rp4);
    let rp6 = fold(rp6);
    let rp8 = fold(rp8);
    let rp10 = fold(rp10);
    let rp12 = fold(rp12);
    let rp14 = fold(rp14);
    let rp16 = fold(rp16);

    // Little-endian Linux SM-order parity packing.
    let rp3 = fold(par >> 16);
    let low16 = par & 0xFFFF;
    let rp2 = (low16 ^ (low16 >> 8)) as u8;
    let p16 = par ^ (par >> 16);
    let rp1 = ((p16 >> 8) & 0xFF) as u8;
    let rp0 = p16 as u8;
    let par8 = (p16 ^ (p16 >> 8)) as u8;

    let rp5 = par8 ^ rp4;
    let rp7 = par8 ^ rp6;
    let rp9 = par8 ^ rp8;
    let rp11 = par8 ^ rp10;
    let rp13 = par8 ^ rp12;
    let rp15 = par8 ^ rp14;
    let rp17 = par8 ^ rp16;

    let ecc0 = (inv_parity(rp7) << 7)
        | (inv_parity(rp6) << 6)
        | (inv_parity(rp5) << 5)
        | (inv_parity(rp4) << 4)
        | (inv_parity(rp3) << 3)
        | (inv_parity(rp2) << 2)
        | (inv_parity(rp1) << 1)
        | inv_parity(rp0);
    let ecc1 = (inv_parity(rp15) << 7)
        | (inv_parity(rp14) << 6)
        | (inv_parity(rp13) << 5)
        | (inv_parity(rp12) << 4)
        | (inv_parity(rp11) << 3)
        | (inv_parity(rp10) << 2)
        | (inv_parity(rp9) << 1)
        | inv_parity(rp8);
    let ecc2 = (inv_parity(par8 & 0xF0) << 7)
        | (inv_parity(par8 & 0x0F) << 6)
        | (inv_parity(par8 & 0xCC) << 5)
        | (inv_parity(par8 & 0x33) << 4)
        | (inv_parity(par8 & 0xAA) << 3)
        | (inv_parity(par8 & 0x55) << 2)
        | (inv_parity(rp17) << 1)
        | inv_parity(rp16);

    (ecc0 as u32) | ((ecc1 as u32) << 8) | ((ecc2 as u32) << 16)
}

/// Convert 512-byte Linux SM-order Hamming ECC to the STM32 FSMC ECCR layout.
/// Hardware comparisons establish a six-bit left rotation followed by inversion.
#[cfg(feature = "nand-diagnostics")]
fn ecc_linux_to_fsmc(ecc: u32) -> u32 {
    let ecc = ecc & 0x00FF_FFFF;
    let rotated = ((ecc << 6) | (ecc >> 18)) & 0x00FF_FFFF;
    (!rotated) & 0x00FF_FFFF
}

#[cfg(feature = "nand-diagnostics")]
pub(crate) fn ecc_software_512_fsmc(data: &[u8]) -> u32 {
    ecc_linux_to_fsmc(ecc_software_512(data))
}

/// Reference vectors from Linux ecc_sw_hamming_calculate() using
/// NAND_ECC_SOFT_HAMMING_SM_ORDER and 512-byte steps.
#[cfg(feature = "nand-diagnostics")]
pub(crate) fn ecc_software_selftest() -> [u32; 4] {
    let zero = [0u8; ECC_CHUNK_SIZE];
    let erased = [0xFFu8; ECC_CHUNK_SIZE];
    let mut single_bit = [0u8; ECC_CHUNK_SIZE];
    single_bit[0] = 1;
    let mut pseudo_random = [0u8; ECC_CHUNK_SIZE];
    let mut state = 0x1234_5678u32;
    for byte in &mut pseudo_random {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        *byte = state as u8;
    }
    [
        ecc_software_512(&zero),
        ecc_software_512(&erased),
        ecc_software_512(&single_bit),
        ecc_software_512(&pseudo_random),
    ]
}
