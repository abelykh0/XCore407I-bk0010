//! Cortex-M4F word copy used by the canvas renderer.
//!
//! The FPU register bank acts as a 128-byte transfer buffer. Placed in
//! initialized SRAM (`.data.ramcode`) so the USB-facing copy path executes
//! with zero flash wait states.
//!
//! All inputs MUST be 4-byte aligned and `words` is the number of `u32` words.

use core::arch::global_asm;

global_asm!(r#"
    .syntax unified
    .arch armv7e-m
    .fpu fpv4-sp-d16
    .thumb
    .section .data.ramcode,"awx",%progbits
    .balign 4
    .global rust_copy_words_asm
    .type rust_copy_words_asm,%function
    .thumb_func
rust_copy_words_asm:
    src   .req r0
    dst   .req r1
    count .req r2

    cmp     count, #16
    bhs.n   2f

    @ Fast path for 1..=15 words: uses only caller-saved s0-s15 (no vpush/vpop)
    lsrs    count, count, #1
    itt     cs
    vldmcs.32 src!, {s0}
    vstmcs.32 dst!, {s0}

    lsrs    count, count, #1
    itt     cs
    vldmcs.32 src!, {s0-s1}
    vstmcs.32 dst!, {s0-s1}

    lsrs    count, count, #1
    itt     cs
    vldmcs.32 src!, {s0-s3}
    vstmcs.32 dst!, {s0-s3}

    lsrs    count, count, #1
    itt     cs
    vldmcs.32 src!, {s0-s7}
    vstmcs.32 dst!, {s0-s7}

    bx      lr

2:  @ Bulk path for >= 16 words: saves callee-saved s16-s31 for 128-byte bursts
    vpush   {s16-s31}

    lsrs    count, count, #1
    itt     cs
    vldmcs.32 src!, {s0}
    vstmcs.32 dst!, {s0}

    lsrs    count, count, #1
    itt     cs
    vldmcs.32 src!, {s0-s1}
    vstmcs.32 dst!, {s0-s1}

    lsrs    count, count, #1
    itt     cs
    vldmcs.32 src!, {s0-s3}
    vstmcs.32 dst!, {s0-s3}

    lsrs    count, count, #1
    itt     cs
    vldmcs.32 src!, {s0-s7}
    vstmcs.32 dst!, {s0-s7}

    lsrs    count, count, #1
    itt     cs
    vldmcs.32 src!, {s0-s15}
    vstmcs.32 dst!, {s0-s15}

    beq.n   1f

    .balign 4
0:
    vldm.32 src!, {s0-s31}
    vstm.32 dst!, {s0-s31}
    subs    count, count, #1
    bne.n   0b

1:
    vpop    {s16-s31}
    bx      lr

    .size rust_copy_words_asm, .-rust_copy_words_asm
    .unreq src
    .unreq dst
    .unreq count
"#, options(raw));

unsafe extern "C" {
    fn rust_copy_words_asm(src: *const u32, dst: *mut u32, words: u32);
}

/// Copy `words` 32-bit words from `src` to `dst` using the FPU register bank.
///
/// # Safety
/// - `src` and `dst` must be 4-byte aligned, non-overlapping, and valid for
///   `words` `u32` elements.
#[inline(always)]
pub(crate) unsafe fn copy_words_u32(src: *const u32, dst: *mut u32, words: usize) {
    debug_assert!((src as usize) & 3 == 0, "src must be 4-byte aligned");
    debug_assert!((dst as usize) & 3 == 0, "dst must be 4-byte aligned");

    if words != 0 {
        unsafe {
            rust_copy_words_asm(src, dst, words as u32);
        }
    }
}

/// Byte-length wrapper for callers passing byte counts (must be a multiple of 4).
///
/// # Safety
/// - `src` and `dst` must be 4-byte aligned, non-overlapping, and valid for
///   `bytes` bytes.
/// - `bytes` must be a multiple of 4.
#[inline(always)]
pub(crate) unsafe fn copy_words(src: *const u8, dst: *mut u8, bytes: usize) {
    debug_assert!((src as usize) & 3 == 0, "src must be 4-byte aligned");
    debug_assert!((dst as usize) & 3 == 0, "dst must be 4-byte aligned");
    debug_assert!(bytes & 3 == 0, "bytes must be a multiple of 4");

    unsafe {
        copy_words_u32(src.cast(), dst.cast(), bytes >> 2);
    }
}
