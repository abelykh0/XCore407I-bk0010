//! Cortex-M4F word copy used by the original C canvas renderer.
//!
//! The FPU register bank acts as a 128-byte transfer buffer. Keep this code in
//! initialized SRAM (`.data`) so the USB-facing copy path avoids flash wait
//! states. Inputs must be 4-byte aligned and the byte count a multiple of 4.

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
    src .req r0
    dst .req r1
    count .req r2
    vpush {s16-s31}
    lsrs.n count, #1
    itt cs
    vldmcs.32 src!, {s0}
    vstmcs.32 dst!, {s0}
    .macro XFER n
        lsrs.n count, #1
        itt cs
        vldmcs.32 src!, {s0-s\n}
        vstmcs.32 dst!, {s0-s\n}
    .endm
    XFER 1
    XFER 3
    XFER 7
    XFER 15
    beq.w 1f
0:
    vldm.32 src!, {s0-s31}
    vstm.32 dst!, {s0-s31}
    subs.n count, #1
    bne.n 0b
1:
    vpop {s16-s31}
    bx lr
    .size rust_copy_words_asm, .-rust_copy_words_asm
    .unreq src
    .unreq dst
    .unreq count
"#, options(raw));

unsafe extern "C" {
    fn rust_copy_words_asm(src: *const u32, dst: *mut u32, words: u32);
}

/// Copy aligned words using the original FPU-register transfer trick.
///
/// # Safety
/// `src` and `dst` must be valid for `bytes` bytes, non-overlapping, and
/// readable/writable respectively. The assembly path additionally requires
/// the common alignment handled below and a word-multiple transfer length.
#[inline(always)]
pub(crate) unsafe fn copy_words(src: *const u8, dst: *mut u8, bytes: usize) {
    unsafe {
    if bytes == 0 {
        return;
    }

    // The VFP transfer instructions require both addresses to be word
    // aligned. If the two pointers have different alignments, there is no
    // common word boundary, so keep the whole operation byte-wise.
    if ((src as usize) ^ (dst as usize)) & 3 != 0 {
        core::ptr::copy_nonoverlapping(src, dst, bytes);
        return;
    }

    let prefix = (4 - ((src as usize) & 3)) & 3;
    let prefix = prefix.min(bytes);
    if prefix != 0 {
        core::ptr::copy_nonoverlapping(src, dst, prefix);
    }

    let aligned_bytes = (bytes - prefix) & !3;
    if aligned_bytes != 0 {
        rust_copy_words_asm(
            src.add(prefix).cast(),
            dst.add(prefix).cast(),
            (aligned_bytes / 4) as u32,
        );
    }

    let suffix = bytes - prefix - aligned_bytes;
    if suffix != 0 {
        core::ptr::copy_nonoverlapping(
            src.add(prefix + aligned_bytes),
            dst.add(prefix + aligned_bytes),
            suffix,
        );
    }
    }
}
