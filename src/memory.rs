//! Shared static memory layout for the camera canvas and CDC networking.
//!
//! The same physical canvas allocation is used for the BK display in normal
//! mode and for CDC transfer/network storage in CDC mode. The regions below
//! retain their existing SRAM/CCM placement and sizes.

use core::ptr::NonNull;
use core::sync::atomic::{AtomicBool, Ordering};

pub(crate) const CANVAS_TAIL_BYTES: usize = 46_848;
pub(crate) const CANVAS_MAIN_BYTES: usize = 512 * 256 - CANVAS_TAIL_BYTES;

pub(crate) const CDC_ZMODEM_STORAGE_BYTES: usize = 4096;
pub(crate) const CDC_ETHERNET_DMA_STORAGE_BYTES: usize = 2 * 16 + 2 * 1536 + 16 + 1536;
pub(crate) const CDC_HTTP_RX_BYTES: usize = 4096;
pub(crate) const CDC_HTTP_BODY_BYTES: usize = 62_416;
pub(crate) const CDC_HTTP_STORAGE_BYTES: usize = CDC_HTTP_RX_BYTES + CDC_HTTP_BODY_BYTES;
const CDC_ALT_STACK_BYTES: usize = 8 * 1024 + 768;
const CDC_ALT_STACK_BOTTOM: usize = CANVAS_MAIN_BYTES - CDC_ALT_STACK_BYTES;

#[derive(Clone, Copy)]
#[repr(C, align(4))]
struct AlignedCanvas<const N: usize>([u8; N]);

#[repr(C, align(8))]
struct DmaAlignedCanvas<const N: usize>([u8; N]);

static mut DEMO_CANVAS: DmaAlignedCanvas<CANVAS_MAIN_BYTES> =
    DmaAlignedCanvas([0; CANVAS_MAIN_BYTES]);

#[repr(C, align(8))]
union CanvasTailStorage {
    canvas: AlignedCanvas<CANVAS_TAIL_BYTES>,
    cpu: [u8; CANVAS_TAIL_BYTES],
}

#[unsafe(link_section = ".ccmram.canvas")]
static mut DEMO_CANVAS_TAIL: CanvasTailStorage =
    CanvasTailStorage { canvas: AlignedCanvas([0; CANVAS_TAIL_BYTES]) };

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

pub(crate) fn canvas_main_ptr() -> *mut u8 {
    unsafe { core::ptr::addr_of_mut!(DEMO_CANVAS.0).cast::<u8>() }
}

pub(crate) fn canvas_main_const_ptr() -> *const u8 {
    unsafe { core::ptr::addr_of!(DEMO_CANVAS.0).cast::<u8>() }
}

pub(crate) fn canvas_tail_ptr() -> *mut u8 {
    unsafe { core::ptr::addr_of_mut!(DEMO_CANVAS_TAIL.canvas.0).cast::<u8>() }
}

pub(crate) fn canvas_tail_const_ptr() -> *const u8 {
    unsafe { core::ptr::addr_of!(DEMO_CANVAS_TAIL.canvas.0).cast::<u8>() }
}

#[allow(dead_code)]
pub(crate) fn cdc_alternate_stack_top() -> *mut u8 {
    unsafe { canvas_main_ptr().add(CANVAS_MAIN_BYTES) }
}

pub(crate) fn ethernet_dma_storage_ptr() -> *mut u8 {
    unsafe { canvas_main_ptr().add(CDC_ZMODEM_STORAGE_BYTES) }
}

pub(crate) fn ethernet_http_storage_ptr() -> *mut u8 {
    unsafe {
        canvas_main_ptr().add(CDC_ZMODEM_STORAGE_BYTES + CDC_ETHERNET_DMA_STORAGE_BYTES)
    }
}

pub(crate) fn ethernet_socket_storage_ptr() -> *mut u8 {
    unsafe { core::ptr::addr_of_mut!(DEMO_CANVAS_TAIL.cpu).cast::<u8>() }
}

/// Exclusive lease for the canvas region reused by one ZMODEM transfer.
/// Dropping the lease makes the region available to the next transfer.
pub(crate) struct CdcZmodemStorage {
    ptr: NonNull<u8>,
}

impl CdcZmodemStorage {
    pub(crate) const fn len(&self) -> usize {
        CDC_ZMODEM_STORAGE_BYTES
    }

    pub(crate) fn as_mut_ptr(&mut self) -> *mut u8 {
        self.ptr.as_ptr()
    }
}

impl Drop for CdcZmodemStorage {
    fn drop(&mut self) {
        CDC_ZMODEM_STORAGE_IN_USE.store(false, Ordering::Release);
    }
}

static CDC_ZMODEM_STORAGE_IN_USE: AtomicBool = AtomicBool::new(false);

pub(crate) fn claim_cdc_zmodem_storage() -> Option<CdcZmodemStorage> {
    CDC_ZMODEM_STORAGE_IN_USE
        .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
        .ok()?;
    let Some(ptr) = NonNull::new(canvas_main_ptr()) else {
        CDC_ZMODEM_STORAGE_IN_USE.store(false, Ordering::Release);
        return None;
    };
    Some(CdcZmodemStorage { ptr })
}
