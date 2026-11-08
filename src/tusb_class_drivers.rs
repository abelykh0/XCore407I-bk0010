//! Rust side of the TinyUSB custom-class callback boundary.
//!
//! The class-driver registration structure stays in C because it is private
//! to TinyUSB. This module implements UVC parsing, alternate settings,
//! Probe/Commit controls, and the high-bandwidth ISO IN transfer path.

use core::ffi::c_void;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::camera::{HEIGHT, WIDTH};
use crate::usb_audio::BkEmulatorAudio;
use crate::uvc::{ControlResponse, UvcClass, FRAME_BIT_RATE, FRAME_INTERVAL_100NS};

const USB_DESC_INTERFACE: u8 = 0x04;
const USB_DESC_ENDPOINT: u8 = 0x05;
const USB_DESC_INTERFACE_ASSOCIATION: u8 = 0x0b;
const USB_DESC_CS_INTERFACE: u8 = 0x24;
const USB_CLASS_VIDEO: u8 = 0x0e;
const USB_SUBCLASS_VIDEO_CONTROL: u8 = 0x01;
const USB_SUBCLASS_VIDEO_STREAMING: u8 = 0x02;
const UVC_CONTROL_INTERFACE: u8 = 0;
const UVC_STREAMING_INTERFACE: u8 = 1;
const UVC_STREAM_ENDPOINT: u8 = 0x81;
const UVC_MAX_PACKET_SIZE: u16 = 768;
const UVC_TRANSACTIONS_PER_MICROFRAME: u8 = 3;
const UVC_VS_TOTAL_LENGTH: u16 = 77;
const UVC_NV12_GUID: [u8; 16] = [
    0x4e, 0x56, 0x31, 0x32, 0x00, 0x00, 0x10, 0x00,
    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71,
];

const CONTROL_STAGE_SETUP: u8 = 0;
const CONTROL_STAGE_DATA: u8 = 1;
const CONTROL_STAGE_ACK: u8 = 2;
const REQUEST_GET_INTERFACE: u8 = 10;
const REQUEST_SET_INTERFACE: u8 = 11;
const REQUEST_SET_CUR: u8 = 0x01;
const REQUEST_TYPE_MASK: u8 = 0x60;
const REQUEST_TYPE_STANDARD: u8 = 0x00;
const REQUEST_TYPE_CLASS: u8 = 0x20;
const REQUEST_RECIPIENT_MASK: u8 = 0x1f;
const REQUEST_RECIPIENT_INTERFACE: u8 = 0x01;
const REQUEST_DIRECTION_IN: u8 = 0x80;

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct UsbControlRequest {
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    length: u16,
}

#[derive(Clone, Copy)]
struct ParsedUvcFunction {
    length: usize,
    endpoint_offset: usize,
}

static UVC_STATE_READY: AtomicBool = AtomicBool::new(false);
static UVC_ENDPOINT_ALLOCATED: AtomicBool = AtomicBool::new(false);
static mut UVC_STATE: MaybeUninit<UvcClass> = MaybeUninit::uninit();
static mut UVC_ENDPOINT_DESCRIPTOR: [u8; 7] = [0; 7];
static mut UVC_ALT_RESPONSE: u8 = 0;

unsafe extern "C" {
    fn xcore_tud_control_xfer(
        rhport: u8,
        request: *const c_void,
        buffer: *mut c_void,
        length: u16,
    ) -> bool;
    fn xcore_tud_control_status(rhport: u8, request: *const c_void) -> bool;
    fn xcore_usbd_control_buffer() -> *mut u8;
    fn xcore_usbd_edpt_iso_alloc(rhport: u8, ep_addr: u8, packet_size: u16) -> bool;
    fn xcore_usbd_edpt_iso_activate(rhport: u8, endpoint_desc: *const c_void) -> bool;
    fn xcore_usbd_edpt_close(rhport: u8, ep_addr: u8);
    fn xcore_usbd_edpt_claim(rhport: u8, ep_addr: u8) -> bool;
    fn xcore_usbd_edpt_release(rhport: u8, ep_addr: u8) -> bool;
    fn xcore_usbd_edpt_xfer(rhport: u8, ep_addr: u8, buffer: *mut u8, length: u16) -> bool;
    fn xcore_usbd_edpt_xfer_isr(rhport: u8, ep_addr: u8, buffer: *mut u8, length: u16) -> bool;
    fn xcore_usbd_edpt_ready(rhport: u8, ep_addr: u8) -> bool;
}

fn uvc_state_ptr() -> Option<*mut UvcClass> {
    if !UVC_STATE_READY.load(Ordering::Acquire) {
        return None;
    }
    Some(core::ptr::addr_of_mut!(UVC_STATE).cast::<UvcClass>())
}

fn with_uvc_state<R>(f: impl FnOnce(&mut UvcClass) -> R) -> Option<R> {
    let state = uvc_state_ptr()?;
    Some(unsafe { f(&mut *state) })
}

fn initialize_uvc_state() -> bool {
    if UVC_STATE_READY.load(Ordering::Acquire) {
        return true;
    }
    let Some(state) = UvcClass::try_new() else {
        return false;
    };
    unsafe {
        core::ptr::addr_of_mut!(UVC_STATE).cast::<UvcClass>().write(state);
    }
    UVC_STATE_READY.store(true, Ordering::Release);
    true
}

/// Validate and measure the complete UVC function beginning at interface 0.
fn parse_uvc_function(descriptors: &[u8]) -> Option<ParsedUvcFunction> {
    if descriptors.len() < 9
        || descriptors[0] != 9
        || descriptors[1] != USB_DESC_INTERFACE
        || descriptors[2] != UVC_CONTROL_INTERFACE
        || descriptors[3] != 0
        || descriptors[4] != 0
        || descriptors[5] != USB_CLASS_VIDEO
        || descriptors[6] != USB_SUBCLASS_VIDEO_CONTROL
        || descriptors[7] != 0
    {
        return None;
    }

    let mut offset = 0usize;
    let mut control_interface_seen = false;
    let mut streaming_alt0_seen = false;
    let mut streaming_alt1_seen = false;
    let mut streaming_endpoint_offset = None;
    let mut vc_header_seen = false;
    let mut vc_input_terminal_seen = false;
    let mut vc_output_terminal_seen = false;
    let mut vs_header_seen = false;
    let mut nv12_format_seen = false;
    let mut frame_seen = false;
    let mut color_matching_seen = false;
    let mut function_end = descriptors.len();
    let mut current_interface = u8::MAX;
    let mut current_alternate = u8::MAX;

    while offset + 2 <= descriptors.len() {
        let length = descriptors[offset] as usize;
        let descriptor_type = descriptors[offset + 1];
        if length < 2 || offset.checked_add(length)? > descriptors.len() {
            return None;
        }

        // The UVC function ends before the next IAD. The configuration
        // parser has already skipped the UVC IAD before calling this driver;
        // do not consume the following UAC2 IAD or bind its interfaces to UVC.
        if descriptor_type == USB_DESC_INTERFACE_ASSOCIATION {
            function_end = offset;
            break;
        }

        if descriptor_type == USB_DESC_INTERFACE {
            if length != 9 {
                return None;
            }
            let number = descriptors[offset + 2];
            let alternate = descriptors[offset + 3];
            let endpoint_count = descriptors[offset + 4];
            let class = descriptors[offset + 5];
            let subclass = descriptors[offset + 6];
            let protocol = descriptors[offset + 7];

            if offset != 0 && number != UVC_CONTROL_INTERFACE && number != UVC_STREAMING_INTERFACE {
                function_end = offset;
                break;
            }
            if class != USB_CLASS_VIDEO || protocol != 0 {
                return None;
            }

            current_interface = number;
            current_alternate = alternate;
            match (number, alternate, subclass) {
                (UVC_CONTROL_INTERFACE, 0, USB_SUBCLASS_VIDEO_CONTROL) => {
                    if control_interface_seen || endpoint_count != 0 {
                        return None;
                    }
                    control_interface_seen = true;
                }
                (UVC_STREAMING_INTERFACE, 0, USB_SUBCLASS_VIDEO_STREAMING) => {
                    if streaming_alt0_seen || endpoint_count != 0 {
                        return None;
                    }
                    streaming_alt0_seen = true;
                }
                (UVC_STREAMING_INTERFACE, 1, USB_SUBCLASS_VIDEO_STREAMING) => {
                    if streaming_alt1_seen || endpoint_count != 1 {
                        return None;
                    }
                    streaming_alt1_seen = true;
                }
                _ => return None,
            }
        } else if descriptor_type == USB_DESC_CS_INTERFACE
            && current_interface == UVC_CONTROL_INTERFACE
        {
            if length < 3 {
                return None;
            }
            match descriptors[offset + 2] {
                // UVC VideoControl Header.
                1 => {
                    if length != 13
                        || vc_header_seen
                        || u16::from_le_bytes([descriptors[offset + 3], descriptors[offset + 4]]) != 0x0110
                        || u16::from_le_bytes([descriptors[offset + 5], descriptors[offset + 6]]) != 30
                        || u32::from_le_bytes([
                            descriptors[offset + 7], descriptors[offset + 8],
                            descriptors[offset + 9], descriptors[offset + 10],
                        ]) != 48_000_000
                        || descriptors[offset + 11] != 1
                        || descriptors[offset + 12] != UVC_STREAMING_INTERFACE
                    {
                        return None;
                    }
                    vc_header_seen = true;
                }
                // Input terminal descriptor used by the existing camera function.
                2 => {
                    if length != 8
                        || descriptors[offset + 3] != 1
                        || u16::from_le_bytes([descriptors[offset + 4], descriptors[offset + 5]]) != 0x0200
                    {
                        return None;
                    }
                    vc_input_terminal_seen = true;
                }
                // USB streaming output terminal.
                3 => {
                    if length != 9
                        || descriptors[offset + 3] != 2
                        || u16::from_le_bytes([descriptors[offset + 4], descriptors[offset + 5]]) != 0x0101
                        || descriptors[offset + 7] != 1
                    {
                        return None;
                    }
                    vc_output_terminal_seen = true;
                }
                _ => return None,
            }
        } else if descriptor_type == USB_DESC_CS_INTERFACE
            && current_interface == UVC_STREAMING_INTERFACE
            && current_alternate == 0
        {
            if length < 3 {
                return None;
            }
            match descriptors[offset + 2] {
                // VideoStreaming input header emitted by UvcUsbClass.
                1 => {
                    if length != 14
                        || vs_header_seen
                        || descriptors[offset + 3] != 1
                        || u16::from_le_bytes([descriptors[offset + 4], descriptors[offset + 5]]) != UVC_VS_TOTAL_LENGTH
                        || descriptors[offset + 6] != UVC_STREAM_ENDPOINT
                        || descriptors[offset + 8] != 2
                        || descriptors[offset + 12] != 1
                    {
                        return None;
                    }
                    vs_header_seen = true;
                }
                // Uncompressed format descriptor: exactly one NV12 format.
                4 => {
                    if length != 27
                        || nv12_format_seen
                        || descriptors[offset + 3] != 1
                        || descriptors[offset + 4] != 1
                        || descriptors[offset + 5..offset + 21] != UVC_NV12_GUID
                        || descriptors[offset + 21] != 12
                        || descriptors[offset + 22] != 1
                    {
                        return None;
                    }
                    nv12_format_seen = true;
                }
                // Uncompressed frame descriptor for the configured camera.
                5 => {
                    if length != 30
                        || frame_seen
                        || descriptors[offset + 3] != 1
                        || descriptors[offset + 4] != 0
                        || u16::from_le_bytes([descriptors[offset + 5], descriptors[offset + 6]]) != WIDTH as u16
                        || u16::from_le_bytes([descriptors[offset + 7], descriptors[offset + 8]]) != HEIGHT as u16
                    {
                        return None;
                    }
                    let min_bitrate = u32::from_le_bytes([
                        descriptors[offset + 9], descriptors[offset + 10],
                        descriptors[offset + 11], descriptors[offset + 12],
                    ]);
                    let max_bitrate = u32::from_le_bytes([
                        descriptors[offset + 13], descriptors[offset + 14],
                        descriptors[offset + 15], descriptors[offset + 16],
                    ]);
                    let frame_bytes = u32::from_le_bytes([
                        descriptors[offset + 17], descriptors[offset + 18],
                        descriptors[offset + 19], descriptors[offset + 20],
                    ]);
                    let interval = u32::from_le_bytes([
                        descriptors[offset + 21], descriptors[offset + 22],
                        descriptors[offset + 23], descriptors[offset + 24],
                    ]);
                    if min_bitrate != FRAME_BIT_RATE
                        || max_bitrate != FRAME_BIT_RATE
                        || frame_bytes != crate::camera::FRAME_BYTES as u32
                        || interval != FRAME_INTERVAL_100NS
                        || descriptors[offset + 25] != 1
                        || u32::from_le_bytes([
                            descriptors[offset + 26], descriptors[offset + 27],
                            descriptors[offset + 28], descriptors[offset + 29],
                        ]) != FRAME_INTERVAL_100NS
                    {
                        return None;
                    }
                    frame_seen = true;
                }
                // Color-matching descriptor used by this NV12 function.
                0x0d => {
                    if length != 6
                        || descriptors[offset + 3] != 1
                        || descriptors[offset + 4] != 1
                        || descriptors[offset + 5] != 4
                    {
                        return None;
                    }
                    color_matching_seen = true;
                }
                _ => return None,
            }
        } else if descriptor_type == USB_DESC_ENDPOINT {
            if length != 7 {
                return None;
            }
            let address = descriptors[offset + 2];
            if address == UVC_STREAM_ENDPOINT {
                if current_interface != UVC_STREAMING_INTERFACE || current_alternate != 1 {
                    return None;
                }
                let attributes = descriptors[offset + 3];
                let max_packet = u16::from_le_bytes([
                    descriptors[offset + 4], descriptors[offset + 5],
                ]);
                let packet_bytes = max_packet & 0x07ff;
                let transactions = (((max_packet >> 11) & 0x03) + 1) as u8;
                if attributes != 0x05
                    || packet_bytes != UVC_MAX_PACKET_SIZE
                    || transactions != UVC_TRANSACTIONS_PER_MICROFRAME
                    || descriptors[offset + 6] != 1
                    || streaming_endpoint_offset.is_some()
                {
                    return None;
                }
                streaming_endpoint_offset = Some(offset);
            }
        }

        offset += length;
    }

    if control_interface_seen
        && streaming_alt0_seen
        && streaming_alt1_seen
        && vc_header_seen
        && vc_input_terminal_seen
        && vc_output_terminal_seen
        && vs_header_seen
        && nv12_format_seen
        && frame_seen
        && color_matching_seen
    {
        Some(ParsedUvcFunction {
            length: function_end,
            endpoint_offset: streaming_endpoint_offset?,
        })
    } else {
        None
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_open(
    rhport: u8,
    interface_desc: *const c_void,
    max_len: u16,
) -> u16 {
    if interface_desc.is_null() || max_len < 9 {
        return 0;
    }
    let descriptors = unsafe {
        core::slice::from_raw_parts(interface_desc.cast::<u8>(), max_len as usize)
    };
    let Some(parsed) = parse_uvc_function(descriptors) else {
        return 0;
    };
    if parsed.length < 9 || parsed.length > max_len as usize || !initialize_uvc_state() {
        return 0;
    }

    let endpoint_desc = unsafe { descriptors.as_ptr().add(parsed.endpoint_offset) };
    let packet_size = u16::from_le_bytes(unsafe { [*endpoint_desc.add(4), *endpoint_desc.add(5)] }) & 0x07ff;
    if !UVC_ENDPOINT_ALLOCATED.load(Ordering::Acquire) {
        if !unsafe { xcore_usbd_edpt_iso_alloc(rhport, UVC_STREAM_ENDPOINT, packet_size) } {
            return 0;
        }
        UVC_ENDPOINT_ALLOCATED.store(true, Ordering::Release);
    }
    unsafe {
        core::ptr::copy_nonoverlapping(
            endpoint_desc,
            core::ptr::addr_of_mut!(UVC_ENDPOINT_DESCRIPTOR).cast::<u8>(),
            7,
        );
    }
    parsed.length as u16
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_reset(rhport: u8) {
    cortex_m::interrupt::free(|_| {
    let _ = with_uvc_state(|state| {
        let _ = state.set_interface(UVC_STREAMING_INTERFACE, 0);
    });
    UVC_ENDPOINT_ALLOCATED.store(false, Ordering::Release);

    })
}

fn submit_uvc_batch(rhport: u8, batch: (&'static [u8], u8)) -> bool {
    let (bytes, _packet_count) = batch;
    if bytes.is_empty() || bytes.len() > u16::MAX as usize {
        return false;
    }
    if !unsafe { xcore_usbd_edpt_claim(rhport, UVC_STREAM_ENDPOINT) } {
        return false;
    }
    let submitted = unsafe {
        xcore_usbd_edpt_xfer(
            rhport,
            UVC_STREAM_ENDPOINT,
            bytes.as_ptr() as *mut u8,
            bytes.len() as u16,
        )
    };
    if !submitted {
        unsafe { xcore_usbd_edpt_release(rhport, UVC_STREAM_ENDPOINT); }
    }
    submitted
}

/// Re-arm the ISO endpoint directly from TinyUSB's DCD completion ISR.
/// TinyUSB has already released the endpoint's busy/claimed state before
/// invoking xfer_isr, so no endpoint claim is needed here.
fn submit_uvc_batch_isr(rhport: u8, batch: (&'static [u8], u8)) -> bool {
    let (bytes, _packet_count) = batch;
    if bytes.is_empty() || bytes.len() > u16::MAX as usize {
        return false;
    }
    unsafe {
        xcore_usbd_edpt_xfer_isr(
            rhport,
            UVC_STREAM_ENDPOINT,
            bytes.as_ptr() as *mut u8,
            bytes.len() as u16,
        )
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_control_xfer(
    rhport: u8,
    stage: u8,
    request_ptr: *const c_void,
) -> bool {
    cortex_m::interrupt::free(|_| {
    if request_ptr.is_null() {
        return false;
    }
    let request = unsafe {
        core::ptr::read_unaligned(request_ptr.cast::<UsbControlRequest>())
    };
    let request_type = request.request_type;
    let request_code = request.request;
    let value = u16::from_le(request.value);
    let index = u16::from_le(request.index);
    let length = u16::from_le(request.length);
    let interface = index as u8;

    if (request_type & REQUEST_TYPE_MASK) == REQUEST_TYPE_STANDARD
        && (request_type & REQUEST_RECIPIENT_MASK) == REQUEST_RECIPIENT_INTERFACE
    {
        if stage != CONTROL_STAGE_SETUP {
            return true;
        }
        match request_code {
            REQUEST_GET_INTERFACE => {
                if interface != UVC_CONTROL_INTERFACE && interface != UVC_STREAMING_INTERFACE {
                    return false;
                }
                let alternate = if interface == UVC_STREAMING_INTERFACE {
                    let Some(alt) = with_uvc_state(|state| state.alternate_setting()) else {
                        return false;
                    };
                    alt
                } else {
                    0
                };
                unsafe {
                    UVC_ALT_RESPONSE = alternate;
                    xcore_tud_control_xfer(
                        rhport,
                        request_ptr,
                        core::ptr::addr_of_mut!(UVC_ALT_RESPONSE).cast::<c_void>(),
                        1,
                    )
                }
            }
            REQUEST_SET_INTERFACE => {
                if length != 0 || (value >> 8) != 0 {
                    return false;
                }
                if interface == UVC_CONTROL_INTERFACE {
                    if (value as u8) != 0 {
                        return false;
                    }
                    return unsafe { xcore_tud_control_status(rhport, request_ptr) };
                }
                if interface != UVC_STREAMING_INTERFACE || value as u8 > 1 {
                    return false;
                }
                let alternate = value as u8;
                if alternate == 1 {
                    let endpoint_desc = core::ptr::addr_of!(UVC_ENDPOINT_DESCRIPTOR).cast::<c_void>();
                    if !unsafe { xcore_usbd_edpt_iso_activate(rhport, endpoint_desc) } {
                        return false;
                    }
                }
                let changed = with_uvc_state(|state| {
                    state.set_interface(UVC_STREAMING_INTERFACE, alternate).is_ok()
                }).unwrap_or(false);
                if !changed {
                    if alternate == 1 {
                        unsafe { xcore_usbd_edpt_close(rhport, UVC_STREAM_ENDPOINT); }
                    }
                    return false;
                }
                if alternate == 0 {
                    unsafe { xcore_usbd_edpt_close(rhport, UVC_STREAM_ENDPOINT); }
                } else {
                    let first = with_uvc_state(|state| state.start_stream_batch()).flatten();
                    if let Some(batch) = first {
                        if !submit_uvc_batch(rhport, batch) {
                            return false;
                        }
                    }
                }
                unsafe { xcore_tud_control_status(rhport, request_ptr) }
            }
            _ => false,
        }
    } else if (request_type & REQUEST_TYPE_MASK) == REQUEST_TYPE_CLASS {
        if stage == CONTROL_STAGE_SETUP {
            if (request_type & REQUEST_DIRECTION_IN) != 0 {
                let Some(state) = uvc_state_ptr() else {
                    return false;
                };
                let response = unsafe {
                    (&mut *state).control(request_type, request_code, value, index, length, &[])
                };
                match response {
                    Ok(ControlResponse::Data(data)) => unsafe {
                        xcore_tud_control_xfer(
                            rhport,
                            request_ptr,
                            data.as_ptr() as *mut c_void,
                            data.len() as u16,
                        )
                    },
                    _ => false,
                }
            } else if request_code == REQUEST_SET_CUR && length <= crate::uvc::PROBE_LENGTH as u16 {
                let buffer = unsafe { xcore_usbd_control_buffer() };
                if buffer.is_null() {
                    return false;
                }
                unsafe {
                    xcore_tud_control_xfer(
                        rhport,
                        request_ptr,
                        buffer.cast::<c_void>(),
                        length,
                    )
                }
            } else {
                false
            }
        } else if stage == CONTROL_STAGE_DATA && (request_type & REQUEST_DIRECTION_IN) == 0 {
            if length > crate::uvc::PROBE_LENGTH as u16 {
                return false;
            }
            let buffer = unsafe { xcore_usbd_control_buffer() };
            if buffer.is_null() {
                return false;
            }
            let data = unsafe { core::slice::from_raw_parts(buffer, length as usize) };
            let Some(state) = uvc_state_ptr() else {
                return false;
            };
            matches!(
                unsafe {
                    (&mut *state).control(request_type, request_code, value, index, length, data)
                },
                Ok(ControlResponse::Ack)
            )
        } else {
            true
        }
    } else {
        false
    }

    })
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_xfer(
    rhport: u8,
    ep_addr: u8,
    result: i32,
    _xferred_bytes: u32,
) -> bool {
    cortex_m::interrupt::free(|_| {
    if ep_addr != UVC_STREAM_ENDPOINT {
        return false;
    }
    let batch = with_uvc_state(|state| {
        if result == 0 {
            state.next_in_batch()
        } else {
            state.restart_after_iso_recovery()
        }
    }).flatten();
    if let Some(batch) = batch {
        return submit_uvc_batch(rhport, batch);
    }
    true

    })
}

/// ISR completion path: keep the UVC high-bandwidth endpoint primed without
/// waiting for the main-loop TinyUSB task to drain the event queue.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_xfer_isr(
    rhport: u8,
    ep_addr: u8,
    result: i32,
    _xferred_bytes: u32,
) -> bool {
    if ep_addr != UVC_STREAM_ENDPOINT {
        return false;
    }
    let batch = with_uvc_state(|state| {
        if result == 0 {
            state.next_in_batch()
        } else {
            state.restart_after_iso_recovery()
        }
    }).flatten();
    match batch {
        Some(batch) => submit_uvc_batch_isr(rhport, batch),
        None => true,
    }
}

/// Thread-context kick for the first batch if streaming starts before the
/// producer has published a READY buffer. Call from the main loop.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_task(rhport: u8) {
    cortex_m::interrupt::free(|_| {
    let ready = unsafe { xcore_usbd_edpt_ready(rhport, UVC_STREAM_ENDPOINT) };
    if !ready {
        return;
    }
    let batch = with_uvc_state(|state| {
        if state.is_streaming() {
            state.next_in_batch()
        } else {
            None
        }
    }).flatten();
    if let Some(batch) = batch {
        let _ = submit_uvc_batch(rhport, batch);
    }

    })
}

const UAC_STREAM_INTERFACE: u8 = 3;

static AUDIO_STATE_READY: AtomicBool = AtomicBool::new(false);
static mut AUDIO_STATE: MaybeUninit<BkEmulatorAudio> = MaybeUninit::uninit();

fn audio_state_ptr() -> Option<*mut BkEmulatorAudio> {
    if !AUDIO_STATE_READY.load(Ordering::Acquire) {
        return None;
    }
    Some(core::ptr::addr_of_mut!(AUDIO_STATE).cast::<BkEmulatorAudio>())
}

fn initialize_audio_state() -> bool {
    if AUDIO_STATE_READY.load(Ordering::Acquire) {
        return true;
    }
    unsafe {
        core::ptr::addr_of_mut!(AUDIO_STATE)
            .cast::<BkEmulatorAudio>()
            .write(BkEmulatorAudio::new());
    }
    AUDIO_STATE_READY.store(true, Ordering::Release);
    true
}

/// Called by TinyUSB after it has applied SET_INTERFACE to the audio stream.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_audio_set_interface(interface: u8, alternate: u8) -> bool {
    if interface != UAC_STREAM_INTERFACE || alternate > 1 {
        return false;
    }
    cortex_m::interrupt::free(|_| {
        if !initialize_audio_state() {
            return false;
        }
        let Some(state) = audio_state_ptr() else {
            return false;
        };
        unsafe { (&mut *state).tinyusb_set_active(alternate == 1); }
        true
    })
}

/// Generate one 6-sample / 12-byte packet for TinyUSB's audio software FIFO.
/// TinyUSB copies these bytes into its own DMA-capable endpoint buffer.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_audio_fill_packet(buffer: *mut u8, capacity: u16) -> u16 {
    if buffer.is_null() || capacity < 12 {
        return 0;
    }
    let Some(state) = audio_state_ptr() else {
        return 0;
    };
    let packet = unsafe { (&mut *state).tinyusb_prepare_packet() };
    let Some((source, length)) = packet else {
        return 0;
    };
    if length > capacity {
        unsafe { (&mut *state).tinyusb_transfer_retired(); }
        return 0;
    }
    unsafe {
        core::ptr::copy_nonoverlapping(source, buffer, length as usize);
        (&mut *state).tinyusb_transfer_retired();
    }
    length
}

/// TinyUSB calls this on unmount/reset; the next SET_INTERFACE will reinitialize
/// capture state and start a fresh audio stream.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_audio_stop() {
    cortex_m::interrupt::free(|_| {
        if let Some(state) = audio_state_ptr() {
            unsafe { (&mut *state).tinyusb_set_active(false); }
        }
    })
}
