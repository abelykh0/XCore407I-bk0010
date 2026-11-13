//! Rust side of the TinyUSB custom-class callback boundary.
//!
//! The class-driver registration structure stays in C because it is private
//! to TinyUSB. This module implements UVC parsing, alternate settings,
//! Probe/Commit controls, and the high-bandwidth ISO IN transfer path.

use core::ffi::c_void;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::camera::{HEIGHT, WIDTH};
use crate::bk_audio::BkEmulatorAudio;
use crate::uvc::{ControlResponse, UvcClass, active_frame_interval_100ns, active_frame_rate_fps};

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
const UVC_VS_TOTAL_LENGTH: u16 = 83;
const UVC_NV12_GUID: [u8; 16] = [
    0x4e, 0x56, 0x31, 0x32, 0x00, 0x00, 0x10, 0x00,
    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71,
];

#[derive(Clone, Copy)]
struct ParsedUvcFunction {
    length: usize,
    endpoint_offset: usize,
}

static UVC_STATE_READY: AtomicBool = AtomicBool::new(false);
static UVC_ENDPOINT_ALLOCATED: AtomicBool = AtomicBool::new(false);
static mut UVC_STATE: MaybeUninit<UvcClass> = MaybeUninit::uninit();
static mut UVC_ENDPOINT_DESCRIPTOR: [u8; 7] = [0; 7];

unsafe extern "C" {
    fn xcore_usbd_edpt_iso_alloc(rhport: u8, ep_addr: u8, packet_size: u16) -> bool;
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
    let expected_frame_interval = active_frame_interval_100ns();
    let expected_frame_bit_rate =
        crate::camera::FRAME_BYTES as u32 * 8 * active_frame_rate_fps();
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
                // Still-image frame descriptor with no alternate still sizes.
                3 => {
                    if length != 6
                        || descriptors[offset + 3] != UVC_STREAM_ENDPOINT
                        || descriptors[offset + 4] != 0
                        || descriptors[offset + 5] != 0
                    {
                        return None;
                    }
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
                    if min_bitrate != expected_frame_bit_rate
                        || max_bitrate != expected_frame_bit_rate
                        || frame_bytes != crate::camera::FRAME_BYTES as u32
                        || interval != expected_frame_interval
                        || descriptors[offset + 25] != 1
                        || u32::from_le_bytes([
                            descriptors[offset + 26], descriptors[offset + 27],
                            descriptors[offset + 28], descriptors[offset + 29],
                        ]) != expected_frame_interval
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
pub extern "C" fn xcore_tud_uvc_reset(_rhport: u8) {
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
pub extern "C" fn xcore_tud_uvc_endpoint_descriptor() -> *const c_void {
    core::ptr::addr_of!(UVC_ENDPOINT_DESCRIPTOR).cast::<u8>().cast::<c_void>()
}

/// C class-driver helpers: request decoding and EP0 stages live in TinyUSB's C layer.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_get_interface(interface: u8, alternate: *mut u8) -> bool {
    if alternate.is_null() || (interface != UVC_CONTROL_INTERFACE && interface != UVC_STREAMING_INTERFACE) {
        return false;
    }
    let value = if interface == UVC_STREAMING_INTERFACE {
        let Some(value) = with_uvc_state(|state| state.alternate_setting()) else {
            return false;
        };
        value
    } else {
        0
    };
    unsafe { alternate.write(value); }
    true
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_set_interface(alternate: u8) -> bool {
    with_uvc_state(|state| state.set_interface(UVC_STREAMING_INTERFACE, alternate).is_ok())
        .unwrap_or(false)
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_start_stream(rhport: u8) -> bool {
    let first = with_uvc_state(|state| state.start_stream_batch()).flatten();
    match first {
        Some(batch) => submit_uvc_batch(rhport, batch),
        None => true,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_class_control_in(
    request_type: u8,
    request_code: u8,
    value: u16,
    index: u16,
    length: u16,
    data_out: *mut *const u8,
    length_out: *mut u16,
) -> bool {
    if data_out.is_null() || length_out.is_null() {
        return false;
    }
    let Some(state) = uvc_state_ptr() else {
        return false;
    };
    let response = unsafe {
        (&mut *state).control(request_type, request_code, value, index, length, &[])
    };
    match response {
        Ok(ControlResponse::Data(data)) => {
            unsafe {
                data_out.write(data.as_ptr());
                length_out.write(data.len() as u16);
            }
            true
        }
        _ => false,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_class_control_out(
    request_type: u8,
    request_code: u8,
    value: u16,
    index: u16,
    length: u16,
    data: *const u8,
) -> bool {
    if data.is_null() || length > crate::uvc::PROBE_LENGTH as u16 {
        return false;
    }
    let Some(state) = uvc_state_ptr() else {
        return false;
    };
    let data = unsafe { core::slice::from_raw_parts(data, length as usize) };
    matches!(
        unsafe { (&mut *state).control(request_type, request_code, value, index, length, data) },
        Ok(ControlResponse::Ack)
    )
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
    if interface != UAC_STREAM_INTERFACE {
        return true;
    }
    if alternate > 1 {
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

/// Render a bounded batch of PCM samples from the main loop.
#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_audio_render() {
    if let Some(state) = audio_state_ptr() {
        unsafe { (&mut *state).render_available(448); }
    }
}

/// USB completion callback: copy pre-rendered PCM samples only.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn xcore_tud_audio_fill_packet(buffer: *mut u8, capacity: u16) -> u16 {
    unsafe { crate::bk_audio::fill_usb_packet(buffer, capacity) }
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
