//! Rust side of the TinyUSB custom-class callback boundary.
//!
//! The class-driver registration structure stays in C because it is private
//! to TinyUSB. UVC descriptor parsing is implemented here, but the callback
//! still returns zero so TinyUSB does not claim the interface until endpoint
//! activation and the complete control/data paths are ready.

use core::ffi::c_void;

const USB_DESC_INTERFACE: u8 = 0x04;
const USB_DESC_ENDPOINT: u8 = 0x05;
const USB_DESC_CS_INTERFACE: u8 = 0x24;
const USB_CLASS_VIDEO: u8 = 0x0e;
const USB_SUBCLASS_VIDEO_CONTROL: u8 = 0x01;
const USB_SUBCLASS_VIDEO_STREAMING: u8 = 0x02;
const UVC_CONTROL_INTERFACE: u8 = 0;
const UVC_STREAMING_INTERFACE: u8 = 1;
const UVC_STREAM_ENDPOINT: u8 = 0x81;
const UVC_MAX_PACKET_SIZE: u16 = 768;
const UVC_TRANSACTIONS_PER_MICROFRAME: u8 = 3;

/// Validate and measure the complete UVC function beginning at interface 0.
///
/// The configuration parser passes open() a slice beginning at the
/// VideoControl interface (the preceding IAD is not included). The returned
/// span ends immediately before the next function's interface descriptor.
/// This parser deliberately does not claim the interfaces yet.
fn parse_uvc_function(descriptors: &[u8]) -> Option<usize> {
    if descriptors.len() < 9
        || descriptors[0] < 9
        || descriptors[1] != USB_DESC_INTERFACE
        || descriptors[2] != UVC_CONTROL_INTERFACE
        || descriptors[3] != 0
        || descriptors[5] != USB_CLASS_VIDEO
        || descriptors[6] != USB_SUBCLASS_VIDEO_CONTROL
    {
        return None;
    }

    let mut offset = 0usize;
    let mut control_interface_seen = false;
    let mut streaming_alt0_seen = false;
    let mut streaming_alt1_seen = false;
    let mut streaming_endpoint_seen = false;
    let mut vc_header_seen = false;
    let mut function_end = descriptors.len();
    let mut current_interface = u8::MAX;
    let mut current_alternate = u8::MAX;

    while offset + 2 <= descriptors.len() {
        let length = descriptors[offset] as usize;
        let descriptor_type = descriptors[offset + 1];
        if length < 2 || offset.checked_add(length)? > descriptors.len() {
            return None;
        }

        if descriptor_type == USB_DESC_INTERFACE {
            if length < 9 {
                return None;
            }
            let number = descriptors[offset + 2];
            let alternate = descriptors[offset + 3];
            let class = descriptors[offset + 5];
            let subclass = descriptors[offset + 6];
            current_interface = number;
            current_alternate = alternate;

            if offset != 0 && number != UVC_CONTROL_INTERFACE && number != UVC_STREAMING_INTERFACE {
                function_end = offset;
                break;
            }

            if class != USB_CLASS_VIDEO {
                return None;
            }

            match (number, alternate, subclass) {
                (UVC_CONTROL_INTERFACE, 0, USB_SUBCLASS_VIDEO_CONTROL) => {
                    control_interface_seen = true;
                }
                (UVC_STREAMING_INTERFACE, 0, USB_SUBCLASS_VIDEO_STREAMING) => {
                    streaming_alt0_seen = true;
                }
                (UVC_STREAMING_INTERFACE, 1, USB_SUBCLASS_VIDEO_STREAMING) => {
                    streaming_alt1_seen = true;
                }
                _ => return None,
            }
        } else if descriptor_type == USB_DESC_CS_INTERFACE
            && current_interface == UVC_CONTROL_INTERFACE
            && length >= 13
            && descriptors[offset + 2] == 1
        {
            // UVC VideoControl Header: UVC 1.1, one streaming interface,
            // and that interface is number 1.
            if u16::from_le_bytes([descriptors[offset + 3], descriptors[offset + 4]]) != 0x0110
                || descriptors[offset + 11] != 1
                || descriptors[offset + 12] != UVC_STREAMING_INTERFACE
            {
                return None;
            }
            vc_header_seen = true;
        } else if descriptor_type == USB_DESC_ENDPOINT {
            if length < 7 {
                return None;
            }
            let address = descriptors[offset + 2];
            let attributes = descriptors[offset + 3];
            let max_packet = u16::from_le_bytes([
                descriptors[offset + 4],
                descriptors[offset + 5],
            ]);
            let packet_bytes = max_packet & 0x07ff;
            let transactions = (((max_packet >> 11) & 0x03) + 1) as u8;

            if address == UVC_STREAM_ENDPOINT {
                if current_interface != UVC_STREAMING_INTERFACE || current_alternate != 1 {
                    return None;
                }
                if attributes & 0x03 != 0x01
                    || packet_bytes != UVC_MAX_PACKET_SIZE
                    || transactions != UVC_TRANSACTIONS_PER_MICROFRAME
                {
                    return None;
                }
                streaming_endpoint_seen = true;
            }
        }

        offset += length;
    }

    if control_interface_seen
        && streaming_alt0_seen
        && streaming_alt1_seen
        && streaming_endpoint_seen
        && vc_header_seen
    {
        Some(function_end)
    } else {
        None
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_open(
    _rhport: u8,
    interface_desc: *const c_void,
    max_len: u16,
) -> u16 {
    if interface_desc.is_null() || max_len < 9 {
        return 0;
    }

    // TinyUSB supplies a descriptor span bounded by the configuration
    // descriptor. Read only within that span and validate every bLength.
    let descriptors = unsafe {
        core::slice::from_raw_parts(interface_desc.cast::<u8>(), max_len as usize)
    };
    let _validated_function_len = parse_uvc_function(descriptors);

    // Keep the old class stack in charge until the TinyUSB control,
    // alternate-setting and ISO transfer paths are implemented together.
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_control_xfer(
    _rhport: u8,
    _stage: u8,
    _request: *const c_void,
) -> bool {
    false
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_uvc_xfer(
    _rhport: u8,
    _ep_addr: u8,
    _result: i32,
    _xferred_bytes: u32,
) -> bool {
    false
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_audio_open(
    _rhport: u8,
    _interface_desc: *const c_void,
    _max_len: u16,
) -> u16 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_audio_control_xfer(
    _rhport: u8,
    _stage: u8,
    _request: *const c_void,
) -> bool {
    false
}

#[unsafe(no_mangle)]
pub extern "C" fn xcore_tud_audio_xfer(
    _rhport: u8,
    _ep_addr: u8,
    _result: i32,
    _xferred_bytes: u32,
) -> bool {
    false
}