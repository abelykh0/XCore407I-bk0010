//! TinyUSB device descriptors matching the existing usb-device class layout.
//!
//! These bytes preserve the prior UVC/UAC2 interface numbering, IADs,
//! endpoint addresses, high-bandwidth UVC transactions, and packet sizes.

use crate::camera::{HEIGHT, WIDTH};
use crate::uvc::{
    active_frame_interval_100ns, active_frame_rate_fps, FRAME_BIT_RATE, FRAME_BYTES,
    FRAME_INTERVAL_100NS,
    ISO_PACKETS_PER_MICROFRAME, MAX_PACKET_SIZE,
};

const CONFIGURATION_DESCRIPTOR_LEN: usize = 230;

const fn push_u8(
    descriptor: &mut [u8; CONFIGURATION_DESCRIPTOR_LEN],
    offset: &mut usize,
    value: u8,
) {
    descriptor[*offset] = value;
    *offset += 1;
}

const fn push_u16(
    descriptor: &mut [u8; CONFIGURATION_DESCRIPTOR_LEN],
    offset: &mut usize,
    value: u16,
) {
    push_u8(descriptor, offset, value as u8);
    push_u8(descriptor, offset, (value >> 8) as u8);
}

const fn push_u32(
    descriptor: &mut [u8; CONFIGURATION_DESCRIPTOR_LEN],
    offset: &mut usize,
    value: u32,
) {
    push_u8(descriptor, offset, value as u8);
    push_u8(descriptor, offset, (value >> 8) as u8);
    push_u8(descriptor, offset, (value >> 16) as u8);
    push_u8(descriptor, offset, (value >> 24) as u8);
}

const fn push_bytes(
    descriptor: &mut [u8; CONFIGURATION_DESCRIPTOR_LEN],
    offset: &mut usize,
    bytes: &[u8],
) {
    let mut index = 0;
    while index < bytes.len() {
        push_u8(descriptor, offset, bytes[index]);
        index += 1;
    }
}

const fn build_configuration_descriptor() -> [u8; CONFIGURATION_DESCRIPTOR_LEN] {
    let mut descriptor = [0u8; CONFIGURATION_DESCRIPTOR_LEN];
    let mut offset = 0usize;

    // Configuration header: 4 interfaces, bus-powered, 100 mA.
    push_bytes(&mut descriptor, &mut offset, &[
        9, 0x02, 0xe6, 0x00, 4, 1, 0, 0x80, 50,
    ]);

    // UVC IAD and VideoControl interface.
    push_bytes(&mut descriptor, &mut offset, &[
        8, 0x0b, 0, 2, 0x0e, 0x03, 0, 4,
        9, 0x04, 0, 0, 0, 0x0e, 0x01, 0, 0,
        13, 0x24, 0x01, 0x10, 0x01, 30, 0, 0, 0x6c, 0xdc, 0x02, 1, 1,
        8, 0x24, 0x02, 1, 0, 2, 0, 0,
        9, 0x24, 0x03, 2, 1, 1, 0, 1, 0,
    ]);

    // UVC VideoStreaming alternate setting 0 and class-specific descriptors.
    push_bytes(&mut descriptor, &mut offset, &[
        9, 0x04, 1, 0, 0, 0x0e, 0x02, 0, 4,
        14, 0x24, 0x01, 1, 77, 0, 0x81, 0, 2, 0, 0, 0, 1, 0,
        27, 0x24, 0x04, 1, 1,
        0x4e, 0x56, 0x31, 0x32, 0, 0, 0x10, 0, 0x80, 0, 0, 0xaa, 0, 0x38, 0x9b, 0x71,
        6, 0x24, 0x03, 0x81, 0, 0,
        30, 0x24, 0x05, 1, 0,
    ]);

    // UVC frame descriptor: dimensions, bitrates, frame size and interval.
    push_u16(&mut descriptor, &mut offset, WIDTH as u16);
    push_u16(&mut descriptor, &mut offset, HEIGHT as u16);
    push_u32(&mut descriptor, &mut offset, FRAME_BIT_RATE);
    push_u32(&mut descriptor, &mut offset, FRAME_BIT_RATE);
    push_u32(&mut descriptor, &mut offset, FRAME_BYTES);
    push_u32(&mut descriptor, &mut offset, FRAME_INTERVAL_100NS);
    push_u8(&mut descriptor, &mut offset, 1);
    push_u32(&mut descriptor, &mut offset, FRAME_INTERVAL_100NS);

    // UVC color matching, alternate setting 1, and 3 x 768-byte HS ISO EP1 IN.
    let uvc_w_max_packet =
        MAX_PACKET_SIZE | (((ISO_PACKETS_PER_MICROFRAME - 1) as u16) << 11);
    push_bytes(&mut descriptor, &mut offset, &[
        6, 0x24, 0x0d, 1, 1, 4,
        9, 0x04, 1, 1, 1, 0x0e, 0x02, 0, 4,
        7, 0x05, 0x81, 0x05,
    ]);
    push_u16(&mut descriptor, &mut offset, uvc_w_max_packet);
    push_u8(&mut descriptor, &mut offset, 1);

    // CDC ACM function: interfaces 2 (control) and 3 (data).
    // EP4 IN notification; EP3 OUT/IN bulk. HS bulk endpoints use 512-byte MPS.
    push_bytes(&mut descriptor, &mut offset, &[
        8, 0x0b, 2, 2, 0x02, 0x02, 0x01, 0,
        9, 0x04, 2, 0, 1, 0x02, 0x02, 0x01, 0,
        5, 0x24, 0x00, 0x10, 0x01,
        5, 0x24, 0x01, 0x00, 3,
        4, 0x24, 0x02, 0x02,
        5, 0x24, 0x06, 2, 3,
        7, 0x05, 0x84, 0x03, 8, 0, 16,
        9, 0x04, 3, 0, 2, 0x0a, 0, 0, 0,
        7, 0x05, 0x03, 0x02, 0x00, 0x02, 0,
        7, 0x05, 0x83, 0x02, 0x00, 0x02, 0,
    ]);

    if offset != CONFIGURATION_DESCRIPTOR_LEN {
        panic!("TinyUSB configuration descriptor length mismatch");
    }
    descriptor
}

static DEVICE_DESCRIPTOR: [u8; 18] = [
    18, 0x01, 0x10, 0x02, 0xef, 0x02, 0x01, 64,
    0x09, 0x12, 0x71, 0x40, 0x10, 0x00, 0, 0, 0, 1,
];

// usb-device emitted an empty language list because no device-level strings
// were configured. Preserve that exact two-byte string descriptor.
static EMPTY_LANGUAGE_DESCRIPTOR: [u16; 1] = [0x0302];
static CAMERA_INTERFACE_STRING: [u16; 22] = [
    0x032c,
    b'X' as u16, b'C' as u16, b'o' as u16, b'r' as u16, b'e' as u16,
    b'4' as u16, b'0' as u16, b'7' as u16, b'I' as u16, b' ' as u16,
    b'N' as u16, b'V' as u16, b'1' as u16, b'2' as u16, b' ' as u16,
    b'c' as u16, b'a' as u16, b'm' as u16, b'e' as u16, b'r' as u16, b'a' as u16,
];

// usb-device's BosWriter emits a BOS header plus a USB 2.0 Extension
// capability with bmAttributes = 0.
static BOS_DESCRIPTOR: [u8; 12] = [
    5, 0x0f, 12, 0, 1,
    7, 0x10, 2, 0, 0, 0, 0,
];

// Match usb-device's Device Qualifier for the high-speed composite device.
static DEVICE_QUALIFIER_DESCRIPTOR: [u8; 10] = [
    10, 0x06, 0x10, 0x02, 0xef, 0x02, 0x01, 64, 1, 0,
];

static mut CONFIGURATION_DESCRIPTOR: [u8; CONFIGURATION_DESCRIPTOR_LEN] =
    build_configuration_descriptor();

#[unsafe(no_mangle)]
pub extern "C" fn tud_descriptor_device_cb() -> *const u8 {
    DEVICE_DESCRIPTOR.as_ptr()
}

#[unsafe(no_mangle)]
pub extern "C" fn tud_descriptor_configuration_cb(index: u8) -> *const u8 {
    if index != 0 {
        return core::ptr::null();
    }

    // The same composite descriptor serves both startup modes. Update the
    // frame interval and bitrate before TinyUSB exposes it to the host.
    let interval = active_frame_interval_100ns();
    let bitrate = FRAME_BYTES * 8 * active_frame_rate_fps();
    let descriptor = core::ptr::addr_of_mut!(CONFIGURATION_DESCRIPTOR).cast::<u8>();
    unsafe {
        for (offset, value) in [
            (121usize, bitrate as u8),
            (122, (bitrate >> 8) as u8),
            (123, (bitrate >> 16) as u8),
            (124, (bitrate >> 24) as u8),
            (125, bitrate as u8),
            (126, (bitrate >> 8) as u8),
            (127, (bitrate >> 16) as u8),
            (128, (bitrate >> 24) as u8),
            (133, interval as u8),
            (134, (interval >> 8) as u8),
            (135, (interval >> 16) as u8),
            (136, (interval >> 24) as u8),
            (138, interval as u8),
            (139, (interval >> 8) as u8),
            (140, (interval >> 16) as u8),
            (141, (interval >> 24) as u8),
        ] {
            descriptor.add(offset).write(value);
        }
    }
    descriptor
}

#[unsafe(no_mangle)]
pub extern "C" fn tud_descriptor_device_qualifier_cb() -> *const u8 {
    DEVICE_QUALIFIER_DESCRIPTOR.as_ptr()
}

#[unsafe(no_mangle)]
pub extern "C" fn tud_descriptor_bos_cb() -> *const u8 {
    BOS_DESCRIPTOR.as_ptr()
}

#[unsafe(no_mangle)]
pub extern "C" fn tud_descriptor_string_cb(index: u8, _lang_id: u16) -> *const u16 {
    match index {
        0 => EMPTY_LANGUAGE_DESCRIPTOR.as_ptr(),
        4 => CAMERA_INTERFACE_STRING.as_ptr(),
        _ => core::ptr::null(),
    }
}
