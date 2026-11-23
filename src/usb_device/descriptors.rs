//! TinyUSB device descriptors matching the existing usb-device class layout.
//!
//! These bytes preserve the prior UVC/UAC2 interface numbering, IADs,
//! endpoint addresses, high-bandwidth UVC transactions, and packet sizes.

use crate::camera::{HEIGHT, WIDTH};
use crate::usb_device::uvc::{
    active_frame_interval_100ns, active_frame_rate_fps, FRAME_BIT_RATE, FRAME_BYTES,
    ENDPOINT_IN, FRAME_INTERVAL_100NS, ISO_PACKETS_PER_MICROFRAME, MAX_PACKET_SIZE, NV12_GUID,
};

const CDC_CONFIGURATION_DESCRIPTOR_LEN: usize = 224;

const fn push_u8<const N: usize>(
    descriptor: &mut [u8; N],
    offset: &mut usize,
    value: u8,
) {
    descriptor[*offset] = value;
    *offset += 1;
}

const fn push_u16<const N: usize>(
    descriptor: &mut [u8; N],
    offset: &mut usize,
    value: u16,
) {
    push_u8(descriptor, offset, value as u8);
    push_u8(descriptor, offset, (value >> 8) as u8);
}

const fn push_u32<const N: usize>(
    descriptor: &mut [u8; N],
    offset: &mut usize,
    value: u32,
) {
    push_u8(descriptor, offset, value as u8);
    push_u8(descriptor, offset, (value >> 8) as u8);
    push_u8(descriptor, offset, (value >> 16) as u8);
    push_u8(descriptor, offset, (value >> 24) as u8);
}

const fn push_bytes<const N: usize>(
    descriptor: &mut [u8; N],
    offset: &mut usize,
    bytes: &[u8],
) {
    let mut index = 0;
    while index < bytes.len() {
        push_u8(descriptor, offset, bytes[index]);
        index += 1;
    }
}

const fn push_uvc_descriptors<const N: usize>(
    descriptor: &mut [u8; N],
    offset: &mut usize,
    frame_bit_rate: u32,
    frame_interval_100ns: u32,
) {
    // Interface Association Descriptor: UVC function, interfaces 0–1.
    push_bytes(descriptor, offset, &[
        8, 0x0b, 0, 2, 0x0e, 0x03, 0, 4,
    ]);

    // VideoControl interface 0 (no endpoints).
    push_bytes(descriptor, offset, &[
        9, 0x04, 0, 0, 0, 0x0e, 0x01, 0, 0,
    ]);

    // Class-specific VC header: UVC 1.1, 48 MHz clock, one streaming interface.
    push_bytes(descriptor, offset, &[
        13, 0x24, 0x01, 0x10, 0x01, 30, 0, 0, 0x6c, 0xdc, 0x02, 1, 1,
    ]);

    // Camera Input Terminal (ID 1) and Processing Unit (ID 2).
    push_bytes(descriptor, offset, &[
        8, 0x24, 0x02, 1, 0, 2, 0, 0,
        9, 0x24, 0x03, 2, 1, 1, 0, 1, 0,
    ]);

    // VideoStreaming interface 1, alternate setting 0 (zero-bandwidth).
    push_bytes(descriptor, offset, &[
        9, 0x04, 1, 0, 0, 0x0e, 0x02, 0, 4,
    ]);

    // VS input header: one format, endpoint address, terminal link.
    push_bytes(descriptor, offset, &[
        14, 0x24, 0x01, 1, 77, 0, ENDPOINT_IN, 0, 2, 0, 0, 0, 1, 0,
    ]);

    // Uncompressed format descriptor (format index 1), followed by the NV12 GUID.
    push_bytes(descriptor, offset, &[
        27, 0x24, 0x04, 1, 1,
    ]);
    push_bytes(descriptor, offset, &NV12_GUID);
    // Remaining format fields: 12 bits per pixel, frame index 1, no aspect
    // ratio, no interlacing, and no copy protection.
    push_bytes(descriptor, offset, &[12, 1, 0, 0, 0, 0]);

    // Uncompressed frame descriptor (frame index 1).
    push_bytes(descriptor, offset, &[30, 0x24, 0x05, 1, 0]);

    // Frame fields: dimensions, bitrates, frame size, and interval.
    push_u16(descriptor, offset, WIDTH as u16);
    push_u16(descriptor, offset, HEIGHT as u16);
    push_u32(descriptor, offset, frame_bit_rate);
    push_u32(descriptor, offset, frame_bit_rate);
    push_u32(descriptor, offset, FRAME_BYTES);
    push_u32(descriptor, offset, frame_interval_100ns);
    push_u8(descriptor, offset, 1);
    push_u32(descriptor, offset, frame_interval_100ns);

    // Color matching, alternate setting 1, and high-bandwidth ISO IN endpoint.
    let encoded_max_packet =
        MAX_PACKET_SIZE | (((ISO_PACKETS_PER_MICROFRAME - 1) as u16) << 11);
    push_bytes(descriptor, offset, &[
        6, 0x24, 0x0d, 1, 1, 4,
        9, 0x04, 1, 1, 1, 0x0e, 0x02, 0, 4,
        7, 0x05, ENDPOINT_IN, 0x05,
    ]);
    push_u16(descriptor, offset, encoded_max_packet);
    push_u8(descriptor, offset, 1);
}

const fn build_cdc_configuration_descriptor() -> [u8; CDC_CONFIGURATION_DESCRIPTOR_LEN] {
    let mut descriptor = [0u8; CDC_CONFIGURATION_DESCRIPTOR_LEN];
    let mut offset = 0usize;

    // Configuration header: 4 interfaces, bus-powered, 100 mA.
    push_bytes(&mut descriptor, &mut offset, &[
        9, 0x02, 0xe0, 0x00, 4, 1, 0, 0x80, 50,
    ]);

    push_uvc_descriptors(
        &mut descriptor,
        &mut offset,
        FRAME_BYTES * 8,
        10_000_000,
    );

    // CDC ACM Interface Association Descriptor (IAD).
    // Groups control interface 2 with data interface 3.
    push_bytes(&mut descriptor, &mut offset, &[
        8, 0x0b, 2, 2, 0x02, 0x02, 0x01, 0,
    ]);

    // CDC ACM control interface 2: one interrupt notification endpoint.
    push_bytes(&mut descriptor, &mut offset, &[
        9, 0x04, 2, 0, 1, 0x02, 0x02, 0x01, 0,
    ]);

    // CDC Header Functional Descriptor: CDC specification version 1.10.
    push_bytes(&mut descriptor, &mut offset, &[
        5, 0x24, 0x00, 0x10, 0x01,
    ]);

    // CDC Call Management Functional Descriptor: data interface 3.
    push_bytes(&mut descriptor, &mut offset, &[
        5, 0x24, 0x01, 0x00, 3,
    ]);

    // CDC Abstract Control Management Functional Descriptor.
    push_bytes(&mut descriptor, &mut offset, &[
        4, 0x24, 0x02, 0x02,
    ]);

    // CDC Union Functional Descriptor: master 2 (control), slave 3 (data).
    push_bytes(&mut descriptor, &mut offset, &[
        5, 0x24, 0x06, 2, 3,
    ]);

    // Interrupt IN notification endpoint EP4; 8-byte packets, 16-ms interval.
    push_bytes(&mut descriptor, &mut offset, &[
        7, 0x05, 0x84, 0x03, 8, 0, 16,
    ]);

    // CDC data interface 3: bulk OUT and bulk IN endpoints.
    push_bytes(&mut descriptor, &mut offset, &[
        9, 0x04, 3, 0, 2, 0x0a, 0, 0, 0,
    ]);

    // Bulk OUT endpoint EP3; 512-byte maximum packet size at high speed.
    push_bytes(&mut descriptor, &mut offset, &[
        7, 0x05, 0x03, 0x02, 0x00, 0x02, 0,
    ]);

    // Bulk IN endpoint EP3; 512-byte maximum packet size at high speed.
    push_bytes(&mut descriptor, &mut offset, &[
        7, 0x05, 0x83, 0x02, 0x00, 0x02, 0,
    ]);

    if offset != CDC_CONFIGURATION_DESCRIPTOR_LEN {
        panic!("TinyUSB configuration descriptor length mismatch");
    }
    descriptor
}

const NORMAL_CONFIGURATION_DESCRIPTOR_LEN: usize = 290;

const fn build_normal_configuration_descriptor() -> [u8; NORMAL_CONFIGURATION_DESCRIPTOR_LEN] {
    let mut descriptor = [0u8; NORMAL_CONFIGURATION_DESCRIPTOR_LEN];
    let mut offset = 0usize;

    // Configuration header: 4 interfaces, bus-powered, 100 mA.
    push_bytes(&mut descriptor, &mut offset, &[
        9, 0x02, 0x22, 0x01, 4, 1, 0, 0x80, 50,
    ]);

    push_uvc_descriptors(
        &mut descriptor,
        &mut offset,
        FRAME_BIT_RATE,
        FRAME_INTERVAL_100NS,
    );

    // UAC2 IAD, AudioControl interface, and class-specific AC descriptors.
    // The fixed sample rate is reported by the Clock Source GET_CUR/GET_RANGE callback,
    // which reads the shared AUDIO_KHZ constant from the Rust application.
    // AudioControl function association: interfaces 2 (control) and 3 (streaming).
    push_bytes(&mut descriptor, &mut offset, &[
        8, 0x0b, 2, 2, 1, 1, 0x20, 0,
    ]);

    // AudioControl interface 2 and class-specific AC header (UAC2, category 7).
    push_bytes(&mut descriptor, &mut offset, &[
        9, 0x04, 2, 0, 0, 1, 1, 0x20, 0,
        9, 0x24, 1, 0, 2, 7, 60, 0, 0,
    ]);

    // Clock Source entity ID 1: internal fixed clock.
    push_bytes(&mut descriptor, &mut offset, &[
        8, 0x24, 0x0a, 1, 1, 5, 0, 0,
    ]);

    // Input Terminal ID 2: USB streaming input, linked to Clock Source 1.
    push_bytes(&mut descriptor, &mut offset, &[
        17, 0x24, 0x02, 2, 0x03, 0x06, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
    ]);

    // Feature Unit ID 3, sourced from Input Terminal 2.
    push_bytes(&mut descriptor, &mut offset, &[
        14, 0x24, 0x06, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ]);

    // Output Terminal ID 4: USB streaming output, sourced from Feature Unit 3.
    push_bytes(&mut descriptor, &mut offset, &[
        12, 0x24, 0x03, 4, 0x01, 0x01, 0, 3, 1, 0, 0, 0,
    ]);

    // AudioStreaming interface 3: alternate 0 is idle; alternate 1 carries audio.
    push_bytes(&mut descriptor, &mut offset, &[
        9, 0x04, 3, 0, 0, 1, 2, 0x20, 0,
        9, 0x04, 3, 1, 1, 1, 2, 0x20, 0,
    ]);

    // AS General descriptor: terminal link 4, PCM format, two-channel stream.
    push_bytes(&mut descriptor, &mut offset, &[
        16, 0x24, 1, 4, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0,
    ]);

    // Type I format: two-byte subslot, 16-bit PCM samples.
    push_bytes(&mut descriptor, &mut offset, &[
        6, 0x24, 2, 1, 2, 16,
    ]);

    // Isochronous OUT endpoint EP2 and its class-specific endpoint descriptor.
    push_bytes(&mut descriptor, &mut offset, &[
        7, 0x05, 0x82, 0x05, 10, 0, 1,
        8, 0x25, 1, 0, 0, 0, 0, 0,
    ]);

    if offset != NORMAL_CONFIGURATION_DESCRIPTOR_LEN {
        panic!("TinyUSB configuration descriptor length mismatch");
    }
    descriptor
}

static NORMAL_CONFIGURATION_DESCRIPTOR: [u8; NORMAL_CONFIGURATION_DESCRIPTOR_LEN] =
    build_normal_configuration_descriptor();

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

static mut CDC_CONFIGURATION_DESCRIPTOR: [u8; CDC_CONFIGURATION_DESCRIPTOR_LEN] =
    build_cdc_configuration_descriptor();

#[unsafe(no_mangle)]
pub extern "C" fn tud_descriptor_device_cb() -> *const u8 {
    DEVICE_DESCRIPTOR.as_ptr()
}

#[unsafe(no_mangle)]
pub extern "C" fn tud_descriptor_configuration_cb(index: u8) -> *const u8 {
    if index != 0 {
        return core::ptr::null();
    }

    // Preserve the original UVC+UAC2 descriptor byte-for-byte in normal mode.
    // CDC mode uses its separate UVC+CDC configuration.
    if !crate::usb_device::uvc::cdc_mode_enabled() {
        return NORMAL_CONFIGURATION_DESCRIPTOR.as_ptr();
    }

    // Update only the CDC-mode UVC frame interval and bitrate before enumeration.
    let interval = active_frame_interval_100ns();
    let bitrate = FRAME_BYTES * 8 * active_frame_rate_fps();
    let descriptor = core::ptr::addr_of_mut!(CDC_CONFIGURATION_DESCRIPTOR).cast::<u8>();
    unsafe {
        for (offset, value) in [
            // Uncompressed frame descriptor starts at byte 106:
            // dwMinBitRate=115, dwMaxBitRate=119,
            // dwDefaultFrameInterval=127, dwFrameInterval[0]=132.
            (115usize, bitrate as u8),
            (116, (bitrate >> 8) as u8),
            (117, (bitrate >> 16) as u8),
            (118, (bitrate >> 24) as u8),
            (119, bitrate as u8),
            (120, (bitrate >> 8) as u8),
            (121, (bitrate >> 16) as u8),
            (122, (bitrate >> 24) as u8),
            (127, interval as u8),
            (128, (interval >> 8) as u8),
            (129, (interval >> 16) as u8),
            (130, (interval >> 24) as u8),
            (132, interval as u8),
            (133, (interval >> 8) as u8),
            (134, (interval >> 16) as u8),
            (135, (interval >> 24) as u8),
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
