use core::ffi::c_void;

use crate::tusb;

pub(super) fn write_hex_u32(value: u32) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let mut digits = [b'0'; 8];
    for (index, digit) in digits.iter_mut().enumerate() {
        let shift = (7 - index) * 4;
        *digit = HEX[((value >> shift) & 0x0F) as usize];
    }
    write_status(&digits);
}

/// Format a 32-bit decimal value into the supplied scratch buffer.
/// The returned slice contains either plain digits or comma-grouped digits.
fn format_decimal<'a>(mut value: usize, grouped: bool, digits: &'a mut [u8; 13]) -> &'a [u8] {
    let mut start = digits.len();
    let mut group_digits = 0usize;

    loop {
        start -= 1;
        digits[start] = b'0' + (value % 10) as u8;
        value /= 10;
        group_digits += 1;

        if value == 0 {
            break;
        }
        if grouped && group_digits == 3 {
            start -= 1;
            digits[start] = b',';
            group_digits = 0;
        }
    }

    &digits[start..]
}

pub(super) fn write_decimal(value: usize) {
    let mut digits = [0u8; 13];
    write_status(format_decimal(value, false, &mut digits));
}

pub(super) fn write_decimal_grouped(value: usize) {
    let mut digits = [0u8; 13];
    write_status(format_decimal(value, true, &mut digits));
}

fn write_transfer_duration(seconds: u32) {
    let minutes = seconds / 60;
    let seconds = seconds % 60;
    if minutes < 10 {
        write_status(b"0");
    }
    write_decimal(minutes as usize);
    write_status(b":");
    if seconds < 10 {
        write_status(b"0");
    }
    write_decimal(seconds as usize);
}

pub(crate) fn write_send_complete(name: &[u8], size: u32, elapsed_seconds: u32) {
    write_status(b"\r\nSent file ");
    write_status(name);
    write_status(b" size ");
    write_decimal_grouped(size as usize);
    write_status(b" bytes in ");
    write_transfer_duration(elapsed_seconds);
    write_status(b".\r\n> ");
}

fn append_receive_bytes(buffer: &mut [u8], len: &mut usize, bytes: &[u8]) {
    let end = (*len + bytes.len()).min(buffer.len());
    let count = end - *len;
    buffer[*len..end].copy_from_slice(&bytes[..count]);
    *len = end;
}

fn append_receive_decimal(buffer: &mut [u8], len: &mut usize, value: usize) {
    let mut digits = [0u8; 13];
    append_receive_bytes(buffer, len, format_decimal(value, false, &mut digits));
}

fn append_receive_grouped(buffer: &mut [u8], len: &mut usize, value: usize) {
    let mut digits = [0u8; 13];
    append_receive_bytes(buffer, len, format_decimal(value, true, &mut digits));
}

/// Queue one complete receive notification so the caller can retain it if
/// the CDC TX FIFO does not accept the entire message.
pub(crate) fn write_receive_complete(name: &[u8], size: u32, elapsed_seconds: u32) -> bool {
    let mut buffer = [0u8; 160];
    let mut len = 0usize;
    append_receive_bytes(&mut buffer, &mut len, b"\r\nReceived file ");
    append_receive_bytes(&mut buffer, &mut len, name);
    append_receive_bytes(&mut buffer, &mut len, b" size ");
    append_receive_grouped(&mut buffer, &mut len, size as usize);
    append_receive_bytes(&mut buffer, &mut len, b" bytes in ");
    let minutes = elapsed_seconds / 60;
    let seconds = elapsed_seconds % 60;
    if minutes < 10 {
        append_receive_bytes(&mut buffer, &mut len, b"0");
    }
    append_receive_decimal(&mut buffer, &mut len, minutes as usize);
    append_receive_bytes(&mut buffer, &mut len, b":");
    if seconds < 10 {
        append_receive_bytes(&mut buffer, &mut len, b"0");
    }
    append_receive_decimal(&mut buffer, &mut len, seconds as usize);
    append_receive_bytes(&mut buffer, &mut len, b".\r\n> ");
    write_status_counted(&buffer[..len]) == len
}

/// Queue one summary for a multi-file ZMODEM receive session.
pub(crate) fn write_receive_summary(file_count: u32, total_size: u32) -> bool {
    let mut buffer = [0u8; 96];
    let mut len = 0usize;
    append_receive_bytes(&mut buffer, &mut len, b"\r\nReceived ");
    append_receive_grouped(&mut buffer, &mut len, file_count as usize);
    append_receive_bytes(&mut buffer, &mut len, b" files, total size ");
    append_receive_grouped(&mut buffer, &mut len, total_size as usize);
    append_receive_bytes(&mut buffer, &mut len, b" bytes.\r\n> ");
    write_status_counted(&buffer[..len]) == len
}

pub(crate) fn write_status(bytes: &[u8]) {
    let _ = write_status_counted(bytes);
}

/// Queue bytes through TinyUSB and return the number accepted into its TX FIFO.
pub(super) fn write_status_counted(bytes: &[u8]) -> usize {
    if bytes.is_empty() {
        return 0;
    }
    #[cfg(feature = "list-profile")]
    let profile_started = if crate::usb_device::cdc::console::list_diag::is_active() {
        Some(crate::usb_device::cdc::console::list_diag::cycle_count())
    } else {
        None
    };

    // TinyUSB's CDC write may accept fewer bytes than requested when its TX
    // FIFO is full. Do not silently discard the tail of diagnostic reports:
    // flush and service the device task until all bytes are queued.
    let mut offset = 0usize;
    let mut stalled = 0u32;
    while offset < bytes.len() && stalled < 4096 {
        let written = unsafe {
            tusb::tud_cdc_n_write(
                0,
                bytes[offset..].as_ptr().cast::<c_void>(),
                (bytes.len() - offset) as u32,
            )
        } as usize;
        offset += written;
        let _ = unsafe { tusb::tud_cdc_n_write_flush(0) };

        if written == 0 {
            stalled += 1;
            unsafe {
                let _ = tusb::tud_task_ext(1, false);
            }
            core::hint::spin_loop();
        } else {
            stalled = 0;
        }
    }
    let _ = unsafe { tusb::tud_cdc_n_write_flush(0) };
    #[cfg(feature = "list-profile")]
    if let Some(started) = profile_started {
        crate::usb_device::cdc::console::list_diag::record_cdc_write(
            crate::usb_device::cdc::console::list_diag::cycle_count().wrapping_sub(started),
            offset,
        );
    }
    offset
}
