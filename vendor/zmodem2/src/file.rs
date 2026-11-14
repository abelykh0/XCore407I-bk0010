// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2017-2020 Alexey Arbuzov
// Copyright (c) 2023-2026 Jarkko Sakkinen

//! ZMODEM file metadata helpers.

use crate::buffer::Buffer;
use crate::error::Error;
use crate::header::{Encoding, Frame, Header};
use crate::io::Write;
use crate::wire::{SUBPACKET_MAX_SIZE, SubpacketType, write_subpacket};
use core::fmt::Write as _;

/// Parses a u32 from a slice of ASCII octal bytes; `None` when the field
/// is empty or malformed, as these fields are advisory.
pub(crate) fn parse_octal(bytes: &[u8]) -> Option<u32> {
    if bytes.is_empty() {
        return None;
    }
    let mut result: u32 = 0;
    for &byte in bytes {
        let digit = match byte {
            b'0'..=b'7' => u32::from(byte - b'0'),
            _ => return None,
        };
        result = result.checked_mul(8)?.checked_add(digit)?;
    }
    Some(result)
}

/// Parses a u32 from a slice of ASCII decimal bytes.
pub(crate) fn parse_file_size(bytes: &[u8]) -> Result<u32, Error> {
    if bytes.is_empty() {
        return Ok(0);
    }

    let mut result: u32 = 0;
    for &byte in bytes {
        let digit = match byte {
            b'0'..=b'9' => u32::from(byte - b'0'),
            _ => return Err(Error::MalformedFileSize),
        };
        result = result
            .checked_mul(10)
            .and_then(|r| r.checked_add(digit))
            .ok_or(Error::MalformedFileSize)?;
    }
    Ok(result)
}

/// Write ZRFILE
pub(crate) fn write_zfile<P>(
    port: &mut P,
    buf: &mut Buffer<SUBPACKET_MAX_SIZE>,
    name: &[u8],
    size: u32,
    modified: Option<u32>,
    mode: Option<u32>,
) -> Result<Option<()>, Error>
where
    P: Write + ?Sized,
{
    buf.clear();
    buf.extend_from_slice(name)
        .map_err(|_| Error::OutOfMemory)?;
    buf.push(b'\0').map_err(|_| Error::OutOfMemory)?;

    write!(buf, "{size}").map_err(|_| Error::OutOfMemory)?;
    // The fields after the length are optional but cannot be skipped: a
    // mode without a modification time needs a zero ("unknown") in its
    // place. Both are octal, as YMODEM batch has them.
    if modified.is_some() || mode.is_some() {
        let modified = modified.unwrap_or(0);
        write!(buf, " {modified:o}").map_err(|_| Error::OutOfMemory)?;
    }
    if let Some(mode) = mode {
        write!(buf, " {mode:o}").map_err(|_| Error::OutOfMemory)?;
    }
    buf.push(b'\0').map_err(|_| Error::OutOfMemory)?;

    if Header::new(Encoding::ZBIN32, Frame::ZFILE, [0; 4])
        .write(port)?
        .is_none()
    {
        return Ok(None);
    }
    write_subpacket(port, Encoding::ZBIN32, SubpacketType::ZCRCW, buf)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_octal_fields() {
        assert_eq!(parse_octal(b"100644"), Some(0o100_644));
        assert_eq!(parse_octal(b"0"), Some(0));
        assert_eq!(parse_octal(b""), None);
        assert_eq!(parse_octal(b"8"), None);
        assert_eq!(parse_octal(b"37777777777777"), None);
    }

    #[test]
    fn parse_empty_file_size_as_zero() {
        assert_eq!(parse_file_size(b""), Ok(0));
    }

    #[test]
    fn reject_bad_file_size() {
        assert_eq!(parse_file_size(b"12x"), Err(Error::MalformedFileSize));
    }
}
