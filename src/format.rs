/// Format a u32 as decimal ASCII without allocation.
///
/// The returned slice borrows the supplied buffer and contains only the
/// digits, with no sign or terminator.
pub fn u32_to_decimal(mut value: u32, out: &mut [u8; 10]) -> &[u8] {
    let mut pos = out.len();
    if value == 0 {
        out[9] = b'0';
        return &out[9..];
    }
    while value != 0 && pos != 0 {
        pos -= 1;
        out[pos] = b'0' + (value % 10) as u8;
        value /= 10;
    }
    &out[pos..]
}
