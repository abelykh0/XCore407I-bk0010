//! USB HID boot-keyboard key translation and display labels.

pub(crate) fn append_keyboard_char(out: &mut [u8; 32], used: &mut usize, ch: u8) {
    if *used < out.len() {
        out[*used] = ch;
        *used += 1;
    }
}

pub(crate) fn append_keyboard_text(out: &mut [u8; 32], used: &mut usize, text: &[u8]) {
    for &ch in text {
        append_keyboard_char(out, used, ch);
    }
}

pub(crate) fn key_to_ascii(key: u8, shift: bool, caps: bool, num: bool, control_or_alt: bool) -> Option<u8> {
    // AZBK standard PC-to-BK special-key assignments.
    // Values are the BK control-key codes consumed by the emulator.
    match key {
        0x29 => return Some(0x1b), // Esc -> КТ
        0x49 => return Some(0x17), // Insert -> ВС
        0x4c => return Some(0x1d), // Delete -> ГРАФ
        0x3c => return Some(0x1e), // F3 -> УСТ
        0x4d => return Some(0x1f), // End -> ЗАП
        0x4a => return Some(0x14), // Home -> ТАБ
        0x3e => return Some(0x10), // F5 -> ИНД СУ
        0x3f => return Some(0x11), // F6 -> БЛОК РЕД
        0x40 => return Some(0x12), // F7 -> ШАГ
        0x41 => return Some(0x13), // F8 -> СБР
        0x4e => return Some(0x18), // Page Down -> СТИР
        // The keypad navigation usages act as arrows when Num Lock is off.
        0x4f => return Some(0x19), // Right
        0x50 => return Some(0x08), // Left
        0x51 => return Some(0x1b), // Down
        0x52 => return Some(0x1a), // Up
        0x5a if !num => return Some(0x1b), // Keypad Down
        0x5c if !num => return Some(0x08), // Keypad Left
        0x5e if !num => return Some(0x19), // Keypad Right
        0x60 if !num => return Some(0x1a), // Keypad Up
        _ => {}
    }

    if control_or_alt { return None; }

    match key {
        0x04..=0x1d => {
            let lower = b'a' + (key - 0x04);
            Some(if shift ^ caps { lower - 32 } else { lower })
        }
        0x1e..=0x27 => {
            const NORMAL: &[u8; 10] = b"1234567890";
            const SHIFTED: &[u8; 10] = b"!@#$%^&*()";
            let i = (key - 0x1e) as usize;
            Some(if shift { SHIFTED[i] } else { NORMAL[i] })
        }
        0x28 => Some(b'\n'), // BK OnKey maps Enter to 0x0A.
        0x2a => Some(0x18), // BK OnKey maps Backspace to 0x18.
        0x2c => Some(b' '),
        0x2d => Some(if shift { b'_' } else { b'-' }),
        0x2e => Some(if shift { b'+' } else { b'=' }),
        0x2f => Some(if shift { b'{' } else { b'[' }),
        0x30 => Some(if shift { b'}' } else { b']' }),
        0x31 => Some(if shift { b'|' } else { b'\\' }),
        0x33 => Some(if shift { b':' } else { b';' }),
        0x34 => Some(if shift { b'"' } else { b'\'' }),
        0x35 => Some(if shift { b'~' } else { b'`' }),
        0x36 => Some(if shift { b'<' } else { b',' }),
        0x37 => Some(if shift { b'>' } else { b'.' }),
        0x38 => Some(if shift { b'?' } else { b'/' }),
        0x54 => Some(b'/'),
        0x55 => Some(b'*'),
        0x56 => Some(b'-'),
        0x57 => Some(b'+'),
        0x58 => Some(b'\n'),
        0x59..=0x61 if num => Some(b'1' + (key - 0x59)),
        0x62 if num => Some(b'0'),
        0x63 if num => Some(b'.'),
        _ => None,
    }
}

pub(crate) fn special_key_label(key: u8, num: bool) -> Option<&'static [u8]> {
    match key {
        0x28 => Some(b"[ENTER]"),
        0x29 => Some(b"[ESC]"),
        0x2a => Some(b"[BACKSPACE]"),
        0x2b => Some(b"[TAB]"),
        0x39 => Some(b"[CAPS]"),
        0x46 => Some(b"[PRTSC]"),
        0x47 => Some(b"[SCROLL]"),
        0x48 => Some(b"[PAUSE]"),
        0x49 => Some(b"[INS]"),
        0x4a => Some(b"[HOME]"),
        0x4b => Some(b"[PGUP]"),
        0x4c => Some(b"[DEL]"),
        0x4d => Some(b"[END]"),
        0x4e => Some(b"[PGDN]"),
        0x4f => Some(b"[RIGHT]"),
        0x50 => Some(b"[LEFT]"),
        0x51 => Some(b"[DOWN]"),
        0x52 => Some(b"[UP]"),
        0x53 => Some(b"[NUM]"),
        0x59 if !num => Some(b"[KP-END]"),
        0x5a if !num => Some(b"[KP-DOWN]"),
        0x5b if !num => Some(b"[KP-PGDN]"),
        0x5c if !num => Some(b"[KP-LEFT]"),
        0x5d if !num => Some(b"[KP5]"),
        0x5e if !num => Some(b"[KP-RIGHT]"),
        0x5f if !num => Some(b"[KP-HOME]"),
        0x60 if !num => Some(b"[KP-UP]"),
        0x61 if !num => Some(b"[KP-PGUP]"),
        0x62 if !num => Some(b"[KP-INS]"),
        0x63 if !num => Some(b"[KP-DEL]"),
        0x3a => Some(b"[F1]"),
        0x3b => Some(b"[F2]"),
        0x3c => Some(b"[F3]"),
        0x3d => Some(b"[F4]"),
        0x3e => Some(b"[F5]"),
        0x3f => Some(b"[F6]"),
        0x40 => Some(b"[F7]"),
        0x41 => Some(b"[F8]"),
        0x42 => Some(b"[F9]"),
        0x43 => Some(b"[F10]"),
        0x44 => Some(b"[F11]"),
        0x45 => Some(b"[F12]"),
        0x65 => Some(b"[MENU]"),
        _ => None,
    }
}

// Keyboard input is paced in thread context alongside emulator execution.
