use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, Ordering};

use crate::file_browser::{
    FILE_BROWSER_ACTIVE, FILE_BROWSER_ACTION, FILE_BROWSER_OPEN, FILE_BROWSER_UP,
    FILE_BROWSER_DOWN, FILE_BROWSER_ENTER, FILE_BROWSER_ESCAPE,
    SAVE_DIALOG_IDLE, SAVE_DIALOG_INPUT, SAVE_DIALOG_CHECK, SAVE_DIALOG_CONFIRM,
    SAVE_DIALOG_WRITE, SAVE_RESULT_CANCELLED, SAVE_NAME_CAPACITY,
    SAVE_FILENAME, SAVE_FILENAME_LEN, SAVE_DIALOG_RESULT, SAVE_DIALOG_STATE,
    SAVE_DIALOG_DIRTY, SAVE_OVERWRITE,
    FORMAT_STATE, FORMAT_CONFIRM, FORMAT_CONFIRM_FINAL, FORMAT_REQUEST,
    FORMAT_CANCELLED,
    INIT_STATE, INIT_CONFIRM, INIT_CONFIRM_FINAL, INIT_REQUEST, INIT_CANCELLED,
};
use crate::usb_host::USB_FS_STATUS;
use crate::keyboard_map::{append_keyboard_char, append_keyboard_text, key_to_ascii, special_key_label};
use xcore407i_blinker::camera::request_camera_mode_toggle;
use xcore407i_blinker::tusb;

// Keyboard input is paced at most once per PDP frame.
static LAST_KEYBOARD_CYCLE: AtomicU32 = AtomicU32::new(0);
static KEYBOARD_CYCLE_INITIALIZED: AtomicBool = AtomicBool::new(false);

static mut PREVIOUS_KEYS: [u8; 6] = [0; 6];
static mut PREVIOUS_MODIFIERS: u8 = 0;
static KEYBOARD_LOCKS: AtomicU8 = AtomicU8::new(0);
static KEYBOARD_DEVICE: AtomicU8 = AtomicU8::new(0);
static KEYBOARD_INSTANCE: AtomicU8 = AtomicU8::new(0);
static mut KEYBOARD_LED_REPORT: u8 = 0;
const KEYBOARD_QUEUE_CAPACITY: u16 = 128;
const KEYBOARD_QUEUE_MASK: u16 = KEYBOARD_QUEUE_CAPACITY - 1;
static mut KEYBOARD_QUEUE: [u8; KEYBOARD_QUEUE_CAPACITY as usize] = [0; KEYBOARD_QUEUE_CAPACITY as usize];
static KEYBOARD_QUEUE_HEAD: AtomicU16 = AtomicU16::new(0);
static KEYBOARD_QUEUE_TAIL: AtomicU16 = AtomicU16::new(0);
const PDP_CYCLES_PER_FRAME: u32 = 60_000;

unsafe extern "C" {
    fn bk_timing_cycles() -> u32;
    fn bk_try_send_input_byte(value: u8) -> u8;
    fn bk_request_stop_irq();
    static mut RamBuffer: [u8; 0x4000];
}

fn keyboard_queue_push(bytes: &[u8]) {
    for &byte in bytes {
        let head = KEYBOARD_QUEUE_HEAD.load(Ordering::Relaxed);
        let tail = KEYBOARD_QUEUE_TAIL.load(Ordering::Acquire);
        if head.wrapping_sub(tail) >= KEYBOARD_QUEUE_CAPACITY { break; }
        unsafe { core::ptr::addr_of_mut!(KEYBOARD_QUEUE).cast::<u8>()
            .add((head & KEYBOARD_QUEUE_MASK) as usize).write(byte); }
        KEYBOARD_QUEUE_HEAD.store(head.wrapping_add(1), Ordering::Release);
    }
}
fn keyboard_queue_peek() -> Option<u8> {
    let tail = KEYBOARD_QUEUE_TAIL.load(Ordering::Relaxed);
    if tail == KEYBOARD_QUEUE_HEAD.load(Ordering::Acquire) { return None; }
    Some(unsafe { core::ptr::addr_of!(KEYBOARD_QUEUE).cast::<u8>()
        .add((tail & KEYBOARD_QUEUE_MASK) as usize).read() })
}
fn keyboard_queue_pop() {
    let tail = KEYBOARD_QUEUE_TAIL.load(Ordering::Relaxed);
    KEYBOARD_QUEUE_TAIL.store(tail.wrapping_add(1), Ordering::Release);
}
pub(crate) fn keyboard_queue_discard() {
    let head = KEYBOARD_QUEUE_HEAD.load(Ordering::Acquire);
    KEYBOARD_QUEUE_TAIL.store(head, Ordering::Release);
}

#[unsafe(no_mangle)]
pub extern "C" fn tuh_hid_mount_cb(dev_addr: u8, instance: u8, _desc: *const u8, _len: u16) {
    // HID interface protocol 1 is a boot keyboard; ignore unrelated HID
    // interfaces rather than decoding a mouse/gamepad report as keystrokes.
    if unsafe { tusb::tuh_hid_interface_protocol(dev_addr, instance) } != 1 {
        return;
    }
    KEYBOARD_DEVICE.store(dev_addr, Ordering::Release);
    KEYBOARD_INSTANCE.store(instance, Ordering::Release);
    KEYBOARD_LOCKS.store(0, Ordering::Release);
    keyboard_queue_discard();
    unsafe {
        PREVIOUS_KEYS = [0; 6];
        PREVIOUS_MODIFIERS = 0;
        KEYBOARD_LED_REPORT = 0;
    }
    USB_FS_STATUS.store(4, Ordering::Release);

    // TinyUSB normally selects boot protocol for boot keyboards during
    // enumeration (CFG_TUH_HID_SET_PROTOCOL_ON_ENUM). Verify that before
    // interpreting reports as the standard 8-byte boot-keyboard packet.
    if unsafe { tusb::tuh_hid_get_protocol(dev_addr, instance) } != 0 {
        // Request boot protocol; the completion callback starts reception.
        if !unsafe { tusb::tuh_hid_set_protocol(dev_addr, instance, 0) } {
            USB_FS_STATUS.store(9, Ordering::Release);
        }
    } else {
        unsafe { tusb::tuh_hid_receive_report(dev_addr, instance); }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn tuh_hid_umount_cb(dev_addr: u8, instance: u8) {
    if KEYBOARD_DEVICE.load(Ordering::Acquire) == dev_addr
        && KEYBOARD_INSTANCE.load(Ordering::Acquire) == instance
    {
        KEYBOARD_DEVICE.store(0, Ordering::Release);
        KEYBOARD_INSTANCE.store(0, Ordering::Release);
        KEYBOARD_LOCKS.store(0, Ordering::Release);
        keyboard_queue_discard();
        unsafe {
            PREVIOUS_KEYS = [0; 6];
            PREVIOUS_MODIFIERS = 0;
        }
    }
    USB_FS_STATUS.store(5, Ordering::Release);
}

#[unsafe(no_mangle)]
pub extern "C" fn tuh_hid_set_protocol_complete_cb(dev_addr: u8, instance: u8, protocol: u8) {
    // Ignore late control-transfer completions from a keyboard that has
    // already disconnected or been replaced by another HID interface.
    if KEYBOARD_DEVICE.load(Ordering::Acquire) != dev_addr
        || KEYBOARD_INSTANCE.load(Ordering::Acquire) != instance
    {
        return;
    }

    if protocol == 0 {
        unsafe { tusb::tuh_hid_receive_report(dev_addr, instance); }
    } else {
        USB_FS_STATUS.store(9, Ordering::Release);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn tuh_hid_report_received_cb(
    dev_addr: u8,
    instance: u8,
    report: *const u8,
    len: u16,
) {
    if KEYBOARD_DEVICE.load(Ordering::Acquire) != dev_addr
        || KEYBOARD_INSTANCE.load(Ordering::Acquire) != instance
    {
        return;
    }

    if len >= 8 && !report.is_null() {
        let bytes = unsafe { core::slice::from_raw_parts(report, len as usize) };
        process_boot_keyboard_report(dev_addr, instance, bytes);
    }
    // Re-arm only the still-active keyboard. A disconnect/reconnect can
    // reuse device addresses, so stale callbacks must not target a new HID.
    if KEYBOARD_DEVICE.load(Ordering::Acquire) == dev_addr
        && KEYBOARD_INSTANCE.load(Ordering::Acquire) == instance
    {
        unsafe { tusb::tuh_hid_receive_report(dev_addr, instance); }
    }
}

/// Process an 8-byte USB HID boot-keyboard report:
/// [modifier, reserved, key1..key6].
fn process_boot_keyboard_report(dev_addr: u8, instance: u8, report: &[u8]) {
    if KEYBOARD_DEVICE.load(Ordering::Acquire) != dev_addr
        || KEYBOARD_INSTANCE.load(Ordering::Acquire) != instance
    {
        return;
    }

    let modifiers = report[0];
    let shift = modifiers & 0x22 != 0;
    let control = modifiers & 0x11 != 0;
    let gui = modifiers & 0x88 != 0;
    let control_or_alt = modifiers & 0xdd != 0;
    let previous_modifiers = unsafe { PREVIOUS_MODIFIERS };
    let control_pressed = control && previous_modifiers & 0x11 == 0;
    let gui_pressed = gui && previous_modifiers & 0x88 == 0;
    let locks_before = KEYBOARD_LOCKS.load(Ordering::Acquire);
    let mut locks = locks_before;
    let mut text = [b' '; 32];
    let mut used = 0usize;
    let mut emulator_input = [0u8; 32];
    let mut emulator_used = 0usize;

    let previous = unsafe { PREVIOUS_KEYS };
    let mut current = [0u8; 6];

    // AZBK modifier keys are only forwarded while the emulator owns input.
    let keyboard_captured = SAVE_DIALOG_STATE.load(Ordering::Acquire) != SAVE_DIALOG_IDLE
        || FORMAT_STATE.load(Ordering::Acquire) != 0
        || INIT_STATE.load(Ordering::Acquire) != 0
        || FILE_BROWSER_ACTIVE.load(Ordering::Acquire)
;
    if !keyboard_captured && control_pressed && modifiers & 0x44 == 0 {
        append_keyboard_char(&mut emulator_input, &mut emulator_used, 0x1c); // СУ
    }
    // АР2 is a modifier (Alt/AltGr), not a standalone BK key code.
    if !keyboard_captured && gui_pressed {
        // A single Win key toggles the language. Read the live BK flag so
        // changes made by the emulator itself are respected:
        // bit 7 set = РУС, clear = ЛАТ.
        let language_flag = unsafe { RamBuffer[0x23] };
        let language_key = if language_flag & 0x80 != 0 {
            0x0f // Currently РУС: request ЛАТ.
        } else {
            0x0e // Currently ЛАТ: request РУС.
        };
        append_keyboard_char(&mut emulator_input, &mut emulator_used, language_key);
    }
    current.copy_from_slice(&report[2..8]);

    for &key in current.iter() {
        if key == 0 || previous.contains(&key) {
            continue;
        }

        // While the save dialog is active, capture filename editing and
        // confirmation keys instead of forwarding them to the BK emulator.
        let save_state = SAVE_DIALOG_STATE.load(Ordering::Acquire);
        if save_state == SAVE_DIALOG_INPUT {
            match key {
                0x28 => {
                    if unsafe { SAVE_FILENAME_LEN } != 0 {
                        SAVE_DIALOG_STATE.store(SAVE_DIALOG_CHECK, Ordering::Release);
                    }
                }
                0x29 => {
                    SAVE_DIALOG_STATE.store(SAVE_DIALOG_IDLE, Ordering::Release);
                    SAVE_DIALOG_RESULT.store(SAVE_RESULT_CANCELLED, Ordering::Release);
                }
                0x2a => unsafe {
                    if SAVE_FILENAME_LEN != 0 {
                        SAVE_FILENAME_LEN -= 1;
                        core::ptr::addr_of_mut!(SAVE_FILENAME)
                            .cast::<u8>()
                            .add(SAVE_FILENAME_LEN)
                            .write(0);
                        SAVE_DIALOG_DIRTY.store(true, Ordering::Release);
                    }
                },
                _ => {
                    if let Some(ch) = key_to_ascii(key, shift, false, false, control_or_alt) {
                        if ch.is_ascii_alphanumeric() || ch == b'_' || ch == b'-' || ch == b' ' {
                            unsafe {
                                if SAVE_FILENAME_LEN < SAVE_NAME_CAPACITY {
                                    core::ptr::addr_of_mut!(SAVE_FILENAME)
                                        .cast::<u8>()
                                        .add(SAVE_FILENAME_LEN)
                                        .write(ch);
                                    SAVE_FILENAME_LEN += 1;
                                    SAVE_DIALOG_DIRTY.store(true, Ordering::Release);
                                }
                            }
                        }
                    }
                }
            }
            continue;
        } else if save_state == SAVE_DIALOG_CONFIRM {
            match key {
                0x29 => {
                    SAVE_DIALOG_STATE.store(SAVE_DIALOG_IDLE, Ordering::Release);
                    SAVE_DIALOG_RESULT.store(SAVE_RESULT_CANCELLED, Ordering::Release);
                }
                _ => match key_to_ascii(key, shift, false, false, control_or_alt) {
                    Some(b'y') | Some(b'Y') => {
                        SAVE_OVERWRITE.store(true, Ordering::Release);
                        SAVE_DIALOG_STATE.store(SAVE_DIALOG_WRITE, Ordering::Release);
                    }
                    Some(b'n') | Some(b'N') => {
                        SAVE_DIALOG_STATE.store(SAVE_DIALOG_IDLE, Ordering::Release);
                        SAVE_DIALOG_RESULT.store(SAVE_RESULT_CANCELLED, Ordering::Release);
                    }
                    _ => {}
                }
            }
            continue;
        } else if save_state != SAVE_DIALOG_IDLE {
            continue;
        }

        // LittleFS formatting requires two explicit confirmations.
        // The actual flash operation runs later in main-loop context.
        let format_state = FORMAT_STATE.load(Ordering::Acquire);
        if format_state == FORMAT_CONFIRM || format_state == FORMAT_CONFIRM_FINAL {
            match key {
                0x29 => FORMAT_STATE.store(FORMAT_CANCELLED, Ordering::Release),
                _ => match key_to_ascii(key, shift, false, false, control_or_alt) {
                    Some(b'y') | Some(b'Y') if format_state == FORMAT_CONFIRM => {
                        FORMAT_STATE.store(FORMAT_CONFIRM_FINAL, Ordering::Release);
                    }
                    Some(b'y') | Some(b'Y') if format_state == FORMAT_CONFIRM_FINAL => {
                        FORMAT_STATE.store(FORMAT_REQUEST, Ordering::Release);
                    }
                    Some(b'n') | Some(b'N') => {
                        FORMAT_STATE.store(FORMAT_CANCELLED, Ordering::Release);
                    }
                    _ => {}
                }
            }
            continue;
        }

        // First-time NAND provisioning is deliberately destructive and
        // requires two confirmations. The erase/provision operation runs in main().
        let init_state = INIT_STATE.load(Ordering::Acquire);
        if init_state == INIT_CONFIRM || init_state == INIT_CONFIRM_FINAL {
            match key {
                0x29 => INIT_STATE.store(INIT_CANCELLED, Ordering::Release),
                _ => match key_to_ascii(key, shift, false, false, control_or_alt) {
                    Some(b'y') | Some(b'Y') if init_state == INIT_CONFIRM => {
                        INIT_STATE.store(INIT_CONFIRM_FINAL, Ordering::Release);
                    }
                    Some(b'y') | Some(b'Y') if init_state == INIT_CONFIRM_FINAL => {
                        INIT_STATE.store(INIT_REQUEST, Ordering::Release);
                    }
                    Some(b'n') | Some(b'N') => {
                        INIT_STATE.store(INIT_CANCELLED, Ordering::Release);
                    }
                    _ => {}
                }
            }
            continue;
        }

        // Ctrl+Alt+F12 opens the explicit NAND reinitialization flow.
        // This is intentionally available even when a valid Dhara volume
        // already exists; the two confirmation screens guard the erase.
        if key == 0x45 && modifiers & 0x55 == 0x55 {
            if INIT_STATE.load(Ordering::Acquire) == 0 {
                keyboard_queue_discard();
                INIT_STATE.store(INIT_CONFIRM, Ordering::Release);
            }
            continue;
        }

        // STOP is a hardware IRQ1, not a character sent through 177662.
        // Pause remains available as the host key for the BK STOP signal.
        if key == 0x48 {
            unsafe { bk_request_stop_irq(); }
            continue;
        }

        // F1 requests a camera mode change. The camera applies it at the
        // next UVC frame boundary and clears the canvas in the new format.
        if key == 0x3a {
            request_camera_mode_toggle();
            continue;
        }

        // F2 opens the NAND file browser; directory access runs in main().
        if key == 0x3b {
            FILE_BROWSER_ACTIVE.store(true, Ordering::Release);
            FILE_BROWSER_ACTION.store(FILE_BROWSER_OPEN, Ordering::Release);
            continue;
        }

        // While the browser is active, consume navigation/selection keys and
        // prevent them from reaching the BK emulator.
        if FILE_BROWSER_ACTIVE.load(Ordering::Acquire) {
            let action = match key {
                0x52 => FILE_BROWSER_UP,    // Up Arrow
                0x51 => FILE_BROWSER_DOWN,  // Down Arrow
                0x28 => FILE_BROWSER_ENTER, // Enter
                0x29 => FILE_BROWSER_ESCAPE,// Escape
                _ => 0,
            };
            if action != 0 {
                FILE_BROWSER_ACTION.store(action, Ordering::Release);
            }
            continue;
        }

        // F3 starts a RAM save; the extension is appended automatically.
        if key == 0x3c {
            keyboard_queue_discard();
            unsafe {
                core::ptr::write_bytes(
                    core::ptr::addr_of_mut!(SAVE_FILENAME).cast::<u8>(),
                    0,
                    SAVE_NAME_CAPACITY,
                );
                SAVE_FILENAME_LEN = 0;
            }
            SAVE_DIALOG_RESULT.store(0, Ordering::Release);
            SAVE_DIALOG_STATE.store(SAVE_DIALOG_INPUT, Ordering::Release);
            SAVE_DIALOG_DIRTY.store(true, Ordering::Release);
            continue;
        }

        // HID keyboard usages for the three toggle keys.
        match key {
            0x39 => locks ^= 0x02, // Caps Lock
            0x53 => locks ^= 0x01, // Num Lock
            0x47 => locks ^= 0x04, // Scroll Lock
            _ => {}
        }

        if let Some(ch) = key_to_ascii(key, shift, locks & 0x02 != 0, locks & 0x01 != 0, control_or_alt) {
            append_keyboard_char(&mut text, &mut used, ch);
            append_keyboard_char(&mut emulator_input, &mut emulator_used, ch);
        } else if let Some(label) = special_key_label(key, locks & 0x01 != 0) {
            if used != 0 { append_keyboard_char(&mut text, &mut used, b' '); }
            append_keyboard_text(&mut text, &mut used, label);
        }
    }

    unsafe {
        PREVIOUS_KEYS = current;
        PREVIOUS_MODIFIERS = modifiers;
    }

    if locks != locks_before {
        KEYBOARD_LOCKS.store(locks, Ordering::Release);
        // HID LED output bits: Num=bit0, Caps=bit1, Scroll=bit2.
        // SET_REPORT is asynchronous; the one-byte buffer is static and
        // remains valid until TinyUSB completes the control transfer.
        unsafe {
            KEYBOARD_LED_REPORT = locks & 0x07;
            let led_report = core::ptr::addr_of_mut!(KEYBOARD_LED_REPORT);
            let _ = tusb::tuh_hid_set_report(
                dev_addr, instance, 0, 2, led_report.cast(), 1,
            );
        }

        /*
        let event: &[u8] = match locks & 0x07 {
            0 => b"USB KEYBOARD: LOCKS OFF",
            1 => b"USB KEYBOARD: NUM LOCK",
            2 => b"USB KEYBOARD: CAPS LOCK",
            3 => b"USB KEYBOARD: CAPS + NUM LOCK",
            4 => b"USB KEYBOARD: SCROLL LOCK",
            5 => b"USB KEYBOARD: NUM + SCROLL LOCK",
            6 => b"USB KEYBOARD: CAPS + SCROLL LOCK",
            _ => b"USB KEYBOARD: ALL LOCKS",
        };
        show_usb_status(event);
        */
    }

    // Only decoded key bytes go to the emulator; display labels are not BK input.
    if emulator_used != 0 {
        keyboard_queue_push(&emulator_input[..emulator_used]);
    }
}

pub(crate) fn service_keyboard_frame_pacing() {
    let now = unsafe { bk_timing_cycles() };
    if !KEYBOARD_CYCLE_INITIALIZED.load(Ordering::Relaxed) {
        LAST_KEYBOARD_CYCLE.store(now, Ordering::Relaxed);
        KEYBOARD_CYCLE_INITIALIZED.store(true, Ordering::Relaxed);
        return;
    }

    let last = LAST_KEYBOARD_CYCLE.load(Ordering::Relaxed);
    let elapsed = now.wrapping_sub(last);
    if (elapsed as i32) < 0 {
        // The emulator cycle counter was reset; rebase the frame interval.
        LAST_KEYBOARD_CYCLE.store(now, Ordering::Relaxed);
    } else if elapsed >= PDP_CYCLES_PER_FRAME {
        if let Some(key) = keyboard_queue_peek() {
            // Leave the byte at the queue head if the BK keyboard port is busy.
            if unsafe { bk_try_send_input_byte(key) } != 0 {
                keyboard_queue_pop();
            }
        }
        // At most one byte per 60,000 PDP cycles, including after delays.
        LAST_KEYBOARD_CYCLE.store(now, Ordering::Relaxed);
    }
}
