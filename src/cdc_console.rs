//! Minimal CDC command console for the boot-selected CDC device mode.
//!
//! This module is called only from the CDC-mode device loop. It does not
//! initialize or service the USB host, keyboard, audio, emulator, or storage.

use crate::tusb;
use core::ffi::c_void;

const LINE_CAPACITY: usize = 80;

static mut LINE: [u8; LINE_CAPACITY] = [0; LINE_CAPACITY];
static mut LINE_LEN: usize = 0;
static mut WAS_CONNECTED: bool = false;

const BANNER: &[u8] = b"\r\nXCore407I CDC console\r\nType HELP for commands.\r\n> ";
const HELP: &[u8] = b"\r\nCommands:\r\n  HELP     Show this help\r\n  STATUS   Show USB mode status\r\n  VERSION  Show firmware target\r\n> ";
const STATUS: &[u8] = b"\r\nUSB: High-Speed UVC camera + CDC serial\r\nAudio: disabled\r\nUSB host / keyboard: disabled\r\nEmulator: disabled\r\n> ";
const VERSION: &[u8] = b"\r\nXCore407I BK-0010 firmware\r\n> ";

#[inline]
unsafe fn write(bytes: &[u8]) {
    if bytes.is_empty() {
        return;
    }
    let _ = unsafe {
        tusb::tud_cdc_n_write(0, bytes.as_ptr().cast::<c_void>(), bytes.len() as u32)
    };
    let _ = unsafe { tusb::tud_cdc_n_write_flush(0) };
}

fn command_is(line: &[u8], command: &[u8]) -> bool {
    line.len() == command.len()
        && line.iter().zip(command).all(|(a, b)| a.to_ascii_uppercase() == *b)
}

/// Service CDC input and output from the CDC-mode foreground loop.
/// TinyUSB's device task must be serviced by the caller before this function.
pub fn service() {
    unsafe {
        let connected = tusb::tud_cdc_n_connected(0);
        if connected && !WAS_CONNECTED {
            LINE_LEN = 0;
            write(BANNER);
        }
        if !connected {
            LINE_LEN = 0;
            WAS_CONNECTED = false;
            return;
        }
        WAS_CONNECTED = true;

        let mut byte = 0u8;
        while tusb::tud_cdc_n_read(0, (&mut byte as *mut u8).cast::<c_void>(), 1) != 0 {
            match byte {
                b'\r' | b'\n' => {
                    write(b"\r\n");
                    let line = &LINE[..LINE_LEN];
                    if command_is(line, b"HELP") {
                        write(HELP);
                    } else if command_is(line, b"STATUS") {
                        write(STATUS);
                    } else if command_is(line, b"VERSION") {
                        write(VERSION);
                    } else if LINE_LEN == 0 {
                        write(b"> ");
                    } else {
                        write(b"\r\nUnknown command. Type HELP.\r\n> ");
                    }
                    LINE_LEN = 0;
                }
                0x08 | 0x7f => {
                    if LINE_LEN != 0 {
                        LINE_LEN -= 1;
                        write(b"\x08 \x08");
                    }
                }
                0x20..=0x7e => {
                    if LINE_LEN < LINE_CAPACITY {
                        LINE[LINE_LEN] = byte;
                        LINE_LEN += 1;
                        write(&[byte]);
                    } else {
                        write(b"\x07");
                    }
                }
                _ => {}
            }
        }
    }
}
