//! ZMODEM receive integration for the CDC-only boot mode.
//!
//! The protocol engine is the no_std, heapless state machine from zmodem2.
//! Incoming file data is appended to LittleFS in bounded chunks.

use core::ffi::c_void;

use littlefs2::{
    fs::Filesystem,
    io::{Error as LfsError, Write},
    path::Path,
};
use zmodem2::{Action, Event, Receiver};

use crate::{nand_fs::ActiveStorage, tusb};

const NAME_CAPACITY: usize = 64;
const WIRE_CAPACITY: usize = 64;
const TRANSFER_TIMEOUT_MS: u32 = 60_000;

pub struct State {
    receiver: Option<Receiver>,
    path: [u8; NAME_CAPACITY + 2],
    path_len: usize,
    wire: [u8; WIRE_CAPACITY],
    wire_len: usize,
    wire_pos: usize,
    last_activity_ms: u32,
    transfer_failed: bool,
    transfer_started: bool,
    timeout_pending: bool,
}

impl State {
    pub const fn new() -> Self {
        Self {
            receiver: None,
            path: [0; NAME_CAPACITY + 2],
            path_len: 0,
            wire: [0; WIRE_CAPACITY],
            wire_len: 0,
            wire_pos: 0,
            last_activity_ms: 0,
            transfer_failed: false,
            transfer_started: false,
            timeout_pending: false,
        }
    }

    pub fn active(&self) -> bool {
        self.receiver.is_some()
    }
}

impl Default for State {
    fn default() -> Self {
        Self::new()
    }
}

pub fn cancel(state: &mut State) {
    state.receiver = None;
    state.path_len = 0;
    state.wire_len = 0;
    state.wire_pos = 0;
    state.transfer_started = false;
    state.timeout_pending = false;
}

fn make_path(name: &[u8], path: &mut [u8; NAME_CAPACITY + 2]) -> Option<usize> {
    if name.is_empty() || name.len() > NAME_CAPACITY || name == b"." || name == b".." {
        return None;
    }
    if !name.iter().all(|byte| {
        byte.is_ascii_alphanumeric()
            || matches!(*byte, b'.' | b'_' | b'-' | b'+' | b' ' | b'(' | b')')
    }) {
        return None;
    }

    path[0] = b'/';
    path[1..1 + name.len()].copy_from_slice(name);
    path[1 + name.len()] = 0;
    Some(name.len() + 2)
}

fn create_new_file(fs: &Filesystem<'_, ActiveStorage>, path_bytes: &[u8]) -> bool {
    let Ok(path) = Path::from_bytes_with_nul(path_bytes) else {
        return false;
    };
    fs.open_file_with_options_and_then(
        |options| options.write(true).create_new(true),
        path,
        |_file| Ok(()),
    )
    .is_ok()
}

fn remove_partial_file(fs: &Filesystem<'_, ActiveStorage>, state: &mut State) {
    if state.path_len == 0 {
        return;
    }
    if let Ok(path) = Path::from_bytes_with_nul(&state.path[..state.path_len]) {
        let _ = fs.remove(path);
    }
    state.path_len = 0;
}

fn append_file(
    fs: &Filesystem<'_, ActiveStorage>,
    path_bytes: &[u8],
    data: &[u8],
) -> bool {
    let Ok(path) = Path::from_bytes_with_nul(path_bytes) else {
        return false;
    };
    fs.open_file_with_options_and_then(
        |options| options.write(true).append(true),
        path,
        |file| {
            let mut written = 0usize;
            while written < data.len() {
                let count = file.write(&data[written..])?;
                if count == 0 {
                    return Err(LfsError::IO);
                }
                written += count;
            }
            file.flush()
        },
    )
    .is_ok()
}

pub fn start(state: &mut State, storage_available: bool, now_ms: u32) -> Result<(), ()> {
    if !storage_available || state.receiver.is_some() {
        return Err(());
    }
    let mut receiver = Receiver::new().map_err(|_| ())?;
    receiver.set_manual_file_accept(true);
    state.path_len = 0;
    state.wire_len = 0;
    state.wire_pos = 0;
    state.last_activity_ms = now_ms;
    state.transfer_failed = false;
    state.transfer_started = false;
    state.timeout_pending = false;
    state.receiver = Some(receiver);
    Ok(())
}

/// Advance the receiver state machine in the CDC foreground loop.
pub fn service(
    fs: &Filesystem<'_, ActiveStorage>,
    state: &mut State,
    now_ms: u32,
) {
    let Some(mut receiver) = state.receiver.take() else {
        return;
    };
    let mut finished = false;

    // Bound work per iteration so UVC buffer production keeps running.
    for _ in 0..8 {
        match receiver.poll() {
            Action::WriteWire(bytes) => {
                let bytes_len = bytes.len();
                let count = unsafe {
                    tusb::tud_cdc_n_write(
                        0,
                        bytes.as_ptr().cast::<c_void>(),
                        bytes.len() as u32,
                    )
                } as usize;
                if count == 0 {
                    break;
                }
                unsafe { let _ = tusb::tud_cdc_n_write_flush(0); }
                receiver.wire_written(count);
                state.last_activity_ms = now_ms;
                if count < bytes_len {
                    break;
                }
            }
            Action::WriteFile(bytes) => {
                let bytes_len = bytes.len();
                if state.path_len == 0
                    || !append_file(fs, &state.path[..state.path_len], bytes)
                {
                    state.transfer_failed = true;
                    let _ = receiver.abort();
                    break;
                }
                if receiver.file_written(bytes_len).is_err() {
                    state.transfer_failed = true;
                    let _ = receiver.abort();
                    break;
                }
                state.last_activity_ms = now_ms;
            }
            Action::Event(Event::FileStarted(info)) => {
                state.transfer_started = true;
                let path_len = make_path(info.name, &mut state.path);
                let created = match path_len {
                    Some(len) => create_new_file(fs, &state.path[..len]),
                    None => false,
                };
                if created {
                    state.path_len = path_len.unwrap_or(0);
                    if receiver.accept_file_at(0).is_err() {
                        state.transfer_failed = true;
                        let _ = receiver.abort();
                        break;
                    }
                } else {
                    state.path_len = 0;
                    if receiver.skip_file().is_err() {
                        state.transfer_failed = true;
                        let _ = receiver.abort();
                        break;
                    }
                }
            }
            Action::Event(Event::FileCompleted) => {
                state.path_len = 0;
            }
            Action::Event(Event::SessionCompleted) => {
                finished = true;
                if state.transfer_failed {
                    super::cdc_console::write_status(
                        b"\r\nZMODEM stopped: storage/protocol error\r\n> ",
                    );
                } else {
                    super::cdc_console::write_status(b"\r\nZMODEM upload complete\r\n> ");
                }
                break;
            }
            Action::Event(Event::Aborted) => {
                finished = true;
                remove_partial_file(fs, state);
                if state.timeout_pending {
                    super::cdc_console::write_status(
                        b"\r\nZMODEM timed out after 60 seconds of inactivity\r\n> ",
                    );
                } else if state.transfer_started {
                    super::cdc_console::write_status(b"\r\nZMODEM transfer cancelled\r\n> ");
                } else {
                    super::cdc_console::write_status(b"\r\nZMODEM session cancelled\r\n> ");
                }
                state.timeout_pending = false;
                state.transfer_started = false;
                break;
            }
            Action::Idle => {
                if state.wire_pos == state.wire_len {
                    state.wire_len = unsafe {
                        tusb::tud_cdc_n_read(
                            0,
                            state.wire.as_mut_ptr().cast::<c_void>(),
                            WIRE_CAPACITY as u32,
                        )
                    } as usize;
                    state.wire_pos = 0;
                    if state.wire_len != 0 {
                        state.last_activity_ms = now_ms;
                    }
                }

                if state.wire_pos < state.wire_len {
                    let input = &state.wire[state.wire_pos..state.wire_len];
                    match receiver.submit_wire(input) {
                        Ok(consumed) if consumed != 0 => state.wire_pos += consumed,
                        Ok(_) => break,
                        Err(_) => {
                            state.transfer_failed = true;
                            let _ = receiver.abort();
                            break;
                        }
                    }
                } else if now_ms.wrapping_sub(state.last_activity_ms) >= TRANSFER_TIMEOUT_MS {
                    state.timeout_pending = true;
                    if receiver.abort().is_err() {
                        state.transfer_failed = true;
                        finished = true;
                        remove_partial_file(fs, state);
                        super::cdc_console::write_status(
                            b"\r\nZMODEM timeout; unable to queue abort\r\n> ",
                        );
                        break;
                    }
                    state.last_activity_ms = now_ms;
                } else {
                    break;
                }
            }
            _ => break,
        }
    }

    if !finished {
        state.receiver = Some(receiver);
    }
}
