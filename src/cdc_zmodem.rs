//! ZMODEM receive integration for the CDC-only boot mode.
//!
//! The protocol engine is the no_std, heapless state machine from zmodem2.
//! Incoming file data is appended to LittleFS in bounded chunks.

use core::{ffi::c_void, mem::MaybeUninit, ptr::NonNull};

use littlefs2::{
    fs::Filesystem,
    io::{Error as LfsError, Write},
    path::Path,
};
use zmodem2::{Action, Error as ZmodemError, Event, Receiver};

use crate::{camera, nand_fs::ActiveStorage, tusb};

const NAME_CAPACITY: usize = 64;
const WIRE_CAPACITY: usize = 64;
const TRANSFER_TIMEOUT_MS: u32 = 60_000;
const TEMP_PATH: &[u8] = b"/.zmodem-upload\0";

pub struct State {
    receiver: Option<NonNull<Receiver>>,
    final_path: [u8; NAME_CAPACITY + 2],
    final_path_len: usize,
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
            final_path: [0; NAME_CAPACITY + 2],
            final_path_len: 0,
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

pub fn cancel(state: &mut State, fs: Option<&Filesystem<'_, ActiveStorage>>) {
    if let Some(fs) = fs {
        remove_partial_file(fs, state);
    }
    if state.receiver.take().is_some() {
        camera::release_cdc_receiver_storage();
    }
    state.final_path_len = 0;
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
    if state.final_path_len != 0 {
        if let Ok(path) = Path::from_bytes_with_nul(TEMP_PATH) {
            let _ = fs.remove(path);
        }
    }
    state.final_path_len = 0;
}

fn create_temp_file(fs: &Filesystem<'_, ActiveStorage>) -> bool {
    let Ok(path) = Path::from_bytes_with_nul(TEMP_PATH) else {
        return false;
    };
    // Never delete an existing hidden file: it may belong to the user.
    if fs.exists(path) {
        return false;
    }
    create_new_file(fs, TEMP_PATH)
}

fn commit_temp_file(fs: &Filesystem<'_, ActiveStorage>, state: &mut State) -> bool {
    if state.final_path_len == 0 {
        return true;
    }
    let Ok(temp) = Path::from_bytes_with_nul(TEMP_PATH) else {
        return false;
    };
    let Ok(final_path) = Path::from_bytes_with_nul(&state.final_path[..state.final_path_len]) else {
        return false;
    };
    if fs.exists(final_path) || fs.rename(temp, final_path).is_err() {
        let _ = fs.remove(temp);
        state.final_path_len = 0;
        return false;
    }
    state.final_path_len = 0;
    true
}

fn append_file(
    fs: &Filesystem<'_, ActiveStorage>,
    data: &[u8],
) -> bool {
    let Ok(path) = Path::from_bytes_with_nul(TEMP_PATH) else {
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
    let (storage, storage_len) = camera::claim_cdc_receiver_storage().ok_or(())?;
    if storage_len < core::mem::size_of::<Receiver>()
        || (storage as usize) % core::mem::align_of::<Receiver>() != 0
    {
        camera::release_cdc_receiver_storage();
        return Err(());
    }
    let storage = unsafe { &mut *storage.cast::<MaybeUninit<Receiver>>() };
    let receiver = match Receiver::new_in_place(storage) {
        Ok(receiver) => receiver,
        Err(_) => {
            camera::release_cdc_receiver_storage();
            return Err(());
        }
    };
    receiver.set_manual_file_accept(true);
    let receiver = NonNull::from(receiver);
    state.final_path_len = 0;
    state.wire_len = 0;
    state.wire_pos = 0;
    state.last_activity_ms = now_ms;
    state.transfer_failed = false;
    state.transfer_started = false;
    state.timeout_pending = false;
    state.receiver = Some(receiver);
    Ok(())
}

/// Preserve the ZMODEM startup header detected by the CDC console.
pub fn seed_input(state: &mut State, bytes: &[u8]) -> bool {
    if state.receiver.is_none() || bytes.len() > WIRE_CAPACITY || state.wire_len != 0 {
        return false;
    }
    state.wire[..bytes.len()].copy_from_slice(bytes);
    state.wire_len = bytes.len();
    state.wire_pos = 0;
    true
}

/// Advance the receiver state machine in the CDC foreground loop.
pub fn service(
    fs: &Filesystem<'_, ActiveStorage>,
    state: &mut State,
    now_ms: u32,
) {
    let Some(mut receiver_ptr) = state.receiver else {
        return;
    };
    let receiver = unsafe { receiver_ptr.as_mut() };
    let mut finished = false;

    // Bound work per iteration so UVC buffer production keeps running.
    for _ in 0..8 {
        // Check inactivity independently of the protocol action. The
        // receiver can remain in a non-Idle action while CDC is backpressured.
        if !state.timeout_pending
            && now_ms.wrapping_sub(state.last_activity_ms) >= TRANSFER_TIMEOUT_MS
        {
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
        }

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
                if state.final_path_len == 0
                    || !append_file(fs, bytes)
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
                let path_len = make_path(info.name, &mut state.final_path);
                let available = match path_len {
                    Some(len) => {
                        match Path::from_bytes_with_nul(&state.final_path[..len]) {
                            Ok(path) => !fs.exists(path) && create_temp_file(fs),
                            Err(_) => false,
                        }
                    }
                    None => false,
                };
                if available {
                    state.final_path_len = path_len.unwrap_or(0);
                    if receiver.accept_file_at(0).is_err() {
                        state.transfer_failed = true;
                        let _ = receiver.abort();
                        break;
                    }
                } else {
                    state.final_path_len = 0;
                    if receiver.skip_file().is_err() {
                        state.transfer_failed = true;
                        let _ = receiver.abort();
                        break;
                    }
                }
            }
            Action::Event(Event::FileCompleted) => {
                if !commit_temp_file(fs, state) {
                    state.transfer_failed = true;
                    let _ = receiver.abort();
                    break;
                }
            }
            Action::Event(Event::SessionCompleted) => {
                finished = true;
                remove_partial_file(fs, state);
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
                } else if state.transfer_failed {
                    super::cdc_console::write_status(
                        b"\r\nZMODEM stopped: storage/protocol error\r\n> ",
                    );
                } else if state.transfer_started {
                    super::cdc_console::write_status(b"\r\nZMODEM transfer cancelled\r\n> ");
                } else {
                    super::cdc_console::write_status(b"\r\nZMODEM session cancelled\r\n> ");
                }
                state.transfer_failed = false;
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
                        Err(error) => {
                            state.transfer_failed = true;
                            let message: &[u8] = match error {
                                ZmodemError::MalformedEncoding(_) => b"\r\nZMODEM input: malformed encoding\r\n",
                                ZmodemError::MalformedFileSize => b"\r\nZMODEM input: malformed file size\r\n",
                                ZmodemError::MalformedFileName => b"\r\nZMODEM input: malformed filename\r\n",
                                ZmodemError::MalformedFrame(_) => b"\r\nZMODEM input: malformed frame\r\n",
                                ZmodemError::MalformedHeader => b"\r\nZMODEM input: malformed header\r\n",
                                ZmodemError::MalformedPacket(_) => b"\r\nZMODEM input: malformed packet\r\n",
                                ZmodemError::InvalidState => b"\r\nZMODEM input: invalid state\r\n",
                                ZmodemError::Backpressure => b"\r\nZMODEM input: backpressure\r\n",
                                ZmodemError::OutOfMemory => b"\r\nZMODEM input: buffer exhausted\r\n",
                                ZmodemError::UnexpectedCrc16 => b"\r\nZMODEM input: CRC-16 mismatch\r\n",
                                ZmodemError::UnexpectedCrc32 => b"\r\nZMODEM input: CRC-32 mismatch\r\n",
                                ZmodemError::UnexpectedEof => b"\r\nZMODEM input: unexpected EOF\r\n",
                                ZmodemError::UnsupportedFeature => b"\r\nZMODEM input: unsupported feature\r\n",
                            };
                            super::cdc_console::write_status(message);
                            let _ = receiver.abort();
                            break;
                        }
                    }
                } else {
                    break;
                }
            }
            _ => break,
        }
    }

    if finished {
        // Do not let trailing ZMODEM bytes leak into the interactive console.
        state.wire_len = 0;
        state.wire_pos = 0;
        loop {
            let count = unsafe {
                tusb::tud_cdc_n_read(
                    0,
                    state.wire.as_mut_ptr().cast::<c_void>(),
                    WIRE_CAPACITY as u32,
                )
            } as usize;
            if count == 0 {
                break;
            }
        }
        state.receiver = None;
        state.final_path_len = 0;
        state.transfer_failed = false;
        state.transfer_started = false;
        state.timeout_pending = false;
        camera::release_cdc_receiver_storage();
    }
}
