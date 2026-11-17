//! ZMODEM receive integration for the CDC-only boot mode.
//!
//! The protocol engine is the no_std, heapless state machine from zmodem2.
//! Incoming file data is appended to LittleFS in bounded chunks.

use core::{ffi::c_void, mem::MaybeUninit, ptr::NonNull};

use littlefs2::{
    fs::Filesystem,
    fs::FileAllocation,
    io::{Error as LfsError, SeekFrom, Write},
    path::Path,
};
use zmodem2::{Action, Error as ZmodemError, Event, FileInfo, Position, Receiver, Sender};

use crate::{camera, nand_fs::ActiveStorage, tusb};

const NAME_CAPACITY: usize = 64;
const WIRE_CAPACITY: usize = 64;
const RECEIVER_STORAGE_CAPACITY: usize = 4096;
const _: () = assert!(core::mem::size_of::<Receiver>() <= RECEIVER_STORAGE_CAPACITY);
const TRANSFER_TIMEOUT_MS: u32 = 60_000;
const TEMP_PATH: &[u8] = b"/.zmodem-upload\0";

#[inline]
fn rtc_now_millis() -> u32 {
    let rtc = unsafe { &*stm32f4::stm32f407::RTC::ptr() };
    crate::rtc::unix_time_millis(rtc)
}

pub struct State {
    receiver: Option<NonNull<Receiver>>,
    sender: Option<Sender>,
    send_path: [u8; NAME_CAPACITY + 2],
    send_path_len: usize,
    send_size: u32,
    send_started_rtc_ms: u32,
    send_buffer: [u8; 1024],
    send_wire: [u8; WIRE_CAPACITY],
    send_wire_len: usize,
    send_wire_pos: usize,
    send_last_activity_ms: u32,
    send_last_handshake_ms: u32,
    file_allocation: FileAllocation<ActiveStorage>,
    final_path: [u8; NAME_CAPACITY + 2],
    final_path_len: usize,
    receive_name: [u8; NAME_CAPACITY],
    receive_name_len: usize,
    receive_size: u32,
    receive_file_committed: bool,
    receive_started_rtc_ms: u32,
    receive_elapsed_seconds: u32,
    wire: [u8; WIRE_CAPACITY],
    wire_len: usize,
    wire_pos: usize,
    last_activity_ms: u32,
    discard_input_until_ms: u32,
    transfer_failed: bool,
    transfer_started: bool,
    timeout_pending: bool,
    input_accepted: bool,
    response_queued: bool,
    response_sent: bool,
    session_complete_pending: bool,
    last_error: u8,
    protocol_error: u8,
    protocol_error_detail: u8,
    wire_received_bytes: u32,
    wire_consumed_bytes: u32,
    append_lfs_error: i32,
    append_storage_operation: u32,
    append_storage_location: u32,
    append_storage_error: u32,
    append_nand_stage: u32,
    append_nand_detail: u32,
    append_nand_page: u32,
    append_nand_stored_ecc: u32,
    append_nand_calculated_ecc: u32,
    commit_stage: u32,
    commit_lfs_error: i32,
}

impl State {
    pub fn new() -> Self {
        Self {
            receiver: None,
            sender: None,
            send_path: [0; NAME_CAPACITY + 2],
            send_path_len: 0,
            send_size: 0,
            send_started_rtc_ms: 0,
            send_buffer: [0; 1024],
            send_wire: [0; WIRE_CAPACITY],
            send_wire_len: 0,
            send_wire_pos: 0,
            send_last_activity_ms: 0,
            send_last_handshake_ms: 0,
            file_allocation: FileAllocation::new(),
            final_path: [0; NAME_CAPACITY + 2],
            final_path_len: 0,
            receive_name: [0; NAME_CAPACITY],
            receive_name_len: 0,
            receive_size: 0,
            receive_file_committed: false,
            receive_started_rtc_ms: 0,
            receive_elapsed_seconds: 0,
            wire: [0; WIRE_CAPACITY],
            wire_len: 0,
            wire_pos: 0,
            last_activity_ms: 0,
            discard_input_until_ms: 0,
            transfer_failed: false,
            transfer_started: false,
            timeout_pending: false,
            input_accepted: false,
            response_queued: false,
            response_sent: false,
            session_complete_pending: false,
            last_error: 0,
            protocol_error: 0,
            protocol_error_detail: 0,
            wire_received_bytes: 0,
            wire_consumed_bytes: 0,
            append_lfs_error: 0,
            append_storage_operation: 0,
            append_storage_location: 0,
            append_storage_error: 0,
            append_nand_stage: 0,
            append_nand_detail: 0,
            append_nand_page: 0,
            append_nand_stored_ecc: 0,
            append_nand_calculated_ecc: 0,
            commit_stage: 0,
            commit_lfs_error: 0,
        }
    }

    pub fn active(&self) -> bool {
        self.receiver.is_some()
            || self.sender.is_some()
            || self.session_complete_pending
            || self.file_allocation.is_open()
    }

    /// Discard trailing sender bytes after a failed session so they cannot
    /// be interpreted as interactive console commands. Resume the console
    /// only after one second without incoming transfer data.
    pub fn discard_trailing_input(&mut self, now_ms: u32) -> bool {
        if self.discard_input_until_ms == 0 {
            return false;
        }

        let mut discarded = [0u8; WIRE_CAPACITY];
        let count = unsafe {
            tusb::tud_cdc_n_read(
                0,
                discarded.as_mut_ptr().cast::<c_void>(),
                discarded.len() as u32,
            )
        } as usize;
        if count != 0 {
            self.discard_input_until_ms = now_ms.wrapping_add(1_000);
            return true;
        }

        if (now_ms.wrapping_sub(self.discard_input_until_ms) as i32) < 0 {
            return true;
        }

        self.discard_input_until_ms = 0;
        false
    }

    pub fn handshake_status(&self) -> (bool, bool, bool) {
        (self.input_accepted, self.response_queued, self.response_sent)
    }

    /// Deliver a committed receive notification from the normal console path.
    /// Keep the record until the complete message has entered the CDC TX FIFO.
    pub fn report_receive_complete(&mut self) -> bool {
        if !self.receive_file_committed || self.receive_name_len == 0 {
            return false;
        }

        if !super::cdc_console::write_receive_complete(
            &self.receive_name[..self.receive_name_len],
            self.receive_size,
            self.receive_elapsed_seconds,
        ) {
            return false;
        }

        self.receive_name_len = 0;
        self.receive_size = 0;
        self.receive_file_committed = false;
        self.receive_elapsed_seconds = 0;
        true
    }

    pub fn last_error(&self) -> u8 {
        self.last_error
    }

    pub fn protocol_diagnostics(&self) -> (u8, u8, u32, u32) {
        (self.protocol_error, self.protocol_error_detail, self.wire_received_bytes, self.wire_consumed_bytes)
    }

    pub fn commit_failure_details(&self) -> (u32, i32) {
        (self.commit_stage, self.commit_lfs_error)
    }

    pub fn append_failure_details(&self) -> (i32, u32, u32, u32, u32, u32, u32, u32, u32) {
        (
            self.append_lfs_error,
            self.append_storage_operation,
            self.append_storage_location,
            self.append_storage_error,
            self.append_nand_stage,
            self.append_nand_detail,
            self.append_nand_page,
            self.append_nand_stored_ecc,
            self.append_nand_calculated_ecc,
        )
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
    state.sender = None;
    state.session_complete_pending = false;
    state.send_path_len = 0;
    state.send_wire_len = 0;
    state.send_wire_pos = 0;
    state.final_path_len = 0;
    // A committed receive notification survives a CDC disconnect. The
    // normal console path will deliver it after the host reconnects.
    if !state.receive_file_committed {
        state.receive_name_len = 0;
        state.receive_size = 0;
    }
    state.wire_len = 0;
    state.wire_pos = 0;
    state.transfer_started = false;
    state.timeout_pending = false;
    state.session_complete_pending = false;
    state.discard_input_until_ms = 0;
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

fn create_new_file(
    fs: &Filesystem<'_, ActiveStorage>,
    allocation: &mut FileAllocation<ActiveStorage>,
    path_bytes: &[u8],
) -> Result<(), u8> {
    let path = Path::from_bytes_with_nul(path_bytes).map_err(|_| 1)?;
    fs.open_file_with_options_and_then_with_allocation(
        |options| options.write(true).create_new(true),
        path,
        allocation,
        |_file| Ok(()),
    )
    .map_err(|_| 2)
}

fn remove_partial_file(fs: &Filesystem<'_, ActiveStorage>, state: &mut State) {
    if state.file_allocation.is_open() {
        #[cfg(feature = "list-profile")]
        let close_started = crate::write_diag::cycle_count();
        if unsafe { state.file_allocation.close(fs) }.is_err() {
            if state.last_error == 0 {
                state.last_error = 8;
            }
            return;
        }
        #[cfg(feature = "list-profile")]
        crate::write_diag::record_lfs_append_close(
            crate::write_diag::cycle_count().wrapping_sub(close_started),
        );
    }
    if state.final_path_len != 0 {
        if let Ok(path) = Path::from_bytes_with_nul(TEMP_PATH) {
            if fs.remove(path).is_err() && state.last_error == 0 {
                state.last_error = 8;
            }
        }
    }
    state.final_path_len = 0;
}

fn create_temp_file(
    fs: &Filesystem<'_, ActiveStorage>,
    allocation: &mut FileAllocation<ActiveStorage>,
) -> Result<(), u8> {
    let path = Path::from_bytes_with_nul(TEMP_PATH).map_err(|_| 1)?;
    // This path is reserved exclusively for an interrupted ZMODEM upload.
    // Remove a stale partial file before starting the next transfer.
    if fs.exists(path) {
        fs.remove(path).map_err(|_| 2)?;
    }
    create_new_file(fs, allocation, TEMP_PATH).map_err(|stage| if stage == 1 { 3 } else { 4 })?;
    #[cfg(feature = "list-profile")]
    let open_started = crate::write_diag::cycle_count();
    let open_result = unsafe {
        allocation.open_with_options(
            fs,
            |options| options.write(true),
            path,
        )
    };
    #[cfg(feature = "list-profile")]
    crate::write_diag::record_lfs_append_open(
        crate::write_diag::cycle_count().wrapping_sub(open_started),
    );
    if open_result.is_err() {
        let _ = fs.remove(path);
        return Err(7);
    }
    Ok(())
}

fn commit_temp_file(fs: &Filesystem<'_, ActiveStorage>, state: &mut State) -> Result<(), (u32, i32)> {
    if state.final_path_len == 0 {
        return Ok(());
    }
    let temp = Path::from_bytes_with_nul(TEMP_PATH).map_err(|_| (1, LfsError::IO.code()))?;
    let final_path = Path::from_bytes_with_nul(&state.final_path[..state.final_path_len])
        .map_err(|_| (2, LfsError::IO.code()))?;
    if fs.exists(final_path) {
        return Err((3, 0));
    }
    if state.file_allocation.is_open() {
        #[cfg(feature = "list-profile")]
        let close_started = crate::write_diag::cycle_count();
        unsafe { state.file_allocation.close(fs) }.map_err(|error| (5, error.code()))?;
        #[cfg(feature = "list-profile")]
        crate::write_diag::record_lfs_append_close(
            crate::write_diag::cycle_count().wrapping_sub(close_started),
        );
    }
    fs.rename(temp, final_path).map_err(|error| (4, error.code()))?;
    state.final_path_len = 0;
    Ok(())
}

fn append_file(
    fs: &Filesystem<'_, ActiveStorage>,
    allocation: &mut FileAllocation<ActiveStorage>,
    data: &[u8],
) -> Result<(), i32> {
    #[cfg(feature = "list-profile")]
    let append_started = crate::write_diag::cycle_count();
    let mut written = 0usize;
    while written < data.len() {
        #[cfg(feature = "list-profile")]
        let write_started = crate::write_diag::cycle_count();
        let count = unsafe { allocation.write(fs, &data[written..]) }
            .map_err(|error| error.code())?;
        #[cfg(feature = "list-profile")]
        crate::write_diag::record_lfs_write(
            crate::write_diag::cycle_count().wrapping_sub(write_started), count,
        );
        if count == 0 {
            return Err(LfsError::IO.code());
        }
        written += count;
    }
    // littlefs2's Write::flush is intentionally a no-op. The persistent
    // handle is synchronized once, when it is closed at commit/abort.
    #[cfg(feature = "list-profile")]
    crate::write_diag::record_lfs_append_callback(
        crate::write_diag::cycle_count().wrapping_sub(append_started),
    );
    Ok(())
}

/// Create a deterministic 4 KiB file and read it twice through LittleFS/Dhara:
/// once with ECC validation bypassed, then through the normal ECC path.
/// Returns 0 on success; 1..N identify the failing stage.
pub fn run_ecc_storage_diagnostic(
    fs: &Filesystem<'_, ActiveStorage>,
    state: &mut State,
) -> u8 {
    const TEST_PATH: &[u8] = b"/eccdiag.bin\0";
    const TEST_SIZE: usize = 4096;
    const CHUNK: usize = 512;
    let Ok(path) = Path::from_bytes_with_nul(TEST_PATH) else { return 1; };
    if fs.exists(path) && fs.remove(path).is_err() { return 2; }

    let create = fs.open_file_with_options_and_then_with_allocation(
        |options| options.write(true).create_new(true),
        path,
        &mut state.file_allocation,
        |_file| Ok(()),
    );
    if create.is_err() { return 3; }

    let write_result = fs.open_file_with_options_and_then_with_allocation(
        |options| options.write(true).append(true),
        path,
        &mut state.file_allocation,
        |file| {
            let mut data = [0u8; CHUNK];
            for block in 0..(TEST_SIZE / CHUNK) {
                for (i, byte) in data.iter_mut().enumerate() {
                    let offset = block * CHUNK + i;
                    *byte = ((offset as u32).wrapping_mul(37).wrapping_add(0x5A) ^ (offset as u32 >> 3)) as u8;
                }
                let mut written = 0;
                while written < data.len() {
                    let count = file.write(&data[written..])?;
                    if count == 0 { return Err(LfsError::IO); }
                    written += count;
                }
            }
            file.flush()
        },
    );
    if write_result.is_err() { return 4; }

    let previous_ecc_setting = crate::nand_fs::ecc_check_enabled();
    let mut ecc_read_error = None;
    for ecc_enabled in [false, true] {
        crate::nand_fs::set_ecc_check_enabled(ecc_enabled);
        let read_result = fs.open_file_with_options_and_then_with_allocation(
            |options| options.read(true),
            path,
            &mut state.file_allocation,
            |file| {
                let mut data = [0u8; CHUNK];
                let mut offset = 0usize;
                while offset < TEST_SIZE {
                    let count = file.read(&mut data)?;
                    if count == 0 { return Err(LfsError::IO); }
                    for i in 0..count {
                        let absolute = offset + i;
                        let expected = ((absolute as u32).wrapping_mul(37).wrapping_add(0x5A) ^ (absolute as u32 >> 3)) as u8;
                        if data[i] != expected { return Err(LfsError::IO); }
                    }
                    offset += count;
                }
                Ok(())
            },
        );
        if read_result.is_err() {
            ecc_read_error = Some(if ecc_enabled { 6 } else { 5 });
            break;
        }
    }
    crate::nand_fs::set_ecc_check_enabled(previous_ecc_setting);
    if let Some(error) = ecc_read_error {
        return error;
    }

    if fs.remove(path).is_err() { return 7; }
    0
}

pub fn start(state: &mut State, storage_available: bool, now_ms: u32) -> Result<(), ()> {
    if !storage_available || state.active() {
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
    state.receive_name_len = 0;
    state.receive_size = 0;
    state.receive_file_committed = false;
    state.receive_started_rtc_ms = 0;
    state.receive_elapsed_seconds = 0;
    state.wire_len = 0;
    state.wire_pos = 0;
    state.last_activity_ms = now_ms;
    state.transfer_failed = false;
    state.transfer_started = false;
    state.timeout_pending = false;
    state.input_accepted = false;
    state.response_queued = false;
    state.response_sent = false;
    state.session_complete_pending = false;
    state.last_error = 0;
    state.protocol_error = 0;
    state.protocol_error_detail = 0;
    state.wire_received_bytes = 0;
    state.wire_consumed_bytes = 0;
    state.append_lfs_error = 0;
    state.append_storage_operation = 0;
    state.append_storage_location = 0;
    state.append_storage_error = 0;
    state.append_nand_stage = 0;
    state.append_nand_detail = 0;
    state.append_nand_page = 0;
    state.append_nand_stored_ecc = 0;
    state.append_nand_calculated_ecc = 0;
    state.commit_stage = 0;
    state.commit_lfs_error = 0;
    state.receiver = Some(receiver);
    Ok(())
}

/// Start a device-to-terminal ZMODEM transfer for one root-level LittleFS file.
pub fn start_send(
    fs: &Filesystem<'_, ActiveStorage>,
    state: &mut State,
    name: &[u8],
    now_ms: u32,
) -> Result<(), u8> {
    if state.active() {
        return Err(1);
    }
    let mut path_bytes = [0u8; NAME_CAPACITY + 2];
    let path_len = make_path(name, &mut path_bytes).ok_or(2)?;
    let path = Path::from_bytes_with_nul(&path_bytes[..path_len]).map_err(|_| 2)?;
    let metadata = fs.metadata(path).map_err(|_| 3)?;
    let size = u32::try_from(metadata.len()).map_err(|_| 4)?;

    let mut sender = Sender::new().map_err(|_| 5)?;
    let info = FileInfo::new(name, Some(Position::new(size))).with_mode(0x8000 | 0o644);
    sender.start_file(info).map_err(|_| 5)?;

    // Retain one LittleFS read handle for the complete ZMODEM send.
    // State is stored at a stable address throughout the transfer.
    unsafe {
        state.file_allocation
            .open_with_options(fs, |options| options.read(true), path)
            .map_err(|_| 3)?;
    }

    state.send_path = path_bytes;
    state.send_path_len = path_len;
    state.send_size = size;
    state.send_started_rtc_ms = rtc_now_millis();
    state.send_wire_len = 0;
    state.send_wire_pos = 0;
    state.send_last_activity_ms = now_ms;
    state.send_last_handshake_ms = now_ms;
    state.session_complete_pending = false;
    state.sender = Some(sender);
    Ok(())
}

/// Drive the sender while the CDC terminal receives the file.
fn service_sender(
    fs: &Filesystem<'_, ActiveStorage>,
    state: &mut State,
    now_ms: u32,
) -> bool {
    let Some(mut sender) = state.sender.take() else {
        if state.session_complete_pending {
            state.session_complete_pending = false;
            let elapsed_ms = rtc_now_millis().wrapping_sub(state.send_started_rtc_ms);
            super::cdc_console::write_send_complete(
                &state.send_path[1..state.send_path_len - 1],
                state.send_size,
                elapsed_ms / 1_000,
            );
            state.send_path_len = 0;
            state.send_wire_len = 0;
            state.send_wire_pos = 0;
            return true;
        }
        return false;
    };
    let mut finished = false;

    for _ in 0..8 {
        match sender.poll() {
            Action::WriteWire(bytes) => {
                let len = bytes.len();
                let count = unsafe {
                    tusb::tud_cdc_n_write(0, bytes.as_ptr().cast::<c_void>(), len as u32)
                } as usize;
                if count == 0 {
                    break;
                }
                unsafe { let _ = tusb::tud_cdc_n_write_flush(0); }
                sender.wire_written(count);
                state.send_last_activity_ms = now_ms;
                if count < len {
                    break;
                }
                // SessionCompleted is reported before the final "OO" bytes.
                // Keep the console suspended until a later service pass, so
                // TinyUSB gets an opportunity to transmit the protocol tail
                // before console text is queued on the same CDC endpoint.
                if state.session_complete_pending {
                    finished = true;
                    break;
                }
            }
            Action::ReadFile { offset, max_len } => {
                if state.send_path_len == 0 || max_len == 0 || max_len > state.send_buffer.len() {
                    sender.abort();
                    break;
                }
                let read_result = unsafe {
                    state.file_allocation
                        .seek(fs, SeekFrom::Start(offset.get()))
                        .and_then(|_| state.file_allocation.read(fs, &mut state.send_buffer[..max_len]))
                };
                match read_result {
                    Ok(count) if count != 0 => {
                        if sender.submit_file(&state.send_buffer[..count]).is_err() {
                            sender.abort();
                            break;
                        }
                        state.send_last_activity_ms = now_ms;
                    }
                    _ => {
                        sender.abort();
                        break;
                    }
                }
            }
            Action::Event(Event::FileCompleted) => {
                // This command sends exactly one file. Once the receiver
                // acknowledges ZEOF with ZRINIT, finish the ZMODEM session.
                if sender.finish().is_err() {
                    sender.abort();
                    break;
                }
            }
            Action::Event(Event::SessionCompleted) => {
                // The protocol engine reports completion before exposing its
                // final "OO" acknowledgement as WriteWire.
                state.session_complete_pending = true;
            }
            Action::Event(Event::Aborted) => {
                super::cdc_console::write_status(b"\r\nZMODEM send aborted or failed.\r\n> ");
                finished = true;
                break;
            }
            Action::Event(_) | Action::WriteFile(_) => {
                sender.abort();
                break;
            }
            Action::Idle => {
                if now_ms.wrapping_sub(state.send_last_activity_ms) >= TRANSFER_TIMEOUT_MS {
                    sender.abort();
                    continue;
                }

                if state.send_wire_pos == state.send_wire_len {
                    state.send_wire_len = unsafe {
                        tusb::tud_cdc_n_read(
                            0,
                            state.send_wire.as_mut_ptr().cast::<c_void>(),
                            WIRE_CAPACITY as u32,
                        )
                    } as usize;
                    state.send_wire_pos = 0;
                    if state.send_wire_len != 0 {
                        state.send_last_activity_ms = now_ms;
                    }
                }

                if state.send_wire_pos < state.send_wire_len {
                    let input = &state.send_wire[state.send_wire_pos..state.send_wire_len];
                    match sender.submit_wire(input) {
                        Ok(consumed) if consumed != 0 => {
                            state.send_wire_pos += consumed;
                            state.send_last_activity_ms = now_ms;
                        }
                        Ok(_) => {
                            let remaining = state.send_wire_len - state.send_wire_pos;
                            state.send_wire.copy_within(state.send_wire_pos..state.send_wire_len, 0);
                            state.send_wire_pos = 0;
                            state.send_wire_len = remaining;
                            let count = unsafe {
                                tusb::tud_cdc_n_read(
                                    0,
                                    state.send_wire[state.send_wire_len..].as_mut_ptr().cast::<c_void>(),
                                    (WIRE_CAPACITY - state.send_wire_len) as u32,
                                )
                            } as usize;
                            state.send_wire_len += count;
                            if count == 0 {
                                break;
                            }
                            state.send_last_activity_ms = now_ms;
                        }
                        Err(error) => {
                            let message: &[u8] = match error {
                                ZmodemError::MalformedEncoding(_) => b"\r\nZMODEM send: malformed receiver encoding\r\n",
                                ZmodemError::MalformedFileSize => b"\r\nZMODEM send: malformed receiver file size\r\n",
                                ZmodemError::MalformedFileName => b"\r\nZMODEM send: malformed receiver filename\r\n",
                                ZmodemError::MalformedFrame(_) => b"\r\nZMODEM send: malformed receiver frame\r\n",
                                ZmodemError::MalformedHeader => b"\r\nZMODEM send: malformed receiver header\r\n",
                                ZmodemError::MalformedPacket(_) => b"\r\nZMODEM send: malformed receiver packet\r\n",
                                ZmodemError::InvalidState => b"\r\nZMODEM send: invalid sender state\r\n",
                                ZmodemError::Backpressure => b"\r\nZMODEM send: protocol backpressure\r\n",
                                ZmodemError::OutOfMemory => b"\r\nZMODEM send: protocol buffer exhausted\r\n",
                                ZmodemError::UnexpectedCrc16 => b"\r\nZMODEM send: receiver CRC-16 mismatch\r\n",
                                ZmodemError::UnexpectedCrc32 => b"\r\nZMODEM send: receiver CRC-32 mismatch\r\n",
                                ZmodemError::UnexpectedEof => b"\r\nZMODEM send: unexpected receiver EOF\r\n",
                                ZmodemError::UnsupportedFeature => b"\r\nZMODEM send: unsupported receiver feature\r\n",
                                ZmodemError::SubpacketTooLarge => b"\r\nZMODEM send: receiver packet too large\r\n",
                            };
                            super::cdc_console::write_status(message);
                            sender.abort();
                        }
                    }
                } else if now_ms.wrapping_sub(state.send_last_handshake_ms) >= 2_000 {
                    let _ = sender.timeout();
                    state.send_last_handshake_ms = now_ms;
                } else {
                    break;
                }
            }
            // Action is non-exhaustive; fail closed if the protocol crate adds
            // an action this integration does not understand.
            _ => {
                sender.abort();
                break;
            }
        }
    }

    if finished {
        if state.file_allocation.is_open() {
            if unsafe { state.file_allocation.close(fs) }.is_err() {
                super::cdc_console::write_status(
                    b"\r\nZMODEM send: LittleFS close failed; reset required before another transfer.\r\n> ",
                );
            }
        }
        state.sender = None;
        // SessionCompleted is finalized on the next service pass, which still
        // needs the filename for the completion message. Preserve it until then.
        if !state.session_complete_pending {
            state.send_path_len = 0;
        }
        state.send_wire_len = 0;
        state.send_wire_pos = 0;
    } else {
        state.sender = Some(sender);
    }
    true
}

/// Preserve the ZMODEM startup header detected by the CDC console.
pub fn seed_input(state: &mut State, bytes: &[u8]) -> bool {
    if state.receiver.is_none() || bytes.len() > WIRE_CAPACITY || state.wire_len != 0 {
        return false;
    }
    state.wire[..bytes.len()].copy_from_slice(bytes);
    state.wire_received_bytes = state.wire_received_bytes.wrapping_add(bytes.len() as u32);
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
    if service_sender(fs, state, now_ms) {
        return;
    }
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

        #[cfg(feature = "list-profile")]
        let poll_started = crate::write_diag::cycle_count();
        let action = receiver.poll();
        #[cfg(feature = "list-profile")]
        crate::write_diag::record_zmodem_poll(
            crate::write_diag::cycle_count().wrapping_sub(poll_started),
        );
        match action {
            Action::WriteWire(bytes) => {
                state.response_queued = true;
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
                state.response_sent = true;
                receiver.wire_written(count);
                state.last_activity_ms = now_ms;
                if count < bytes_len {
                    break;
                }
                if state.session_complete_pending {
                    // ZFIN was received and SessionCompleted was surfaced,
                    // but the receiver's final ZFIN reply must reach CDC IN
                    // before the receiver state/storage can be released.
                    finished = true;
                    // Report completion after receiver teardown below. This
                    // keeps console output out of the protocol-action loop.
                    break;
                }
            }
            Action::WriteFile(bytes) => {
                let bytes_len = bytes.len();
                if state.final_path_len == 0 {
                    state.last_error = 1;
                    super::cdc_console::write_status(
                        b"\r\nZMODEM storage error: temp file not active\r\n",
                    );
                    state.transfer_failed = true;
                    let _ = receiver.abort();
                    break;
                }
                crate::dhara_fs::clear_storage_io_diagnostic();
                crate::nand_fs::clear_nand_read_diagnostic();
                if let Err(lfs_error) = append_file(fs, &mut state.file_allocation, bytes) {
                    state.last_error = 2;
                    let (operation, location, storage_error) =
                        crate::dhara_fs::storage_io_diagnostic();
                    state.append_lfs_error = lfs_error;
                    state.append_storage_operation = operation;
                    state.append_storage_location = location;
                    state.append_storage_error = storage_error;
                    let (stage, detail, page, stored_ecc, calculated_ecc) =
                        crate::nand_fs::nand_read_diagnostic();
                    state.append_nand_stage = stage;
                    state.append_nand_detail = detail;
                    state.append_nand_page = page;
                    state.append_nand_stored_ecc = stored_ecc;
                    state.append_nand_calculated_ecc = calculated_ecc;
                    state.transfer_failed = true;
                    let _ = receiver.abort();
                    break;
                }
                state.receive_size = state.receive_size.wrapping_add(bytes_len as u32);
                if receiver.file_written(bytes_len).is_err() {
                    state.last_error = 3;
                    super::cdc_console::write_status(
                        b"\r\nZMODEM protocol error: file_written rejected data\r\n",
                    );
                    state.transfer_failed = true;
                    let _ = receiver.abort();
                    break;
                }
                state.last_activity_ms = now_ms;
            }
            Action::Event(Event::FileStarted(info)) => {
                state.transfer_started = true;
                let path_len = make_path(info.name, &mut state.final_path);
                let rejection = match path_len {
                    None => Some(1),
                    Some(len) => match Path::from_bytes_with_nul(&state.final_path[..len]) {
                        Err(_) => Some(2),
                        Ok(path) if fs.exists(path) => Some(3),
                        Ok(_) => {
                            #[cfg(feature = "list-profile")]
                            crate::write_diag::begin();
                            #[cfg(feature = "list-profile")]
                            let create_started = crate::write_diag::cycle_count();
                            let create_result = create_temp_file(fs, &mut state.file_allocation);
                            #[cfg(feature = "list-profile")]
                            crate::write_diag::record_lfs_create(
                                crate::write_diag::cycle_count().wrapping_sub(create_started),
                            );
                            create_result.err().map(|stage| stage + 3)
                        },
                    },
                };
                if rejection.is_none() {
                    state.final_path_len = path_len.unwrap_or(0);
                    // Copy the borrowed filename before calling another
                    // mutable Receiver method; Event::FileStarted borrows
                    // its name from receiver.poll().
                    state.receive_name_len = info.name.len();
                    state.receive_name[..state.receive_name_len].copy_from_slice(info.name);
                    state.receive_size = 0;
                    state.receive_started_rtc_ms = rtc_now_millis();
                    if receiver.accept_file_at(0).is_err() {
                        state.last_error = 4;
                        state.receive_name_len = 0;
                        super::cdc_console::write_status(
                            b"\r\nZMODEM protocol error: accept_file_at failed\r\n",
                        );
                        state.transfer_failed = true;
                        let _ = receiver.abort();
                        break;
                    }
                } else {
                    state.final_path_len = 0;
                    let message: &[u8] = match rejection.unwrap_or(0) {
                        1 => b"\r\nZMODEM rejected file: invalid filename\r\n",
                        2 => b"\r\nZMODEM rejected file: invalid destination path\r\n",
                        3 => b"\r\nZMODEM rejected file: destination already exists\r\n",
                        4 => b"\r\nZMODEM rejected file: temp path invalid\r\n",
                        5 => b"\r\nZMODEM rejected file: stale temp-file removal failed\r\n",
                        6 => b"\r\nZMODEM rejected file: temp-file path invalid\r\n",
                        _ => b"\r\nZMODEM rejected file: temp-file creation failed\r\n",
                    };
                    super::cdc_console::write_status(message);
                    if receiver.skip_file().is_err() {
                        state.transfer_failed = true;
                        let _ = receiver.abort();
                        break;
                    }
                }
            }
            Action::Event(Event::FileCompleted) => {
                #[cfg(feature = "list-profile")]
                let commit_started = crate::write_diag::cycle_count();
                let commit_result = commit_temp_file(fs, state);
                #[cfg(feature = "list-profile")]
                crate::write_diag::record_lfs_commit(
                    crate::write_diag::cycle_count().wrapping_sub(commit_started),
                );
                if let Err((stage, error)) = commit_result {
                    state.last_error = 5;
                    state.commit_stage = stage;
                    state.commit_lfs_error = error;
                    // Preserve the completed temporary upload for diagnosis or
                    // manual recovery. Clearing final_path_len prevents the
                    // common abort cleanup from deleting it.
                    state.final_path_len = 0;
                    super::cdc_console::write_status(
                        b"\r\nZMODEM storage error: final commit failed; temporary file preserved\r\n",
                    );
                    state.transfer_failed = true;
                    let _ = receiver.abort();
                    break;
                }
                // The file is now atomically renamed to its final path.
                // Defer the console message until the ZMODEM close response
                // has been queued and the receiver is torn down. The CDC
                // endpoint still carries protocol traffic at this point.
                let receive_elapsed_ms =
                    rtc_now_millis().wrapping_sub(state.receive_started_rtc_ms);
                state.receive_elapsed_seconds = receive_elapsed_ms / 1_000;
                state.receive_file_committed = true;
                #[cfg(feature = "list-profile")]
                crate::write_diag::finish(receive_elapsed_ms);
            }
            Action::Event(Event::SessionCompleted) => {
                // Receiver::poll() surfaces events before queued wire bytes.
                // Defer teardown until the final ZFIN reply has been written.
                state.session_complete_pending = true;
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
                // Keep failure flags set until the common teardown below captures
                // them and enables the CDC input-discard window.
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
                    state.wire_received_bytes = state.wire_received_bytes.wrapping_add(state.wire_len as u32);
                    state.wire_pos = 0;
                    if state.wire_len != 0 {
                        state.last_activity_ms = now_ms;
                    }
                }

                if state.wire_pos < state.wire_len {
                    let input = &state.wire[state.wire_pos..state.wire_len];

                    #[cfg(feature = "list-profile")]
                    let submit_started = crate::write_diag::cycle_count();
                    let submit_result = receiver.submit_wire(input);
                    #[cfg(feature = "list-profile")]
                    crate::write_diag::record_zmodem_submit(
                        crate::write_diag::cycle_count().wrapping_sub(submit_started),
                    );
                    match submit_result {
                        Ok(consumed) if consumed != 0 => {
                            state.input_accepted = true;
                            state.wire_consumed_bytes = state.wire_consumed_bytes.wrapping_add(consumed as u32);
                            state.wire_pos += consumed;
                        },
                        Ok(_) => {
                            // The decoder may need more bytes before it can
                            // consume an incomplete header or packet. Preserve
                            // the unconsumed bytes and append more CDC input;
                            // otherwise the seeded ZRQINIT prefix can stall
                            // the receiver before the rest of the header arrives.
                            if state.wire_len == WIRE_CAPACITY {
                                break;
                            }
                            let remaining = state.wire_len - state.wire_pos;
                            state.wire.copy_within(
                                state.wire_pos..state.wire_len,
                                0,
                            );
                            state.wire_pos = 0;
                            state.wire_len = remaining;
                            let count = unsafe {
                                tusb::tud_cdc_n_read(
                                    0,
                                    state.wire[state.wire_len..].as_mut_ptr().cast::<c_void>(),
                                    (WIRE_CAPACITY - state.wire_len) as u32,
                                )
                            } as usize;
                            state.wire_received_bytes = state.wire_received_bytes.wrapping_add(count as u32);
                            if count == 0 {
                                break;
                            }
                            state.wire_len += count;
                            state.last_activity_ms = now_ms;
                        },
                        Err(error) => {
                            state.last_error = 6;
                            let (code, detail) = match error {
                                ZmodemError::MalformedEncoding(v) => (1, v),
                                ZmodemError::MalformedFileSize => (2, 0),
                                ZmodemError::MalformedFileName => (3, 0),
                                ZmodemError::MalformedFrame(v) => (4, v),
                                ZmodemError::MalformedHeader => (5, 0),
                                ZmodemError::MalformedPacket(v) => (6, v),
                                ZmodemError::InvalidState => (7, 0),
                                ZmodemError::Backpressure => (8, 0),
                                ZmodemError::OutOfMemory => (9, 0),
                                ZmodemError::UnexpectedCrc16 => (10, 0),
                                ZmodemError::UnexpectedCrc32 => (11, 0),
                                ZmodemError::UnexpectedEof => (12, 0),
                                ZmodemError::UnsupportedFeature => (13, 0),
                                ZmodemError::SubpacketTooLarge => (14, 0),
                            };
                            state.protocol_error = code;
                            state.protocol_error_detail = detail;
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
                                ZmodemError::SubpacketTooLarge => b"\r\nZMODEM input: subpacket too large; retry requested\r\n",
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
        let failed = state.transfer_failed || state.timeout_pending;
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
        // Keep a committed-file notification pending until the normal CDC
        // console service has accepted the complete message into its TX FIFO.
        let file_committed = state.receive_file_committed;
        state.final_path_len = 0;
        if !file_committed {
            state.receive_name_len = 0;
            state.receive_size = 0;
        }
        state.transfer_failed = false;
        state.transfer_started = false;
        state.timeout_pending = false;
        state.session_complete_pending = false;
        if failed || file_committed {
            // The peer may send the final ASCII "OO" just after our ZFIN
            // reply. Keep it out of the console even when it arrives late.
            state.discard_input_until_ms = now_ms.wrapping_add(1_000);
        }
        camera::release_cdc_receiver_storage();
    }
}
