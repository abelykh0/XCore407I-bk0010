//! USB Still Image/PTP transport used by Windows MTP.
//!
//! MTP is carried over the USB Still Image class (06h/01h/01h). The data
//! plane consists of bulk OUT, bulk IN, and interrupt IN endpoints. This
//! module intentionally contains no UVC/isochronous scheduling code.

use usb_device::bus::{InterfaceNumber, StringIndex, UsbBusAllocator};
use usb_device::class::UsbClass;
use usb_device::descriptor::DescriptorWriter;
use usb_device::endpoint::{EndpointIn, EndpointOut, EndpointType};
use usb_device::Result;
use stm32f4::stm32f407::TIM5;

/// USB Still Image class / PTP subclass / PTP protocol.
pub const STILL_IMAGE_CLASS: u8 = 0x06;
pub const PTP_SUBCLASS: u8 = 0x01;
pub const PTP_PROTOCOL: u8 = 0x01;

/// Read the free-running 1 MHz TIM5 counter used for transfer timing.
#[inline]
fn read_tim5_us() -> u32 {
    // TIM5 is initialized before USB polling is enabled.
    unsafe { (*TIM5::ptr()).cnt().read().bits() }
}

/// USB 2.0 High-Speed maximum packet sizes.
pub const BULK_MAX_PACKET_SIZE: u16 = 512;
pub const EVENT_MAX_PACKET_SIZE: u16 = 64;
pub const EVENT_INTERVAL: u8 = 4;

/// Live transport counters for debugger inspection. They intentionally use
/// atomics because callbacks run from the TIM2 USB polling interrupt.
pub mod diagnostics {
    use core::sync::atomic::AtomicU32;
    pub static OUT_PACKETS: AtomicU32 = AtomicU32::new(0);
    pub static OUT_INVALID: AtomicU32 = AtomicU32::new(0);
    pub static COMMANDS_ACCEPTED: AtomicU32 = AtomicU32::new(0);
    pub static COMMANDS_DEFERRED: AtomicU32 = AtomicU32::new(0);
    pub static IN_SUBMIT_OK: AtomicU32 = AtomicU32::new(0);
    pub static IN_SUBMIT_WOULDBLOCK: AtomicU32 = AtomicU32::new(0);
    pub static IN_COMPLETE: AtomicU32 = AtomicU32::new(0);
    pub static USB_RESETS: AtomicU32 = AtomicU32::new(0);
    pub static PENDING_COMMAND_OVERFLOW: AtomicU32 = AtomicU32::new(0);
    pub static DATA_IN_COMPLETED: AtomicU32 = AtomicU32::new(0);
    pub static RESPONSE_IN_COMPLETED: AtomicU32 = AtomicU32::new(0);
    pub static CONTROL_GET_STATUS: AtomicU32 = AtomicU32::new(0);
    pub static CONTROL_DEVICE_RESET: AtomicU32 = AtomicU32::new(0);
    pub static SEND_OBJECT_INFO_COMMANDS: AtomicU32 = AtomicU32::new(0);
    pub static SEND_OBJECT_INFO_DATA: AtomicU32 = AtomicU32::new(0);
    pub static OBJECT_INFO_DECODE_OK: AtomicU32 = AtomicU32::new(0);
    pub static OBJECT_INFO_DECODE_FAIL: AtomicU32 = AtomicU32::new(0);
    pub static OBJ_BAD_LENGTH: AtomicU32 = AtomicU32::new(0);
    pub static OBJ_BAD_STORAGE: AtomicU32 = AtomicU32::new(0);
    pub static OBJ_LAST_STORAGE_ID: AtomicU32 = AtomicU32::new(0);
    pub static OBJ_LAST_DATASET_LEN: AtomicU32 = AtomicU32::new(0);
    pub static OBJ_EXPECTED_DATASET_LEN: AtomicU32 = AtomicU32::new(0);
    pub static OBJ_BAD_FORMAT: AtomicU32 = AtomicU32::new(0);
    pub static OBJ_BAD_PROTECTION: AtomicU32 = AtomicU32::new(0);
    pub static OBJ_BAD_ASSOC: AtomicU32 = AtomicU32::new(0);
    pub static OBJ_BAD_PARENT: AtomicU32 = AtomicU32::new(0);
    pub static OBJ_BAD_NAME: AtomicU32 = AtomicU32::new(0);
    pub static SEND_OBJECT_COMMANDS: AtomicU32 = AtomicU32::new(0);
    pub static SEND_OBJECT_DATA: AtomicU32 = AtomicU32::new(0);
    pub static UPLOAD_DATA_REJECTED: AtomicU32 = AtomicU32::new(0);
    pub static UPLOAD_ORPHAN_OUT: AtomicU32 = AtomicU32::new(0);
    pub static UPLOAD_INFO_ACCEPTED: AtomicU32 = AtomicU32::new(0);
    pub static UPLOAD_FS_CREATE_OK: AtomicU32 = AtomicU32::new(0);
    pub static UPLOAD_FS_CREATE_FAIL: AtomicU32 = AtomicU32::new(0);
    pub static UPLOAD_FS_WRITE_OK: AtomicU32 = AtomicU32::new(0);
    pub static UPLOAD_FS_WRITE_FAIL: AtomicU32 = AtomicU32::new(0);
    pub static UPLOAD_FS_RENAME_OK: AtomicU32 = AtomicU32::new(0);
    pub static UPLOAD_FS_RENAME_FAIL: AtomicU32 = AtomicU32::new(0);
    pub static UPLOAD_PREFLIGHT_REJECT: AtomicU32 = AtomicU32::new(0);
    pub static UPLOAD_PATH_REJECT: AtomicU32 = AtomicU32::new(0);
    pub static UPLOAD_NAME_REJECT: AtomicU32 = AtomicU32::new(0);
    pub static UPLOAD_TEMP_EXISTS: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_PROPS_SUPPORTED_COMMANDS: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_PROP_DESC_COMMANDS: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_PROP_LIST_COMMANDS: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_PROP_LIST_LAST_FORMAT: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_PROP_LIST_LAST_PROPERTY: AtomicU32 = AtomicU32::new(0);
    pub static SET_OBJECT_PROP_VALUE_COMMANDS: AtomicU32 = AtomicU32::new(0);
    pub static SET_OBJECT_PROP_VALUE_DATA: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_REQUESTS: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_LAST_SIZE: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_BYTES_READ: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_CHUNKS_QUEUED: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_READ_ERRORS: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_USB_SUBMITTED: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_USB_COMPLETED: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_USB_BYTES: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_RESPONSE_COMPLETED: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_READ_US: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_WAIT_US: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_TOTAL_US: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_USB_WOULDBLOCK: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_USB_POLL_LATENCY: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_USB_LATENCY_US: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_USB_LATENCY_MAX_US: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_FINAL_CHUNK_LEN: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_FINAL_CHUNK_ATTEMPTS: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_FINAL_CHUNK_SUBMITTED: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_FINAL_CHUNK_COMPLETED: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_FINAL_DATA_COMPLETED: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_RESPONSE_QUEUED: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_NEXT_COMMAND_DEFERRED: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_DEFERRED_COMMAND_PROCESSED: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_COMMANDS_WHILE_ACTIVE: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_LAST_COMMAND_CODE: AtomicU32 = AtomicU32::new(0);
    pub static GET_OBJECT_LAST_COMMAND_TXID: AtomicU32 = AtomicU32::new(0);
    pub static LAST_COMMAND_PARAM0: AtomicU32 = AtomicU32::new(0);
    pub static LAST_RESPONSE_CODE: AtomicU32 = AtomicU32::new(0);
    pub static LAST_RESPONSE_TXID: AtomicU32 = AtomicU32::new(0);

    fn append(out: &mut [u8], pos: &mut usize, bytes: &[u8]) {
        let count = bytes.len().min(out.len().saturating_sub(*pos));
        out[*pos..*pos + count].copy_from_slice(&bytes[..count]);
        *pos += count;
    }

    fn append_counter(out: &mut [u8], pos: &mut usize, label: &[u8], value: u32) {
        append(out, pos, label);
        let mut digits = [0u8; 10];
        let mut n = value;
        let mut count = 0;
        loop {
            digits[count] = b'0' + (n % 10) as u8;
            count += 1;
            n /= 10;
            if n == 0 { break; }
        }
        while count != 0 {
            count -= 1;
            append(out, pos, &digits[count..count + 1]);
        }
        append(out, pos, b"\r\n");
    }

    /// Compact persistent trace snapshot: retain the transport and transaction
    /// fields most useful for diagnosing a failed GET_OBJECT sequence.
    pub fn write_trace_status(out: &mut [u8]) -> usize {
        use core::sync::atomic::Ordering;
        let mut pos = 0;
        append(out, &mut pos, b"XCore407I MTP trace\r\n");
        append_counter(out, &mut pos, b"GOR=", GET_OBJECT_REQUESTS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GSize=", GET_OBJECT_LAST_SIZE.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GRD=", GET_OBJECT_BYTES_READ.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GCH=", GET_OBJECT_CHUNKS_QUEUED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GER=", GET_OBJECT_READ_ERRORS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GUSB_TX=", GET_OBJECT_USB_SUBMITTED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GUSB_CPL=", GET_OBJECT_USB_COMPLETED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GUSB_B=", GET_OBJECT_USB_BYTES.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GFINAL_LEN=", GET_OBJECT_FINAL_CHUNK_LEN.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GFINAL_ATT=", GET_OBJECT_FINAL_CHUNK_ATTEMPTS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GFINAL_TX=", GET_OBJECT_FINAL_CHUNK_SUBMITTED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GFINAL_CPL=", GET_OBJECT_FINAL_CHUNK_COMPLETED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GUSB_RSP=", GET_OBJECT_RESPONSE_COMPLETED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GFINAL=", GET_OBJECT_FINAL_DATA_COMPLETED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GRSPQ=", GET_OBJECT_RESPONSE_QUEUED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GDEF=", GET_OBJECT_NEXT_COMMAND_DEFERRED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GACTIVE_CMD=", GET_OBJECT_COMMANDS_WHILE_ACTIVE.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GLAST_OP=", GET_OBJECT_LAST_COMMAND_CODE.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GLAST_TID=", GET_OBJECT_LAST_COMMAND_TXID.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GCMD_P0=", LAST_COMMAND_PARAM0.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GRSP_CODE=", LAST_RESPONSE_CODE.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GRSP_TID=", LAST_RESPONSE_TXID.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"INC=", IN_COMPLETE.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"RESP=", RESPONSE_IN_COMPLETED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"RST=", USB_RESETS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"DEF=", COMMANDS_DEFERRED.load(Ordering::Relaxed));
        pos
    }

    /// Create a fixed-size text snapshot suitable for a virtual MTP object.
    pub fn write_status(out: &mut [u8]) -> usize {
        use core::sync::atomic::Ordering;
        let mut pos = 0;
        append(out, &mut pos, b"XCore407I MTP read diagnostics\r\n");
        append_counter(out, &mut pos, b"GOR=", GET_OBJECT_REQUESTS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GSize=", GET_OBJECT_LAST_SIZE.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GRD=", GET_OBJECT_BYTES_READ.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GCH=", GET_OBJECT_CHUNKS_QUEUED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GER=", GET_OBJECT_READ_ERRORS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GUSB_TX=", GET_OBJECT_USB_SUBMITTED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GUSB_CPL=", GET_OBJECT_USB_COMPLETED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GUSB_B=", GET_OBJECT_USB_BYTES.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GUSB_RSP=", GET_OBJECT_RESPONSE_COMPLETED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GREAD_US=", GET_OBJECT_READ_US.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GWAIT_US=", GET_OBJECT_WAIT_US.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GTOTAL_US=", GET_OBJECT_TOTAL_US.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GUSB_WB=", GET_OBJECT_USB_WOULDBLOCK.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GUSB_POLL=", GET_OBJECT_USB_POLL_LATENCY.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GUSB_LAT_US=", GET_OBJECT_USB_LATENCY_US.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GUSB_LAT_MAX_US=", GET_OBJECT_USB_LATENCY_MAX_US.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GFINAL_LEN=", GET_OBJECT_FINAL_CHUNK_LEN.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GFINAL_ATT=", GET_OBJECT_FINAL_CHUNK_ATTEMPTS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GFINAL_TX=", GET_OBJECT_FINAL_CHUNK_SUBMITTED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GFINAL_CPL=", GET_OBJECT_FINAL_CHUNK_COMPLETED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GFINAL=", GET_OBJECT_FINAL_DATA_COMPLETED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GRSPQ=", GET_OBJECT_RESPONSE_QUEUED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GDEF=", GET_OBJECT_NEXT_COMMAND_DEFERRED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GDEFDONE=", GET_OBJECT_DEFERRED_COMMAND_PROCESSED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GACTIVE_CMD=", GET_OBJECT_COMMANDS_WHILE_ACTIVE.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GLAST_OP=", GET_OBJECT_LAST_COMMAND_CODE.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GLAST_TID=", GET_OBJECT_LAST_COMMAND_TXID.load(Ordering::Relaxed));
        append(out, &mut pos, b"----\r\n");
        append_counter(out, &mut pos, b"OUT=", OUT_PACKETS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"BAD=", OUT_INVALID.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"CMD=", COMMANDS_ACCEPTED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"DEF=", COMMANDS_DEFERRED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"INOK=", IN_SUBMIT_OK.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"INWB=", IN_SUBMIT_WOULDBLOCK.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"INC=", IN_COMPLETE.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"RST=", USB_RESETS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"QOVF=", PENDING_COMMAND_OVERFLOW.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"DONE=", DATA_IN_COMPLETED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"RESP=", RESPONSE_IN_COMPLETED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"STAT=", CONTROL_GET_STATUS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"DRESET=", CONTROL_DEVICE_RESET.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"SOIC=", SEND_OBJECT_INFO_COMMANDS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"SOID=", SEND_OBJECT_INFO_DATA.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"DOK=", OBJECT_INFO_DECODE_OK.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"DBAD=", OBJECT_INFO_DECODE_FAIL.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"OLEN=", OBJ_BAD_LENGTH.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"OSTO=", OBJ_BAD_STORAGE.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"OSID=", OBJ_LAST_STORAGE_ID.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"ODL=", OBJ_LAST_DATASET_LEN.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"OEX=", OBJ_EXPECTED_DATASET_LEN.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"OFMT=", OBJ_BAD_FORMAT.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"OPRT=", OBJ_BAD_PROTECTION.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"OASC=", OBJ_BAD_ASSOC.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"OPAR=", OBJ_BAD_PARENT.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"ONAM=", OBJ_BAD_NAME.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"SOC=", SEND_OBJECT_COMMANDS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"SOD=", SEND_OBJECT_DATA.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"REJ=", UPLOAD_DATA_REJECTED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"ORPH=", UPLOAD_ORPHAN_OUT.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"INFO=", UPLOAD_INFO_ACCEPTED.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"CR+=", UPLOAD_FS_CREATE_OK.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"CR-=", UPLOAD_FS_CREATE_FAIL.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"WR+=", UPLOAD_FS_WRITE_OK.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"WR-=", UPLOAD_FS_WRITE_FAIL.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"RN+=", UPLOAD_FS_RENAME_OK.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"RN-=", UPLOAD_FS_RENAME_FAIL.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"PRE=", UPLOAD_PREFLIGHT_REJECT.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"PATH=", UPLOAD_PATH_REJECT.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"NAME=", UPLOAD_NAME_REJECT.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"TMP=", UPLOAD_TEMP_EXISTS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GOPS=", GET_OBJECT_PROPS_SUPPORTED_COMMANDS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GOPD=", GET_OBJECT_PROP_DESC_COMMANDS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GOPL=", GET_OBJECT_PROP_LIST_COMMANDS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GOPLF=", GET_OBJECT_PROP_LIST_LAST_FORMAT.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"GOPLP=", GET_OBJECT_PROP_LIST_LAST_PROPERTY.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"SOPC=", SET_OBJECT_PROP_VALUE_COMMANDS.load(Ordering::Relaxed));
        append_counter(out, &mut pos, b"SOPD=", SET_OBJECT_PROP_VALUE_DATA.load(Ordering::Relaxed));
        pos
    }
}

pub const CONTAINER_HEADER_LEN: usize = 12;
/// Temporary read-only placeholder store, replaced by the mounted LittleFS backend.
const PLACEHOLDER_STORAGE_ID: u32 = 0x0001_0001;
const DEMO_OBJECT_HANDLE: u32 = 1;
const DEMO_OBJECT_NAME: &str = "README.txt";
pub const MTP_OBJECT_CAPACITY: usize = 512;
const DEFAULT_README: &[u8] = b"XCore407I NAND diagnostic pending.\r\n";

/// One entry in the bounded root-directory snapshot supplied by the main loop.
#[derive(Clone, Copy)]
pub struct MtpObject {
    pub handle: u32,
    pub name: [u8; 64],
    pub name_len: u8,
    pub size: u32,
    pub is_dir: bool,
    /// Persistent 128-bit MTP identifier, loaded from the LittleFS file attribute.
    pub puid: [u8; 16],
}

impl MtpObject {
    pub const EMPTY: Self = Self { handle: 0, name: [0; 64], name_len: 0, size: 0, is_dir: false, puid: [0; 16] };
    fn name(&self) -> &str { core::str::from_utf8(&self.name[..self.name_len as usize]).unwrap_or("") }
}

/// Convert UTF-16 filename code units from a PTP string into bounded UTF-8.
/// Invalid surrogate sequences, control characters, path separators, and
/// Windows-invalid filename characters are rejected.
fn decode_utf16_filename(units: &[u16], out: &mut [u8]) -> Option<usize> {
    let mut written = 0usize;
    for decoded in core::char::decode_utf16(units.iter().copied()) {
        let ch = decoded.ok()?;
        if ch.is_control() || matches!(ch, '/' | '\\' | ':' | '*' | '?' | '"' | '<' | '>' | '|') {
            return None;
        }
        let mut encoded = [0u8; 4];
        let bytes = ch.encode_utf8(&mut encoded).as_bytes();
        if written.checked_add(bytes.len())? > out.len() {
            return None;
        }
        out[written..written + bytes.len()].copy_from_slice(bytes);
        written += bytes.len();
    }
    if written == 0 || &out[..written] == b"." || &out[..written] == b".." {
        return None;
    }
    Some(written)
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u16)]
pub enum ContainerType {
    Command = 1,
    Data = 2,
    Response = 3,
    Event = 4,
}

impl TryFrom<u16> for ContainerType {
    type Error = MtpError;

    fn try_from(value: u16) -> core::result::Result<Self, Self::Error> {
        match value {
            1 => Ok(Self::Command),
            2 => Ok(Self::Data),
            3 => Ok(Self::Response),
            4 => Ok(Self::Event),
            _ => Err(MtpError::InvalidContainerType),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MtpError {
    InvalidContainerType,
    InvalidContainerLength,
    TooManyParameters,
    OutputBufferTooSmall,
}

/// Common 12-byte PTP/MTP container header. All multi-byte fields are little-endian.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ContainerHeader {
    pub length: u32,
    pub kind: ContainerType,
    pub code: u16,
    pub transaction_id: u32,
}

impl ContainerHeader {
    pub fn decode(bytes: &[u8]) -> core::result::Result<Self, MtpError> {
        if bytes.len() < CONTAINER_HEADER_LEN {
            return Err(MtpError::InvalidContainerLength);
        }
        let length = u32::from_le_bytes(bytes[0..4].try_into().unwrap());
        if length < CONTAINER_HEADER_LEN as u32 {
            return Err(MtpError::InvalidContainerLength);
        }
        Ok(Self {
            length,
            kind: ContainerType::try_from(u16::from_le_bytes([bytes[4], bytes[5]]))?,
            code: u16::from_le_bytes([bytes[6], bytes[7]]),
            transaction_id: u32::from_le_bytes(bytes[8..12].try_into().unwrap()),
        })
    }

    pub fn encode(&self, out: &mut [u8]) -> core::result::Result<(), MtpError> {
        if out.len() < CONTAINER_HEADER_LEN {
            return Err(MtpError::OutputBufferTooSmall);
        }
        out[0..4].copy_from_slice(&self.length.to_le_bytes());
        out[4..6].copy_from_slice(&(self.kind as u16).to_le_bytes());
        out[6..8].copy_from_slice(&self.code.to_le_bytes());
        out[8..12].copy_from_slice(&self.transaction_id.to_le_bytes());
        Ok(())
    }
}

/// PTP/MTP operation codes used by the initial Windows-compatible subset.
pub mod operation {
    pub const GET_DEVICE_INFO: u16 = 0x1001;
    pub const OPEN_SESSION: u16 = 0x1002;
    pub const CLOSE_SESSION: u16 = 0x1003;
    pub const GET_STORAGE_IDS: u16 = 0x1004;
    pub const GET_STORAGE_INFO: u16 = 0x1005;
    pub const GET_NUM_OBJECTS: u16 = 0x1006;
    pub const GET_OBJECT_HANDLES: u16 = 0x1007;
    pub const GET_OBJECT_INFO: u16 = 0x1008;
    pub const GET_OBJECT: u16 = 0x1009;
    pub const DELETE_OBJECT: u16 = 0x100b;
    pub const SEND_OBJECT_INFO: u16 = 0x100c;
    pub const SEND_OBJECT: u16 = 0x100d;
    pub const SET_OBJECT_PROP_VALUE: u16 = 0x9804;
    pub const GET_DEVICE_PROP_DESC: u16 = 0x1014;
    pub const GET_DEVICE_PROP_VALUE: u16 = 0x1015;
    pub const GET_OBJECT_PROPS_SUPPORTED: u16 = 0x9801;
    pub const GET_OBJECT_PROP_DESC: u16 = 0x9802;
    pub const GET_OBJECT_PROP_VALUE: u16 = 0x9803;
    pub const GET_OBJECT_PROP_LIST: u16 = 0x9805;
    pub const GET_OBJECT_REFERENCES: u16 = 0x9810;
}

pub mod response {
    pub const OK: u16 = 0x2001;
    pub const GENERAL_ERROR: u16 = 0x2002;
    pub const SESSION_NOT_OPEN: u16 = 0x2003;
    pub const INVALID_TRANSACTION_ID: u16 = 0x2004;
    pub const OPERATION_NOT_SUPPORTED: u16 = 0x2005;
    pub const INCOMPLETE_TRANSFER: u16 = 0x2007;
    pub const INVALID_STORAGE_ID: u16 = 0x2008;
    pub const INVALID_OBJECT_HANDLE: u16 = 0x2009;
    pub const STORE_FULL: u16 = 0x200c;
    pub const ACCESS_DENIED: u16 = 0x200f;
    pub const SESSION_ALREADY_OPEN: u16 = 0x201e;
    pub const INVALID_PARAMETER: u16 = 0x201d;
    pub const OBJECT_PROP_NOT_SUPPORTED: u16 = 0xa801;
}

/// Encode a response container, including up to five 32-bit response parameters.
pub fn encode_response(
    transaction_id: u32,
    code: u16,
    parameters: &[u32],
    out: &mut [u8],
) -> core::result::Result<usize, MtpError> {
    if parameters.len() > 5 {
        return Err(MtpError::TooManyParameters);
    }
    let length = CONTAINER_HEADER_LEN + parameters.len() * 4;
    if out.len() < length {
        return Err(MtpError::OutputBufferTooSmall);
    }
    ContainerHeader {
        length: length as u32,
        kind: ContainerType::Response,
        code,
        transaction_id,
    }
    .encode(out)?;
    for (index, value) in parameters.iter().enumerate() {
        let offset = CONTAINER_HEADER_LEN + index * 4;
        out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }
    Ok(length)
}

/// USB PTP transport endpoints and interface descriptor.
///
/// The class deliberately exposes packet-level endpoint access. MTP data
/// containers can be larger than one USB packet and must be streamed by the
/// protocol engine without allocating a whole-file RAM buffer.
pub struct MtpUsbClass<'a, B: usb_device::bus::UsbBus> {
    interface: InterfaceNumber,
    interface_string: StringIndex,
    bulk_out: EndpointOut<'a, B>,
    bulk_in: EndpointIn<'a, B>,
    event_in: EndpointIn<'a, B>,
    tx: [u8; 4096],
    tx_len: usize,
    tx_offset: usize,
    tx_chunk_len: usize,
    tx_pending: bool,
    tx_in_flight: bool,
    tx_is_response: bool,
    tx_is_final_chunk: bool,
    get_object_tx_active: bool,
    get_object_poll_count: u32,
    get_object_submit_poll: u32,
    get_object_submit_us: u32,
    // MTP commands are serialized: retain one OUT command that arrives while
    // the current transaction's data/response is still being transmitted.
    transaction_active: bool,
    data_transfer_active: bool,
    pending_command: [u8; BULK_MAX_PACKET_SIZE as usize],
    pending_command_len: usize,
    readme: [u8; 192],
    readme_len: usize,
    pending_response: Option<(u16, u32)>,
    pending_get_object: Option<(u32, u32)>,
    pending_delete_object: Option<(u32, u32)>,
    awaiting_set_object_prop_value: Option<(u32, u32)>,
    pending_rename_object: Option<(u32, u32, [u8; 64], u8)>,
    awaiting_send_object_info: Option<u32>,
    pending_send_object_info: Option<(u32, UploadObjectInfo)>,
    upload_info_accepted: bool,
    upload_expected_size: usize,
    awaiting_send_object: Option<u32>,
    upload_data_active: bool,
    upload_received: usize,
    pending_upload_data: bool,
    pending_upload_final: bool,
    pending_upload_abort: bool,
    upload_data_transaction: u32,
    upload_data_len: usize,
    upload_data: [u8; BULK_MAX_PACKET_SIZE as usize],
    session_open: bool,
    objects: [MtpObject; MTP_OBJECT_CAPACITY],
    object_count: usize,
    capacity_bytes: u64,
    free_bytes: u64,
}

impl<'a, B: usb_device::bus::UsbBus> MtpUsbClass<'a, B> {
    pub fn new(allocator: &'a UsbBusAllocator<B>) -> Self {
        let interface = allocator.interface();
        let interface_string = allocator.string();
        let bulk_out = allocator
            .alloc(None, EndpointType::Bulk, BULK_MAX_PACKET_SIZE, 0)
            .expect("MTP bulk OUT endpoint allocation failed");
        let bulk_in = allocator
            .alloc(None, EndpointType::Bulk, BULK_MAX_PACKET_SIZE, 0)
            .expect("MTP bulk IN endpoint allocation failed");
        let event_in = allocator
            .alloc(None, EndpointType::Interrupt, EVENT_MAX_PACKET_SIZE, EVENT_INTERVAL)
            .expect("MTP event IN endpoint allocation failed");

        Self {
            interface,
            interface_string,
            bulk_out,
            bulk_in,
            event_in,
            tx: [0; 4096],
            tx_len: 0,
            tx_offset: 0,
            tx_chunk_len: 0,
            tx_pending: false,
            tx_in_flight: false,
            tx_is_response: false,
            tx_is_final_chunk: false,
            get_object_tx_active: false,
            get_object_poll_count: 0,
            get_object_submit_poll: 0,
            get_object_submit_us: 0,
            transaction_active: false,
            data_transfer_active: false,
            pending_command: [0; BULK_MAX_PACKET_SIZE as usize],
            pending_command_len: 0,
            readme: { let mut bytes = [0u8; 192]; bytes[..DEFAULT_README.len()].copy_from_slice(DEFAULT_README); bytes },
            readme_len: DEFAULT_README.len(),
            pending_response: None,
            pending_get_object: None,
            pending_delete_object: None,
            awaiting_set_object_prop_value: None,
            pending_rename_object: None,
            awaiting_send_object_info: None,
            pending_send_object_info: None,
            upload_info_accepted: false,
            upload_expected_size: 0,
            awaiting_send_object: None,
            upload_data_active: false,
            upload_received: 0,
            pending_upload_data: false,
            pending_upload_final: false,
            pending_upload_abort: false,
            upload_data_transaction: 0,
            upload_data_len: 0,
            upload_data: [0; BULK_MAX_PACKET_SIZE as usize],
            session_open: false,
            objects: [MtpObject::EMPTY; MTP_OBJECT_CAPACITY],
            object_count: 0,
            capacity_bytes: 0,
            free_bytes: 0,
        }
    }

    /// Replace the read-only README.txt contents before USB polling.
    pub fn set_readme(&mut self, contents: &[u8]) -> bool {
        if contents.len() > self.readme.len() { return false; }
        self.readme[..contents.len()].copy_from_slice(contents);
        self.readme_len = contents.len();
        true
    }

    /// Replace the bounded root-directory snapshot. Handles remain stable until refreshed.
    pub fn set_objects(&mut self, objects: &[MtpObject]) {
        self.objects = [MtpObject::EMPTY; MTP_OBJECT_CAPACITY];
        self.object_count = objects.len().min(self.objects.len());
        self.objects[..self.object_count].copy_from_slice(&objects[..self.object_count]);
    }

    pub fn set_storage_stats(&mut self, capacity_bytes: u64, free_bytes: u64) {
        let free_bytes = free_bytes.min(capacity_bytes);
        self.capacity_bytes = capacity_bytes;
        self.free_bytes = free_bytes;
    }

    /// Take a GET_OBJECT request for processing in the main context.
    pub fn take_get_object_request(&mut self) -> Option<(u32, u32)> {
        self.pending_get_object.take()
    }

    /// Take a DELETE_OBJECT request for processing by LittleFS in main context.
    pub fn take_delete_object_request(&mut self) -> Option<(u32, u32)> {
        self.pending_delete_object.take()
    }

    /// Take a validated rename request for main-context filesystem work.
    pub fn take_rename_object_request(&mut self) -> Option<(u32, u32, [u8; 64], u8)> {
        self.pending_rename_object.take()
    }

    /// Take a validated SendObjectInfo request for main-context filesystem work.
    pub fn take_send_object_info_request(&mut self) -> Option<(u32, UploadObjectInfo)> {
        self.pending_send_object_info.take()
    }

    /// Mark a new destination file as safely created in LittleFS.
    pub fn accept_upload_info(&mut self, info: UploadObjectInfo) {
        diagnostics::UPLOAD_INFO_ACCEPTED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        self.upload_expected_size = info.size as usize;
        self.upload_info_accepted = true;
    }

    /// Consume a pending upload-abort notification in main context.
    pub fn take_upload_abort(&mut self) -> bool {
        let pending = self.pending_upload_abort;
        self.pending_upload_abort = false;
        pending
    }

    /// Copy one received SendObject packet payload into main context.
    /// The USB OUT endpoint remains back-pressured until this slot is consumed.
    pub fn take_upload_data(&mut self, out: &mut [u8; BULK_MAX_PACKET_SIZE as usize]) -> Option<(u32, usize, bool)> {
        if !self.pending_upload_data { return None; }
        let len = self.upload_data_len;
        out[..len].copy_from_slice(&self.upload_data[..len]);
        let final_packet = self.pending_upload_final;
        self.pending_upload_data = false;
        self.pending_upload_final = false;
        Some((self.upload_data_transaction, len, final_packet))
    }

    /// Clear upload state after finalization or an error.
    pub fn finish_upload(&mut self) {
        self.upload_info_accepted = false;
        self.upload_expected_size = 0;
        self.awaiting_send_object = None;
        self.upload_data_active = false;
        self.upload_received = 0;
        self.pending_upload_data = false;
        self.pending_upload_final = false;
        self.upload_data_len = 0;
    }

    /// Queue a response with parameters from the main context.
    pub fn queue_main_response_with_params(&mut self, code: u16, transaction_id: u32, params: &[u32]) -> bool {
        if !self.object_tx_ready() { return false; }
        self.data_transfer_active = false;
        self.queue_response_with_params(code, transaction_id, params);
        true
    }

    /// True when the previous bulk-IN packet has completed and another chunk
    /// can be queued by the main context.
    pub fn object_tx_ready(&self) -> bool {
        !self.tx_pending && !self.tx_in_flight
    }

    /// Snapshot the bulk-IN state for persistent GET_OBJECT diagnostics.
    /// The caller must exclude the USB polling interrupt while reading it.
    pub fn transport_trace_state(&self) -> [u32; 8] {
        [
            self.tx_len as u32,
            self.tx_offset as u32,
            self.tx_chunk_len as u32,
            self.tx_pending as u32,
            self.tx_in_flight as u32,
            self.tx_is_response as u32,
            self.data_transfer_active as u32,
            self.pending_response.is_some() as u32,
        ]
    }

    /// Queue one piece of a single MTP GetObject data container. The header is
    /// included only in the first chunk; the final chunk schedules the response.
    pub fn queue_object_data_chunk(
        &mut self,
        transaction_id: u32,
        total_size: u32,
        bytes: &[u8],
        first: bool,
        last: bool,
    ) -> bool {
        let header_len = if first { CONTAINER_HEADER_LEN } else { 0 };
        if bytes.len() + header_len > self.tx.len() || !self.object_tx_ready() {
            return false;
        }
        if first {
            use core::sync::atomic::Ordering;
            diagnostics::GET_OBJECT_USB_SUBMITTED.store(0, Ordering::Relaxed);
            diagnostics::GET_OBJECT_USB_COMPLETED.store(0, Ordering::Relaxed);
            diagnostics::GET_OBJECT_USB_BYTES.store(0, Ordering::Relaxed);
            diagnostics::GET_OBJECT_RESPONSE_COMPLETED.store(0, Ordering::Relaxed);
            diagnostics::GET_OBJECT_USB_WOULDBLOCK.store(0, Ordering::Relaxed);
            diagnostics::GET_OBJECT_USB_POLL_LATENCY.store(0, Ordering::Relaxed);
            diagnostics::GET_OBJECT_USB_LATENCY_US.store(0, Ordering::Relaxed);
            diagnostics::GET_OBJECT_USB_LATENCY_MAX_US.store(0, Ordering::Relaxed);
            diagnostics::GET_OBJECT_FINAL_CHUNK_LEN.store(0, Ordering::Relaxed);
            diagnostics::GET_OBJECT_FINAL_CHUNK_ATTEMPTS.store(0, Ordering::Relaxed);
            diagnostics::GET_OBJECT_FINAL_CHUNK_SUBMITTED.store(0, Ordering::Relaxed);
            diagnostics::GET_OBJECT_FINAL_CHUNK_COMPLETED.store(0, Ordering::Relaxed);
            self.get_object_tx_active = true;
            let length = CONTAINER_HEADER_LEN as u32 + total_size;
            self.tx[..4].copy_from_slice(&length.to_le_bytes());
            self.tx[4..6].copy_from_slice(&(ContainerType::Data as u16).to_le_bytes());
            self.tx[6..8].copy_from_slice(&operation::GET_OBJECT.to_le_bytes());
            self.tx[8..12].copy_from_slice(&transaction_id.to_le_bytes());
        }
        self.tx[header_len..header_len + bytes.len()].copy_from_slice(bytes);
        self.tx_len = header_len + bytes.len();
        self.tx_pending = true;
        self.tx_is_response = false;
        self.tx_is_final_chunk = last;
        if last {
            diagnostics::GET_OBJECT_FINAL_CHUNK_LEN.store(bytes.len() as u32, core::sync::atomic::Ordering::Relaxed);
        }
        if first {
            self.data_transfer_active = true;
        }
        if last {
            self.pending_response = Some((response::OK, transaction_id));
        }
        true
    }

    /// Queue a complete small GetObject data container using the same
    /// single-transfer path as the other MTP data responses.
    pub fn queue_object_data(&mut self, transaction_id: u32, payload: &[u8]) -> bool {
        if payload.len() + CONTAINER_HEADER_LEN > self.tx.len() || !self.object_tx_ready() {
            return false;
        }
        self.queue_data(operation::GET_OBJECT, transaction_id, payload, response::OK);
        true
    }

    /// Queue a response from the main context after a storage-side failure.
    pub fn queue_main_response(&mut self, code: u16, transaction_id: u32) -> bool {
        if !self.object_tx_ready() {
            return false;
        }
        self.data_transfer_active = false;
        self.queue_response(code, transaction_id);
        true
    }

    fn object(&self, handle: u32) -> Option<&MtpObject> {
        self.objects[..self.object_count].iter().find(|object| object.handle == handle)
    }

    pub fn bulk_out(&self) -> &EndpointOut<'a, B> { &self.bulk_out }
    pub fn bulk_in(&self) -> &EndpointIn<'a, B> { &self.bulk_in }
    pub fn event_in(&self) -> &EndpointIn<'a, B> { &self.event_in }
    pub fn interface(&self) -> InterfaceNumber { self.interface }

    /// Clear protocol state after the Still Image class Device Reset request.
    /// This does not reset the USB peripheral or increment the USB bus-reset counter.
    fn reset_protocol_state(&mut self) {
        self.clear_transport_stalls();
        self.session_open = false;
        self.pending_get_object = None;
        if self.upload_info_accepted || self.awaiting_send_object.is_some()
            || self.awaiting_send_object_info.is_some() || self.upload_data_active
            || self.pending_upload_data
        {
            self.pending_upload_abort = true;
        }
        self.pending_delete_object = None;
        self.awaiting_set_object_prop_value = None;
        self.pending_rename_object = None;
        self.awaiting_send_object_info = None;
        self.pending_send_object_info = None;
        self.awaiting_send_object = None;
        self.upload_data_active = false;
        self.upload_received = 0;
        self.pending_upload_data = false;
        self.pending_upload_final = false;
        self.upload_info_accepted = false;
        self.upload_expected_size = 0;
        self.upload_data_len = 0;
        self.pending_response = None;
        self.pending_command_len = 0;
        self.transaction_active = false;
        self.data_transfer_active = false;
        self.tx_len = 0;
        self.tx_offset = 0;
        self.tx_chunk_len = 0;
        self.tx_pending = false;
        self.tx_in_flight = false;
        self.tx_is_response = false;
        self.tx_is_final_chunk = false;
    }

    /// Clear endpoint stalls after a PTP Cancel/Reset recovery sequence.
    pub fn clear_transport_stalls(&self) {
        self.bulk_in.unstall();
        self.bulk_out.unstall();
    }
}


impl<'a, B: usb_device::bus::UsbBus> MtpUsbClass<'a, B> {
    fn append_u16(out: &mut [u8], pos: &mut usize, value: u16) {
        out[*pos..*pos + 2].copy_from_slice(&value.to_le_bytes());
        *pos += 2;
    }

    fn append_u32(out: &mut [u8], pos: &mut usize, value: u32) {
        out[*pos..*pos + 4].copy_from_slice(&value.to_le_bytes());
        *pos += 4;
    }

    fn append_ptp_string(out: &mut [u8], pos: &mut usize, value: &str) {
        let units = value.encode_utf16().count();
        out[*pos] = (units + 1) as u8;
        *pos += 1;
        for unit in value.encode_utf16() {
            out[*pos..*pos + 2].copy_from_slice(&unit.to_le_bytes());
            *pos += 2;
        }
        out[*pos..*pos + 2].copy_from_slice(&0u16.to_le_bytes());
        *pos += 2;
    }

    fn queue_data(&mut self, code: u16, transaction_id: u32, payload: &[u8], response_code: u16) {
        let length = CONTAINER_HEADER_LEN + payload.len();
        self.tx[..4].copy_from_slice(&(length as u32).to_le_bytes());
        self.tx[4..6].copy_from_slice(&(ContainerType::Data as u16).to_le_bytes());
        self.tx[6..8].copy_from_slice(&code.to_le_bytes());
        self.tx[8..12].copy_from_slice(&transaction_id.to_le_bytes());
        self.tx[12..length].copy_from_slice(payload);
        self.tx_len = length;
        self.tx_offset = 0;
        self.tx_chunk_len = 0;
        self.tx_pending = true;
        self.tx_is_response = false;
        self.pending_response = Some((response_code, transaction_id));
        self.data_transfer_active = true;
    }

    fn queue_response(&mut self, code: u16, transaction_id: u32) {
        self.queue_response_with_params(code, transaction_id, &[]);
    }

    fn queue_response_with_params(&mut self, code: u16, transaction_id: u32, params: &[u32]) {
        diagnostics::LAST_RESPONSE_CODE.store(code as u32, core::sync::atomic::Ordering::Relaxed);
        diagnostics::LAST_RESPONSE_TXID.store(transaction_id, core::sync::atomic::Ordering::Relaxed);
        self.pending_response = None;
        if let Ok(length) = encode_response(transaction_id, code, params, &mut self.tx) {
            self.tx_len = length;
            self.tx_offset = 0;
            self.tx_chunk_len = 0;
            self.tx_pending = true;
            self.tx_is_response = true;
        } else {
            self.tx_len = 0;
            self.tx_offset = 0;
            self.tx_chunk_len = 0;
            self.tx_is_response = true;
        }
    }

    /// Submit queued MTP data only when the USB IN endpoint accepts it.
    /// A WouldBlock is normal while the previous packet is still in flight;
    /// keep the packet queued and retry on the next USB poll.
    fn transmit_pending(&mut self) {
        if !self.tx_pending || self.tx_in_flight || self.tx_len == 0 {
            return;
        }
        let remaining = self.tx_len.saturating_sub(self.tx_offset);
        let chunk_len = remaining.min(BULK_MAX_PACKET_SIZE as usize);
        if self.tx_is_final_chunk {
            diagnostics::GET_OBJECT_FINAL_CHUNK_ATTEMPTS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
        match self.bulk_in.write(&self.tx[self.tx_offset..self.tx_offset + chunk_len]) {
            Ok(length) if length == chunk_len => {
                diagnostics::IN_SUBMIT_OK.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                if self.get_object_tx_active && !self.tx_is_response {
                    diagnostics::GET_OBJECT_USB_SUBMITTED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                    diagnostics::GET_OBJECT_USB_BYTES.fetch_add(chunk_len as u32, core::sync::atomic::Ordering::Relaxed);
                    self.get_object_submit_poll = self.get_object_poll_count;
                    self.get_object_submit_us = read_tim5_us();
                }
                if self.tx_is_final_chunk {
                    diagnostics::GET_OBJECT_FINAL_CHUNK_SUBMITTED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                }
                self.tx_pending = false;
                self.tx_in_flight = true;
                self.tx_chunk_len = chunk_len;
            }
            Ok(_) | Err(usb_device::UsbError::WouldBlock) => {
                diagnostics::IN_SUBMIT_WOULDBLOCK.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                if self.get_object_tx_active && !self.tx_is_response {
                    diagnostics::GET_OBJECT_USB_WOULDBLOCK.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                }
            }
            Err(_) => {
                // Keep the packet queued. A later poll retries transient bus errors.
            }
        }
    }

    fn device_info(&mut self, transaction_id: u32) {
        let mut data = [0u8; 240];
        let mut pos = 0;
        Self::append_u16(&mut data, &mut pos, 100); // StandardVersion
        Self::append_u32(&mut data, &mut pos, 6); // Microsoft MTP vendor extension
        Self::append_u16(&mut data, &mut pos, 100);
        Self::append_ptp_string(&mut data, &mut pos, "microsoft.com: 1.0;");
        Self::append_u16(&mut data, &mut pos, 0); // FunctionalMode

        let operations: [u16; 16] = [
            operation::GET_DEVICE_INFO,
            operation::OPEN_SESSION,
            operation::CLOSE_SESSION,
            operation::GET_STORAGE_IDS,
            operation::GET_STORAGE_INFO,
            operation::GET_NUM_OBJECTS,
            operation::GET_OBJECT_HANDLES,
            operation::GET_OBJECT_INFO,
            operation::GET_OBJECT,
            operation::DELETE_OBJECT,
            operation::SEND_OBJECT_INFO,
            operation::SEND_OBJECT,
            operation::GET_OBJECT_PROPS_SUPPORTED,
            operation::GET_OBJECT_PROP_DESC,
            operation::GET_OBJECT_PROP_LIST,
            operation::SET_OBJECT_PROP_VALUE,
        ];
        Self::append_u32(&mut data, &mut pos, operations.len() as u32);
        for value in operations { Self::append_u16(&mut data, &mut pos, value); }

        Self::append_u32(&mut data, &mut pos, 0); // EventsSupported
        Self::append_u32(&mut data, &mut pos, 0); // DevicePropertiesSupported
        Self::append_u32(&mut data, &mut pos, 0); // CaptureFormats
        Self::append_u32(&mut data, &mut pos, 2); // PlaybackFormats
        Self::append_u16(&mut data, &mut pos, 0x3000); // Undefined
        Self::append_u16(&mut data, &mut pos, 0x3004); // Text

        Self::append_ptp_string(&mut data, &mut pos, "OpenAI XCore407I");
        Self::append_ptp_string(&mut data, &mut pos, "XCore407I");
        Self::append_ptp_string(&mut data, &mut pos, "0.1");
        Self::append_ptp_string(&mut data, &mut pos, "XCORE407I-002");

        self.queue_data(operation::GET_DEVICE_INFO, transaction_id, &data[..pos], response::OK);
    }


    fn storage_info(&mut self, transaction_id: u32, storage_id: u32) {
        if storage_id != PLACEHOLDER_STORAGE_ID {
            self.queue_response(response::INVALID_STORAGE_ID, transaction_id);
            return;
        }
        // Fixed RAM-backed hierarchical filesystem with read/write access.
        let mut data = [0u8; 96];
        let mut pos = 0;
        Self::append_u16(&mut data, &mut pos, 0x0003); // Fixed RAM
        Self::append_u16(&mut data, &mut pos, 0x0002); // Hierarchical filesystem
        Self::append_u16(&mut data, &mut pos, 0x0000); // Read/write
        Self::append_u64(&mut data, &mut pos, self.capacity_bytes);
        Self::append_u64(&mut data, &mut pos, self.free_bytes);
        // FreeSpaceInObjects is unknown: zero would tell Windows that the
        // storage has no remaining object slots, which can make its capacity
        // indicator appear full even when FreeSpaceInBytes is healthy.
        Self::append_u32(&mut data, &mut pos, 0xffff_ffff);
        Self::append_ptp_string(&mut data, &mut pos, "Data");
        Self::append_ptp_string(&mut data, &mut pos, "XCORE407I");
        self.queue_data(operation::GET_STORAGE_INFO, transaction_id, &data[..pos], response::OK);
    }

    fn append_u64(out: &mut [u8], pos: &mut usize, value: u64) {
        out[*pos..*pos + 8].copy_from_slice(&value.to_le_bytes());
        *pos += 8;
    }

    fn append_u128(out: &mut [u8], pos: &mut usize, value: &[u8; 16]) {
        out[*pos..*pos + 16].copy_from_slice(value);
        *pos += 16;
    }

    fn object_info(&mut self, transaction_id: u32, handle: u32) {
        let Some(object) = self.object(handle).copied() else {
            self.queue_response(response::INVALID_OBJECT_HANDLE, transaction_id);
            return;
        };
        let mut data = [0u8; 192];
        let mut pos = 0;
        Self::append_u32(&mut data, &mut pos, PLACEHOLDER_STORAGE_ID);
        Self::append_u16(&mut data, &mut pos, if object.is_dir { 0x3001 } else { 0x3000 });
        Self::append_u16(&mut data, &mut pos, 0); // Read/write
        Self::append_u32(&mut data, &mut pos, object.size);
        Self::append_u16(&mut data, &mut pos, 0);
        Self::append_u32(&mut data, &mut pos, 0);
        Self::append_u32(&mut data, &mut pos, 0);
        Self::append_u32(&mut data, &mut pos, 0);
        Self::append_u32(&mut data, &mut pos, 0);
        Self::append_u32(&mut data, &mut pos, 0);
        Self::append_u32(&mut data, &mut pos, 0);
        Self::append_u32(&mut data, &mut pos, 0); // ParentObject: root-level object
        Self::append_u16(&mut data, &mut pos, if object.is_dir { 1 } else { 0 });
        Self::append_u32(&mut data, &mut pos, 0);
        Self::append_u32(&mut data, &mut pos, 0);
        Self::append_ptp_string(&mut data, &mut pos, object.name());
        Self::append_ptp_string(&mut data, &mut pos, "");
        Self::append_ptp_string(&mut data, &mut pos, "");
        Self::append_ptp_string(&mut data, &mut pos, "");
        self.queue_data(operation::GET_OBJECT_INFO, transaction_id, &data[..pos], response::OK);
    }

    fn object_props_supported(&mut self, transaction_id: u32, format: u32) {
        if format != 0 && format != 0x3000 && format != 0x3001 {
            // GetObjectPropsSupported always has a data phase. An unsupported
            // format is represented by an empty property-code array.
            self.queue_data(
                operation::GET_OBJECT_PROPS_SUPPORTED,
                transaction_id,
                &[0, 0, 0, 0],
                response::OK,
            );
            return;
        }
        // Required core properties plus the filesystem name and parent. Name
        // (DC44) is distinct from ObjectFileName (DC07), but both map to the
        // same root-level LittleFS filename for these generic objects.
        // DateModified is omitted because no timestamp is available.
        let props: [u16; 8] = [0xdc01, 0xdc02, 0xdc03, 0xdc04, 0xdc07, 0xdc0b, 0xdc41, 0xdc44];
        let mut data = [0u8; 32];
        data[..4].copy_from_slice(&(props.len() as u32).to_le_bytes());
        for (i, prop) in props.iter().enumerate() {
            let at = 4 + i * 2;
            data[at..at + 2].copy_from_slice(&prop.to_le_bytes());
        }
        self.queue_data(operation::GET_OBJECT_PROPS_SUPPORTED, transaction_id, &data[..20], response::OK);
    }

    fn object_prop_desc(&mut self, transaction_id: u32, property: u32, format: u32) {
        if format != 0 && format != 0x3000 && format != 0x3001 {
            self.queue_response(response::OBJECT_PROP_NOT_SUPPORTED, transaction_id);
            return;
        }

        // Keep descriptors consistent with GetObjectPropsSupported and
        // GetObjectPropList. Both filename properties are writable aliases.
        let (data_type, writable) = match property {
            0xdc01 | 0xdc0b => (0x0006, false), // StorageID, ParentObject: UINT32
            0xdc02 | 0xdc03 => (0x0004, false), // ObjectFormat, ProtectionStatus: UINT16
            0xdc04 => (0x0008, false),          // ObjectSize: UINT64
            0xdc41 => (0x000a, false),          // PersistentUniqueObjectIdentifier: UINT128
            0xdc07 | 0xdc44 => (0xffff, true),   // ObjectFileName / Name: STRING
            _ => {
                self.queue_response(response::OBJECT_PROP_NOT_SUPPORTED, transaction_id);
                return;
            }
        };

        let mut data = [0u8; 32];
        let mut pos = 0;
        Self::append_u16(&mut data, &mut pos, property as u16);
        Self::append_u16(&mut data, &mut pos, data_type);
        data[pos] = if writable { 1 } else { 0 }; // GetSet
        pos += 1;
        match data_type {
            0x0004 => Self::append_u16(&mut data, &mut pos, 0),
            0x0006 => Self::append_u32(&mut data, &mut pos, 0),
            0x0008 => Self::append_u64(&mut data, &mut pos, 0),
            0x000a => { data[pos..pos + 16].fill(0); pos += 16; }
            0xffff => {
                // Empty PTP STRING: zero UTF-16 code units.
                data[pos] = 0;
                pos += 1;
            }
            _ => unreachable!(),
        }
        Self::append_u32(&mut data, &mut pos, 0); // GroupCode
        data[pos] = 0; // FormFlag: none
        pos += 1;
        self.queue_data(operation::GET_OBJECT_PROP_DESC, transaction_id, &data[..pos], response::OK);
    }

    fn append_prop_u16(out: &mut [u8], pos: &mut usize, handle: u32, code: u16, value: u16) {
        Self::append_u32(out, pos, handle);
        Self::append_u16(out, pos, code);
        Self::append_u16(out, pos, 0x0004); // UINT16
        Self::append_u16(out, pos, value);
    }

    fn append_prop_u32(out: &mut [u8], pos: &mut usize, handle: u32, code: u16, value: u32) {
        Self::append_u32(out, pos, handle);
        Self::append_u16(out, pos, code);
        Self::append_u16(out, pos, 0x0006); // UINT32
        Self::append_u32(out, pos, value);
    }

    fn append_prop_u64(out: &mut [u8], pos: &mut usize, handle: u32, code: u16, value: u64) {
        Self::append_u32(out, pos, handle);
        Self::append_u16(out, pos, code);
        Self::append_u16(out, pos, 0x0008); // UINT64
        Self::append_u64(out, pos, value);
    }

    fn append_prop_u128(out: &mut [u8], pos: &mut usize, handle: u32, code: u16, value: &[u8; 16]) {
        Self::append_u32(out, pos, handle);
        Self::append_u16(out, pos, code);
        Self::append_u16(out, pos, 0x000a); // UINT128
        Self::append_u128(out, pos, value);
    }

    fn append_prop_string(out: &mut [u8], pos: &mut usize, handle: u32, code: u16, value: &str) {
        Self::append_u32(out, pos, handle);
        Self::append_u16(out, pos, code);
        Self::append_u16(out, pos, 0xffff); // STRING
        Self::append_ptp_string(out, pos, value);
    }

    fn object_prop_list(&mut self, transaction_id: u32, handle: u32, format_code: u32, property_code: u32) {
        // ObjectHandle 0xffffffff requests every object. The current response
        // builder is bounded to one 4096-byte transaction buffer, so do not
        // misrepresent that request by returning only the first object's data.
        if handle == 0xffff_ffff {
            self.queue_response(response::OPERATION_NOT_SUPPORTED, transaction_id);
            return;
        }
        let Some(object) = self.object(handle).copied() else {
            self.queue_response(response::INVALID_OBJECT_HANDLE, transaction_id);
            return;
        };

        // FormatCode == 0 means any format. A nonzero format restricts
        // the result to objects of that format.
        let object_format = if object.is_dir { 0x3001 } else { 0x3000 };
        if format_code != 0 && format_code != object_format {
            self.queue_data(operation::GET_OBJECT_PROP_LIST, transaction_id, &[0, 0, 0, 0], response::OK);
            return;
        }

        // PropertyCode == 0 requests all properties. Otherwise return only
        // the requested property, as required by GetObjectPropList.
        let include_storage_id = property_code == 0 || property_code == 0xdc01;
        let include_format = property_code == 0 || property_code == 0xdc02;
        let include_protection = property_code == 0 || property_code == 0xdc03;
        let include_size = property_code == 0 || property_code == 0xdc04;
        let include_filename = property_code == 0 || property_code == 0xdc07;
        let include_parent = property_code == 0 || property_code == 0xdc0b;
        let include_puid = property_code == 0 || property_code == 0xdc41;
        let include_name = property_code == 0 || property_code == 0xdc44;

        if !(include_storage_id || include_format || include_protection || include_size
            || include_filename || include_parent || include_puid || include_name)
        {
            self.queue_response(response::OBJECT_PROP_NOT_SUPPORTED, transaction_id);
            return;
        }

        let count = include_storage_id as u32
            + include_format as u32
            + include_protection as u32
            + include_size as u32
            + include_filename as u32
            + include_parent as u32
            + include_puid as u32
            + include_name as u32;

        let mut data = [0u8; 512];
        let mut pos = 0;
        Self::append_u32(&mut data, &mut pos, count);
        let h = object.handle;
        if include_storage_id {
            Self::append_prop_u32(&mut data, &mut pos, h, 0xdc01, PLACEHOLDER_STORAGE_ID);
        }
        if include_format {
            Self::append_prop_u16(&mut data, &mut pos, h, 0xdc02, if object.is_dir { 0x3001 } else { 0x3000 });
        }
        if include_protection {
            Self::append_prop_u16(&mut data, &mut pos, h, 0xdc03, 0);
        }
        if include_size {
            Self::append_prop_u64(&mut data, &mut pos, h, 0xdc04, object.size as u64);
        }
        if include_filename {
            Self::append_prop_string(&mut data, &mut pos, h, 0xdc07, object.name());
        }
        if include_parent {
            Self::append_prop_u32(&mut data, &mut pos, h, 0xdc0b, 0); // Root-level object
        }
        if include_puid {
            Self::append_prop_u128(&mut data, &mut pos, h, 0xdc41, &object.puid);
        }
        if include_name {
            Self::append_prop_string(&mut data, &mut pos, h, 0xdc44, object.name());
        }
        self.queue_data(operation::GET_OBJECT_PROP_LIST, transaction_id, &data[..pos], response::OK);
    }

    fn handle_command(&mut self, packet: &[u8]) {
        let Ok(header) = ContainerHeader::decode(packet) else { return; };
        diagnostics::GET_OBJECT_LAST_COMMAND_CODE.store(header.code as u32, core::sync::atomic::Ordering::Relaxed);
        diagnostics::GET_OBJECT_LAST_COMMAND_TXID.store(header.transaction_id, core::sync::atomic::Ordering::Relaxed);
        diagnostics::LAST_COMMAND_PARAM0.store(
            if packet.len() >= CONTAINER_HEADER_LEN + 4 {
                u32::from_le_bytes(packet[CONTAINER_HEADER_LEN..CONTAINER_HEADER_LEN + 4].try_into().unwrap())
            } else { 0 },
            core::sync::atomic::Ordering::Relaxed,
        );
        if header.kind != ContainerType::Command
            || header.length as usize > packet.len()
            || header.length < CONTAINER_HEADER_LEN as u32
        {
            return;
        }
        let params = &packet[CONTAINER_HEADER_LEN..header.length as usize];
        let parameter = |index: usize| -> Option<u32> {
            let start = index * 4;
            if params.len() < start + 4 { None } else {
                Some(u32::from_le_bytes(params[start..start + 4].try_into().unwrap()))
            }
        };

        match header.code {
            operation::GET_DEVICE_INFO => self.device_info(header.transaction_id),
            operation::OPEN_SESSION => {
                if self.session_open {
                    self.queue_response(response::SESSION_ALREADY_OPEN, header.transaction_id);
                } else if parameter(0).unwrap_or(0) == 0 || params.len() != 4 {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                } else {
                    self.session_open = true;
                    self.queue_response(response::OK, header.transaction_id);
                }
            }
            operation::CLOSE_SESSION => {
                if !self.session_open {
                    self.queue_response(response::SESSION_NOT_OPEN, header.transaction_id);
                } else {
                    if self.upload_info_accepted || self.pending_send_object_info.is_some()
                        || self.awaiting_send_object_info.is_some() || self.awaiting_send_object.is_some()
                        || self.pending_upload_data
                    {
                        self.pending_upload_abort = true;
                        self.finish_upload();
                        self.pending_send_object_info = None;
                        self.awaiting_send_object_info = None;
                    }
                    self.session_open = false;
                    self.queue_response(response::OK, header.transaction_id);
                }
            }
            operation::GET_STORAGE_IDS => {
                if !self.session_open {
                    self.queue_response(response::SESSION_NOT_OPEN, header.transaction_id);
                } else {
                    let mut ids = [0u8; 8];
                    ids[..4].copy_from_slice(&1u32.to_le_bytes());
                    ids[4..8].copy_from_slice(&PLACEHOLDER_STORAGE_ID.to_le_bytes());
                    self.queue_data(operation::GET_STORAGE_IDS, header.transaction_id, &ids, response::OK);
                }
            }
            operation::GET_STORAGE_INFO => {
                if !self.session_open {
                    self.queue_response(response::SESSION_NOT_OPEN, header.transaction_id);
                } else if let Some(storage_id) = parameter(0) {
                    self.storage_info(header.transaction_id, storage_id);
                } else {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                }
            }
            operation::GET_NUM_OBJECTS | operation::GET_OBJECT_HANDLES => {
                if !self.session_open {
                    self.queue_response(response::SESSION_NOT_OPEN, header.transaction_id);
                } else if parameter(0) != Some(PLACEHOLDER_STORAGE_ID)
                    && parameter(0) != Some(0xffff_ffff)
                {
                    self.queue_response(response::INVALID_STORAGE_ID, header.transaction_id);
                } else {
                    // ObjectFormatCode is optional; zero (or omission) means
                    // all formats. This implementation exposes files (0x3000)
                    // and directories (0x3001).
                    let format = parameter(1).unwrap_or(0);
                    if format != 0 && format != 0x3000 && format != 0x3001 {
                        if header.code == operation::GET_NUM_OBJECTS {
                            self.queue_response_with_params(response::OK, header.transaction_id, &[0]);
                        } else {
                            self.queue_data(header.code, header.transaction_id, &[0, 0, 0, 0], response::OK);
                        }
                    } else {
                        let matching_count = self.objects[..self.object_count]
                            .iter()
                            .filter(|object| format == 0
                                || (object.is_dir && format == 0x3001)
                                || (!object.is_dir && format == 0x3000))
                            .count();

                        if header.code == operation::GET_NUM_OBJECTS {
                            // GetNumObjects returns the filtered count as a response parameter.
                            self.queue_response_with_params(response::OK, header.transaction_id, &[matching_count as u32]);
                        } else {
                            let mut handles = [0u8; 4 + MTP_OBJECT_CAPACITY * 4];
                            handles[..4].copy_from_slice(&(matching_count as u32).to_le_bytes());
                            let mut index = 0;
                            for object in self.objects[..self.object_count].iter() {
                                if format == 0
                                    || (object.is_dir && format == 0x3001)
                                    || (!object.is_dir && format == 0x3000)
                                {
                                    let at = 4 + index * 4;
                                    handles[at..at + 4].copy_from_slice(&object.handle.to_le_bytes());
                                    index += 1;
                                }
                            }
                            self.queue_data(header.code, header.transaction_id, &handles[..4 + matching_count * 4], response::OK);
                        }
                    }
                }
            }
            operation::GET_OBJECT_INFO => {
                if !self.session_open {
                    self.queue_response(response::SESSION_NOT_OPEN, header.transaction_id);
                } else if let Some(handle) = parameter(0) {
                    if params.len() == 4 {
                        self.object_info(header.transaction_id, handle);
                    } else {
                        self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                    }
                } else {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                }
            }
            operation::GET_OBJECT_PROPS_SUPPORTED => {
                diagnostics::GET_OBJECT_PROPS_SUPPORTED_COMMANDS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                if !self.session_open {
                    self.queue_response(response::SESSION_NOT_OPEN, header.transaction_id);
                } else if params.len() != 4 {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                } else if let Some(format) = parameter(0) {
                    self.object_props_supported(header.transaction_id, format);
                } else {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                }
            }
            operation::GET_OBJECT_PROP_DESC => {
                diagnostics::GET_OBJECT_PROP_DESC_COMMANDS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                if !self.session_open {
                    self.queue_response(response::SESSION_NOT_OPEN, header.transaction_id);
                } else if params.len() != 8 {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                } else if let (Some(property), Some(format)) = (parameter(0), parameter(1)) {
                    self.object_prop_desc(header.transaction_id, property, format);
                } else {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                }
            }
            operation::GET_OBJECT_PROP_LIST => {
                diagnostics::GET_OBJECT_PROP_LIST_COMMANDS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                if !self.session_open {
                    self.queue_response(response::SESSION_NOT_OPEN, header.transaction_id);
                } else if params.len() != 20 {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                } else if let (Some(handle), Some(format_code), Some(property_code), Some(group_code), Some(depth)) =
                    (parameter(0), parameter(1), parameter(2), parameter(3), parameter(4))
                {
                    // This implementation currently supports only the
                    // ungrouped, direct-object form. Do not silently ignore
                    // GroupCode or Depth and return a misleading property set.
                    if group_code != 0 || depth != 0 {
                        self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                    } else {
                        diagnostics::GET_OBJECT_PROP_LIST_LAST_FORMAT.store(format_code, core::sync::atomic::Ordering::Relaxed);
                        diagnostics::GET_OBJECT_PROP_LIST_LAST_PROPERTY.store(property_code, core::sync::atomic::Ordering::Relaxed);
                        self.object_prop_list(header.transaction_id, handle, format_code, property_code);
                    }
                } else {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                }
            }
            operation::SET_OBJECT_PROP_VALUE => {
                diagnostics::SET_OBJECT_PROP_VALUE_COMMANDS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                if !self.session_open {
                    self.queue_response(response::SESSION_NOT_OPEN, header.transaction_id);
                } else if params.len() != 8 {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                } else if let (Some(handle), Some(property)) = (parameter(0), parameter(1)) {
                    if property != 0xdc07 && property != 0xdc44 {
                        self.queue_response(response::OPERATION_NOT_SUPPORTED, header.transaction_id);
                    } else if self.object(handle).is_none() {
                        self.queue_response(response::INVALID_OBJECT_HANDLE, header.transaction_id);
                    } else if self.object(handle).map(|o| o.is_dir).unwrap_or(false) {
                        self.queue_response(response::ACCESS_DENIED, header.transaction_id);
                    } else if self.object(handle).map(|o| matches!(o.name(), "MTP-README.txt" | "MTP-STATUS.txt")).unwrap_or(false) {
                        self.queue_response(response::ACCESS_DENIED, header.transaction_id);
                    } else if self.awaiting_set_object_prop_value.is_some() || self.pending_rename_object.is_some() {
                        self.queue_response(response::GENERAL_ERROR, header.transaction_id);
                    } else {
                        self.awaiting_set_object_prop_value = Some((handle, header.transaction_id));
                    }
                } else {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                }
            }
            operation::SEND_OBJECT_INFO => {
                diagnostics::SEND_OBJECT_INFO_COMMANDS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                if !self.session_open {
                    self.queue_response(response::SESSION_NOT_OPEN, header.transaction_id);
                } else if params.len() != 8 {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                } else if parameter(0) != Some(PLACEHOLDER_STORAGE_ID)
                    || (parameter(1) != Some(0) && parameter(1) != Some(0xffff_ffff)) {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                } else if self.awaiting_send_object_info.is_some()
                    || self.pending_send_object_info.is_some()
                    || self.upload_info_accepted {
                    self.queue_response(response::GENERAL_ERROR, header.transaction_id);
                } else {
                    self.awaiting_send_object_info = Some(header.transaction_id);
                }
            }
            operation::SEND_OBJECT => {
                diagnostics::SEND_OBJECT_COMMANDS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                if !self.session_open {
                    self.queue_response(response::SESSION_NOT_OPEN, header.transaction_id);
                } else if !params.is_empty() {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                } else if !self.upload_info_accepted
                    || self.awaiting_send_object.is_some()
                    || self.pending_upload_data {
                    self.queue_response(response::GENERAL_ERROR, header.transaction_id);
                } else {
                    self.awaiting_send_object = Some(header.transaction_id);
                }
            }
            operation::DELETE_OBJECT => {
                if !self.session_open {
                    self.queue_response(response::SESSION_NOT_OPEN, header.transaction_id);
                } else if (params.len() != 4 && params.len() != 8) {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                } else if let Some(handle) = parameter(0) {
                    if self.object(handle).is_none() {
                        self.queue_response(response::INVALID_OBJECT_HANDLE, header.transaction_id);
                    } else if self.object(handle).map(|object| object.name()) == Some("MTP-README.txt")
                        || self.object(handle).map(|object| object.name()) == Some("MTP-STATUS.txt") {
                        self.queue_response(response::ACCESS_DENIED, header.transaction_id);
                    } else if self.pending_delete_object.is_some() {
                        self.queue_response(response::GENERAL_ERROR, header.transaction_id);
                    } else {
                        self.pending_delete_object = Some((handle, header.transaction_id));
                    }
                } else {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                }
            }
            operation::GET_OBJECT => {
                if !self.session_open {
                    self.queue_response(response::SESSION_NOT_OPEN, header.transaction_id);
                } else if params.len() != 4 {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                } else if let Some(handle) = parameter(0) {
                    if self.object(handle).is_none() {
                        self.queue_response(response::INVALID_OBJECT_HANDLE, header.transaction_id);
                    } else if self.pending_get_object.is_some() {
                        self.queue_response(response::GENERAL_ERROR, header.transaction_id);
                    } else if self.object(handle).map(|object| object.name()) == Some("MTP-STATUS.txt") {
                        // Snapshot counters directly into the transfer buffer;
                        // this is read-only and does not touch LittleFS.
                        let mut payload = [0u8; 1536];
                        let payload_len = diagnostics::write_status(&mut payload);
                        payload[payload_len..].fill(b' ');
                        // ObjectInfo advertises exactly 1536 bytes. Send the
                        // complete object, including padding, so the MTP data
                        // container length agrees with the advertised size.
                        self.queue_data(
                            operation::GET_OBJECT,
                            header.transaction_id,
                            &payload,
                            response::OK,
                        );
                    } else if self.object(handle).map(|object| object.name()) == Some("MTP-README.txt") {
                        // Keep one known-good, in-memory object on the original
                        // command/response path. This isolates USB MTP transport
                        // from the deferred LittleFS read path.
                        let mut payload = [0u8; 192];
                        let payload_len = self.readme_len;
                        payload[..payload_len].copy_from_slice(&self.readme[..payload_len]);
                        self.queue_data(
                            operation::GET_OBJECT,
                            header.transaction_id,
                            &payload[..payload_len],
                            response::OK,
                        );
                    } else {
                        self.pending_get_object = Some((handle, header.transaction_id));
                    }
                } else {
                    self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                }
            }
            _ => self.queue_response(response::OPERATION_NOT_SUPPORTED, header.transaction_id),
        }
    }
}

impl<B: usb_device::bus::UsbBus> UsbClass<B> for MtpUsbClass<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        writer.interface(self.interface, STILL_IMAGE_CLASS, PTP_SUBCLASS, PTP_PROTOCOL)?;
        writer.endpoint(&self.bulk_out)?;
        writer.endpoint(&self.bulk_in)?;
        writer.endpoint(&self.event_in)?;
        Ok(())
    }

    fn get_string(
        &self,
        index: StringIndex,
        _lang_id: usb_device::descriptor::lang_id::LangID,
    ) -> Option<&str> {
        if index == self.interface_string { Some("XCore407I MTP data") } else { None }
    }

    fn control_in(&mut self, xfer: usb_device::class::ControlIn<'_, '_, '_, B>) {
        let req = *xfer.request();
        if req.request_type != usb_device::control::RequestType::Class
            || req.recipient != usb_device::control::Recipient::Interface
            || req.index > u8::MAX as u16
            || u8::from(self.interface) != req.index as u8
            || req.value != 0
        {
            return;
        }

        // USB Still Image class request 0x67: Get Device Status.
        // The returned structure is { uint16 length, uint16 PTP response code }.
        if req.request == 0x67 && req.length >= 4 {
            diagnostics::CONTROL_GET_STATUS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            let code = if self.transaction_active
                || self.tx_pending
                || self.tx_in_flight
                || self.pending_get_object.is_some()
                || self.pending_delete_object.is_some()
                || self.pending_send_object_info.is_some()
                || self.awaiting_send_object_info.is_some()
                || self.upload_info_accepted
                || self.awaiting_send_object.is_some()
                || self.pending_upload_data
            {
                0x2019u16 // Device Busy
            } else {
                response::OK
            };
            let status = [4u8, 0, code as u8, (code >> 8) as u8];
            let _ = xfer.accept_with(&status);
        }
    }

    fn control_out(&mut self, xfer: usb_device::class::ControlOut<'_, '_, '_, B>) {
        let req = *xfer.request();
        if req.request_type != usb_device::control::RequestType::Class
            || req.recipient != usb_device::control::Recipient::Interface
            || req.index > u8::MAX as u16
            || u8::from(self.interface) != req.index as u8
            || req.value != 0
        {
            return;
        }

        // USB Still Image class request 0x66: Device Reset. This is a
        // protocol-level reset, not a USB bus reset.
        if req.request == 0x66 && req.length == 0 {
            if xfer.accept().is_ok() {
                diagnostics::CONTROL_DEVICE_RESET.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                self.reset_protocol_state();
            }
        }
    }

    fn endpoint_out(&mut self, address: usb_device::endpoint::EndpointAddress) {
        if address != self.bulk_out.address() { return; }
        // Do not consume another OUT packet until main context has copied and
        // persisted the previous packet. Leaving the endpoint unread applies
        // USB bulk NAK back-pressure instead of dropping data.
        if self.pending_upload_data { return; }

        let mut packet = [0u8; BULK_MAX_PACKET_SIZE as usize];
        if let Ok(length) = self.bulk_out.read(&mut packet) {
            diagnostics::OUT_PACKETS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);

            // Once the first SendObject packet has supplied the 12-byte MTP
            // header, subsequent USB packets are continuation payload only.
            // If upload metadata is still active but continuation mode has
            // unexpectedly cleared, count packets before trying to parse their
            // raw payload as a new MTP container.
            if self.upload_info_accepted
                && self.awaiting_send_object.is_none()
                && !self.upload_data_active
            {
                diagnostics::UPLOAD_ORPHAN_OUT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
            if self.upload_data_active {
                let next_received = self.upload_received.checked_add(length);
                if length == 0
                    || length < BULK_MAX_PACKET_SIZE as usize
                        && next_received != Some(self.upload_expected_size)
                    || next_received.map_or(true, |n| n > self.upload_expected_size)
                {
                    diagnostics::UPLOAD_DATA_REJECTED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                    let transaction_id = self.upload_data_transaction;
                    self.pending_upload_abort = true;
                    self.finish_upload();
                    self.queue_response(response::INCOMPLETE_TRANSFER, transaction_id);
                    return;
                }
                self.upload_data[..length].copy_from_slice(&packet[..length]);
                self.upload_data_len = length;
                self.upload_received += length;
                self.pending_upload_final = self.upload_received == self.upload_expected_size;
                self.upload_data_active = !self.pending_upload_final;
                self.pending_upload_data = true;
                return;
            }

            if length < CONTAINER_HEADER_LEN {
                diagnostics::OUT_INVALID.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                return;
            }
            let Ok(header) = ContainerHeader::decode(&packet[..length]) else {
                diagnostics::OUT_INVALID.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                return;
            };
            if header.length < CONTAINER_HEADER_LEN as u32 {
                diagnostics::OUT_INVALID.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                return;
            }

            if header.kind == ContainerType::Data {
                match header.code {
                    operation::SET_OBJECT_PROP_VALUE
                        if self.awaiting_set_object_prop_value.map(|(_, tx)| tx) == Some(header.transaction_id) =>
                    {
                        diagnostics::SET_OBJECT_PROP_VALUE_DATA.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                        if header.length as usize != length || length < CONTAINER_HEADER_LEN + 3 {
                            self.awaiting_set_object_prop_value = None;
                            self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                            return;
                        }
                        let data = &packet[CONTAINER_HEADER_LEN..length];
                        let count = data[0] as usize;
                        if count < 2 || data.len() != 1 + count * 2 || count > 65 {
                            self.awaiting_set_object_prop_value = None;
                            self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                            return;
                        }
                        let mut units = [0u16; 64];
                        let unit_count = count - 1;
                        let terminator = 1 + unit_count * 2;
                        if unit_count > units.len()
                            || u16::from_le_bytes([data[terminator], data[terminator + 1]]) != 0
                        {
                            self.awaiting_set_object_prop_value = None;
                            self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                            return;
                        }
                        for i in 0..unit_count {
                            let at = 1 + i * 2;
                            units[i] = u16::from_le_bytes([data[at], data[at + 1]]);
                        }
                        let mut name = [0u8; 64];
                        let Some(name_len) = decode_utf16_filename(&units[..unit_count], &mut name) else {
                            self.awaiting_set_object_prop_value = None;
                            self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                            return;
                        };
                        let (handle, _) = self.awaiting_set_object_prop_value.take().unwrap();
                        self.pending_rename_object = Some((handle, header.transaction_id, name, name_len as u8));
                    }
                    operation::SEND_OBJECT_INFO
                        if self.awaiting_send_object_info == Some(header.transaction_id) =>
                    {
                        if header.length as usize != length {
                            diagnostics::OUT_INVALID.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                            return;
                        }
                        diagnostics::SEND_OBJECT_INFO_DATA.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                        match UploadObjectInfo::decode(&packet[CONTAINER_HEADER_LEN..length]) {
                            Ok(info) => {
                                diagnostics::OBJECT_INFO_DECODE_OK.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                                self.pending_send_object_info = Some((header.transaction_id, info));
                                self.awaiting_send_object_info = None;
                            }
                            Err(_) => {
                                diagnostics::OBJECT_INFO_DECODE_FAIL.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                                self.awaiting_send_object_info = None;
                                self.queue_response(response::INVALID_PARAMETER, header.transaction_id);
                            }
                        }
                    }
                    operation::SEND_OBJECT
                        if self.awaiting_send_object == Some(header.transaction_id) =>
                    {
                        diagnostics::SEND_OBJECT_DATA.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                        let payload_len = length - CONTAINER_HEADER_LEN;
                        let total_payload = (header.length as usize).saturating_sub(CONTAINER_HEADER_LEN);
                        if total_payload == self.upload_expected_size
                            && payload_len <= total_payload
                            && header.length as usize >= length
                            && (length == BULK_MAX_PACKET_SIZE as usize
                                || header.length as usize == length)
                        {
                            self.upload_data_transaction = header.transaction_id;
                            self.upload_received = payload_len;
                            self.upload_data[..payload_len]
                                .copy_from_slice(&packet[CONTAINER_HEADER_LEN..length]);
                            self.upload_data_len = payload_len;
                            self.upload_data_active = self.upload_received < self.upload_expected_size;
                            self.pending_upload_final = !self.upload_data_active;
                            self.pending_upload_data = true;
                            self.awaiting_send_object = None;
                        } else {
                            diagnostics::UPLOAD_DATA_REJECTED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                            self.pending_upload_abort = true;
                            self.finish_upload();
                            self.queue_response(response::INCOMPLETE_TRANSFER, header.transaction_id);
                        }
                    }
                    _ => {
                        diagnostics::OUT_INVALID.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                    },
                }
                return;
            }
            if header.length as usize != length || header.kind != ContainerType::Command {
                diagnostics::OUT_INVALID.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                return;
            }
            if self.transaction_active {
                if self.pending_command_len == 0 && length <= self.pending_command.len() {
                    if self.get_object_tx_active {
                        diagnostics::GET_OBJECT_COMMANDS_WHILE_ACTIVE.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                    }
                    self.pending_command[..length].copy_from_slice(&packet[..length]);
                    self.pending_command_len = length;
                    diagnostics::COMMANDS_DEFERRED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                    if self.get_object_tx_active {
                        diagnostics::GET_OBJECT_NEXT_COMMAND_DEFERRED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                    }
                } else {
                    diagnostics::PENDING_COMMAND_OVERFLOW.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                }
            } else {
                self.transaction_active = true;
                diagnostics::COMMANDS_ACCEPTED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                self.handle_command(&packet[..length]);
            }
        }
    }

    fn endpoint_in_complete(&mut self, address: usb_device::endpoint::EndpointAddress) {
        if address != self.bulk_in.address() { return; }
        diagnostics::IN_COMPLETE.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        if self.get_object_tx_active && !self.tx_is_response {
            let elapsed_us = read_tim5_us().wrapping_sub(self.get_object_submit_us);
            diagnostics::GET_OBJECT_USB_LATENCY_US.fetch_add(
                elapsed_us,
                core::sync::atomic::Ordering::Relaxed,
            );
            let mut previous = diagnostics::GET_OBJECT_USB_LATENCY_MAX_US.load(
                core::sync::atomic::Ordering::Relaxed,
            );
            while elapsed_us > previous {
                match diagnostics::GET_OBJECT_USB_LATENCY_MAX_US.compare_exchange_weak(
                    previous,
                    elapsed_us,
                    core::sync::atomic::Ordering::Relaxed,
                    core::sync::atomic::Ordering::Relaxed,
                ) {
                    Ok(_) => break,
                    Err(actual) => previous = actual,
                }
            }
            diagnostics::GET_OBJECT_USB_POLL_LATENCY.fetch_add(
                self.get_object_poll_count.wrapping_sub(self.get_object_submit_poll),
                core::sync::atomic::Ordering::Relaxed,
            );
        }
        if self.tx_is_response {
            diagnostics::RESPONSE_IN_COMPLETED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            if self.get_object_tx_active {
                diagnostics::GET_OBJECT_RESPONSE_COMPLETED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                self.get_object_tx_active = false;
            }
        } else if self.get_object_tx_active {
            diagnostics::GET_OBJECT_USB_COMPLETED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
        if !self.tx_is_response && self.pending_response.is_some() {
            diagnostics::DATA_IN_COMPLETED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
        if self.tx_is_final_chunk {
            diagnostics::GET_OBJECT_FINAL_CHUNK_COMPLETED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
        self.tx_in_flight = false;
        self.tx_offset += self.tx_chunk_len;
        self.tx_chunk_len = 0;
        if self.tx_offset < self.tx_len {
            self.tx_pending = true;
            return;
        }
        self.tx_offset = 0;
        self.tx_len = 0;

        // A data container is followed by its MTP response. The transaction
        // remains active until that response has itself completed.
        if let Some((code, transaction_id)) = self.pending_response.take() {
            self.tx_is_final_chunk = false;
            if self.get_object_tx_active {
                diagnostics::GET_OBJECT_FINAL_DATA_COMPLETED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                diagnostics::GET_OBJECT_RESPONSE_QUEUED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
            self.data_transfer_active = false;
            self.queue_response(code, transaction_id);
            return;
        }

        // Non-final chunks complete without an MTP response. Keep the
        // transaction locked until the main loop queues the next chunk.
        if self.data_transfer_active {
            return;
        }

        self.transaction_active = false;

        // Process at most one deferred command, and only after the preceding
        // transaction is completely off the wire.
        if self.pending_command_len != 0 {
            let length = self.pending_command_len;
            let mut packet = [0u8; BULK_MAX_PACKET_SIZE as usize];
            packet[..length].copy_from_slice(&self.pending_command[..length]);
            self.pending_command_len = 0;
            self.transaction_active = true;
            if diagnostics::GET_OBJECT_LAST_COMMAND_CODE.load(core::sync::atomic::Ordering::Relaxed)
                == operation::GET_OBJECT as u32
            {
                diagnostics::GET_OBJECT_DEFERRED_COMMAND_PROCESSED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
            self.handle_command(&packet[..length]);
        }
    }

    fn poll(&mut self) {
        self.get_object_poll_count = self.get_object_poll_count.wrapping_add(1);
        self.transmit_pending();
    }

    fn reset(&mut self) {
        diagnostics::USB_RESETS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        self.reset_protocol_state();
    }
}

/// Minimal storage contract for the MTP object layer. A LittleFS-backed
/// implementation will map object handles to paths and stream file contents
/// through caller-provided buffers; it must never expose NAND sectors to MTP.
pub trait MtpStorage {
    fn storage_id(&self) -> u32;
    fn capacity_bytes(&self) -> u64;
    fn free_bytes(&self) -> u64;
    fn object_count(&self) -> u32;
    fn read_object(&mut self, handle: u32, offset: u64, out: &mut [u8]) -> core::result::Result<usize, MtpStorageError>;
    fn write_object(&mut self, handle: u32, offset: u64, data: &[u8]) -> core::result::Result<usize, MtpStorageError>;
    fn delete_object(&mut self, handle: u32) -> core::result::Result<(), MtpStorageError>;
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MtpStorageError {
    NotFound,
    ReadOnly,
    NoSpace,
    Io,
}


/// Validated root-level file metadata received in an MTP SendObjectInfo data
/// container. Names are decoded from UTF-16 into bounded UTF-8 and validated
/// before they are passed to the embedded filesystem.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct UploadObjectInfo {
    pub storage_id: u32,
    pub parent: u32,
    pub size: u32,
    pub name: [u8; 64],
    pub name_len: u8,
}

impl UploadObjectInfo {
    /// Parse the fixed fields and PTP UTF-16 string in an ObjectInfo dataset.
    /// This parser intentionally accepts only root-level regular files.
    pub fn decode(dataset: &[u8]) -> core::result::Result<Self, MtpError> {
        const FIXED_LEN: usize = 52;
        diagnostics::OBJ_LAST_DATASET_LEN.store(dataset.len() as u32, core::sync::atomic::Ordering::Relaxed);
        if dataset.len() < FIXED_LEN + 1 {
            diagnostics::OBJ_EXPECTED_DATASET_LEN.store((FIXED_LEN + 1) as u32, core::sync::atomic::Ordering::Relaxed);
            diagnostics::OBJ_BAD_LENGTH.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            return Err(MtpError::InvalidContainerLength);
        }
        let u16_at = |at: usize| u16::from_le_bytes([dataset[at], dataset[at + 1]]);
        let u32_at = |at: usize| u32::from_le_bytes([
            dataset[at], dataset[at + 1], dataset[at + 2], dataset[at + 3],
        ]);

        let storage_id = u32_at(0);
        diagnostics::OBJ_LAST_STORAGE_ID.store(storage_id, core::sync::atomic::Ordering::Relaxed);
        let format = u16_at(4);
        let protection = u16_at(6);
        let size = u32_at(8);
        let parent = u32_at(38);
        let association_type = u16_at(42);
        let count = dataset[FIXED_LEN] as usize;
        // Some Windows MTP clients put 0 (unspecified) in the ObjectInfo
        // dataset even though SendObjectInfo's command parameter identifies
        // the advertised storage. Normalize that sentinel to our storage ID.
        if storage_id != 0 && storage_id != PLACEHOLDER_STORAGE_ID {
            diagnostics::OBJ_BAD_STORAGE.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            return Err(MtpError::InvalidContainerLength);
        }
        let storage_id = PLACEHOLDER_STORAGE_ID;
        if format == 0x3001 {
            diagnostics::OBJ_BAD_FORMAT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            return Err(MtpError::InvalidContainerLength);
        }
        if protection != 0 {
            diagnostics::OBJ_BAD_PROTECTION.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            return Err(MtpError::InvalidContainerLength);
        }
        if association_type != 0 {
            diagnostics::OBJ_BAD_ASSOC.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            return Err(MtpError::InvalidContainerLength);
        }
        if parent != 0 && parent != 0xffff_ffff {
            diagnostics::OBJ_BAD_PARENT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            return Err(MtpError::InvalidContainerLength);
        }
        if count < 2 {
            diagnostics::OBJ_BAD_NAME.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            return Err(MtpError::InvalidContainerLength);
        }

        let string_bytes = count.checked_mul(2).ok_or(MtpError::InvalidContainerLength)?;
        let end = FIXED_LEN + 1 + string_bytes;
        diagnostics::OBJ_EXPECTED_DATASET_LEN.store(end as u32, core::sync::atomic::Ordering::Relaxed);
        if end > dataset.len() {
            diagnostics::OBJ_BAD_LENGTH.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            return Err(MtpError::InvalidContainerLength);
        }
        let unit_count = count - 1;
        if unit_count > 64 {
            diagnostics::OBJ_BAD_NAME.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            return Err(MtpError::InvalidContainerLength);
        }
        let mut units = [0u16; 64];
        for index in 0..unit_count {
            let at = FIXED_LEN + 1 + index * 2;
            units[index] = u16::from_le_bytes([dataset[at], dataset[at + 1]]);
        }
        let terminator_at = FIXED_LEN + 1 + unit_count * 2;
        if u16::from_le_bytes([dataset[terminator_at], dataset[terminator_at + 1]]) != 0 {
            diagnostics::OBJ_BAD_NAME.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            return Err(MtpError::InvalidContainerLength);
        }
        let mut name = [0u8; 64];
        let Some(name_len) = decode_utf16_filename(&units[..unit_count], &mut name) else {
            diagnostics::OBJ_BAD_NAME.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            return Err(MtpError::InvalidContainerLength);
        };

        // ObjectInfo has four PTP strings: filename, capture date,
        // modification date, and keywords. Consume the three metadata
        // strings after the filename; their contents are not needed here.
        let mut cursor = end;
        for _ in 0..3 {
            if cursor >= dataset.len() {
                diagnostics::OBJ_BAD_LENGTH.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                return Err(MtpError::InvalidContainerLength);
            }
            let metadata_count = dataset[cursor] as usize;
            let metadata_bytes = metadata_count.checked_mul(2).ok_or(MtpError::InvalidContainerLength)?;
            let metadata_start = cursor + 1;
            let metadata_end = metadata_start.checked_add(metadata_bytes).ok_or(MtpError::InvalidContainerLength)?;
            if metadata_end > dataset.len() {
                diagnostics::OBJ_BAD_LENGTH.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                return Err(MtpError::InvalidContainerLength);
            }
            if metadata_count != 0 {
                let terminator = metadata_end - 2;
                if u16::from_le_bytes([dataset[terminator], dataset[terminator + 1]]) != 0 {
                    diagnostics::OBJ_BAD_LENGTH.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                    return Err(MtpError::InvalidContainerLength);
                }
            }
            cursor = metadata_end;
        }
        diagnostics::OBJ_EXPECTED_DATASET_LEN.store(cursor as u32, core::sync::atomic::Ordering::Relaxed);
        if cursor != dataset.len() {
            diagnostics::OBJ_BAD_LENGTH.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            return Err(MtpError::InvalidContainerLength);
        }

        Ok(Self { storage_id, parent, size, name, name_len: name_len as u8 })
    }
}

#[cfg(test)]
mod filename_unicode_tests {
    use super::*;

    #[test]
    fn decodes_basic_multilingual_plane_filename() {
        let mut out = [0u8; 64];
        let len = decode_utf16_filename(&[0x4e2d, 0x6587, b'.' as u16, b't' as u16], &mut out).unwrap();
        assert_eq!(&out[..len], "中文.t".as_bytes());
    }

    #[test]
    fn decodes_supplementary_character_filename() {
        let mut out = [0u8; 64];
        let len = decode_utf16_filename(&[0xd83d, 0xde00, b'.' as u16], &mut out).unwrap();
        assert_eq!(&out[..len], "😀.".as_bytes());
    }

    #[test]
    fn rejects_invalid_surrogates_and_path_characters() {
        let mut out = [0u8; 64];
        assert!(decode_utf16_filename(&[0xd800], &mut out).is_none());
        assert!(decode_utf16_filename(&[b'a' as u16, b'/' as u16], &mut out).is_none());
    }

    #[test]
    fn upload_object_info_decodes_unicode_filename() {
        let units = [0x4e2d, 0x6587, 0xd83d, 0xde00, b'.' as u16, b't' as u16, b'x' as u16, b't' as u16];
        let mut data = [0u8; 52 + 1 + (units.len() + 1) * 2 + 3];
        data[0..4].copy_from_slice(&PLACEHOLDER_STORAGE_ID.to_le_bytes());
        data[4..6].copy_from_slice(&0x3000u16.to_le_bytes());
        data[52] = (units.len() + 1) as u8;
        for (index, unit) in units.iter().enumerate() {
            let at = 53 + index * 2;
            data[at..at + 2].copy_from_slice(&unit.to_le_bytes());
        }
        let parsed = UploadObjectInfo::decode(&data).unwrap();
        assert_eq!(&parsed.name[..parsed.name_len as usize], "中文😀.txt".as_bytes());
    }
}

#[cfg(test)]
mod upload_info_tests {
    use super::*;

    #[test]
    fn decodes_root_file_object_info() {
        let mut data = [0u8; 52 + 1 + 12 + 3];
        data[0..4].copy_from_slice(&PLACEHOLDER_STORAGE_ID.to_le_bytes());
        data[4..6].copy_from_slice(&0x3000u16.to_le_bytes());
        data[8..12].copy_from_slice(&1234u32.to_le_bytes());
        data[38..42].copy_from_slice(&0u32.to_le_bytes());
        data[52] = 6;
        for (index, unit) in [b't' as u16, b'e' as u16, b's' as u16, b't' as u16, b'.' as u16].iter().enumerate() {
            let at = 53 + index * 2;
            data[at..at + 2].copy_from_slice(&unit.to_le_bytes());
        }
        let parsed = UploadObjectInfo::decode(&data).unwrap();
        assert_eq!(parsed.size, 1234);
        assert_eq!(&parsed.name[..parsed.name_len as usize], b"test.");
    }

    #[test]
    fn accepts_unspecified_object_info_storage_id() {
        let mut data = [0u8; 52 + 1 + 10 + 3];
        data[0..4].copy_from_slice(&0u32.to_le_bytes());
        data[4..6].copy_from_slice(&0x3000u16.to_le_bytes());
        data[8..12].copy_from_slice(&1234u32.to_le_bytes());
        data[38..42].copy_from_slice(&0u32.to_le_bytes());
        data[52] = 5;
        for (index, unit) in [b't' as u16, b'e' as u16, b's' as u16, b't' as u16].iter().enumerate() {
            let at = 53 + index * 2;
            data[at..at + 2].copy_from_slice(&unit.to_le_bytes());
        }
        let parsed = UploadObjectInfo::decode(&data).unwrap();
        assert_eq!(parsed.storage_id, PLACEHOLDER_STORAGE_ID);
        assert_eq!(parsed.size, 1234);
        assert_eq!(&parsed.name[..parsed.name_len as usize], b"test");
    }

    #[test]
    fn rejects_path_separators() {
        let mut data = [0u8; 52 + 1 + 8 + 3];
        data[0..4].copy_from_slice(&PLACEHOLDER_STORAGE_ID.to_le_bytes());
        data[4..6].copy_from_slice(&0x3000u16.to_le_bytes());
        data[52] = 4;
        for (index, unit) in [b'a' as u16, b'/' as u16, b'b' as u16].iter().enumerate() {
            let at = 53 + index * 2;
            data[at..at + 2].copy_from_slice(&unit.to_le_bytes());
        }
        assert_eq!(UploadObjectInfo::decode(&data), Err(MtpError::InvalidContainerLength));
    }
}
