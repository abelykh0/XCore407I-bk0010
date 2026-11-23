//! HTTP server, LittleFS transfer state, and HTTP diagnostics.

use core::ptr::addr_of_mut;
use super::http_storage_ptr;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

pub(super) const HTTP_RX_BUFFER_SIZE: usize = crate::camera::CDC_HTTP_RX_BYTES;
pub(super) const HTTP_BODY_BUFFER_SIZE: usize = crate::camera::CDC_HTTP_BODY_BYTES;
static HTTP_UPLOAD_DIAG_STAGE: AtomicU8 = AtomicU8::new(0);
static HTTP_UPLOAD_DIAG_OFFSET: AtomicU32 = AtomicU32::new(0);
static HTTP_UPLOAD_DIAG_TOTAL: AtomicU32 = AtomicU32::new(0);
static HTTP_UPLOAD_DIAG_LENGTH: AtomicU32 = AtomicU32::new(0);
static HTTP_UPLOAD_DIAG_WRITTEN: AtomicU32 = AtomicU32::new(0);
static HTTP_UPLOAD_DIAG_SEND_FLAGS: AtomicU32 = AtomicU32::new(0);
static HTTP_POLL_DIAG_STAGE: AtomicU8 = AtomicU8::new(0);
static HTTP_POLL_DIAG_COUNT: AtomicU32 = AtomicU32::new(0);
static HTTP_POLL_DIAG_RECV_COUNT: AtomicU32 = AtomicU32::new(0);
static HTTP_POLL_DIAG_RECV_BYTES: AtomicU32 = AtomicU32::new(0);
static HTTP_POLL_DIAG_NO_SEND: AtomicU32 = AtomicU32::new(0);
static HTTP_POLL_DIAG_NO_RECV: AtomicU32 = AtomicU32::new(0);
static HTTP_POLL_DIAG_EXPECTED: AtomicU32 = AtomicU32::new(0);
static HTTP_POLL_DIAG_QUEUE: AtomicU32 = AtomicU32::new(0);
static HTTP_POLL_DIAG_CAPACITY: AtomicU32 = AtomicU32::new(0);
static HTTP_POLL_DIAG_STATE: AtomicU8 = AtomicU8::new(0);

pub fn http_upload_diagnostics() -> (u8, u32, u32, u32, u32, u32) {
    (
        HTTP_UPLOAD_DIAG_STAGE.load(Ordering::Acquire),
        HTTP_UPLOAD_DIAG_OFFSET.load(Ordering::Acquire),
        HTTP_UPLOAD_DIAG_TOTAL.load(Ordering::Acquire),
        HTTP_UPLOAD_DIAG_LENGTH.load(Ordering::Acquire),
        HTTP_UPLOAD_DIAG_WRITTEN.load(Ordering::Acquire),
        HTTP_UPLOAD_DIAG_SEND_FLAGS.load(Ordering::Acquire),
    )
}

pub fn http_poll_diagnostics() -> (u8, u32, u32, u32, u32, u32, u32, u32, u8) {
    (
        HTTP_POLL_DIAG_STAGE.load(Ordering::Acquire),
        HTTP_POLL_DIAG_COUNT.load(Ordering::Relaxed),
        HTTP_POLL_DIAG_RECV_COUNT.load(Ordering::Relaxed),
        HTTP_POLL_DIAG_RECV_BYTES.load(Ordering::Acquire),
        HTTP_POLL_DIAG_NO_SEND.load(Ordering::Relaxed),
        HTTP_POLL_DIAG_NO_RECV.load(Ordering::Relaxed),
        HTTP_POLL_DIAG_EXPECTED.load(Ordering::Acquire),
        HTTP_POLL_DIAG_QUEUE.load(Ordering::Acquire),
        HTTP_POLL_DIAG_STATE.load(Ordering::Acquire),
    )
}


const HTTP_PAGE: &[u8] = include_bytes!("../../resources/http_page.html");

fn http_name_ends_with(name: &[u8], suffix: &[u8]) -> bool {
    name.len() >= suffix.len()
        && name[name.len() - suffix.len()..]
            .iter()
            .zip(suffix)
            .all(|(left, right)| left.to_ascii_lowercase() == *right)
}

fn http_escaped_name_len(name: &[u8]) -> usize {
    name.iter()
        .map(|byte| match byte {
            b'&' => 5,
            b'<' | b'>' => 4,
            b'"' => 6,
            _ => 1,
        })
        .sum()
}

fn http_directory_entry_len(name: &[u8], is_dir: bool, size: usize) -> usize {
    let safe_image_name = !name.is_empty()
        && name.len() <= 64
        && name != b"."
        && name != b".."
        && name.iter().all(|byte| byte.is_ascii_alphanumeric() || matches!(*byte, b'.' | b'_' | b'-'));
    let image_file = !is_dir
        && safe_image_name
        && (http_name_ends_with(name, b".jpg")
            || http_name_ends_with(name, b".jpeg")
            || http_name_ends_with(name, b".png"));
    let escaped_name_len = http_escaped_name_len(name);
    let mut entry_len = b"<li data-name=\"".len() + escaped_name_len + b"\">".len()
        + escaped_name_len + b"</li>".len();
    if image_file {
        entry_len += b"<a target=\"_blank\" rel=\"noopener\" href=\"/file/".len()
            + name.len() + b"\">".len() + b"</a>".len();
    }
    if is_dir {
        entry_len += 1; // trailing slash
    } else {
        let mut digits = [0u8; 10];
        let digit_count = crate::format::u32_to_decimal(size as u32, &mut digits).len();
        entry_len += b" (".len() + digit_count + digit_count.saturating_sub(1) / 3 + b" bytes)".len();
    }
    entry_len
}

fn append_http_bytes(buffer: &mut [u8], length: &mut usize, bytes: &[u8]) -> bool {
    if bytes.len() > buffer.len().saturating_sub(*length) {
        return false;
    }
    buffer[*length..*length + bytes.len()].copy_from_slice(bytes);
    *length += bytes.len();
    true
}

fn append_http_decimal_formatted(
    buffer: &mut [u8],
    length: &mut usize,
    value: usize,
    grouped: bool,
) {
    let mut digits = [0u8; 10];
    let digits = crate::format::u32_to_decimal(value as u32, &mut digits);
    for (index, &digit) in digits.iter().enumerate() {
        if grouped && index != 0 && (digits.len() - index) % 3 == 0 {
            append_http_bytes(buffer, length, b",");
        }
        append_http_bytes(buffer, length, core::slice::from_ref(&digit));
    }
}

fn append_http_decimal(buffer: &mut [u8], length: &mut usize, value: usize) {
    append_http_decimal_formatted(buffer, length, value, false);
}

fn append_http_grouped_decimal(buffer: &mut [u8], length: &mut usize, value: usize) {
    append_http_decimal_formatted(buffer, length, value, true);
}


const HTTP_UPLOAD_CHUNK_SIZE: usize = 2048;

pub struct HttpDownloadState {
    size: usize,
    offset: usize,
    chunk_len: usize,
    chunk_pos: usize,
    header_offset: usize,
    content_type: &'static [u8],
    header_sent: bool,
    active: bool,
}

impl HttpDownloadState {
    pub fn new() -> Self {
        Self {
            size: 0,
            offset: 0,
            chunk_len: 0,
            chunk_pos: 0,
            header_offset: 0,
            content_type: b"image/jpeg",
            header_sent: false,
            active: false,
        }
    }

    fn start(
        &mut self,
        filesystem: &littlefs2::fs::Filesystem<'_, crate::nand_fs::ActiveStorage>,
        name: &[u8],
        allocation: &mut littlefs2::fs::FileAllocation<crate::nand_fs::ActiveStorage>,
    ) -> bool {
        use littlefs2::path::Path;
        if self.active || allocation.is_open() || name.is_empty() || name.len() > 64 { return false; }
        let mut path_bytes = [0u8; 66];
        path_bytes[0] = b'/';
        path_bytes[1..1 + name.len()].copy_from_slice(name);
        path_bytes[1 + name.len()] = 0;
        let Ok(path) = Path::from_bytes_with_nul(&path_bytes[..name.len() + 2]) else { return false; };
        let Ok(metadata) = filesystem.metadata(path) else { return false; };
        let size = metadata.len();
        self.content_type = if http_name_ends_with(name, b".png") { b"image/png" } else { b"image/jpeg" };
        if unsafe { allocation.open_with_options(filesystem, |options| options.read(true), path) }.is_err() {
            return false;
        }
        self.size = size;
        self.offset = 0;
        self.chunk_len = 0;
        self.chunk_pos = 0;
        self.header_offset = 0;
        self.header_sent = false;
        self.active = true;
        true
    }
}

pub struct HttpUploadState {
    allocation: littlefs2::fs::FileAllocation<crate::nand_fs::ActiveStorage>,
    temp_path: [u8; 80],
    temp_path_len: usize,
    final_path: [u8; 66],
    final_path_len: usize,
    total: usize,
    received: usize,
    rx_len: usize,
}

impl HttpUploadState {
    pub fn new() -> Self {
        Self {
            allocation: littlefs2::fs::FileAllocation::new(),
            temp_path: [0; 80],
            temp_path_len: 0,
            final_path: [0; 66],
            final_path_len: 0,
            total: 0,
            received: 0,
            rx_len: 0,
        }
    }

    fn abort(&mut self, filesystem: &littlefs2::fs::Filesystem<'_, crate::nand_fs::ActiveStorage>) {
        use littlefs2::path::Path;
        if self.allocation.is_open() {
            let _ = unsafe { self.allocation.close(filesystem) };
        }
        if self.temp_path_len != 0 {
            if let Ok(path) = Path::from_bytes_with_nul(&self.temp_path[..self.temp_path_len]) {
                let _ = filesystem.remove(path);
            }
        }
        self.temp_path_len = 0;
        self.final_path_len = 0;
        self.total = 0;
        self.received = 0;
    }

    fn write_chunk(
        &mut self,
        filesystem: &littlefs2::fs::Filesystem<'_, crate::nand_fs::ActiveStorage>,
        name: &[u8],
        offset: usize,
        total: usize,
        data: &[u8],
    ) -> bool {
        use littlefs2::path::Path;

        HTTP_UPLOAD_DIAG_STAGE.store(2, Ordering::Release);
        HTTP_UPLOAD_DIAG_WRITTEN.store(0, Ordering::Release);
        if name.is_empty() || name.len() > 64 || total > filesystem.total_space()
            || offset > total || data.len() > total.saturating_sub(offset)
        {
            return false;
        }

        if offset == 0 {
            if self.temp_path_len != 0 || self.allocation.is_open() {
                self.abort(filesystem);
            }

            let mut final_len = 0usize;
            self.final_path[final_len] = b'/';
            final_len += 1;
            self.final_path[final_len..final_len + name.len()].copy_from_slice(name);
            final_len += name.len();
            self.final_path[final_len] = 0;
            final_len += 1;

            let mut temp_len = 9usize;
            self.temp_path[..9].copy_from_slice(b"/.upload-");
            self.temp_path[temp_len..temp_len + name.len()].copy_from_slice(name);
            temp_len += name.len();
            self.temp_path[temp_len] = 0;
            temp_len += 1;

            let Ok(temp) = Path::from_bytes_with_nul(&self.temp_path[..temp_len]) else { return false; };
            let Ok(final_path) = Path::from_bytes_with_nul(&self.final_path[..final_len]) else { return false; };
            if filesystem.exists(temp) && filesystem.remove(temp).is_err() { return false; }
            if filesystem.open_file_with_options_and_then_with_allocation(
                |options| options.write(true).create_new(true),
                temp,
                &mut self.allocation,
                |_file| Ok(()),
            ).is_err() {
                return false;
            }
            if unsafe { self.allocation.open_with_options(filesystem, |options| options.write(true), temp) }.is_err() {
                let _ = filesystem.remove(temp);
                return false;
            }
            self.temp_path_len = temp_len;
            self.final_path_len = final_len;
            self.total = total;
            self.received = 0;
            let _ = final_path;
        } else if self.temp_path_len == 0
            || self.final_path_len == 0
            || self.total != total
            || self.received != offset
            || &self.final_path[1..self.final_path_len - 1] != name
            || !self.allocation.is_open()
        {
            return false;
        }

        let mut written = 0usize;
        while written < data.len() {
            let count = match unsafe { self.allocation.write(filesystem, &data[written..]) } {
                Ok(count) if count != 0 => count,
                _ => { self.abort(filesystem); return false; }
            };
            written += count;
            HTTP_UPLOAD_DIAG_WRITTEN.store(written as u32, Ordering::Release);
        }
        self.received += data.len();
        HTTP_UPLOAD_DIAG_STAGE.store(3, Ordering::Release);

        if self.received == self.total {
            HTTP_UPLOAD_DIAG_STAGE.store(4, Ordering::Release);
            if unsafe { self.allocation.close(filesystem) }.is_err() {
                self.abort(filesystem);
                return false;
            }
            HTTP_UPLOAD_DIAG_STAGE.store(5, Ordering::Release);
            let Ok(temp) = Path::from_bytes_with_nul(&self.temp_path[..self.temp_path_len]) else {
                self.abort(filesystem); return false;
            };
            let Ok(final_path) = Path::from_bytes_with_nul(&self.final_path[..self.final_path_len]) else {
                self.abort(filesystem); return false;
            };
            if filesystem.exists(final_path) && filesystem.remove(final_path).is_err() {
                self.abort(filesystem); return false;
            }
            HTTP_UPLOAD_DIAG_STAGE.store(6, Ordering::Release);
            if filesystem.rename(temp, final_path).is_err() {
                self.abort(filesystem); return false;
            }
            HTTP_UPLOAD_DIAG_STAGE.store(7, Ordering::Release);
            self.temp_path_len = 0;
            self.final_path_len = 0;
            self.total = 0;
            self.received = 0;
        }
        true
    }
}

fn parse_http_decimal(bytes: &[u8]) -> Option<usize> {
    let bytes = bytes.trim_ascii();
    if bytes.is_empty() { return None; }
    let mut value = 0usize;
    for &byte in bytes {
        if !byte.is_ascii_digit() { return None; }
        value = value.checked_mul(10)?.checked_add((byte - b'0') as usize)?;
    }
    Some(value)
}

impl<'a> super::EthernetNetwork<'a> {
    /// Serve a small page shell and an AJAX-paginated LittleFS directory listing.
    pub(super) fn poll_http(
        &mut self,
        filesystem: &littlefs2::fs::Filesystem<'_, crate::nand_fs::ActiveStorage>,
        http_upload: &mut HttpUploadState,
        http_download: &mut HttpDownloadState,
    ) {
        use smoltcp::socket::tcp::Socket as TcpSocket;

        HTTP_POLL_DIAG_COUNT.fetch_add(1, Ordering::Relaxed);
        HTTP_POLL_DIAG_STAGE.store(1, Ordering::Release);
        let socket = self.sockets.get_mut::<TcpSocket>(self.http_handle);
        if !socket.is_open() {
            let _ = socket.listen(80);
            return;
        }

        // A browser may close a keep-alive connection after a fetch. In
        // CLOSE-WAIT, is_open() and can_send() can still be true, but the
        // receive half is finished. Close our transmit half so smoltcp can
        // complete the TCP teardown and the listening socket can be reused.
        if http_download.active {
            Self::poll_http_download(socket, filesystem, http_download, &mut http_upload.allocation);
            return;
        }

        if !socket.may_recv() {
            HTTP_POLL_DIAG_STAGE.store(2, Ordering::Release);
            HTTP_POLL_DIAG_NO_RECV.fetch_add(1, Ordering::Relaxed);
            if socket.may_send() {
                socket.close();
            }
            return;
        }
        if !socket.can_send() {
            HTTP_POLL_DIAG_STAGE.store(3, Ordering::Release);
            HTTP_POLL_DIAG_NO_SEND.fetch_add(1, Ordering::Relaxed);
            return;
        }

        HTTP_POLL_DIAG_STAGE.store(4, Ordering::Release);
        let mut request_kind = 0u8; // 0=page, 1=directory, 2=upload, 3=bad request
        let mut listing_offset = 0usize;
        let mut upload_name = [0u8; 64];
        let mut upload_name_len = 0usize;
        let mut upload_offset = 0usize;
        let mut upload_total = 0usize;
        let mut upload_len = 0usize;
        let mut upload_body_offset = 0usize;
        let mut image_name = [0u8; 64];
        let mut image_name_len = 0usize;
        let upload_body = unsafe {
            core::slice::from_raw_parts_mut(
                addr_of_mut!((*http_storage_ptr()).body).cast::<u8>(),
                HTTP_BODY_BUFFER_SIZE,
            )
        };

        let mut request_ready = false;
        let mut parse_request = |data: &[u8]| {
            HTTP_POLL_DIAG_RECV_COUNT.fetch_add(1, Ordering::Relaxed);
            HTTP_POLL_DIAG_RECV_BYTES.store(data.len() as u32, Ordering::Release);
            let Some(separator) = data.windows(4).position(|window| window == b"\r\n\r\n") else {
                HTTP_POLL_DIAG_STAGE.store(5, Ordering::Release);
                return (0, false);
            };
            let header_len = separator + 4;
            let Some(line_end) = data.windows(2).position(|window| window == b"\r\n") else {
                return (0, false);
            };
            let line = &data[..line_end];

            if let Some(rest) = line.strip_prefix(b"GET /file/") {
                let Some(http_pos) = rest.windows(9).position(|window| window == b" HTTP/1.1") else {
                    request_kind = 3;
                    request_ready = true;
                    return (header_len, true);
                };
                let name = &rest[..http_pos];
                let valid_name = !name.is_empty() && name.len() <= image_name.len()
                    && name != b"." && name != b".."
                    && name.iter().all(|byte| byte.is_ascii_alphanumeric() || matches!(*byte, b'.' | b'_' | b'-'));
                let is_image = http_name_ends_with(name, b".jpg")
                    || http_name_ends_with(name, b".jpeg")
                    || http_name_ends_with(name, b".png");
                if valid_name && is_image {
                    image_name[..name.len()].copy_from_slice(name);
                    image_name_len = name.len();
                    request_kind = 4;
                } else {
                    request_kind = 3;
                }
                request_ready = true;
                return (header_len, true);
            }

            if let Some(rest) = line.strip_prefix(b"GET /api/files?offset=") {
                request_kind = 1;
                for &byte in rest {
                    if !byte.is_ascii_digit() { break; }
                    listing_offset = listing_offset.saturating_mul(10)
                        .saturating_add((byte - b'0') as usize)
                        .min(1_000_000);
                }
                return (header_len, true);
            }

            if let Some(rest) = line.strip_prefix(b"POST /api/upload?name=") {
                let Some(http_pos) = rest.windows(9).position(|window| window == b" HTTP/1.1") else {
                    request_kind = 3;
                    return (header_len, true);
                };
                let query = &rest[..http_pos];
                let mut fields = query.split(|&byte| byte == b'&');
                let name = fields.next().unwrap_or(&[]);
                let offset = fields.next().and_then(|field| field.strip_prefix(b"offset="))
                    .and_then(parse_http_decimal);
                let total = fields.next().and_then(|field| field.strip_prefix(b"total="))
                    .and_then(parse_http_decimal);
                let valid_name = !name.is_empty()
                    && name != b"."
                    && name != b".."
                    && name.len() <= upload_name.len()
                    && name.iter().all(|byte| byte.is_ascii_alphanumeric() || matches!(*byte, b'.' | b'_' | b'-'));
                let mut content_length = None;
                for header in data[..separator].split(|&byte| byte == b'\n').skip(1) {
                    let header = header.strip_suffix(b"\r").unwrap_or(header);
                    if header.len() >= 15 && header[..15].eq_ignore_ascii_case(b"Content-Length:") {
                        content_length = parse_http_decimal(header[15..].trim_ascii());
                    }
                }
                let Some(content_length) = content_length else {
                    request_kind = 3;
                    request_ready = true;
                    return (header_len, true);
                };
                if !valid_name || offset.is_none() || total.is_none()
                    || content_length > HTTP_UPLOAD_CHUNK_SIZE
                    || content_length > HTTP_BODY_BUFFER_SIZE
                {
                    request_kind = 3;
                    request_ready = true;
                    return (header_len, true);
                }
                if data.len() < header_len + content_length {
                    HTTP_POLL_DIAG_EXPECTED.store((header_len + content_length) as u32, Ordering::Release);
                    HTTP_POLL_DIAG_STAGE.store(6, Ordering::Release);
                    return (0, false);
                }
                upload_name[..name.len()].copy_from_slice(name);
                upload_name_len = name.len();
                upload_offset = offset.unwrap();
                upload_total = total.unwrap();
                upload_len = content_length;
                if upload_offset > upload_total
                    || upload_len > upload_total.saturating_sub(upload_offset)
                {
                    request_kind = 3;
                } else {
                    upload_body_offset = header_len;
                    request_kind = 2;
                }
                HTTP_UPLOAD_DIAG_OFFSET.store(upload_offset as u32, Ordering::Release);
                HTTP_UPLOAD_DIAG_TOTAL.store(upload_total as u32, Ordering::Release);
                HTTP_UPLOAD_DIAG_LENGTH.store(upload_len as u32, Ordering::Release);
                HTTP_UPLOAD_DIAG_WRITTEN.store(0, Ordering::Release);
                HTTP_UPLOAD_DIAG_SEND_FLAGS.store(0, Ordering::Release);
                HTTP_UPLOAD_DIAG_STAGE.store(1, Ordering::Release);
                request_ready = true;
                return (header_len + content_length, true);
            }

            request_kind = if line.starts_with(b"GET / ") { 0 } else { 3 };
            request_ready = true;
            (header_len, true)
        };

        // smoltcp recv() exposes only the current contiguous ring-buffer slice.
        // Accumulate successive slices so a wrapped TCP request is parsed as one
        // complete HTTP message. Keep partial requests across polling passes.
        while http_upload.rx_len < HTTP_RX_BUFFER_SIZE {
            let tail = &mut upload_body[http_upload.rx_len..HTTP_RX_BUFFER_SIZE];
            match socket.recv(|data| {
                let count = data.len().min(tail.len());
                tail[..count].copy_from_slice(&data[..count]);
                (count, count)
            }) {
                Ok(count) if count != 0 => {
                    http_upload.rx_len += count;
                    HTTP_POLL_DIAG_RECV_COUNT.fetch_add(1, Ordering::Relaxed);
                    HTTP_POLL_DIAG_RECV_BYTES.store(http_upload.rx_len as u32, Ordering::Release);
                }
                _ => break,
            }
        }
        let (_, request_ready) = parse_request(&upload_body[..http_upload.rx_len]);

        HTTP_POLL_DIAG_QUEUE.store(socket.recv_queue() as u32, Ordering::Release);
        HTTP_POLL_DIAG_CAPACITY.store(socket.recv_capacity() as u32, Ordering::Release);
        let tcp_state = match socket.state() {
            smoltcp::socket::tcp::State::Closed => 0,
            smoltcp::socket::tcp::State::Listen => 1,
            smoltcp::socket::tcp::State::SynSent => 2,
            smoltcp::socket::tcp::State::SynReceived => 3,
            smoltcp::socket::tcp::State::Established => 4,
            smoltcp::socket::tcp::State::FinWait1 => 5,
            smoltcp::socket::tcp::State::FinWait2 => 6,
            smoltcp::socket::tcp::State::CloseWait => 7,
            smoltcp::socket::tcp::State::Closing => 8,
            smoltcp::socket::tcp::State::LastAck => 9,
            smoltcp::socket::tcp::State::TimeWait => 10,
        };
        HTTP_POLL_DIAG_STATE.store(tcp_state, Ordering::Release);
        if !request_ready || !socket.can_send() {
            if request_ready { HTTP_POLL_DIAG_STAGE.store(8, Ordering::Release); }
            return;
        }
        HTTP_POLL_DIAG_STAGE.store(9, Ordering::Release);

        if request_kind == 4 {
            http_upload.rx_len = 0;
            if http_download.start(filesystem, &image_name[..image_name_len], &mut http_upload.allocation) {
                Self::poll_http_download(socket, filesystem, http_download, &mut http_upload.allocation);
            } else {
                let _ = socket.send_slice(b"HTTP/1.1 404 Not Found\r\nContent-Length: 9\r\nConnection: close\r\n\r\nNot found");
                socket.close();
            }
            return;
        }

        let mut status_code = 200u16;
        let body: &[u8] = match request_kind {
            0 => HTTP_PAGE,
            1 => {
                let body = upload_body;
                let mut length = 0usize;
                let mut seen = 0usize;
                let mut emitted = 0usize;
                let mut has_more = false;
                append_http_bytes(body, &mut length, b"<ul>");
                let _ = crate::nand_fs::read_root_directory(
                    filesystem,
                    true,
                    |path, is_dir, size| {
                        if seen < listing_offset {
                            seen += 1;
                            return Ok(());
                        }
                        let name = path.strip_prefix(b"/").unwrap_or(path);
                        let name = name.strip_suffix(b"\0").unwrap_or(name);
                        // Check the complete rendered entry before writing any of it.
                        // Reserve room for the closing list and pagination marker.
                        let pagination_reserve =
                            b"</ul>".len() + b"<span data-next=\"".len() + 10 + b"\"></span>".len();
                        let required = http_directory_entry_len(name, is_dir, size)
                            + pagination_reserve;
                        if required > body.len().saturating_sub(length) {
                            has_more = true;
                            return Ok(());
                        }
                        seen += 1;
                        emitted += 1;
                        append_http_bytes(body, &mut length, b"<li data-name=\"");
                        for &byte in name {
                            let escaped = match byte {
                                b'&' => b"&amp;" as &[u8],

                                b'<' => b"&lt;",
                                b'>' => b"&gt;",
                                b'"' => b"&quot;",
                                _ => core::slice::from_ref(&byte),
                            };
                            append_http_bytes(body, &mut length, escaped);
                        }
                        append_http_bytes(body, &mut length, b"\">");
                        let safe_image_name = !name.is_empty()
                            && name.len() <= 64
                            && name != b"."
                            && name != b".."
                            && name.iter().all(|byte| byte.is_ascii_alphanumeric() || matches!(*byte, b'.' | b'_' | b'-'));
                        let image_file = !is_dir && safe_image_name
                            && (http_name_ends_with(name, b".jpg")
                                || http_name_ends_with(name, b".jpeg")
                                || http_name_ends_with(name, b".png"));
                        if image_file {
                            append_http_bytes(body, &mut length, b"<a target=\"_blank\" rel=\"noopener\" href=\"/file/");
                            append_http_bytes(body, &mut length, name);
                            append_http_bytes(body, &mut length, b"\">");
                        }
                        for &byte in name {
                            let escaped = match byte {
                                b'&' => b"&amp;" as &[u8],
                                b'<' => b"&lt;",
                                b'>' => b"&gt;",
                                b'"' => b"&quot;",
                                _ => core::slice::from_ref(&byte),
                            };
                            append_http_bytes(body, &mut length, escaped);
                        }
                        if image_file { append_http_bytes(body, &mut length, b"</a>"); }
                        if is_dir {
                            append_http_bytes(body, &mut length, b"/");
                        } else {
                            append_http_bytes(body, &mut length, b" (");
                            append_http_grouped_decimal(body, &mut length, size);
                            append_http_bytes(body, &mut length, b" bytes)");
                        }
                        append_http_bytes(body, &mut length, b"</li>");
                        Ok(())
                    },
                );
                append_http_bytes(body, &mut length, b"</ul>");
                if has_more {
                    append_http_bytes(body, &mut length, b"<span data-next=\"");
                    append_http_decimal(body, &mut length, listing_offset.saturating_add(emitted));
                    append_http_bytes(body, &mut length, b"\"></span>");
                }
                &body[..length]
            }
            2 => {
                let name = &upload_name[..upload_name_len];
                if http_upload.write_chunk(filesystem, name, upload_offset, upload_total, &upload_body[upload_body_offset..upload_body_offset + upload_len]) {
                    b"OK"
                } else {
                    status_code = 500;
                    b"Upload failed"
                }
            }
            _ => {
                status_code = 400;
                b"Bad request"
            }
        };

        let mut header = [0u8; 160];
        let mut header_len = 0usize;
        let status = match status_code {
            200 => b"HTTP/1.1 200 OK\r\n" as &[u8],
            400 => b"HTTP/1.1 400 Bad Request\r\n",
            _ => b"HTTP/1.1 500 Internal Server Error\r\n",
        };
        append_http_bytes(&mut header, &mut header_len, status);
        let content_type = if request_kind <= 1 {
            b"Content-Type: text/html; charset=utf-8\r\n" as &[u8]
        } else {
            b"Content-Type: text/plain; charset=utf-8\r\n"
        };
        append_http_bytes(&mut header, &mut header_len, content_type);
        append_http_bytes(&mut header, &mut header_len, b"Connection: keep-alive\r\nContent-Length: ");
        append_http_decimal(&mut header, &mut header_len, body.len());
        append_http_bytes(&mut header, &mut header_len, b"\r\n\r\n");
        if request_kind == 2 { HTTP_UPLOAD_DIAG_STAGE.store(8, Ordering::Release); }
        let header_sent = socket.send_slice(&header[..header_len]).is_ok();
        if request_kind == 2 {
            HTTP_UPLOAD_DIAG_SEND_FLAGS.store(header_sent as u32, Ordering::Release);
            HTTP_UPLOAD_DIAG_STAGE.store(9, Ordering::Release);
        }
        let body_sent = socket.send_slice(body).is_ok();
        if request_kind == 2 {
            HTTP_UPLOAD_DIAG_SEND_FLAGS.store((header_sent as u32) | ((body_sent as u32) << 1), Ordering::Release);
            HTTP_UPLOAD_DIAG_STAGE.store(10, Ordering::Release);
        }
        HTTP_POLL_DIAG_STAGE.store(10, Ordering::Release);
        http_upload.rx_len = 0;
    }

    fn poll_http_download(
        socket: &mut smoltcp::socket::tcp::Socket<'_>,
        filesystem: &littlefs2::fs::Filesystem<'_, crate::nand_fs::ActiveStorage>,
        state: &mut HttpDownloadState,
        allocation: &mut littlefs2::fs::FileAllocation<crate::nand_fs::ActiveStorage>,
    ) {
        if !state.active || !socket.can_send() { return; }
        if !state.header_sent {
            let mut header = [0u8; 192];
            let mut len = 0usize;
            append_http_bytes(&mut header, &mut len, b"HTTP/1.1 200 OK\r\nContent-Type: ");
            append_http_bytes(&mut header, &mut len, state.content_type);
            append_http_bytes(&mut header, &mut len, b"\r\nContent-Length: ");
            append_http_decimal(&mut header, &mut len, state.size);
            append_http_bytes(&mut header, &mut len, b"\r\nContent-Disposition: inline\r\nX-Content-Type-Options: nosniff\r\nConnection: close\r\n\r\n");
            match socket.send_slice(&header[state.header_offset..len]) {
                Ok(sent) => {
                    state.header_offset += sent;
                    if state.header_offset == len {
                        state.header_sent = true;
                    }
                }
                Err(_) => return,
            }
            return;
        }
        if state.offset >= state.size {
            if allocation.is_open() { let _ = unsafe { allocation.close(filesystem) }; }
            state.active = false;
            socket.close();
            return;
        }
        let buffer = unsafe {
            core::slice::from_raw_parts_mut(
                addr_of_mut!((*http_storage_ptr()).body).cast::<u8>(),
                1024,
            )
        };
        // Match the ZMODEM sender's buffered file-read pattern: keep the
        // allocation open and read forward. Retain a chunk until TCP accepts
        // it, so a temporarily full TX queue never causes the file cursor and
        // HTTP byte offset to diverge.
        if state.chunk_pos < state.chunk_len {
            let pending = &buffer[state.chunk_pos..state.chunk_len];
            match socket.send_slice(pending) {
                Ok(sent) => {
                    state.chunk_pos += sent;
                    state.offset += sent;
                }
                Err(_) => return,
            }
            return;
        }

        let count = core::cmp::min(buffer.len(), state.size - state.offset);
        match unsafe { allocation.read(filesystem, &mut buffer[..count]) } {
            Ok(read) if read != 0 => {
                state.chunk_len = read;
                state.chunk_pos = 0;
            }
            _ => {
                if allocation.is_open() { let _ = unsafe { allocation.close(filesystem) }; }
                state.active = false;
                socket.close();
            }
        }
    }
}