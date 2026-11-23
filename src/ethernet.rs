//! Foreground-polled Ethernet networking.
//!
//! PHY/MAC/DMA support is in `mac`; HTTP handling and file transfers are in `http`.

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use smoltcp::time::Instant;

mod http;
mod mac;
pub use http::{
    HttpDownloadState, HttpUploadState, http_poll_diagnostics, http_upload_diagnostics,
};
pub use mac::{
    EthernetDevice, EthernetRxToken, EthernetTxToken, PhyStatus,
    enable_dma_interrupts, initialize, is_initialized, on_dma_interrupt, probe_dp83848,
};
use http::{HTTP_BODY_BUFFER_SIZE, HTTP_RX_BUFFER_SIZE};
use mac::MAC_ADDRESS;

#[repr(C, align(8))]
struct EthernetHttpStorage {
    rx: [u8; HTTP_RX_BUFFER_SIZE],
    body: [u8; HTTP_BODY_BUFFER_SIZE],
}

#[repr(C, align(8))]
struct EthernetSocketStorage {
    sockets: [smoltcp::iface::SocketStorage<'static>; 2],
}

const HTTP_TX_BUFFER_SIZE: usize =
    crate::camera::CANVAS_TAIL_BYTES - core::mem::size_of::<EthernetSocketStorage>();

const _: () = assert!(core::mem::size_of::<EthernetHttpStorage>() == crate::camera::CDC_HTTP_STORAGE_BYTES);
const _: () = assert!(core::mem::size_of::<EthernetSocketStorage>() + HTTP_TX_BUFFER_SIZE <= crate::camera::CANVAS_TAIL_BYTES);
#[inline]
fn http_storage_ptr() -> *mut EthernetHttpStorage {
    crate::camera::cdc_ethernet_http_storage_ptr().cast()
}

#[inline]
fn socket_storage_ptr() -> *mut EthernetSocketStorage {
    crate::camera::cdc_ethernet_socket_storage_ptr().cast()
}

#[inline]
fn http_tx_storage_ptr() -> *mut u8 {
    unsafe {
        socket_storage_ptr()
            .cast::<u8>()
            .add(core::mem::size_of::<EthernetSocketStorage>())
    }
}

pub fn socket_storage() -> &'static mut [smoltcp::iface::SocketStorage<'static>] {
    unsafe {
        let storage = socket_storage_ptr();
        core::ptr::addr_of_mut!((*storage).sockets).write([
            smoltcp::iface::SocketStorage::EMPTY,
            smoltcp::iface::SocketStorage::EMPTY,
        ]);
        &mut *core::ptr::addr_of_mut!((*storage).sockets)
    }
}

static IPV4_ADDRESS: AtomicU32 = AtomicU32::new(0);
static IPV4_PREFIX: AtomicU8 = AtomicU8::new(0);

/// Current DHCP address and prefix, if a lease has been acquired.
pub fn ipv4_config() -> Option<([u8; 4], u8)> {
    let address = IPV4_ADDRESS.load(Ordering::Acquire);
    if address == 0 {
        None
    } else {
        Some((address.to_be_bytes(), IPV4_PREFIX.load(Ordering::Acquire)))
    }
}

/// Small no-heap DHCP client around the polled Ethernet device.
pub struct EthernetNetwork<'a> {
    device: EthernetDevice,
    iface: smoltcp::iface::Interface,
    sockets: smoltcp::iface::SocketSet<'a>,
    dhcp_handle: smoltcp::iface::SocketHandle,
    http_handle: smoltcp::iface::SocketHandle,
}

impl<'a> EthernetNetwork<'a> {
    /// Create a network interface using caller-provided fixed socket storage.
    pub fn new(storage: &'a mut [smoltcp::iface::SocketStorage<'a>]) -> Self {
        use smoltcp::iface::{Config, Interface, SocketSet};
        use smoltcp::socket::dhcpv4::Socket as Dhcpv4Socket;
        use smoltcp::socket::tcp::{Socket as TcpSocket, SocketBuffer as TcpSocketBuffer};
        use smoltcp::wire::{EthernetAddress, HardwareAddress};

        let mut device = EthernetDevice::new();
        let mut config = Config::new(HardwareAddress::Ethernet(EthernetAddress(MAC_ADDRESS)));
        config.random_seed = 0x4071_1209_2026_0001;
        let iface = Interface::new(config, &mut device, Instant::from_millis(0));
        let mut sockets = SocketSet::new(storage);
        let dhcp_handle = sockets.add(Dhcpv4Socket::new());

        // HTTP RX/body buffers are in DEMO_CANVAS. The TCP TX buffer follows
        // socket metadata in CCM, where CPU access is sufficient.
        let http_storage = http_storage_ptr();
        let http_rx: &'static mut [u8] = unsafe {
            core::slice::from_raw_parts_mut(
                addr_of_mut!((*http_storage).rx).cast::<u8>(),
                HTTP_RX_BUFFER_SIZE,
            )
        };
        let http_tx: &'static mut [u8] = unsafe {
            core::slice::from_raw_parts_mut(http_tx_storage_ptr(), HTTP_TX_BUFFER_SIZE)
        };
        let http_socket = TcpSocket::new(
            TcpSocketBuffer::new(http_rx),
            TcpSocketBuffer::new(http_tx),
        );
        let http_handle = sockets.add(http_socket);

        Self { device, iface, sockets, dhcp_handle, http_handle }
    }

    /// Advance Ethernet, DHCP, and interface timers. Call frequently from the
    /// foreground; no interrupt or executor is required.
    pub fn poll(
        &mut self,
        now_ms: u32,
        filesystem: &littlefs2::fs::Filesystem<'_, crate::nand_fs::ActiveStorage>,
        http_upload: &mut HttpUploadState,
        http_download: &mut HttpDownloadState,
    ) {
        if !is_initialized() {
            return;
        }

        let timestamp = Instant::from_millis(now_ms as i64);

        // Process ingress one frame at a time and give the HTTP application a
        // chance to consume request data before a following TCP FIN is handled.
        // smoltcp's TCP recv API reports Finished in CloseWait, even when the
        // receive queue still contains bytes, so draining after FIN is too late.
        self.iface.poll_maintenance(timestamp);
        loop {
            match self.iface.poll_ingress_single(timestamp, &mut self.device, &mut self.sockets) {
                smoltcp::iface::PollIngressSingleResult::None => break,
                smoltcp::iface::PollIngressSingleResult::PacketProcessed
                | smoltcp::iface::PollIngressSingleResult::SocketStateChanged => {
                    self.poll_http(filesystem, http_upload, http_download);
                }
            }
        }

        use smoltcp::socket::dhcpv4::Event as DhcpEvent;
        if let Some(event) = self.sockets.get_mut::<smoltcp::socket::dhcpv4::Socket>(self.dhcp_handle).poll() {
            match event {
                DhcpEvent::Configured(config) => {
                    let address = config.address;
                    let router = config.router;
                    self.iface.update_ip_addrs(|addrs| {
                        addrs.clear();
                        let _ = addrs.push(smoltcp::wire::IpCidr::Ipv4(address));
                    });
                    self.iface.routes_mut().remove_default_ipv4_route();
                    if let Some(gateway) = router {
                        let _ = self.iface.routes_mut().add_default_ipv4_route(gateway);
                    }
                    IPV4_ADDRESS.store(u32::from_be_bytes(address.address().octets()), Ordering::Release);
                    IPV4_PREFIX.store(address.prefix_len(), Ordering::Release);
                }
                DhcpEvent::Deconfigured => {
                    self.iface.update_ip_addrs(|addrs| addrs.clear());
                    self.iface.routes_mut().remove_default_ipv4_route();
                    IPV4_ADDRESS.store(0, Ordering::Release);
                    IPV4_PREFIX.store(0, Ordering::Release);
                }
            }
        }

        self.poll_http(filesystem, http_upload, http_download);
        let _ = self.iface.poll_egress(timestamp, &mut self.device, &mut self.sockets);
    }

}