//! STM32F407 RMII Ethernet MAC/DMA driver for the XCore407I.
//!
//! DMA descriptors and packet buffers are ordinary statics in SRAM (.bss).
//! Do not move them to CCM: the STM32F407 Ethernet DMA cannot access CCM.
//! DMA interrupts acknowledge hardware events; smoltcp remains foreground-polled.

use core::marker::PhantomData;
use core::ptr::{addr_of_mut, read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
use stm32f4xx_hal::pac;
use smoltcp::phy::{ChecksumCapabilities, Device, DeviceCapabilities, Medium, RxToken, TxToken};
use smoltcp::time::Instant;






const DMA_IRQ_CLEAR_MASK: u32 = (1 << 0) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 13) | (1 << 14) | (1 << 15) | (1 << 16);
const DMA_IRQ_ENABLE_MASK: u32 = (1 << 0) | (1 << 2) | (1 << 6) | (1 << 13) | (1 << 15) | (1 << 16);
const DMA_OMR_INIT: u32 = (1 << 26) | (1 << 25) | (1 << 21);

const DP83848_PHYSTS_LINK: u16 = 1 << 0;
const DP83848_PHYSTS_SPEED_10_MBPS: u16 = 1 << 1;
const DP83848_PHYSTS_FULL_DUPLEX: u16 = 1 << 2;
const DP83848_PHYSTS_AUTONEG_COMPLETE: u16 = 1 << 4;
const PHY_BMSR_AUTONEG_COMPLETE: u16 = 1 << 5;
const MDIO_TIMEOUT: u32 = 1_000_000;
const PHY_NEGOTIATION_POLLS: u32 = 5_000;
const PHY_NEGOTIATION_DELAY: u32 = 50_000;

// Keep the request buffer small. The response buffers grow by only 512 bytes
// in total; directory output is paginated instead of buffering the whole listing.
const HTTP_RX_BUFFER_SIZE: usize = 256;
const HTTP_TX_BUFFER_SIZE: usize = 768;
const HTTP_BODY_BUFFER_SIZE: usize = 640;
const HTTP_DIRECTORY_PAGE_SIZE: usize = 2;
static mut HTTP_RX_BUFFER: [u8; HTTP_RX_BUFFER_SIZE] = [0; HTTP_RX_BUFFER_SIZE];
static mut HTTP_TX_BUFFER: [u8; HTTP_TX_BUFFER_SIZE] = [0; HTTP_TX_BUFFER_SIZE];
static mut HTTP_BODY_BUFFER: [u8; HTTP_BODY_BUFFER_SIZE] = [0; HTTP_BODY_BUFFER_SIZE];
const HTTP_RESPONSE_HEADER: &[u8] =
    b"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nConnection: close\r\n\r\n";
const HTTP_PAGE: &[u8] = br#"<!doctype html><meta charset=utf-8><title>LittleFS</title><h1>LittleFS files</h1><ul id=f></ul><button id=b hidden>More</button><script>
let o=0,f=document.getElementById('f'),b=document.getElementById('b');async function l(){b.disabled=1;try{let r=await fetch('/api/files?offset='+o),d=new DOMParser().parseFromString(await r.text(),'text/html');f.insertAdjacentHTML('beforeend',d.querySelector('ul').innerHTML);let n=d.querySelector('[data-next]');if(n){o=+n.dataset.next;b.hidden=0}else b.hidden=1}catch(e){b.hidden=0;b.textContent='Retry'}b.disabled=0}b.onclick=l;l()</script>"#;

const RX_COUNT: usize = 2;
const FRAME_CAPACITY: usize = 1536;
const DESC_OWN: u32 = 1 << 31;
const RX_DESC_FS: u32 = 1 << 9;
const RX_DESC_LS: u32 = 1 << 8;
const RX_DESC_ES: u32 = 1 << 15;
const RX_DESC_FL_SHIFT: u32 = 16;
const RX_DESC_RCH: u32 = 1 << 14;
const TX_DESC_TCH: u32 = 1 << 20;
const TX_DESC_FS: u32 = 1 << 28;
const TX_DESC_LS: u32 = 1 << 29;
const TX_DESC_IC: u32 = 1 << 30;

const MAC_ADDRESS: [u8; 6] = [0x02, 0x12, 0x09, 0x40, 0x70, 0x01];

const MII_REG_BMSR: u8 = 0x01;
const MII_REG_PHY_ID1: u8 = 0x02;
const MII_REG_PHY_ID2: u8 = 0x03;
const DP83848_REG_PHYSTS: u8 = 0x10;

const DP83848_ID1: u16 = 0x2000;
const DP83848_ID2_MASK: u16 = 0xfff0;
const DP83848_ID2: u16 = 0x5c90;

#[repr(C, align(8))]
struct DmaDescriptor {
    words: [u32; 4],
}

#[repr(C, align(4))]
struct FrameBuffer([u8; FRAME_CAPACITY]);

// These statics are deliberately not placed in the CCM section.
static mut RX_DESCRIPTORS: [DmaDescriptor; RX_COUNT] = [
    DmaDescriptor { words: [0; 4] },
    DmaDescriptor { words: [0; 4] },
];
static mut RX_BUFFERS: [FrameBuffer; RX_COUNT] = [
    FrameBuffer([0; FRAME_CAPACITY]),
    FrameBuffer([0; FRAME_CAPACITY]),
];
static mut TX_DESCRIPTOR: DmaDescriptor = DmaDescriptor { words: [0; 4] };
static mut TX_BUFFER: FrameBuffer = FrameBuffer([0; FRAME_CAPACITY]);

#[derive(Clone, Copy)]
pub struct PhyStatus {
    pub found: bool,
    pub address: u8,
    pub id1: u16,
    pub id2: u16,
    pub bmsr: u16,
    pub physts: u16,
    pub mdio_timeout: bool,
}

static MAC_DMA_READY: AtomicBool = AtomicBool::new(false);

/// Synchronize a MAC register write across the TX_CLK/RX_CLK clock domain.
/// ST's F4 HAL performs a read, delay, and same-value write after these updates.
fn mac_register_sync(register: *mut u32) {
    unsafe {
        let value = read_volatile(register);
        for _ in 0..256 {
            core::hint::spin_loop();
        }
        write_volatile(register, value);
        let _ = read_volatile(register);
    }
}

/// Initialize the RMII MAC and DMA rings after board RMII pins are configured.
/// Returns false if the expected DP83848 PHY cannot be read.
pub fn initialize() -> bool {
    unsafe {
        (*pac::RCC::PTR).apb2enr().modify(|_, w| w.syscfgen().set_bit());
        (*pac::SYSCFG::PTR).pmc().modify(|_, w| w.mii_rmii_sel().set_bit());

        // Enable the MAC, TX, and RX clocks through the typed RCC PAC accessor.
        (*pac::RCC::PTR).ahb1enr().modify(|_, w| {
            w.ethmacen().set_bit()
                .ethmactxen().set_bit()
                .ethmacrxen().set_bit()
        });
        let _ = (*pac::RCC::PTR).ahb1enr().read();

        // Reset the Ethernet MAC/DMA block before programming descriptors.
        (*pac::RCC::PTR).ahb1rstr().modify(|_, w| w.ethmacrst().set_bit());
        (*pac::RCC::PTR).ahb1rstr().modify(|_, w| w.ethmacrst().clear_bit());
        let _ = read_volatile((*pac::RCC::PTR).ahb1rstr().as_ptr());

        (*pac::ETHERNET_MAC::PTR).macmiiar().write(|w| w.cr().variant(pac::ethernet_mac::macmiiar::CR::Cr150_168));
        let phy = probe_dp83848();
        if !phy.found || phy.mdio_timeout {
            return false;
        }
        // PHYSTS speed/duplex bits are not valid until autonegotiation completes.
        // Wait a bounded interval so a late link negotiation cannot leave MACCR
        // configured differently from the PHY. A disconnected cable must not
        // prevent the Ethernet stack from starting.
        let phy = wait_for_phy_negotiation(phy);

        // Software-owned, chained descriptor rings. Enhanced descriptors are
        // disabled; each descriptor is the four-word basic format from RM0090.
        let rx_desc = addr_of_mut!(RX_DESCRIPTORS).cast::<DmaDescriptor>();
        let rx_buf = addr_of_mut!(RX_BUFFERS).cast::<FrameBuffer>();
        for i in 0..RX_COUNT {
            let desc = rx_desc.add(i);
            let next = rx_desc.add((i + 1) % RX_COUNT);
            let buffer = rx_buf.add(i);
            write_volatile(addr_of_mut!((*desc).words[1]), RX_DESC_RCH | FRAME_CAPACITY as u32);
            write_volatile(addr_of_mut!((*desc).words[2]), (*buffer).0.as_mut_ptr() as u32);
            write_volatile(addr_of_mut!((*desc).words[3]), next as u32);
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
            write_volatile(addr_of_mut!((*desc).words[0]), DESC_OWN);
        }

        let tx_desc = addr_of_mut!(TX_DESCRIPTOR);
        let tx_buf = addr_of_mut!(TX_BUFFER);
        // In the STM32F4 basic descriptor format, TCH is TDES0 bit 20.
        // TDES1 holds only the buffer byte counts.
        write_volatile(addr_of_mut!((*tx_desc).words[0]), TX_DESC_TCH);
        write_volatile(addr_of_mut!((*tx_desc).words[1]), 0);
        write_volatile(addr_of_mut!((*tx_desc).words[2]), (*tx_buf).0.as_mut_ptr() as u32);
        write_volatile(addr_of_mut!((*tx_desc).words[3]), tx_desc as u32);

        // DMA software reset, then conservative fixed-burst configuration.
        (*pac::ETHERNET_DMA::PTR).dmabmr().write(|w| w.sr().set_bit());
        let mut timeout = MDIO_TIMEOUT;
        while (*pac::ETHERNET_DMA::PTR).dmabmr().read().sr().bit_is_set() {
            if timeout == 0 { return false; }
            timeout -= 1;
        }
        (*pac::ETHERNET_DMA::PTR).dmabmr().write(|w| {
            w.fb().set_bit();
            unsafe { w.pbl().bits(32); }
            unsafe { w.rdp().bits(32); }
            w.usp().set_bit();
            w.aab().set_bit()
        });
        (*pac::ETHERNET_DMA::PTR).dmaomr().write(|w| unsafe { w.bits(DMA_OMR_INIT) });
        write_volatile((*pac::ETHERNET_DMA::PTR).dmardlar().as_ptr(), rx_desc as u32);
        write_volatile((*pac::ETHERNET_DMA::PTR).dmatdlar().as_ptr(), tx_desc as u32);
        (*pac::ETHERNET_DMA::PTR).dmaier().write(|w| unsafe { w.bits(0) });
        (*pac::ETHERNET_DMA::PTR).dmasr().write(|w| unsafe { w.bits(DMA_IRQ_CLEAR_MASK) });

        // Locally administered unicast address 02:12:09:40:70:01.
        write_volatile((*pac::ETHERNET_MAC::PTR).maca0lr().as_ptr(), 0x4009_1202);
        write_volatile((*pac::ETHERNET_MAC::PTR).maca0hr().as_ptr(), 0x0000_0170);

        // Match the PHY only after link and autonegotiation are both valid.
        // If there is no negotiated link, use the conservative 10 Mb/s half-
        // duplex mode until a later link-state update is implemented.
        let negotiated = phy.physts & (DP83848_PHYSTS_AUTONEG_COMPLETE | DP83848_PHYSTS_LINK) == (DP83848_PHYSTS_AUTONEG_COMPLETE | DP83848_PHYSTS_LINK);

        // Match the STM32F4 HAL register synchronization sequence. The MAC
        // configuration crosses into the TX_CLK/RX_CLK clock domain; ST waits
        // at least four clock cycles after each MAC register write.
        (*pac::ETHERNET_MAC::PTR).maccr().write(|w| {
            w.cstf().set_bit()
                .apcs().set_bit()
                .fes().bit(negotiated && phy.physts & DP83848_PHYSTS_SPEED_10_MBPS == 0)
                .dm().bit(negotiated && phy.physts & DP83848_PHYSTS_FULL_DUPLEX != 0)
        });
        mac_register_sync((*pac::ETHERNET_MAC::PTR).maccr().as_ptr());
        write_volatile((*pac::ETHERNET_MAC::PTR).macffr().as_ptr(), 0); // normal destination filtering
        mac_register_sync((*pac::ETHERNET_MAC::PTR).macffr().as_ptr());

        // Enable TE and RE in separate synchronized writes, as in HAL_ETH_Start.
        (*pac::ETHERNET_MAC::PTR).maccr().modify(|_, w| w.te().set_bit());
        mac_register_sync((*pac::ETHERNET_MAC::PTR).maccr().as_ptr());
        (*pac::ETHERNET_MAC::PTR).maccr().modify(|_, w| w.re().set_bit());
        mac_register_sync((*pac::ETHERNET_MAC::PTR).maccr().as_ptr());

        // Flush the transmit FIFO before enabling DMA transmission.
        (*pac::ETHERNET_DMA::PTR).dmaomr().modify(|_, w| w.ftf().set_bit());
        let mut flush_timeout = MDIO_TIMEOUT;
        while (*pac::ETHERNET_DMA::PTR).dmaomr().read().ftf().bit_is_set() {
            if flush_timeout == 0 {
                return false;
            }
            flush_timeout -= 1;
        }

        // Start DMA only after the MAC is ready, then demand descriptor fetch.
        (*pac::ETHERNET_DMA::PTR).dmaomr().modify(|_, w| w.sr().set_bit().st().set_bit());
        write_volatile((*pac::ETHERNET_DMA::PTR).dmarpdr().as_ptr(), 0);
        write_volatile((*pac::ETHERNET_DMA::PTR).dmatpdr().as_ptr(), 0);
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        cortex_m::asm::dsb();

        MAC_DMA_READY.store(true, Ordering::Release);
    }
    true
}


/// Enable receive, transmit-complete, and DMA-error interrupt sources.
/// The NVIC line is enabled separately by the board startup code.
pub fn enable_dma_interrupts() {
    unsafe {
        // Clear stale write-one-to-clear events before enabling sources.
        (*pac::ETHERNET_DMA::PTR).dmasr().write(|w| unsafe { w.bits(DMA_IRQ_CLEAR_MASK) });
        (*pac::ETHERNET_DMA::PTR).dmaier().write(|w| unsafe { w.bits(DMA_IRQ_ENABLE_MASK) });
    }
}

/// Ethernet DMA IRQ top half: acknowledge pending events only.
/// Packet handling and all smoltcp/DHCP work remain in foreground context.
pub fn on_dma_interrupt() {
    unsafe {
        let pending = (*pac::ETHERNET_DMA::PTR).dmasr().read().bits() & DMA_IRQ_CLEAR_MASK;
        if pending != 0 {
            (*pac::ETHERNET_DMA::PTR).dmasr().write(|w| unsafe { w.bits(pending) });
        }
    }
}


/// Append bytes to the bounded HTTP response body.
fn append_http_bytes(buffer: &mut [u8], length: &mut usize, bytes: &[u8]) -> bool {
    if bytes.len() > buffer.len().saturating_sub(*length) {
        return false;
    }
    buffer[*length..*length + bytes.len()].copy_from_slice(bytes);
    *length += bytes.len();
    true
}

fn append_http_decimal(buffer: &mut [u8], length: &mut usize, mut value: usize) {
    let mut digits = [0u8; 10];
    let mut count = 0;
    loop {
        digits[count] = b'0' + (value % 10) as u8;
        count += 1;
        value /= 10;
        if value == 0 { break; }
    }
    while count != 0 {
        count -= 1;
        if !append_http_bytes(buffer, length, &digits[count..count + 1]) {
            break;
        }
    }
}

/// Whether MAC/DMA initialization completed successfully.
pub fn is_initialized() -> bool {
    MAC_DMA_READY.load(Ordering::Acquire)
}

/// Result of a bounded MDIO scan for the board's DP83848 PHY.
pub fn probe_dp83848() -> PhyStatus {
    unsafe {
        (*pac::RCC::PTR).apb2enr().modify(|_, w| w.syscfgen().set_bit());
        (*pac::RCC::PTR).ahb1enr().modify(|_, w| w.ethmacen().set_bit());
        let _ = (*pac::RCC::PTR).ahb1enr().read();
        (*pac::ETHERNET_MAC::PTR).macmiiar().modify(|_, w| w.cr().variant(pac::ethernet_mac::macmiiar::CR::Cr150_168).mb().clear_bit());

        for address in 0u8..32 {
            let id1 = match mdio_read(address, MII_REG_PHY_ID1) {
                Some(value) => value,
                None => return empty_status(true),
            };
            if id1 != DP83848_ID1 { continue; }
            let id2 = match mdio_read(address, MII_REG_PHY_ID2) {
                Some(value) => value,
                None => return empty_status(true),
            };
            if id2 & DP83848_ID2_MASK != DP83848_ID2 { continue; }
            let _ = match mdio_read(address, MII_REG_BMSR) {
                Some(value) => value,
                None => return empty_status(true),
            };
            let bmsr = match mdio_read(address, MII_REG_BMSR) {
                Some(value) => value,
                None => return empty_status(true),
            };
            let physts = match mdio_read(address, DP83848_REG_PHYSTS) {
                Some(value) => value,
                None => return empty_status(true),
            };
            return PhyStatus { found: true, address, id1, id2, bmsr, physts, mdio_timeout: false };
        }
    }
    empty_status(false)
}

fn empty_status(mdio_timeout: bool) -> PhyStatus {
    PhyStatus { found: false, address: 0xff, id1: 0, id2: 0, bmsr: 0, physts: 0, mdio_timeout }
}

/// Wait for DP83848 link and autonegotiation completion without blocking startup
/// indefinitely. Returns the most recent PHY status even if the wait expires.
fn wait_for_phy_negotiation(mut phy: PhyStatus) -> PhyStatus {
    for _ in 0..PHY_NEGOTIATION_POLLS {
        // BMSR link and autoneg-complete bits are latch-low; read twice.
        let _ = mdio_read(phy.address, MII_REG_BMSR);
        if let Some(bmsr) = mdio_read(phy.address, MII_REG_BMSR) {
            phy.bmsr = bmsr;
        } else {
            break;
        }
        if let Some(physts) = mdio_read(phy.address, DP83848_REG_PHYSTS) {
            phy.physts = physts;
        } else {
            break;
        }
        if phy.bmsr & PHY_BMSR_AUTONEG_COMPLETE != 0
            && phy.physts & (DP83848_PHYSTS_AUTONEG_COMPLETE | DP83848_PHYSTS_LINK) == (DP83848_PHYSTS_AUTONEG_COMPLETE | DP83848_PHYSTS_LINK)
        {
            break;
        }
        for _ in 0..PHY_NEGOTIATION_DELAY {
            core::hint::spin_loop();
        }
    }
    phy
}

fn mdio_read(phy_address: u8, register: u8) -> Option<u16> {
    unsafe {
        let mut remaining = MDIO_TIMEOUT;
        while (*pac::ETHERNET_MAC::PTR).macmiiar().read().mb().bit_is_set() {
            if remaining == 0 { return None; }
            remaining -= 1;
        }
        (*pac::ETHERNET_MAC::PTR).macmiiar().write(|w| {
            unsafe { w.pa().bits(phy_address).mr().bits(register); }
            w.cr().variant(pac::ethernet_mac::macmiiar::CR::Cr150_168).mb().set_bit()
        });
        remaining = MDIO_TIMEOUT;
        while (*pac::ETHERNET_MAC::PTR).macmiiar().read().mb().bit_is_set() {
            if remaining == 0 { return None; }
            remaining -= 1;
        }
        Some((read_volatile((*pac::ETHERNET_MAC::PTR).macmiidr().as_ptr()) & 0xffff) as u16)
    }
}

/// Minimal polled Ethernet device for smoltcp. One TX descriptor deliberately
/// serializes transmission; RX uses two descriptors to absorb short bursts.
pub struct EthernetDevice {
    rx_index: usize,
}

impl EthernetDevice {
    pub const fn new() -> Self { Self { rx_index: 0 } }

    fn rx_ready(&mut self) -> Option<(usize, usize)> {
        unsafe {
            let desc = addr_of_mut!(RX_DESCRIPTORS).cast::<DmaDescriptor>().add(self.rx_index);
            let status = read_volatile(addr_of_mut!((*desc).words[0]));
            if status & DESC_OWN != 0 { return None; }
            let length = ((status >> RX_DESC_FL_SHIFT) & 0x3fff) as usize;
            if status & RX_DESC_ES != 0 || status & (RX_DESC_FS | RX_DESC_LS) != (RX_DESC_FS | RX_DESC_LS)
                || length < 14 || length > FRAME_CAPACITY
            {
                write_volatile(addr_of_mut!((*desc).words[0]), DESC_OWN);
                self.rx_index = (self.rx_index + 1) % RX_COUNT;
                write_volatile((*pac::ETHERNET_DMA::PTR).dmarpdr().as_ptr(), 0);
                return None;
            }
            Some((self.rx_index, length))
        }
    }

    fn tx_ready() -> bool {
        unsafe { read_volatile(addr_of_mut!(TX_DESCRIPTOR.words[0])) & DESC_OWN == 0 }
    }
}

impl Default for EthernetDevice {
    fn default() -> Self { Self::new() }
}

impl Device for EthernetDevice {
    type RxToken<'a> = EthernetRxToken<'a> where Self: 'a;
    type TxToken<'a> = EthernetTxToken<'a> where Self: 'a;

    fn receive(&mut self, _timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        if !is_initialized() || !Self::tx_ready() { return None; }
        let (index, length) = self.rx_ready()?;
        self.rx_index = (index + 1) % RX_COUNT;
        Some((
            EthernetRxToken { index, length, _borrow: PhantomData },
            EthernetTxToken { _borrow: PhantomData },
        ))
    }

    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        if is_initialized() && Self::tx_ready() {
            Some(EthernetTxToken { _borrow: PhantomData })
        } else {
            None
        }
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.medium = Medium::Ethernet;
        caps.max_transmission_unit = 1514;
        caps.max_burst_size = Some(1);
        caps.checksum = ChecksumCapabilities::default();
        caps
    }
}

pub struct EthernetRxToken<'a> {
    index: usize,
    length: usize,
    _borrow: PhantomData<&'a EthernetDevice>,
}

impl RxToken for EthernetRxToken<'_> {
    fn consume<R, F>(self, f: F) -> R where F: FnOnce(&[u8]) -> R {
        unsafe {
            let buffer = addr_of_mut!(RX_BUFFERS).cast::<FrameBuffer>().add(self.index);
            let bytes = core::slice::from_raw_parts((*buffer).0.as_ptr(), self.length);
            let result = f(bytes);
            let desc = addr_of_mut!(RX_DESCRIPTORS).cast::<DmaDescriptor>().add(self.index);
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
            write_volatile(addr_of_mut!((*desc).words[0]), DESC_OWN);
            cortex_m::asm::dsb();
            write_volatile((*pac::ETHERNET_DMA::PTR).dmarpdr().as_ptr(), 0);
            result
        }
    }
}

pub struct EthernetTxToken<'a> {
    _borrow: PhantomData<&'a EthernetDevice>,
}

impl TxToken for EthernetTxToken<'_> {
    fn consume<R, F>(self, length: usize, f: F) -> R where F: FnOnce(&mut [u8]) -> R {
        assert!(length <= FRAME_CAPACITY);
        unsafe {
            let buffer = addr_of_mut!(TX_BUFFER);
            let bytes = core::slice::from_raw_parts_mut((*buffer).0.as_mut_ptr(), length);
            let result = f(bytes);
            // TDES1 holds the buffer byte count; TCH is in TDES0 on STM32F4.
            write_volatile(addr_of_mut!(TX_DESCRIPTOR.words[1]), length as u32);
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
            cortex_m::asm::dsb();
            // Give the chained descriptor to DMA only after its buffer is ready.
            write_volatile(
                addr_of_mut!(TX_DESCRIPTOR.words[0]),
                DESC_OWN | TX_DESC_TCH | TX_DESC_FS | TX_DESC_LS | TX_DESC_IC,
            );
            // The STM32F4 HAL writes the next free TX descriptor address
            // to DMATPDR when issuing transmit poll demand.
            write_volatile((*pac::ETHERNET_DMA::PTR).dmatpdr().as_ptr(), addr_of_mut!(TX_DESCRIPTOR) as u32);
            result
        }
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

        // The HTTP buffers are static SRAM objects, not stack allocations.
        let http_rx: &'static mut [u8] = unsafe {
            core::slice::from_raw_parts_mut(
                addr_of_mut!(HTTP_RX_BUFFER).cast::<u8>(),
                HTTP_RX_BUFFER_SIZE,
            )
        };
        let http_tx: &'static mut [u8] = unsafe {
            core::slice::from_raw_parts_mut(
                addr_of_mut!(HTTP_TX_BUFFER).cast::<u8>(),
                HTTP_TX_BUFFER_SIZE,
            )
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
    ) {
        if !is_initialized() {
            return;
        }

        let timestamp = Instant::from_millis(now_ms as i64);
        let _ = self.iface.poll(timestamp, &mut self.device, &mut self.sockets);

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

        self.poll_http(filesystem);
    }

    /// Serve a small page shell and an AJAX-paginated LittleFS directory listing.
    fn poll_http(
        &mut self,
        filesystem: &littlefs2::fs::Filesystem<'_, crate::nand_fs::ActiveStorage>,
    ) {
        use smoltcp::socket::tcp::Socket as TcpSocket;

        let socket = self.sockets.get_mut::<TcpSocket>(self.http_handle);
        if !socket.is_open() {
            let _ = socket.listen(80);
            return;
        }

        if socket.can_recv() {
            let mut api_request = false;
            let mut offset = 0usize;
            let received = socket.recv(|data| {
                const API_PREFIX: &[u8] = b"GET /api/files?offset=";
                if let Some(line_end) = data.iter().position(|&byte| byte == b'\n') {
                    let line = &data[..line_end];
                    if let Some(rest) = line.strip_prefix(API_PREFIX) {
                        api_request = true;
                        for &byte in rest {
                            if !byte.is_ascii_digit() { break; }
                            offset = offset.saturating_mul(10)
                                .saturating_add((byte - b'0') as usize)
                                .min(1_000_000);
                        }
                    }
                }
                (data.len(), !data.is_empty())
            }).unwrap_or(false);

            if received && socket.can_send() {
                if !api_request {
                    let _ = socket.send_slice(HTTP_RESPONSE_HEADER);
                    let _ = socket.send_slice(HTTP_PAGE);
                } else {
                    let body = unsafe {
                        core::slice::from_raw_parts_mut(
                            addr_of_mut!(HTTP_BODY_BUFFER).cast::<u8>(),
                            HTTP_BODY_BUFFER_SIZE,
                        )
                    };
                    let mut length = 0usize;
                    let mut seen = 0usize;
                    let mut emitted = 0usize;
                    let mut has_more = false;
                    append_http_bytes(body, &mut length, b"<ul>");

                    let _ = crate::nand_fs::read_root_directory(
                        filesystem,
                        true,
                        |path, is_dir, size| {
                            if seen < offset {
                                seen += 1;
                                return Ok(());
                            }
                            if emitted >= HTTP_DIRECTORY_PAGE_SIZE {
                                has_more = true;
                                return Ok(());
                            }
                            seen += 1;
                            emitted += 1;

                            let name = path.strip_prefix(b"/").unwrap_or(path);
                            let name = name.strip_suffix(b"\0").unwrap_or(name);
                            // Bound worst-case HTML expansion while retaining useful names.
                            let mut name_len = core::cmp::min(name.len(), 32);
                            while name_len > 0 && name_len < name.len()
                                && name[name_len] & 0xc0 == 0x80
                            {
                                name_len -= 1;
                            }
                            append_http_bytes(body, &mut length, b"<li>");
                            for &byte in &name[..name_len] {
                                let escaped = match byte {
                                    b'&' => b"&amp;" as &[u8],
                                    b'<' => b"&lt;",
                                    b'>' => b"&gt;",
                                    b'"' => b"&quot;",
                                    _ => core::slice::from_ref(&byte),
                                };
                                append_http_bytes(body, &mut length, escaped);
                            }
                            if name_len < name.len() {
                                append_http_bytes(body, &mut length, b"... ");
                            }
                            if is_dir {
                                append_http_bytes(body, &mut length, b"/");
                            } else {
                                append_http_bytes(body, &mut length, b" (");
                                append_http_decimal(body, &mut length, size);
                                append_http_bytes(body, &mut length, b" bytes)");
                            }
                            append_http_bytes(body, &mut length, b"</li>");
                            Ok(())
                        },
                    );

                    append_http_bytes(body, &mut length, b"</ul>");
                    if has_more {
                        append_http_bytes(body, &mut length, b"<span data-next=\"");
                        append_http_decimal(body, &mut length, offset.saturating_add(emitted));
                        append_http_bytes(body, &mut length, b"\"></span>");
                    }
                    let _ = socket.send_slice(HTTP_RESPONSE_HEADER);
                    let _ = socket.send_slice(&body[..length]);
                }
                socket.close();
            }
        }
    }
}
