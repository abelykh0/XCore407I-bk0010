//! STM32F407 RMII Ethernet MAC/DMA driver for the XCore407I.
//!
//! DMA descriptors and packet buffers are ordinary statics in SRAM (.bss).
//! Do not move them to CCM: the STM32F407 Ethernet DMA cannot access CCM.
//! DMA interrupts acknowledge hardware events; smoltcp remains foreground-polled.

use core::marker::PhantomData;
use core::ptr::{addr_of_mut, read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
use smoltcp::phy::{ChecksumCapabilities, Device, DeviceCapabilities, Medium, RxToken, TxToken};
use smoltcp::time::Instant;

const RCC_BASE: usize = 0x4002_3800;
const RCC_AHB1ENR: *mut u32 = (RCC_BASE + 0x30) as *mut u32;
const RCC_AHB1RSTR: *mut u32 = (RCC_BASE + 0x10) as *mut u32;
const RCC_APB2ENR: *mut u32 = (RCC_BASE + 0x44) as *mut u32;
const SYSCFG_PMC: *mut u32 = 0x4001_3804 as *mut u32;

const ETH_MAC_BASE: usize = 0x4002_8000;
const ETH_DMA_BASE: usize = 0x4002_9000;
const ETH_MACCR: *mut u32 = ETH_MAC_BASE as *mut u32;
const ETH_MACDBGR: *mut u32 = (ETH_MAC_BASE + 0x34) as *mut u32;
const ETH_MACFFR: *mut u32 = (ETH_MAC_BASE + 0x04) as *mut u32;
const ETH_MACMIIAR: *mut u32 = (ETH_MAC_BASE + 0x10) as *mut u32;
const ETH_MACMIIDR: *mut u32 = (ETH_MAC_BASE + 0x14) as *mut u32;
const ETH_MACA0HR: *mut u32 = (ETH_MAC_BASE + 0x40) as *mut u32;
const ETH_MACA0LR: *mut u32 = (ETH_MAC_BASE + 0x44) as *mut u32;
const ETH_DMABMR: *mut u32 = ETH_DMA_BASE as *mut u32;
const ETH_DMATPDR: *mut u32 = (ETH_DMA_BASE + 0x04) as *mut u32;
const ETH_DMARPDR: *mut u32 = (ETH_DMA_BASE + 0x08) as *mut u32;
const ETH_DMARDLAR: *mut u32 = (ETH_DMA_BASE + 0x0c) as *mut u32;
const ETH_DMATDLAR: *mut u32 = (ETH_DMA_BASE + 0x10) as *mut u32;
const ETH_DMASR: *mut u32 = (ETH_DMA_BASE + 0x14) as *mut u32;
const ETH_DMAOMR: *mut u32 = (ETH_DMA_BASE + 0x18) as *mut u32;
const ETH_DMAIER: *mut u32 = (ETH_DMA_BASE + 0x1c) as *mut u32;
const ETH_DMACHTDR: *mut u32 = (ETH_DMA_BASE + 0x48) as *mut u32;
const DMA_IRQ_CLEAR_MASK: u32 = (1 << 0) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 13) | (1 << 14) | (1 << 15) | (1 << 16);
const DMA_IRQ_ENABLE_MASK: u32 = (1 << 0) | (1 << 2) | (1 << 6) | (1 << 13) | (1 << 15) | (1 << 16);

const ETHMACEN: u32 = 1 << 25;
const ETHMACTXEN: u32 = 1 << 26;
const ETHMACRXEN: u32 = 1 << 27;
const ETHMACRST: u32 = 1 << 25;
const SYSCFGEN: u32 = 1 << 14;
const MII_RMII_SEL: u32 = 1 << 23;
const MII_BUSY: u32 = 1;
const MII_CR_DIV_102: u32 = 0b100 << 2;
const MDIO_TIMEOUT: u32 = 1_000_000;
const PHY_NEGOTIATION_POLLS: u32 = 5_000;
const PHY_NEGOTIATION_DELAY: u32 = 50_000;

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
static DMA_IRQ_COUNT: AtomicU32 = AtomicU32::new(0);

/// Initialize the RMII MAC and DMA rings after board RMII pins are configured.
/// Returns false if the expected DP83848 PHY cannot be read.
pub fn initialize() -> bool {
    unsafe {
        write_volatile(RCC_APB2ENR, read_volatile(RCC_APB2ENR) | SYSCFGEN);
        write_volatile(SYSCFG_PMC, read_volatile(SYSCFG_PMC) | MII_RMII_SEL);

        write_volatile(
            RCC_AHB1ENR,
            read_volatile(RCC_AHB1ENR) | ETHMACEN | ETHMACTXEN | ETHMACRXEN,
        );
        let _ = read_volatile(RCC_AHB1ENR);

        // Reset the Ethernet MAC/DMA block before programming descriptors.
        write_volatile(RCC_AHB1RSTR, read_volatile(RCC_AHB1RSTR) | ETHMACRST);
        write_volatile(RCC_AHB1RSTR, read_volatile(RCC_AHB1RSTR) & !ETHMACRST);
        let _ = read_volatile(RCC_AHB1RSTR);

        write_volatile(ETH_MACMIIAR, MII_CR_DIV_102);
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
        write_volatile(ETH_DMABMR, 1);
        let mut timeout = MDIO_TIMEOUT;
        while read_volatile(ETH_DMABMR) & 1 != 0 {
            if timeout == 0 { return false; }
            timeout -= 1;
        }
        write_volatile(ETH_DMABMR, (1 << 25) | (1 << 16) | (32 << 8) | (32 << 17) | (1 << 23));
        write_volatile(ETH_DMAOMR, (1 << 26) | (1 << 25) | (1 << 21));
        write_volatile(ETH_DMARDLAR, rx_desc as u32);
        write_volatile(ETH_DMATDLAR, tx_desc as u32);
        write_volatile(ETH_DMAIER, 0);
        write_volatile(ETH_DMASR, 0x1e00_01ff);

        // Locally administered unicast address 02:12:09:40:70:01.
        write_volatile(ETH_MACA0LR, 0x4009_1202);
        write_volatile(ETH_MACA0HR, 0x0000_0170);

        // Match the PHY only after link and autonegotiation are both valid.
        // If there is no negotiated link, use the conservative 10 Mb/s half-
        // duplex mode until a later link-state update is implemented.
        let negotiated = phy.physts & ((1 << 4) | (1 << 0)) == ((1 << 4) | (1 << 0));
        let mut maccr = (1 << 25) | (1 << 7); // strip CRC, automatic pad/CRC stripping
        if negotiated && phy.physts & (1 << 1) == 0 { maccr |= 1 << 14; } // 100 Mb/s
        if negotiated && phy.physts & (1 << 2) != 0 { maccr |= 1 << 11; } // full duplex
        write_volatile(ETH_MACCR, maccr);
        write_volatile(ETH_MACFFR, 0); // normal destination filtering

        // Follow the STM32F4 MAC/DMA startup order: enable the MAC first,
        // flush its TX FIFO, then start the DMA engines.
        let enabled_maccr = maccr | (1 << 3) | (1 << 2);
        write_volatile(ETH_MACCR, enabled_maccr);

        // ST requires at least four TX_CLK/RX_CLK cycles after changing TE/RE.
        // A bounded delay also avoids relying on compiler-only ordering.
        let _ = read_volatile(ETH_MACCR);
        for _ in 0..256 {
            core::hint::spin_loop();
        }
        let _ = read_volatile(ETH_MACCR);

        // Flush the transmit FIFO before enabling DMA transmission.
        write_volatile(ETH_DMAOMR, read_volatile(ETH_DMAOMR) | (1 << 20));
        let mut flush_timeout = MDIO_TIMEOUT;
        while read_volatile(ETH_DMAOMR) & (1 << 20) != 0 {
            if flush_timeout == 0 {
                return false;
            }
            flush_timeout -= 1;
        }

        // Start DMA only after the MAC is ready, then demand descriptor fetch.
        write_volatile(ETH_DMAOMR, read_volatile(ETH_DMAOMR) | (1 << 1) | (1 << 13));
        write_volatile(ETH_DMARPDR, 0);
        write_volatile(ETH_DMATPDR, 0);
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
        write_volatile(ETH_DMASR, DMA_IRQ_CLEAR_MASK);
        write_volatile(ETH_DMAIER, DMA_IRQ_ENABLE_MASK);
    }
}

/// Ethernet DMA IRQ top half: acknowledge pending events only.
/// Packet handling and all smoltcp/DHCP work remain in foreground context.
pub fn on_dma_interrupt() {
    unsafe {
        let status = read_volatile(ETH_DMASR);
        let pending = status & DMA_IRQ_CLEAR_MASK;
        if pending != 0 {
            write_volatile(ETH_DMASR, pending);
            DMA_IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// Number of Ethernet DMA interrupts serviced by the CPU.
pub fn dma_interrupt_count() -> u32 {
    DMA_IRQ_COUNT.load(Ordering::Relaxed)
}

/// Whether MAC/DMA initialization completed successfully.
pub fn is_initialized() -> bool {
    MAC_DMA_READY.load(Ordering::Acquire)
}

/// Result of a bounded MDIO scan for the board's DP83848 PHY.
pub fn probe_dp83848() -> PhyStatus {
    unsafe {
        write_volatile(RCC_APB2ENR, read_volatile(RCC_APB2ENR) | SYSCFGEN);
        write_volatile(RCC_AHB1ENR, read_volatile(RCC_AHB1ENR) | ETHMACEN);
        let _ = read_volatile(RCC_AHB1ENR);
        let initial = read_volatile(ETH_MACMIIAR);
        write_volatile(ETH_MACMIIAR, (initial & !((0b111 << 2) | MII_BUSY)) | MII_CR_DIV_102);

        for address in 0u8..32 {
            let id1 = match mdio_read(address, 2) {
                Some(value) => value,
                None => return empty_status(true),
            };
            if id1 != DP83848_ID1 { continue; }
            let id2 = match mdio_read(address, 3) {
                Some(value) => value,
                None => return empty_status(true),
            };
            if id2 & DP83848_ID2_MASK != DP83848_ID2 { continue; }
            let _ = match mdio_read(address, 1) {
                Some(value) => value,
                None => return empty_status(true),
            };
            let bmsr = match mdio_read(address, 1) {
                Some(value) => value,
                None => return empty_status(true),
            };
            let physts = match mdio_read(address, 0x10) {
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
        unsafe {
            // BMSR link and autoneg-complete bits are latch-low; read twice.
            let _ = mdio_read(phy.address, 1);
            if let Some(bmsr) = mdio_read(phy.address, 1) {
                phy.bmsr = bmsr;
            } else {
                break;
            }
            if let Some(physts) = mdio_read(phy.address, 0x10) {
                phy.physts = physts;
            } else {
                break;
            }
        }
        if phy.bmsr & (1 << 5) != 0
            && phy.physts & ((1 << 4) | (1 << 0)) == ((1 << 4) | (1 << 0))
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
        while read_volatile(ETH_MACMIIAR) & MII_BUSY != 0 {
            if remaining == 0 { return None; }
            remaining -= 1;
        }
        let operation = ((phy_address as u32) << 11)
            | ((register as u32) << 6)
            | MII_CR_DIV_102 | MII_BUSY;
        write_volatile(ETH_MACMIIAR, operation);
        remaining = MDIO_TIMEOUT;
        while read_volatile(ETH_MACMIIAR) & MII_BUSY != 0 {
            if remaining == 0 { return None; }
            remaining -= 1;
        }
        Some((read_volatile(ETH_MACMIIDR) & 0xffff) as u16)
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
                RX_ERRORS.fetch_add(1, Ordering::Relaxed);
                write_volatile(ETH_DMARPDR, 0);
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
            record_rx_protocol(bytes);
            let result = f(bytes);
            RX_FRAMES.fetch_add(1, Ordering::Relaxed);
            let desc = addr_of_mut!(RX_DESCRIPTORS).cast::<DmaDescriptor>().add(self.index);
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
            write_volatile(addr_of_mut!((*desc).words[0]), DESC_OWN);
            cortex_m::asm::dsb();
            write_volatile(ETH_DMARPDR, 0);
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
            write_volatile(ETH_DMATPDR, addr_of_mut!(TX_DESCRIPTOR) as u32);
            TX_FRAMES.fetch_add(1, Ordering::Relaxed);
            result
        }
    }
}


static RX_FRAMES: AtomicU32 = AtomicU32::new(0);
static TX_FRAMES: AtomicU32 = AtomicU32::new(0);
static RX_ERRORS: AtomicU32 = AtomicU32::new(0);
static RX_ARP_FRAMES: AtomicU32 = AtomicU32::new(0);
static RX_IPV4_FRAMES: AtomicU32 = AtomicU32::new(0);
static RX_DHCP_UDP_FRAMES: AtomicU32 = AtomicU32::new(0);
static DHCP_CONFIGURED_EVENTS: AtomicU32 = AtomicU32::new(0);
static DHCP_DECONFIGURED_EVENTS: AtomicU32 = AtomicU32::new(0);

fn record_rx_protocol(bytes: &[u8]) {
    if bytes.len() < 14 { return; }
    let ether_type = u16::from_be_bytes([bytes[12], bytes[13]]);
    if ether_type == 0x0806 {
        RX_ARP_FRAMES.fetch_add(1, Ordering::Relaxed);
    } else if ether_type == 0x0800 {
        RX_IPV4_FRAMES.fetch_add(1, Ordering::Relaxed);
        if bytes.len() < 34 || bytes[23] != 17 { return; }
        let ip_header_len = ((bytes[14] & 0x0f) as usize) * 4;
        if ip_header_len < 20 { return; }
        let udp = 14 + ip_header_len;
        if bytes.len() < udp + 8 { return; }
        let src = u16::from_be_bytes([bytes[udp], bytes[udp + 1]]);
        let dst = u16::from_be_bytes([bytes[udp + 2], bytes[udp + 3]]);
        if (src == 67 && dst == 68) || (src == 68 && dst == 67) {
            RX_DHCP_UDP_FRAMES.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// Protocol counters for frames accepted from the RX descriptors.
pub fn network_diagnostic() -> (u32, u32, u32, u32, u32) {
    (
        RX_ARP_FRAMES.load(Ordering::Relaxed),
        RX_IPV4_FRAMES.load(Ordering::Relaxed),
        RX_DHCP_UDP_FRAMES.load(Ordering::Relaxed),
        DHCP_CONFIGURED_EVENTS.load(Ordering::Relaxed),
        DHCP_DECONFIGURED_EVENTS.load(Ordering::Relaxed),
    )
}

/// Snapshot DMA engine and RX descriptor state without allocating memory.
pub fn dma_diagnostic() -> (u32, u32, u32, u32, u32, u32, u32, u32, u32, u32, u32, u32, u32) {
    unsafe {
        let rx = addr_of_mut!(RX_DESCRIPTORS).cast::<DmaDescriptor>();
        (
            read_volatile(ETH_DMASR),
            read_volatile(ETH_DMAOMR),
            read_volatile(addr_of_mut!((*rx).words[0])),
            read_volatile(addr_of_mut!((*rx.add(1)).words[0])),
            read_volatile(addr_of_mut!(TX_DESCRIPTOR.words[0])),
            read_volatile(addr_of_mut!(TX_DESCRIPTOR.words[1])),
            read_volatile(addr_of_mut!(TX_DESCRIPTOR.words[2])),
            read_volatile(addr_of_mut!(TX_DESCRIPTOR.words[3])),
            read_volatile(ETH_DMACHTDR),
            read_volatile(ETH_DMAIER),
            read_volatile(ETH_MACCR),
            read_volatile(SYSCFG_PMC),
            read_volatile(ETH_MACDBGR),
        )
    }
}

/// Ethernet frames accepted by the stack, transmitted, and rejected by RX.
pub fn frame_counts() -> (u32, u32, u32) {
    (
        RX_FRAMES.load(Ordering::Relaxed),
        TX_FRAMES.load(Ordering::Relaxed),
        RX_ERRORS.load(Ordering::Relaxed),
    )
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
}

impl<'a> EthernetNetwork<'a> {
    /// Create a network interface using caller-provided fixed socket storage.
    pub fn new(storage: &'a mut [smoltcp::iface::SocketStorage<'a>]) -> Self {
        use smoltcp::iface::{Config, Interface, SocketSet};
        use smoltcp::socket::dhcpv4::Socket as Dhcpv4Socket;
        use smoltcp::wire::{EthernetAddress, HardwareAddress};

        let mut device = EthernetDevice::new();
        let mut config = Config::new(HardwareAddress::Ethernet(EthernetAddress(MAC_ADDRESS)));
        config.random_seed = 0x4071_1209_2026_0001;
        let iface = Interface::new(config, &mut device, Instant::from_millis(0));
        let mut sockets = SocketSet::new(storage);
        let dhcp_handle = sockets.add(Dhcpv4Socket::new());

        Self { device, iface, sockets, dhcp_handle }
    }

    /// Advance Ethernet, DHCP, and interface timers. Call frequently from the
    /// foreground; no interrupt or executor is required.
    pub fn poll(&mut self, now_ms: u32) {
        if !is_initialized() {
            return;
        }

        let timestamp = Instant::from_millis(now_ms as i64);
        let _ = self.iface.poll(timestamp, &mut self.device, &mut self.sockets);

        use smoltcp::socket::dhcpv4::Event as DhcpEvent;
        if let Some(event) = self.sockets.get_mut::<smoltcp::socket::dhcpv4::Socket>(self.dhcp_handle).poll() {
            match event {
                DhcpEvent::Configured(config) => {
                    DHCP_CONFIGURED_EVENTS.fetch_add(1, Ordering::Relaxed);
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
                    DHCP_DECONFIGURED_EVENTS.fetch_add(1, Ordering::Relaxed);
                    self.iface.update_ip_addrs(|addrs| addrs.clear());
                    self.iface.routes_mut().remove_default_ipv4_route();
                    IPV4_ADDRESS.store(0, Ordering::Release);
                    IPV4_PREFIX.store(0, Ordering::Release);
                }
            }
        }

    }
}
