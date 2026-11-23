//! RMII PHY, MAC, and DMA driver plus the smoltcp Ethernet device adapter.
//!
//! PHY negotiation and MAC/DMA setup are kept together because initialization
//! configures the MAC from the negotiated DP83848 link parameters.

use core::marker::PhantomData;
use core::ptr::{addr_of_mut, read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, Ordering};
use stm32f4xx_hal::pac;
use smoltcp::phy::{ChecksumCapabilities, Device, DeviceCapabilities, Medium, RxToken, TxToken};
use smoltcp::time::Instant;

const MII_CR_DIV_102: u32 = 0b100 << 2;

const DP83848_PHYSTS_LINK: u16 = 1 << 0;
const DP83848_PHYSTS_SPEED_10_MBPS: u16 = 1 << 1;
const DP83848_PHYSTS_FULL_DUPLEX: u16 = 1 << 2;
const DP83848_PHYSTS_AUTONEG_COMPLETE: u16 = 1 << 4;
const PHY_BMSR_AUTONEG_COMPLETE: u16 = 1 << 5;
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

pub(super) const MAC_ADDRESS: [u8; 6] = [0x02, 0x12, 0x09, 0x40, 0x70, 0x01];

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

// DMA descriptors and packet buffers occupy DMA-accessible DEMO_CANVAS.
// HTTP/TCP byte buffers follow them in main SRAM; only socket metadata lives
// in CCM, which the Ethernet DMA cannot access.
#[repr(C, align(8))]
struct EthernetDmaStorage {
    rx_descriptors: [DmaDescriptor; RX_COUNT],
    rx_buffers: [FrameBuffer; RX_COUNT],
    tx_descriptor: DmaDescriptor,
    tx_buffer: FrameBuffer,
}

const _: () = assert!(core::mem::size_of::<EthernetDmaStorage>() <= crate::camera::CDC_ETHERNET_DMA_STORAGE_BYTES);

fn dma_storage_ptr() -> *mut EthernetDmaStorage {
    crate::camera::cdc_ethernet_dma_storage_ptr().cast()
}

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

        (*pac::ETHERNET_MAC::PTR).macmiiar().write(|w| w.bits(MII_CR_DIV_102));
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
        let dma = dma_storage_ptr();
        let rx_desc = addr_of_mut!((*dma).rx_descriptors).cast::<DmaDescriptor>();
        let rx_buf = addr_of_mut!((*dma).rx_buffers).cast::<FrameBuffer>();
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

        let tx_desc = addr_of_mut!((*dma).tx_descriptor);
        let tx_buf = addr_of_mut!((*dma).tx_buffer);
        // In the STM32F4 basic descriptor format, TCH is TDES0 bit 20.
        // TDES1 holds only the buffer byte counts.
        write_volatile(addr_of_mut!((*tx_desc).words[0]), TX_DESC_TCH);
        write_volatile(addr_of_mut!((*tx_desc).words[1]), 0);
        write_volatile(addr_of_mut!((*tx_desc).words[2]), (*tx_buf).0.as_mut_ptr() as u32);
        write_volatile(addr_of_mut!((*tx_desc).words[3]), tx_desc as u32);

        // DMA software reset, then conservative fixed-burst configuration.
        (*pac::ETHERNET_DMA::PTR).dmabmr().write(|w| w.bits(1));
        let mut timeout = MDIO_TIMEOUT;
        while (*pac::ETHERNET_DMA::PTR).dmabmr().read().sr().bit_is_set() {
            if timeout == 0 { return false; }
            timeout -= 1;
        }
        (*pac::ETHERNET_DMA::PTR).dmabmr().write(|w| {
            w.sr().clear_bit()
                .da().clear_bit()
                .dsl().bits(0)
                .edfe().clear_bit()
                .pbl().bits(32)
                .pm().bits(0)
                .fb().set_bit()
                .rdp().bits(32)
                .usp().set_bit()
                .fpm().clear_bit()
                .aab().set_bit()
                .mb().clear_bit()
        });
        (*pac::ETHERNET_DMA::PTR).dmaomr().write(|w| {
            w.sr().clear_bit()
                .osf().clear_bit()
                .rtc().bits(0)
                .fugf().clear_bit()
                .fef().clear_bit()
                .st().clear_bit()
                .ttc().bits(0)
                .ftf().clear_bit()
                .tsf().set_bit()
                .dfrf().clear_bit()
                .rsf().set_bit()
                .dtcefd().set_bit()
        });
        write_volatile((*pac::ETHERNET_DMA::PTR).dmardlar().as_ptr(), rx_desc as u32);
        write_volatile((*pac::ETHERNET_DMA::PTR).dmatdlar().as_ptr(), tx_desc as u32);
        (*pac::ETHERNET_DMA::PTR).dmaier().write(|w| {
            w.tie().clear_bit()
                .tpsie().clear_bit()
                .tbuie().clear_bit()
                .tjtie().clear_bit()
                .roie().clear_bit()
                .tuie().clear_bit()
                .rie().clear_bit()
                .rbuie().clear_bit()
                .rpsie().clear_bit()
                .rwtie().clear_bit()
                .etie().clear_bit()
                .erie().clear_bit()
                .fbeie().clear_bit()
                .aise().clear_bit()
                .nise().clear_bit()
        });
        (*pac::ETHERNET_DMA::PTR).dmasr().write(|w| {
            w.ts().set_bit()
                .tbus().set_bit()
                .tjts().set_bit()
                .ros().set_bit()
                .tus().set_bit()
                .rs().set_bit()
                .rbus().set_bit()
                .fbes().set_bit()
                .ers().set_bit()
                .ais().set_bit()
                .nis().set_bit()
        });

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
            w.re().clear_bit()
                .te().clear_bit()
                .dc().clear_bit()
                .apcs().set_bit()
                .rd().clear_bit()
                .ipco().clear_bit()
                .dm().bit(negotiated && phy.physts & DP83848_PHYSTS_FULL_DUPLEX != 0)
                .lm().clear_bit()
                .rod().clear_bit()
                .fes().bit(negotiated && phy.physts & DP83848_PHYSTS_SPEED_10_MBPS == 0)
                .csd().clear_bit()
                .jd().clear_bit()
                .wd().clear_bit()
                .cstf().set_bit();
            w.bl().bits(0);
            w.ifg().bits(0)
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
        (*pac::ETHERNET_DMA::PTR).dmasr().write(|w| {
            w.ts().set_bit()
                .tbus().set_bit()
                .tjts().set_bit()
                .ros().set_bit()
                .tus().set_bit()
                .rs().set_bit()
                .rbus().set_bit()
                .fbes().set_bit()
                .ers().set_bit()
                .ais().set_bit()
                .nis().set_bit()
        });
        (*pac::ETHERNET_DMA::PTR).dmaier().write(|w| {
            w.tie().set_bit()
                .tpsie().clear_bit()
                .tbuie().set_bit()
                .tjtie().clear_bit()
                .roie().clear_bit()
                .tuie().clear_bit()
                .rie().set_bit()
                .rbuie().clear_bit()
                .rpsie().clear_bit()
                .rwtie().clear_bit()
                .etie().clear_bit()
                .erie().clear_bit()
                .fbeie().set_bit()
                .aise().set_bit()
                .nise().set_bit()
        });
    }
}

/// Ethernet DMA IRQ top half: acknowledge pending events only.
/// Packet handling and all smoltcp/DHCP work remain in foreground context.
pub fn on_dma_interrupt() {
    unsafe {
        let status = (*pac::ETHERNET_DMA::PTR).dmasr().read();
        let ts = status.ts().bit_is_set();
        let tbus = status.tbus().bit_is_set();
        let tjts = status.tjts().bit_is_set();
        let ros = status.ros().bit_is_set();
        let tus = status.tus().bit_is_set();
        let rs = status.rs().bit_is_set();
        let rbus = status.rbus().bit_is_set();
        let fbes = status.fbes().bit_is_set();
        let ers = status.ers().bit_is_set();
        let ais = status.ais().bit_is_set();
        let nis = status.nis().bit_is_set();
        if ts || tbus || tjts || ros || tus || rs || rbus || fbes || ers || ais || nis {
            (*pac::ETHERNET_DMA::PTR).dmasr().write(|w| {
                w.ts().bit(ts)
                    .tbus().bit(tbus)
                    .tjts().bit(tjts)
                    .ros().bit(ros)
                    .tus().bit(tus)
                    .rs().bit(rs)
                    .rbus().bit(rbus)
                    .fbes().bit(fbes)
                    .ers().bit(ers)
                    .ais().bit(ais)
                    .nis().bit(nis)
            });
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
        let operation = ((phy_address as u32) << 11)
            | ((register as u32) << 6)
            | MII_CR_DIV_102
            | 1;
        (*pac::ETHERNET_MAC::PTR).macmiiar().write(|w| w.bits(operation));
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
            let desc = addr_of_mut!((*dma_storage_ptr()).rx_descriptors).cast::<DmaDescriptor>().add(self.rx_index);
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
        unsafe { read_volatile(addr_of_mut!((*dma_storage_ptr()).tx_descriptor.words[0])) & DESC_OWN == 0 }
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
            let buffer = addr_of_mut!((*dma_storage_ptr()).rx_buffers).cast::<FrameBuffer>().add(self.index);
            let bytes = core::slice::from_raw_parts((*buffer).0.as_ptr(), self.length);
            let result = f(bytes);
            let desc = addr_of_mut!((*dma_storage_ptr()).rx_descriptors).cast::<DmaDescriptor>().add(self.index);
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
            let buffer = addr_of_mut!((*dma_storage_ptr()).tx_buffer);
            let bytes = core::slice::from_raw_parts_mut((*buffer).0.as_mut_ptr(), length);
            let result = f(bytes);
            // TDES1 holds the buffer byte count; TCH is in TDES0 on STM32F4.
            write_volatile(addr_of_mut!((*dma_storage_ptr()).tx_descriptor.words[1]), length as u32);
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);
            cortex_m::asm::dsb();
            // Give the chained descriptor to DMA only after its buffer is ready.
            write_volatile(
                addr_of_mut!((*dma_storage_ptr()).tx_descriptor.words[0]),
                DESC_OWN | TX_DESC_TCH | TX_DESC_FS | TX_DESC_LS | TX_DESC_IC,
            );
            // The STM32F4 HAL writes the next free TX descriptor address
            // to DMATPDR when issuing transmit poll demand.
            write_volatile((*pac::ETHERNET_DMA::PTR).dmatpdr().as_ptr(), addr_of_mut!((*dma_storage_ptr()).tx_descriptor) as u32);
            result
        }
    }
}

