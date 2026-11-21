//! Minimal STM32F407 Ethernet-PHY management access for CDC diagnostics.
//!
//! This deliberately stops at MDIO/PHY identification. It does not start the
//! Ethernet MAC data path, allocate DMA descriptors, or add network buffers.

use core::ptr::{read_volatile, write_volatile};

const RCC_BASE: usize = 0x4002_3800;
const RCC_AHB1ENR: *mut u32 = (RCC_BASE + 0x30) as *mut u32;
const RCC_APB2ENR: *mut u32 = (RCC_BASE + 0x44) as *mut u32;
const SYSCFG_PMC: *mut u32 = 0x4001_3804 as *mut u32;

const ETH_MAC_BASE: usize = 0x4002_8000;
const ETH_MACMIIAR: *mut u32 = (ETH_MAC_BASE + 0x10) as *mut u32;
const ETH_MACMIIDR: *mut u32 = (ETH_MAC_BASE + 0x14) as *mut u32;

const ETHMACEN: u32 = 1 << 25;
const SYSCFGEN: u32 = 1 << 14;
const MII_RMII_SEL: u32 = 1 << 23;
const MII_BUSY: u32 = 1;
const MII_CR_DIV_102: u32 = 0b100 << 2;
const MDIO_TIMEOUT: u32 = 1_000_000;

const DP83848_ID1: u16 = 0x2000;
const DP83848_ID2_MASK: u16 = 0xfff0;
const DP83848_ID2: u16 = 0x5c90;

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

/// Result of a bounded MDIO scan for the board's DP83848 PHY.
pub fn probe_dp83848() -> PhyStatus {
    unsafe {
        // Enable SYSCFG and the Ethernet MAC clock. Only the MAC management
        // interface is used here; TX, RX, and DMA clocks remain disabled.
        write_volatile(RCC_APB2ENR, read_volatile(RCC_APB2ENR) | SYSCFGEN);
        write_volatile(RCC_AHB1ENR, read_volatile(RCC_AHB1ENR) | ETHMACEN);
        let _ = read_volatile(RCC_AHB1ENR);

        // Select RMII rather than MII. HCLK is 168 MHz in this firmware;
        // CR=100 selects MDC = HCLK/102, below the 2.5 MHz MDIO limit.
        write_volatile(SYSCFG_PMC, read_volatile(SYSCFG_PMC) | MII_RMII_SEL);

        // Clear stale management-operation state and select the /102 divider.
        let initial = read_volatile(ETH_MACMIIAR);
        write_volatile(ETH_MACMIIAR, (initial & !((0b111 << 2) | MII_BUSY)) | MII_CR_DIV_102);

        for address in 0u8..32 {
            let id1 = match mdio_read(address, 2) {
                Some(value) => value,
                None => return empty_status(true),
            };
            if id1 != DP83848_ID1 {
                continue;
            }

            let id2 = match mdio_read(address, 3) {
                Some(value) => value,
                None => return empty_status(true),
            };
            if id2 & DP83848_ID2_MASK != DP83848_ID2 {
                continue;
            }

            // BMSR link status is latch-low; read twice and use the second
            // sample to obtain the current link state.
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

            return PhyStatus {
                found: true,
                address,
                id1,
                id2,
                bmsr,
                physts,
                mdio_timeout: false,
            };
        }
    }

    empty_status(false)
}

fn empty_status(mdio_timeout: bool) -> PhyStatus {
    PhyStatus {
        found: false,
        address: 0xff,
        id1: 0,
        id2: 0,
        bmsr: 0,
        physts: 0,
        mdio_timeout,
    }
}

/// Read one PHY register. Returns None if the MAC management-busy bit does
/// not clear within the bounded wait, so a missing clock cannot hang CDC.
fn mdio_read(phy_address: u8, register: u8) -> Option<u16> {
    unsafe {
        let mut remaining = MDIO_TIMEOUT;
        while read_volatile(ETH_MACMIIAR) & MII_BUSY != 0 {
            if remaining == 0 {
                return None;
            }
            remaining -= 1;
        }

        let operation = ((phy_address as u32) << 11)
            | ((register as u32) << 6)
            | MII_CR_DIV_102
            | MII_BUSY;
        write_volatile(ETH_MACMIIAR, operation);

        remaining = MDIO_TIMEOUT;
        while read_volatile(ETH_MACMIIAR) & MII_BUSY != 0 {
            if remaining == 0 {
                return None;
            }
            remaining -= 1;
        }

        Some((read_volatile(ETH_MACMIIDR) & 0xffff) as u16)
    }
}
