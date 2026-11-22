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

const MII_CR_DIV_102: u32 = 0b100 << 2;

const DP83848_PHYSTS_LINK: u16 = 1 << 0;
const DP83848_PHYSTS_SPEED_10_MBPS: u16 = 1 << 1;
const DP83848_PHYSTS_FULL_DUPLEX: u16 = 1 << 2;
const DP83848_PHYSTS_AUTONEG_COMPLETE: u16 = 1 << 4;
const PHY_BMSR_AUTONEG_COMPLETE: u16 = 1 << 5;
const MDIO_TIMEOUT: u32 = 1_000_000;
const PHY_NEGOTIATION_POLLS: u32 = 5_000;
const PHY_NEGOTIATION_DELAY: u32 = 50_000;

// HTTP uses bounded RX/TX buffers; uploads are sent in small sequential chunks.
// Directory output is paginated instead of buffering the whole listing.
const HTTP_RX_BUFFER_SIZE: usize = 768;
const HTTP_TX_BUFFER_SIZE: usize = 2304;
const HTTP_BODY_BUFFER_SIZE: usize = 768;
const HTTP_DIRECTORY_PAGE_SIZE: usize = 3;
static mut HTTP_RX_BUFFER: [u8; HTTP_RX_BUFFER_SIZE] = [0; HTTP_RX_BUFFER_SIZE];
static mut HTTP_TX_BUFFER: [u8; HTTP_TX_BUFFER_SIZE] = [0; HTTP_TX_BUFFER_SIZE];
static mut HTTP_BODY_BUFFER: [u8; HTTP_BODY_BUFFER_SIZE] = [0; HTTP_BODY_BUFFER_SIZE];
const HTTP_PAGE: &[u8] = br#"<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1"><title>LittleFS</title><h1>LittleFS files</h1><p id=s role=status>Loading files...</p><input id=u type=file><button id=b type=button>Upload</button><p id=p role=status></p><ul id=f></ul><script>
let o=0,count=0,f=document.getElementById('f'),s=document.getElementById('s'),u=document.getElementById('u'),p=document.getElementById('p');async function load(){o=0;count=0;f.textContent='';try{let more=true;while(more){s.textContent='Loading files... ('+count+' loaded)';let r=await fetch('/api/files?offset='+o);if(!r.ok)throw Error('HTTP '+r.status);let d=new DOMParser().parseFromString(await r.text(),'text/html'),ul=d.querySelector('ul');if(!ul)throw Error('Invalid response');f.insertAdjacentHTML('beforeend',ul.innerHTML);count+=ul.children.length;let n=d.querySelector('[data-next]');more=!!n;if(n)o=Number(n.dataset.next);if(more)await new Promise(resolve=>setTimeout(resolve,0));}s.textContent=count+' files loaded.';}catch(e){s.textContent='Could not finish loading files ('+count+' loaded).';}}document.getElementById('b').onclick=async()=>{let file=u.files[0];if(!file){p.textContent='Choose a file first.';return;}if(!/^[A-Za-z0-9._-]{1,64}$/.test(file.name)){p.textContent='Use a filename with 1-64 letters, digits, dot, underscore or hyphen.';return;}try{let chunk=256;for(let offset=0;offset<file.size||offset===0;){let part=file.slice(offset,Math.min(offset+chunk,file.size));p.textContent='Uploading '+file.name+': '+Math.min(offset+part.size,file.size)+' / '+file.size+' bytes';let r=await fetch('/api/upload?name='+encodeURIComponent(file.name)+'&offset='+offset+'&total='+file.size,{method:'POST',body:part});if(!r.ok)throw Error(await r.text());offset+=part.size;if(file.size===0)break;}p.textContent='Upload complete: '+file.name;await load();}catch(e){p.textContent='Upload failed: '+e.message;}};load()</script>"#;

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

        (*pac::ETHERNET_MAC::PTR).macmiiar().write(|w| unsafe { w.bits(MII_CR_DIV_102) });
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
        (*pac::ETHERNET_DMA::PTR).dmabmr().write(|w| unsafe { w.bits(1) });
        let mut timeout = MDIO_TIMEOUT;
        while (*pac::ETHERNET_DMA::PTR).dmabmr().read().sr().bit_is_set() {
            if timeout == 0 { return false; }
            timeout -= 1;
        }
        (*pac::ETHERNET_DMA::PTR).dmabmr().write(|w| unsafe {
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
        (*pac::ETHERNET_DMA::PTR).dmaomr().write(|w| unsafe {
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
            unsafe {
                w.bl().bits(0);
                w.ifg().bits(0)
            }
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


const HTTP_UPLOAD_CHUNK_SIZE: usize = 256;

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

fn write_http_upload(
    filesystem: &littlefs2::fs::Filesystem<'_, crate::nand_fs::ActiveStorage>,
    name: &[u8],
    offset: usize,
    total: usize,
    data: &[u8],
) -> bool {
    use littlefs2::io::prelude::{Seek, Write};
    use littlefs2::io::SeekFrom;
    use littlefs2::path::Path;

    if name.is_empty() || name.len() > 64
        || offset > total || data.len() > total.saturating_sub(offset)
    {
        return false;
    }

    let mut final_path_bytes = [0u8; 66];
    final_path_bytes[0] = b'/';
    final_path_bytes[1..1 + name.len()].copy_from_slice(name);
    final_path_bytes[1 + name.len()] = 0;
    let final_path = match Path::from_bytes_with_nul(&final_path_bytes[..name.len() + 2]) {
        Ok(path) => path,
        Err(_) => return false,
    };

    let mut temp_path_bytes = [0u8; 80];
    let mut temp_len = 0usize;
    if !append_http_bytes(&mut temp_path_bytes, &mut temp_len, b"/.upload-")
        || !append_http_bytes(&mut temp_path_bytes, &mut temp_len, name)
        || !append_http_bytes(&mut temp_path_bytes, &mut temp_len, b"\0")
    {
        return false;
    }
    let temp_path = match Path::from_bytes_with_nul(&temp_path_bytes[..temp_len]) {
        Ok(path) => path,
        Err(_) => return false,
    };

    let written = if offset == 0 {
        if filesystem.exists(temp_path) && filesystem.remove(temp_path).is_err() {
            return false;
        }
        filesystem.create_file_and_then(temp_path, |file| file.write_all(data)).is_ok()
    } else {
        filesystem.open_file_and_then(temp_path, |file| {
            if file.len()? != offset {
                return Ok(false);
            }
            file.seek(SeekFrom::End(0))?;
            file.write_all(data)?;
            Ok(true)
        }).unwrap_or(false)
    };
    if !written {
        return false;
    }

    if offset + data.len() == total {
        if filesystem.exists(final_path) && filesystem.remove(final_path).is_err() {
            return false;
        }
        return filesystem.rename(temp_path, final_path).is_ok();
    }
    true
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

        let mut request_kind = 0u8; // 0=page, 1=directory, 2=upload, 3=bad request
        let mut request_ready = false;
        let mut listing_offset = 0usize;
        let mut upload_name = [0u8; 64];
        let mut upload_name_len = 0usize;
        let mut upload_offset = 0usize;
        let mut upload_total = 0usize;
        let mut upload_len = 0usize;
        let upload_body = unsafe {
            core::slice::from_raw_parts_mut(
                addr_of_mut!(HTTP_BODY_BUFFER).cast::<u8>(),
                HTTP_BODY_BUFFER_SIZE,
            )
        };

        let _ = socket.recv(|data| {
            let Some(separator) = data.windows(4).position(|window| window == b"\r\n\r\n") else {
                return (0, false);
            };
            let header_len = separator + 4;
            let Some(line_end) = data.windows(2).position(|window| window == b"\r\n") else {
                return (0, false);
            };
            let line = &data[..line_end];

            if let Some(rest) = line.strip_prefix(b"GET /api/files?offset=") {
                request_kind = 1;
                for &byte in rest {
                    if !byte.is_ascii_digit() { break; }
                    listing_offset = listing_offset.saturating_mul(10)
                        .saturating_add((byte - b'0') as usize)
                        .min(1_000_000);
                }
                request_ready = true;
                return (header_len, true);
            }

            if let Some(rest) = line.strip_prefix(b"POST /api/upload?name=") {
                let Some(http_pos) = rest.windows(9).position(|window| window == b" HTTP/1.1") else {
                    request_kind = 3;
                    request_ready = true;
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
                    || content_length > upload_body.len()
                {
                    request_kind = 3;
                    request_ready = true;
                    return (header_len, true);
                }
                if data.len() < header_len + content_length {
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
                    upload_body[..upload_len].copy_from_slice(&data[header_len..header_len + upload_len]);
                    request_kind = 2;
                }
                request_ready = true;
                return (header_len + content_length, true);
            }

            request_kind = if line.starts_with(b"GET / ") { 0 } else { 3 };
            request_ready = true;
            (header_len, true)
        });

        if !request_ready || !socket.can_send() {
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
                        if emitted >= HTTP_DIRECTORY_PAGE_SIZE {
                            has_more = true;
                            return Ok(());
                        }
                        seen += 1;
                        emitted += 1;
                        let name = path.strip_prefix(b"/").unwrap_or(path);
                        let name = name.strip_suffix(b"\0").unwrap_or(name);
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
                        if name_len < name.len() { append_http_bytes(body, &mut length, b"... "); }
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
                    append_http_decimal(body, &mut length, listing_offset.saturating_add(emitted));
                    append_http_bytes(body, &mut length, b"\"></span>");
                }
                &body[..length]
            }
            2 => {
                let name = &upload_name[..upload_name_len];
                if write_http_upload(filesystem, name, upload_offset, upload_total, &upload_body[..upload_len]) {
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
        let _ = socket.send_slice(&header[..header_len]);
        let _ = socket.send_slice(body);
    }
}
