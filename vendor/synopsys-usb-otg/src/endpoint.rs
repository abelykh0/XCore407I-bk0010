use crate::endpoint_memory::{EndpointBuffer, EndpointBufferState};
use crate::ral::{
    endpoint0_out, endpoint_in, endpoint_out, modify_reg, otg_device, otg_global, read_reg,
    write_reg,
};
use crate::target::fifo_write;
use crate::target::UsbRegisters;
use crate::transition::EndpointDescriptor;
use crate::UsbPeripheral;
use core::cell::RefCell;
use core::ops::{Deref, DerefMut};
#[cfg(feature = "internal-dma")]
use core::sync::atomic::fence;
#[cfg(any(feature = "internal-dma", feature = "hs"))]
use core::sync::atomic::Ordering;
use critical_section::{CriticalSection, Mutex};
use usb_device::endpoint::{EndpointAddress, EndpointType};
use usb_device::{Result, UsbDirection, UsbError};

#[cfg(feature = "hs")]
static ISO_INCOMPLETE_DIEPCTL: [core::sync::atomic::AtomicU32; 9] =
    [const { core::sync::atomic::AtomicU32::new(0) }; 9];
#[cfg(feature = "hs")]
static ISO_INCOMPLETE_DIEPTSIZ: [core::sync::atomic::AtomicU32; 9] =
    [const { core::sync::atomic::AtomicU32::new(0) }; 9];
#[cfg(feature = "hs")]
static ISO_INCOMPLETE_DIEPINT: [core::sync::atomic::AtomicU32; 9] =
    [const { core::sync::atomic::AtomicU32::new(0) }; 9];
#[cfg(feature = "hs")]
static ISO_INCOMPLETE_FNSOF: [core::sync::atomic::AtomicU32; 9] =
    [const { core::sync::atomic::AtomicU32::new(0) }; 9];

/// Full 14-bit frame number targeted by each armed high-speed isochronous IN transfer.
/// Bit 31 marks the entry valid; bits 13:0 hold the target frame number.
#[cfg(feature = "hs")]
static ISO_IN_TARGET_FRAME: [core::sync::atomic::AtomicU32; 9] =
    [const { core::sync::atomic::AtomicU32::new(0) }; 9];

/// Return the register snapshot taken when an isochronous IN endpoint was
/// reported incomplete: (DIEPCTL, DIEPTSIZ, DIEPINT, DSTS.FNSOF).
#[cfg(feature = "internal-dma")]
pub fn iso_incomplete_snapshot(index: usize) -> (u32, u32, u32, u32) {
    if index >= 9 {
        return (0, 0, 0, 0);
    }
    use core::sync::atomic::Ordering::Relaxed;
    (
        ISO_INCOMPLETE_DIEPCTL[index].load(Relaxed),
        ISO_INCOMPLETE_DIEPTSIZ[index].load(Relaxed),
        ISO_INCOMPLETE_DIEPINT[index].load(Relaxed),
        ISO_INCOMPLETE_FNSOF[index].load(Relaxed),
    )
}

/// Non-DMA builds do not collect endpoint register snapshots.
#[cfg(not(feature = "internal-dma"))]
pub fn iso_incomplete_snapshot(_index: usize) -> (u32, u32, u32, u32) {
    (0, 0, 0, 0)
}

pub fn set_stalled(usb: UsbRegisters, address: EndpointAddress, stalled: bool) {
    critical_section::with(|_| match address.direction() {
        UsbDirection::Out => {
            let ep = usb.endpoint_out(address.index() as usize);
            modify_reg!(endpoint_out, ep, DOEPCTL, STALL: stalled as u32);
        }
        UsbDirection::In => {
            let ep = usb.endpoint_in(address.index() as usize);
            modify_reg!(endpoint_in, ep, DIEPCTL, STALL: stalled as u32);
        }
    })
}

pub fn is_stalled(usb: UsbRegisters, address: EndpointAddress) -> bool {
    let stall = match address.direction() {
        UsbDirection::Out => {
            let ep = usb.endpoint_out(address.index());
            read_reg!(endpoint_out, ep, DOEPCTL, STALL)
        }
        UsbDirection::In => {
            let ep = usb.endpoint_in(address.index());
            read_reg!(endpoint_in, ep, DIEPCTL, STALL)
        }
    };
    stall != 0
}

/// Arbitrates access to the endpoint-specific registers and packet buffer memory.
pub struct Endpoint {
    descriptor: EndpointDescriptor,
    usb: UsbRegisters,
}

impl Endpoint {
    pub fn new<USB: UsbPeripheral>(descriptor: EndpointDescriptor) -> Endpoint {
        Endpoint {
            descriptor,
            usb: UsbRegisters::new::<USB>(),
        }
    }

    pub fn address(&self) -> EndpointAddress {
        self.descriptor.address
    }

    #[inline(always)]
    fn index(&self) -> u8 {
        self.descriptor.address.index() as u8
    }
}

pub struct EndpointIn {
    common: Endpoint,
    #[cfg(feature = "internal-dma")]
    dma_buffer: Mutex<RefCell<Option<DmaTxBuffer>>>,
}

#[cfg(feature = "internal-dma")]
#[repr(align(4))]
struct DmaTxBuffer(&'static mut [u32]);

impl EndpointIn {
    pub fn new<USB: UsbPeripheral>(
        descriptor: EndpointDescriptor,
        #[cfg(feature = "internal-dma")] dma_buffer: Option<&'static mut [u32]>,
    ) -> EndpointIn {
        EndpointIn {
            common: Endpoint::new::<USB>(descriptor),
            #[cfg(feature = "internal-dma")]
            dma_buffer: Mutex::new(RefCell::new(dma_buffer.map(DmaTxBuffer))),
        }
    }

    pub fn configure(&self, _cs: CriticalSection<'_>) {
        if self.index() == 0 {
            let mpsiz = match self.descriptor.max_packet_size {
                8 => 0b11,
                16 => 0b10,
                32 => 0b01,
                64 => 0b00,
                other => panic!("Unsupported EP0 size: {}", other),
            };

            let regs = self.usb.endpoint_in(self.index() as usize);
            write_reg!(endpoint_in, regs, DIEPCTL, MPSIZ: mpsiz as u32, SNAK: 1);
            write_reg!(endpoint_in, regs, DIEPTSIZ, PKTCNT: 0, XFRSIZ: self.descriptor.max_packet_size as u32);
        } else {
            let regs = self.usb.endpoint_in(self.index() as usize);
            write_reg!(endpoint_in, regs, DIEPCTL,
                SNAK: 1,
                USBAEP: 1,
                EPTYP: self.descriptor.ep_type.to_bm_attributes() as u32,
                SD0PID_SEVNFRM: 1,
                TXFNUM: self.index() as u32,
                MPSIZ: self.descriptor.max_packet_size as u32
            );
        }
    }

    pub fn deconfigure(&self, _cs: CriticalSection<'_>) {
        let regs = self.usb.endpoint_in(self.index() as usize);

        // deactivating endpoint
        modify_reg!(endpoint_in, regs, DIEPCTL, USBAEP: 0);

        // TODO: flushing FIFO

        // disabling endpoint
        if read_reg!(endpoint_in, regs, DIEPCTL, EPENA) != 0 && self.index() != 0 {
            modify_reg!(endpoint_in, regs, DIEPCTL, EPDIS: 1)
        }

        // clean EP interrupts
        write_reg!(endpoint_in, regs, DIEPINT, 0xff);

        // TODO: deconfiguring TX FIFO
    }

    /// Abort a missed isochronous IN transaction and flush its TX FIFO.
    /// Returns true when an enabled isochronous endpoint was successfully
    /// stopped, matching the STM32 HAL's incomplete-ISO interrupt handling.
    #[cfg(feature = "hs")]
    pub fn abort_iso_incomplete(&self, current_frame: u32) -> bool {
        if !matches!(self.common.descriptor.ep_type, EndpointType::Isochronous { .. })
            || self.index() == 0
        {
            return false;
        }

        let ep = self.usb.endpoint_in(self.index() as usize);
        let index = self.index() as usize;

        #[cfg(feature = "hs")]
        {
            // Compare the complete 14-bit microframe target recorded when this
            // transfer was armed, using modulo arithmetic across counter wrap.
            let target_state = ISO_IN_TARGET_FRAME[index].load(Ordering::Relaxed);
            if target_state & 0x8000_0000 == 0 {
                return false;
            }
            let target_frame = target_state & 0x3fff;
            let elapsed = current_frame.wrapping_sub(target_frame) & 0x3fff;
            if elapsed >= 0x2000 {
                return false;
            }
        }
        #[cfg(not(feature = "hs"))]
        {
            let _ = current_frame;
            return false;
        }

        if read_reg!(endpoint_in, ep, DIEPCTL, EPENA) == 0 {
            return false;
        }

        // Capture only the endpoint whose armed isochronous transfer reached
        // its target frame and is still enabled.
        ISO_INCOMPLETE_DIEPCTL[index].store(
            read_reg!(endpoint_in, ep, DIEPCTL), Ordering::Relaxed
        );
        ISO_INCOMPLETE_DIEPTSIZ[index].store(
            read_reg!(endpoint_in, ep, DIEPTSIZ), Ordering::Relaxed
        );
        ISO_INCOMPLETE_DIEPINT[index].store(
            read_reg!(endpoint_in, ep, DIEPINT), Ordering::Relaxed
        );
        ISO_INCOMPLETE_FNSOF[index].store(current_frame, Ordering::Relaxed);

        modify_reg!(endpoint_in, ep, DIEPCTL, SNAK: 1);
        modify_reg!(endpoint_in, ep, DIEPCTL, EPDIS: 1);

        let mut timeout = 10_000;
        while read_reg!(endpoint_in, ep, DIEPCTL, EPENA) != 0 && timeout != 0 {
            timeout -= 1;
        }
        if read_reg!(endpoint_in, ep, DIEPCTL, EPENA) != 0 {
            return false;
        }

        write_reg!(endpoint_in, ep, DIEPINT, EPDISD: 1);
        let global = self.usb.global();
        timeout = 10_000;
        while read_reg!(otg_global, global, GRSTCTL, AHBIDL) == 0 && timeout != 0 {
            timeout -= 1;
        }
        if read_reg!(otg_global, global, GRSTCTL, AHBIDL) == 0 {
            return false;
        }

        write_reg!(
            otg_global,
            global,
            GRSTCTL,
            TXFNUM: self.index() as u32,
            TXFFLSH: 1
        );
        timeout = 10_000;
        while read_reg!(otg_global, global, GRSTCTL, TXFFLSH) != 0 && timeout != 0 {
            timeout -= 1;
        }
        read_reg!(otg_global, global, GRSTCTL, TXFFLSH) == 0
    }

    /// Reinitialize an isochronous endpoint after repeated incomplete frames.
    #[cfg(feature = "hs")]
    pub(crate) fn recover_iso_endpoint_hw(&self) {
        if matches!(self.common.descriptor.ep_type, EndpointType::Isochronous { .. }) {
            critical_section::with(|cs| {
                self.deconfigure(cs);
                self.configure(cs);
            });
        }
    }

    pub fn write(&self, buf: &[u8]) -> Result<()> {
        self.write_transfer(buf, 1, 1)
    }

    /// Queue up to two high-speed isochronous packets as one transfer.
    pub fn write_iso_batch(
        &self,
        buf: &[u8],
        packet_count: u8,
        transactions_per_microframe: u8,
    ) -> Result<()> {
        #[cfg(feature = "hs")]
        {
            if !matches!(self.common.descriptor.ep_type, EndpointType::Isochronous { .. })
                || !(1..=3).contains(&packet_count)
                || !(1..=3).contains(&transactions_per_microframe)
                || packet_count > transactions_per_microframe
            {
                return Err(UsbError::Unsupported);
            }
            self.write_transfer(buf, packet_count, transactions_per_microframe)
        }
        #[cfg(not(feature = "hs"))]
        {
            let _ = (buf, packet_count, transactions_per_microframe);
            Err(UsbError::Unsupported)
        }
    }

    /// Queue isochronous IN data directly from the caller's DMA-accessible
    /// buffer instead of copying it into the endpoint's staging buffer.
    ///
    /// # Safety
    /// The buffer address must be 4-byte aligned and point to SRAM accessible
    /// by the USB OTG DMA. The caller must keep the buffer alive and must not
    /// modify it until the endpoint transfer-complete interrupt has fired.
    #[cfg(feature = "internal-dma")]
    pub unsafe fn write_iso_batch_dma(
        &self,
        buf: &[u8],
        packet_count: u8,
        transactions_per_microframe: u8,
    ) -> Result<()> {
        if !matches!(self.common.descriptor.ep_type, EndpointType::Isochronous { .. })
            || !(1..=3).contains(&packet_count)
            || !(1..=3).contains(&transactions_per_microframe)
            || packet_count > transactions_per_microframe
            || (buf.as_ptr() as usize) & 3 != 0
            || buf.len() & 3 != 0
        {
            return Err(UsbError::Unsupported);
        }
        self.write_transfer_inner(
            buf,
            packet_count,
            transactions_per_microframe,
            Some(buf.as_ptr().cast::<u32>()),
            false,
        )
    }

    fn write_transfer(&self, buf: &[u8], packet_count: u8, multi_count: u8) -> Result<()> {
        self.write_transfer_inner(buf, packet_count, multi_count, None, false)
    }

    /// Queue an isochronous IN transfer using the endpoint TX FIFO, even when
    /// the USB peripheral is otherwise configured for internal DMA.
    pub fn write_iso_batch_fifo(
        &self,
        buf: &[u8],
        packet_count: u8,
        transactions_per_microframe: u8,
    ) -> Result<()> {
        #[cfg(feature = "hs")]
        {
            if !matches!(self.common.descriptor.ep_type, EndpointType::Isochronous { .. })
                || !(1..=3).contains(&packet_count)
                || !(1..=3).contains(&transactions_per_microframe)
                || packet_count > transactions_per_microframe
            {
                return Err(UsbError::Unsupported);
            }
            self.write_transfer_inner(buf, packet_count, transactions_per_microframe, None, true)
        }
        #[cfg(not(feature = "hs"))]
        {
            let _ = (buf, packet_count, transactions_per_microframe);
            Err(UsbError::Unsupported)
        }
    }

    fn write_transfer_inner(
        &self,
        buf: &[u8],
        packet_count: u8,
        multi_count: u8,
        direct_dma_address: Option<*const u32>,
        fifo_only: bool,
    ) -> Result<()> {
        let ep = self.usb.endpoint_in(self.index() as usize);
        if self.index() != 0 && read_reg!(endpoint_in, ep, DIEPCTL, EPENA) != 0 {
            return Err(UsbError::WouldBlock);
        }

        let max_packet_size = self.common.descriptor.max_packet_size as usize;
        // USB control transfers use a zero-length IN packet for the status
        // stage (including SET_ADDRESS). Isochronous data packets must remain
        // non-empty, and an empty transfer is only valid as EP0's single ZLP.
        let ep0_status_zlp = self.index() == 0 && buf.is_empty() && packet_count == 1;
        if (!ep0_status_zlp && buf.is_empty())
            || buf.len() > max_packet_size * packet_count as usize
            || (!ep0_status_zlp
                && (buf.len() + max_packet_size - 1) / max_packet_size
                    != packet_count as usize)
        {
            return Err(UsbError::BufferOverflow);
        }

        #[cfg(feature = "internal-dma")]
        if !fifo_only {
            critical_section::with(|cs| -> Result<()> {
                let mut dma = self.dma_buffer.borrow_ref_mut(cs);
                let dma_address = if let Some(address) = direct_dma_address {
                    address
                } else {
                    let Some(buffer) = dma.as_mut() else {
                        return Err(UsbError::BufferOverflow);
                    };
                    if buf.len() > buffer.0.len() * core::mem::size_of::<u32>() {
                        return Err(UsbError::BufferOverflow);
                    }
                    let bytes = unsafe {
                        core::slice::from_raw_parts_mut(
                            buffer.0.as_mut_ptr().cast::<u8>(),
                            buffer.0.len() * core::mem::size_of::<u32>(),
                        )
                    };
                    bytes[..buf.len()].copy_from_slice(buf);
                    buffer.0.as_ptr()
                };
                // Ensure source data reaches SRAM before the OTG core is armed.
                fence(Ordering::Release);
                self.usb
                    .set_in_dma_address(self.index() as usize, dma_address);

                #[cfg(feature = "fs")]
                write_reg!(endpoint_in, ep, DIEPTSIZ, PKTCNT: 1, XFRSIZ: buf.len() as u32);
                #[cfg(feature = "hs")]
                write_reg!(endpoint_in, ep, DIEPTSIZ, MCNT: multi_count as u32, PKTCNT: packet_count as u32, XFRSIZ: buf.len() as u32);

                self.set_iso_frame(ep);
                modify_reg!(endpoint_in, ep, DIEPCTL, CNAK: 1, EPENA: 1);
                Ok(())
            })?;
            return Ok(());
        }

        if !buf.is_empty() {
            // Check for FIFO free space
            let size_words = (buf.len() + 3) / 4;
            if size_words > read_reg!(endpoint_in, ep, DTXFSTS, INEPTFSAV) as usize {
                return Err(UsbError::WouldBlock);
            }
        }

        #[cfg(feature = "fs")]
        write_reg!(endpoint_in, ep, DIEPTSIZ, PKTCNT: 1, XFRSIZ: buf.len() as u32);
        #[cfg(feature = "hs")]
        write_reg!(endpoint_in, ep, DIEPTSIZ, MCNT: multi_count as u32, PKTCNT: packet_count as u32, XFRSIZ: buf.len() as u32);

        self.set_iso_frame(ep);
        modify_reg!(endpoint_in, ep, DIEPCTL, CNAK: 1, EPENA: 1);

        fifo_write(self.usb, self.index(), buf);

        Ok(())
    }

    #[cfg(feature = "hs")]
    fn set_iso_frame(&self, ep: &endpoint_in::RegisterBlock) {
        if matches!(self.common.descriptor.ep_type, EndpointType::Isochronous { .. }) {
            // DWC2 requires every isochronous transfer to select the target
            // frame. Match STM32's LL driver: queue odd when the received SOF
            // frame number is even, and vice versa.
            let frame = read_reg!(otg_device, self.usb.device(), DSTS, FNSOF);
            ISO_IN_TARGET_FRAME[self.index() as usize].store(
                0x8000_0000 | (frame.wrapping_add(1) & 0x3fff),
                Ordering::Relaxed,
            );
            if frame & 1 == 0 {
                modify_reg!(endpoint_in, ep, DIEPCTL, SODDFRM: 1, SD0PID_SEVNFRM: 0);
            } else {
                modify_reg!(endpoint_in, ep, DIEPCTL, SODDFRM: 0, SD0PID_SEVNFRM: 1);
            }
        }
    }
}

pub struct EndpointOut {
    common: Endpoint,
    pub(crate) buffer: Mutex<RefCell<EndpointBuffer>>,
    #[cfg(feature = "internal-dma")]
    dma_transfer_len: Mutex<RefCell<u16>>,
    #[cfg(feature = "internal-dma")]
    ep0_dma_state: Mutex<RefCell<Ep0DmaState>>,
}

#[cfg(feature = "internal-dma")]
#[derive(Default)]
struct Ep0DmaState {
    expected_out: u16,
    received_out: u16,
    status_out: bool,
}

impl EndpointOut {
    pub fn new<USB: UsbPeripheral>(
        descriptor: EndpointDescriptor,
        buffer: EndpointBuffer,
    ) -> EndpointOut {
        EndpointOut {
            common: Endpoint::new::<USB>(descriptor),
            buffer: Mutex::new(RefCell::new(buffer)),
            #[cfg(feature = "internal-dma")]
            dma_transfer_len: Mutex::new(RefCell::new(0)),
            #[cfg(feature = "internal-dma")]
            ep0_dma_state: Mutex::new(RefCell::new(Ep0DmaState::default())),
        }
    }

    pub fn configure(&self, _cs: CriticalSection<'_>) {
        #[cfg(feature = "internal-dma")]
        {
            if self.index() == 0 {
                let regs = self.usb.endpoint0_out();
                write_reg!(endpoint0_out, regs, DOEPCTL0,
                    MPSIZ: 0,
                    USBAEP: 1
                );
                self.arm_setup_dma();
            } else {
                let regs = self.usb.endpoint_out(self.index() as usize);
                write_reg!(endpoint_out, regs, DOEPCTL,
                    SNAK: 1,
                    USBAEP: 1,
                    EPTYP: self.descriptor.ep_type.to_bm_attributes() as u32,
                    MPSIZ: self.descriptor.max_packet_size as u32
                );
                let address = critical_section::with(|cs| {
                    self.buffer.borrow_ref(cs).dma_address()
                });
                self.arm_dma_transfer(address, self.descriptor.max_packet_size, 1);
            }
        }

        #[cfg(not(feature = "internal-dma"))]
        if self.index() == 0 {
            let mpsiz = match self.descriptor.max_packet_size {
                8 => 0b11,
                16 => 0b10,
                32 => 0b01,
                64 => 0b00,
                other => panic!("Unsupported EP0 size: {}", other),
            };

            let regs = self.usb.endpoint0_out();
            write_reg!(endpoint0_out, regs, DOEPTSIZ0, STUPCNT: 1, PKTCNT: 1, XFRSIZ: self.descriptor.max_packet_size as u32);
            modify_reg!(endpoint0_out, regs, DOEPCTL0, MPSIZ: mpsiz as u32, EPENA: 1, CNAK: 1);
        } else {
            let regs = self.usb.endpoint_out(self.index() as usize);
            write_reg!(endpoint_out, regs, DOEPCTL,
                SD0PID_SEVNFRM: 1,
                CNAK: 1,
                EPENA: 1,
                USBAEP: 1,
                EPTYP: self.descriptor.ep_type.to_bm_attributes() as u32,
                MPSIZ: self.descriptor.max_packet_size as u32
            );
        }
    }

    pub fn deconfigure(&self, _cs: CriticalSection<'_>, core_id: u32) {
        let regs = self.usb.endpoint_out(self.index() as usize);

        // GD32VF103 doesn't support endpoint deactivation after reset, so skip this.
        if core_id > 0x0000_1000 {
            // deactivating endpoint
            modify_reg!(endpoint_out, regs, DOEPCTL, USBAEP: 0);

            // disabling endpoint
            if read_reg!(endpoint_out, regs, DOEPCTL, EPENA) != 0 && self.index() != 0 {
                modify_reg!(endpoint_out, regs, DOEPCTL, EPDIS: 1)
            }
        }

        // clean EP interrupts
        write_reg!(endpoint_out, regs, DOEPINT, 0xff);
    }

    pub fn read(&self, buf: &mut [u8]) -> Result<usize> {
        #[cfg(not(feature = "internal-dma"))]
        {
            critical_section::with(|cs| self.buffer.borrow_ref_mut(cs).read_packet(buf))
        }
        #[cfg(feature = "internal-dma")]
        {
            let (read, was_setup) = critical_section::with(|cs| {
                let mut buffer = self.buffer.borrow_ref_mut(cs);
                let was_setup = buffer.is_setup();
                buffer.read_packet(buf).map(|read| (read, was_setup))
            })?;

            if self.index() == 0 {
                if was_setup && read >= 8 {
                    let request_direction_in = buf[0] & 0x80 != 0;
                    let requested = u16::from_le_bytes([buf[6], buf[7]]);
                    critical_section::with(|cs| {
                        let mut state = self.ep0_dma_state.borrow_ref_mut(cs);
                        state.expected_out = if request_direction_in { 0 } else { requested };
                        state.received_out = 0;
                        state.status_out = request_direction_in && requested != 0;
                    });
                    let address = critical_section::with(|cs| {
                        self.buffer.borrow_ref(cs).dma_address()
                    });

                    if !request_direction_in && requested != 0 {
                        let size = requested.min(self.descriptor.max_packet_size);
                        self.arm_dma_transfer(address, size, 1);
                    } else if request_direction_in && requested != 0 {
                        // Device-to-host requests finish with a zero-length OUT status.
                        self.arm_dma_transfer(address, 0, 1);
                    } else {
                        self.arm_setup_dma();
                    }
                } else {
                    let (next_out, continue_data) = critical_section::with(|cs| {
                        let mut state = self.ep0_dma_state.borrow_ref_mut(cs);
                        if state.expected_out != 0 {
                            state.received_out = state.received_out.saturating_add(read as u16);
                            let finished = read < self.descriptor.max_packet_size as usize
                                || state.received_out >= state.expected_out;
                            if finished {
                                state.expected_out = 0;
                                state.received_out = 0;
                                (0, false)
                            } else {
                                (
                                    state.expected_out.saturating_sub(state.received_out)
                                        .min(self.descriptor.max_packet_size),
                                    true,
                                )
                            }
                        } else {
                            state.status_out = false;
                            (0, false)
                        }
                    });
                    if continue_data {
                        let address = critical_section::with(|cs| {
                            self.buffer.borrow_ref(cs).dma_address()
                        });
                        self.arm_dma_transfer(address, next_out, 1);
                    } else {
                        self.arm_setup_dma();
                    }
                }
            } else {
                let address = critical_section::with(|cs| {
                    self.buffer.borrow_ref(cs).dma_address()
                });
                self.arm_dma_transfer(address, self.descriptor.max_packet_size, 1);
            }

            Ok(read)
        }
    }

    #[cfg(feature = "internal-dma")]
    fn arm_setup_dma(&self) {
        let address = critical_section::with(|cs| self.buffer.borrow_ref(cs).dma_address());
        self.usb.set_out_dma_address(self.index() as usize, address);
        let regs = self.usb.endpoint0_out();
        write_reg!(endpoint0_out, regs, DOEPTSIZ0, STUPCNT: 3, PKTCNT: 1, XFRSIZ: 24);
        modify_reg!(endpoint0_out, regs, DOEPCTL0, CNAK: 1, EPENA: 1);
        critical_section::with(|cs| {
            *self.dma_transfer_len.borrow_ref_mut(cs) = 24;
        });
    }

    #[cfg(feature = "internal-dma")]
    fn arm_dma_transfer(&self, address: *mut u32, size: u16, packets: u16) {
        self.usb.set_out_dma_address(self.index() as usize, address);
        if self.index() == 0 {
            let regs = self.usb.endpoint0_out();
            write_reg!(endpoint0_out, regs, DOEPTSIZ0,
                STUPCNT: 0,
                PKTCNT: packets as u32,
                XFRSIZ: size as u32
            );
            modify_reg!(endpoint0_out, regs, DOEPCTL0, CNAK: 1, EPENA: 1);
        } else {
            let regs = self.usb.endpoint_out(self.index() as usize);
            write_reg!(endpoint_out, regs, DOEPTSIZ,
                PKTCNT: packets as u32,
                XFRSIZ: size as u32
            );
            modify_reg!(endpoint_out, regs, DOEPCTL, CNAK: 1, EPENA: 1);
        }
        critical_section::with(|cs| {
            *self.dma_transfer_len.borrow_ref_mut(cs) = size;
        });
    }

    #[cfg(feature = "internal-dma")]
    pub(crate) fn dma_transfer_len(&self) -> u16 {
        critical_section::with(|cs| *self.dma_transfer_len.borrow_ref(cs))
    }

    pub fn buffer_state(&self) -> EndpointBufferState {
        critical_section::with(|cs| self.buffer.borrow_ref(cs).state())
    }
}

impl Deref for EndpointIn {
    type Target = Endpoint;

    fn deref(&self) -> &Self::Target {
        &self.common
    }
}

impl DerefMut for EndpointIn {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.common
    }
}

impl Deref for EndpointOut {
    type Target = Endpoint;

    fn deref(&self) -> &Self::Target {
        &self.common
    }
}

impl DerefMut for EndpointOut {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.common
    }
}
