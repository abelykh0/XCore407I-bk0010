//! `usb-device` class adapter for the board's external USB3300 ULPI PHY.

use crate::camera::{HEIGHT, WIDTH};
use crate::uvc::{
    ControlResponse, UvcClass, ENDPOINT_IN, FRAME_BIT_RATE, FRAME_BYTES, FRAME_INTERVAL_100NS,
    ISO_PACKETS_PER_MICROFRAME, MAX_PACKET_SIZE,
};
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};
use stm32f4::stm32f407::RCC;
use synopsys_usb_otg::{PhyType, UsbPeripheral};
use usb_device::bus::{InterfaceNumber, StringIndex, UsbBus, UsbBusAllocator};
use usb_device::class::{ControlIn, ControlOut, UsbClass};
use usb_device::control::{Recipient, Request, RequestType};
use usb_device::descriptor::DescriptorWriter;
use usb_device::endpoint::{
    EndpointAddress, EndpointIn, EndpointType, IsochronousSynchronizationType, IsochronousUsageType,
};
use usb_device::Result;

const VIDEO_CLASS: u8 = 0x0e;
const VIDEO_CONTROL_SUBCLASS: u8 = 0x01;
const VIDEO_STREAMING_SUBCLASS: u8 = 0x02;

/// F407's OTG_HS controller and external USB3300 PHY.
pub struct F407UsbHs;

// This is a register-compatible STM32 OTG_HS controller. The caller configures
// the board's ULPI GPIO mux and system clock before constructing UsbDevice.
unsafe impl UsbPeripheral for F407UsbHs {
    const REGISTERS: *const () = 0x4004_0000 as *const ();
    const HIGH_SPEED: bool = true;
    const FIFO_DEPTH_WORDS: usize = 1024;
    const ENDPOINT_COUNT: usize = 6;

    fn enable() {
        let rcc = unsafe { &*RCC::ptr() };
        rcc.ahb1enr().modify(|_, w| {
            w.otghsen().set_bit().otghsulpien().set_bit()
        });
        let _ = rcc.ahb1enr().read().bits();
    }

    fn ahb_frequency_hz(&self) -> u32 {
        168_000_000
    }

    fn phy_type(&self) -> PhyType {
        PhyType::ExternalHighSpeed
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum UvcUsbClassInitError {
    CameraAlreadyInUse,
}

pub struct UvcUsbClass<'a, B: UsbBus> {
    function: UvcClass,
    control_interface: InterfaceNumber,
    streaming_interface: InterfaceNumber,
    interface_string: StringIndex,
    endpoint: EndpointIn<'a, B>,
    incomplete_count: u8,
    video_payloads_enabled: bool,
    repeat_video_batch: bool,
}

impl<'a, B: UsbBus> UvcUsbClass<'a, B> {
    pub fn new(
        allocator: &'a UsbBusAllocator<B>,
    ) -> core::result::Result<Self, UvcUsbClassInitError> {
        // The main-loop producer owns the singleton NV12 renderer. This
        // class owns only USB control state and the DMA batch handoff.
        let function = UvcClass::try_new().ok_or(UvcUsbClassInitError::CameraAlreadyInUse)?;
        let control_interface = allocator.interface();
        let streaming_interface = allocator.interface();
        let interface_string = allocator.string();
        let endpoint = allocator
            .alloc(
                Some(EndpointAddress::from(ENDPOINT_IN)),
                EndpointType::Isochronous {
                    synchronization: IsochronousSynchronizationType::Asynchronous,
                    usage: IsochronousUsageType::Data,
                },
                MAX_PACKET_SIZE,
                1,
            )
            .expect("failed to allocate UVC high-speed isochronous IN endpoint");
        Ok(Self {
            function,
            control_interface,
            streaming_interface,
            interface_string,
            endpoint,
            incomplete_count: 0,
            video_payloads_enabled: true,
            repeat_video_batch: false,
        })
    }

    /// Enable or suppress video payload generation/submission while keeping
    /// the UVC interface registered and its control requests serviced.
    pub fn set_video_payloads_enabled(&mut self, enabled: bool) {
        self.video_payloads_enabled = enabled;
    }

    /// Re-submit the first generated batch instead of generating new video.
    pub fn set_repeat_video_batch(&mut self, enabled: bool) {
        self.repeat_video_batch = enabled;
    }

    pub fn set_cpu_metrics(&mut self, available: u8, generate: u8, submit: u8) {
        self.function.set_cpu_metrics(available, generate, submit);
    }

    fn write_video_control_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        writer.interface(
            self.control_interface,
            VIDEO_CLASS,
            VIDEO_CONTROL_SUBCLASS,
            0,
        )?;
        // UVC 1.1 VideoControl header (two-byte descriptor header is added by writer).
        writer.write(
            0x24,
            &[
                1,
                0x10,
                0x01,
                30,
                0,
                0,
                0x6c,
                0xdc,
                0x02,
                1,
                u8::from(self.streaming_interface),
            ],
        )?;
        // Vendor-specific input terminal and USB streaming output terminal.
        writer.write(0x24, &[2, 1, 0x00, 0x02, 0, 0])?;
        writer.write(0x24, &[3, 2, 0x01, 0x01, 0, 1, 0])?;
        Ok(())
    }

    fn write_video_streaming_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        writer.interface_alt(
            self.streaming_interface,
            0,
            VIDEO_CLASS,
            VIDEO_STREAMING_SUBCLASS,
            0,
            Some(self.interface_string),
        )?;
        let endpoint = self.endpoint.address().into();
        // VS input header: all VS format descriptors together occupy 77 bytes.
        writer.write(0x24, &[1, 1, 77, 0, endpoint, 0, 2, 0, 0, 0, 1, 0])?;
        // NV12 GUID and 12 bits per pixel.
        writer.write(
            0x24,
            &[
                4, 1, 1, 0x4e, 0x56, 0x31, 0x32, 0, 0, 0x10, 0, 0x80, 0, 0, 0xaa, 0, 0x38, 0x9b,
                0x71, 12, 1, 0, 0, 0, 0,
            ],
        )?;
        let mut frame = [0u8; 28];
        frame[0..3].copy_from_slice(&[5, 1, 0]);
        frame[3..5].copy_from_slice(&(WIDTH as u16).to_le_bytes());
        frame[5..7].copy_from_slice(&(HEIGHT as u16).to_le_bytes());
        let bitrate = FRAME_BIT_RATE;
        frame[7..11].copy_from_slice(&bitrate.to_le_bytes());
        frame[11..15].copy_from_slice(&bitrate.to_le_bytes());
        frame[15..19].copy_from_slice(&FRAME_BYTES.to_le_bytes());
        frame[19..23].copy_from_slice(&FRAME_INTERVAL_100NS.to_le_bytes());
        frame[23] = 1;
        frame[24..28].copy_from_slice(&FRAME_INTERVAL_100NS.to_le_bytes());
        writer.write(0x24, &frame)?;
        writer.write(0x24, &[0x0d, 1, 1, 4])?;

        writer.interface_alt(
            self.streaming_interface,
            1,
            VIDEO_CLASS,
            VIDEO_STREAMING_SUBCLASS,
            0,
            Some(self.interface_string),
        )?;
        // The endpoint descriptor advertises three 768-byte transactions per HS microframe.
        writer.endpoint_hs_iso(&self.endpoint, ISO_PACKETS_PER_MICROFRAME - 1)?;
        Ok(())
    }

    fn accepts_control(&self, request: &Request) -> bool {
        request.request_type == RequestType::Class
            && request.recipient == Recipient::Interface
            && (request.index as u8 == u8::from(self.control_interface)
                || request.index as u8 == u8::from(self.streaming_interface))
    }

    fn submit_video_batch(endpoint: &EndpointIn<'a, B>, packet: &[u8], packet_count: u8) -> bool {
        // The packet buffer is 4-byte aligned and remains untouched until
        // endpoint_in_complete queues the next batch.
        unsafe {
            endpoint
                // The descriptor advertises the endpoint's two-transaction
                // maximum. Program DIEPTSIZ MCNT for this transfer's actual
                // packet count, which can be one on a short final frame batch.
                .write_iso_batch_dma(packet, packet_count, packet_count)
                .is_ok()
        }
    }
}

impl<B: UsbBus> UsbClass<B> for UvcUsbClass<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        writer.iad(
            self.control_interface,
            2,
            VIDEO_CLASS,
            0x03,
            0,
            Some(self.interface_string),
        )?;
        self.write_video_control_descriptors(writer)?;
        self.write_video_streaming_descriptors(writer)
    }

    fn get_string(
        &self,
        index: StringIndex,
        _lang_id: usb_device::descriptor::lang_id::LangID,
    ) -> Option<&str> {
        if index == self.interface_string {
            Some("XCore407I NV12 camera")
        } else {
            None
        }
    }

    fn reset(&mut self) {
        let _ = self
            .function
            .set_interface(u8::from(self.streaming_interface), 0);
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        let request = *xfer.request();
        if !self.accepts_control(&request) {
            return;
        }
        let request_type = 0xa0 | request.request_type as u8 | request.recipient as u8;
        match self.function.control(
            request_type,
            request.request,
            request.value,
            request.index,
            request.length,
            &[],
        ) {
            Ok(ControlResponse::Data(data)) => {
                let _ = xfer.accept_with(data);
            }
            Ok(ControlResponse::Ack) | Err(_) => {
                let _ = xfer.reject();
            }
        }
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        let request = *xfer.request();
        if !self.accepts_control(&request) {
            return;
        }
        let request_type = 0x20 | request.request_type as u8 | request.recipient as u8;
        let accepted = matches!(
            self.function.control(
                request_type,
                request.request,
                request.value,
                request.index,
                request.length,
                xfer.data(),
            ),
            Ok(ControlResponse::Ack)
        );
        if accepted {
            let _ = xfer.accept();
        } else {
            let _ = xfer.reject();
        }
    }

    fn get_alt_setting(&mut self, interface: InterfaceNumber) -> Option<u8> {
        if interface == self.streaming_interface {
            Some(self.function.alternate_setting())
        } else if interface == self.control_interface {
            Some(0)
        } else {
            None
        }
    }

    fn set_alt_setting(&mut self, interface: InterfaceNumber, alternative: u8) -> bool {
        if interface != self.streaming_interface {
            return false;
        }
        let Ok(()) = self
            .function
            .set_interface(u8::from(interface), alternative)
        else {
            return false;
        };
        self.incomplete_count = 0;
        if alternative == 1 && self.video_payloads_enabled {
            let marker = self.function.start_stream_batch();
            if let Some((marker, packet_count)) = marker {
                if Self::submit_video_batch(&self.endpoint, marker, packet_count) {
                } else {
                }
            }
        }
        true
    }

    fn endpoint_in_complete(&mut self, address: EndpointAddress) {
        if address == self.endpoint.address() {
            self.incomplete_count = 0;
            if !self.video_payloads_enabled {
                return;
            }
            if self.repeat_video_batch {
                if let Some((packet, packet_count)) = self.function.current_in_batch() {
                    if Self::submit_video_batch(&self.endpoint, packet, packet_count) {
                    } else {
                    }
                }
                return;
            }
            let packet = self.function.next_in_batch();
            if let Some((packet, packet_count)) = packet {
                if Self::submit_video_batch(&self.endpoint, packet, packet_count) {
                } else {
                }
            }
        }
    }

    fn endpoint_in_incomplete(&mut self, address: EndpointAddress) {
        if address != self.endpoint.address() || !self.video_payloads_enabled {
            return;
        }

        self.incomplete_count = self.incomplete_count.saturating_add(1);
        if self.incomplete_count < 3 {
            if let Some((packet, packet_count)) = self.function.current_in_batch() {
                if Self::submit_video_batch(&self.endpoint, packet, packet_count) {
                } else {
                }
            }
            return;
        }

        // Match the C class recovery: reconfigure the stuck endpoint and
        // restart image packetization from the start of a frame.
        self.incomplete_count = 0;
        self.endpoint.recover_iso_endpoint();
        if let Some((packet, packet_count)) = self.function.restart_after_iso_recovery() {
            if Self::submit_video_batch(&self.endpoint, packet, packet_count) {
            } else {
            }
        }
    }
}

type BoardUsbBus = synopsys_usb_otg::UsbBus<F407UsbHs>;
type BoardUsbAllocator = UsbBusAllocator<BoardUsbBus>;

static USB_BUS_TAKEN: AtomicBool = AtomicBool::new(false);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct UsbBusAlreadyInitialized;

#[inline(never)]
pub fn usb_bus() -> core::result::Result<&'static BoardUsbAllocator, UsbBusAlreadyInitialized> {
    // EP_MEMORY and BUS are static mutable storage. Claim them permanently so
    // a second safe call cannot create another mutable reference or overwrite
    // the allocator while the first returned reference remains live.
    USB_BUS_TAKEN
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Relaxed)
        .map_err(|_| UsbBusAlreadyInitialized)?;

    // DMA buffers for EP0 OUT (256 bytes) and EP0 IN (64 bytes). The UVC
    // isochronous endpoint submits directly from the camera buffer.
    const EP0_OUT_WORDS: usize = 256 / 4;
    const EP0_IN_WORDS: usize = 64 / 4;
    static mut EP_MEMORY: [u32; EP0_OUT_WORDS + EP0_IN_WORDS] =
        [0; EP0_OUT_WORDS + EP0_IN_WORDS];
    static mut BUS: MaybeUninit<BoardUsbAllocator> = MaybeUninit::uninit();
    let memory = unsafe { &mut *core::ptr::addr_of_mut!(EP_MEMORY) };
    let bus = synopsys_usb_otg::UsbBus::new(F407UsbHs, memory);
    let allocator = unsafe {
        (*core::ptr::addr_of_mut!(BUS)).write(bus);
        (*core::ptr::addr_of!(BUS)).assume_init_ref()
    };
    Ok(allocator)
}
