//! USB Still Image/PTP interface descriptors for Windows MTP discovery.
//!
//! This is intentionally descriptor-only: it allocates the three endpoints
//! and advertises the standard Still Image class interface, but does not
//! implement PTP/MTP control requests or endpoint data transfers.

use usb_device::bus::{InterfaceNumber, StringIndex, UsbBus, UsbBusAllocator};
use usb_device::class::UsbClass;
use usb_device::descriptor::DescriptorWriter;
use usb_device::endpoint::{EndpointAddress, EndpointIn, EndpointOut, EndpointType};
use usb_device::Result;

const STILL_IMAGE_CLASS: u8 = 0x06;
const PTP_SUBCLASS: u8 = 0x01;
const PTP_PROTOCOL: u8 = 0x01;

const BULK_MAX_PACKET_SIZE: u16 = 512;
const EVENT_MAX_PACKET_SIZE: u16 = 64;
const EVENT_INTERVAL: u8 = 4;

/// Descriptor-only MTP transport interface.
///
/// Endpoint numbers are kept separate from the existing UVC (EP1 IN) and
/// UAC2 (EP2 IN) endpoints:
/// - EP3 OUT: bulk
/// - EP3 IN: bulk
/// - EP4 IN: interrupt/event
pub struct MtpDescriptors<'a, B: UsbBus> {
    interface: InterfaceNumber,
    interface_string: StringIndex,
    bulk_out: EndpointOut<'a, B>,
    bulk_in: EndpointIn<'a, B>,
    event_in: EndpointIn<'a, B>,
}

impl<'a, B: UsbBus> MtpDescriptors<'a, B> {
    pub fn new(allocator: &'a UsbBusAllocator<B>) -> Self {
        let interface = allocator.interface();
        let interface_string = allocator.string();

        let bulk_out = allocator
            .alloc(
                Some(EndpointAddress::from(0x03)),
                EndpointType::Bulk,
                BULK_MAX_PACKET_SIZE,
                0,
            )
            .expect("MTP bulk OUT endpoint allocation failed");
        let bulk_in = allocator
            .alloc(
                Some(EndpointAddress::from(0x83)),
                EndpointType::Bulk,
                BULK_MAX_PACKET_SIZE,
                0,
            )
            .expect("MTP bulk IN endpoint allocation failed");
        let event_in = allocator
            .alloc(
                Some(EndpointAddress::from(0x84)),
                EndpointType::Interrupt,
                EVENT_MAX_PACKET_SIZE,
                EVENT_INTERVAL,
            )
            .expect("MTP event IN endpoint allocation failed");

        Self {
            interface,
            interface_string,
            bulk_out,
            bulk_in,
            event_in,
        }
    }
}

impl<B: UsbBus> UsbClass<B> for MtpDescriptors<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        writer.interface(
            self.interface,
            STILL_IMAGE_CLASS,
            PTP_SUBCLASS,
            PTP_PROTOCOL,
        )?;
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
        if index == self.interface_string {
            Some("XCore407I MTP")
        } else {
            None
        }
    }
}
