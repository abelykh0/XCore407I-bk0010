//! USB Audio Class 2.0
//!
//! This crate provides a USB Audio Class 2.0 implementation for
//! [usb-device](https://crates.io/crates/usb-device). It implements all
//! required elements of the specification, however many controls are not
//! implemented (e.g. mixers, effects).
//!
//! Device behaviour is driven by implementing the `ClockSource` and
//! `AudioHandler` traits to configure the audio pipeline and source/sink data
//! to/from USB.
//!
//! Example (creates a UAC2 device with in and out streams):
//!
//! ```ignore
//! let mut audio = YourTraitImpl {...};
//!
//! let config = UsbAudioClassConfig::new(usb_speed, FunctionCode::IoBox, &mut audio)
//!     // base_id is USB entity ID, id 1 is always taken by the clock source, and each stream builds 2 entities
//!     .with_output_config(TerminalConfig::builder().base_id(2).build())
//!     .with_input_config(TerminalConfig::builder().base_id(4).build());
//!
//! let mut uac2 = config.build(&usb_bus).unwrap();
//!
//! let mut usb_dev = usbd_uac2::builder(&usb_bus, UsbVidPid(0x1209, 0x0001))
//!     .strings(&[StringDescriptors::default()
//!         .manufacturer("usbd_uac2")
//!         .product("example")])
//!     .unwrap()
//!     .max_packet_size_0(64) // Required to be 64 on HS
//!     .unwrap()
//!     .build();
//! ```
//!
//! No work needs to be done in the poll loop, the class implementation will
//! call your trait callbacks as required, just call the usb poll as usual:
//!
//! ```ignore
//! loop {
//!     usb_dev.poll(&mut [&mut uac2]);
//! }
//! ```
//!
//! See the trait documentation or examples for additional details.

#![no_std]
#![allow(dead_code)]

pub mod constants;
mod cursor;
pub mod descriptors;
mod log;

use core::cmp::Ordering;
use core::marker::PhantomData;

use constants::*;
use descriptors::*;
#[allow(unused_imports)]
use log::*;

use byteorder_embedded_io::{LittleEndian, ReadBytesExt, WriteBytesExt};
use num_traits::{ConstZero, ToPrimitive};
use usb_device::control::{Recipient, Request, RequestType};
use usb_device::device::{DEFAULT_ALTERNATE_SETTING, UsbDeviceBuilder, UsbVidPid};
use usb_device::endpoint::{self, Endpoint, EndpointDirection, In, Out, EndpointAddress, EndpointType};
use usb_device::{UsbDirection, class_prelude::*};

mod sealed {
    pub trait Sealed {}
}

#[derive(Debug)]
pub enum UsbAudioClassError {
    NotImplemented,
    Other,
}

impl<T: core::error::Error> From<T> for UsbAudioClassError {
    fn from(_: T) -> Self {
        UsbAudioClassError::Other
    }
}

impl From<UsbAudioClassError> for UsbError {
    fn from(_value: UsbAudioClassError) -> Self {
        UsbError::Unsupported
    }
}

pub trait RangeType:
    sealed::Sealed + num_traits::PrimInt + num_traits::ToBytes + ConstZero
{
}
impl sealed::Sealed for i8 {}
impl sealed::Sealed for i16 {}
impl sealed::Sealed for i32 {}
impl sealed::Sealed for u8 {}
impl sealed::Sealed for u16 {}
impl sealed::Sealed for u32 {}

impl RangeType for i8 {}
impl RangeType for i16 {}
impl RangeType for i32 {}
impl RangeType for u8 {}
impl RangeType for u16 {}
impl RangeType for u32 {}

/// Represent a range request/response.
///
/// ref: UAC2 5.2.2
#[derive(PartialEq, Eq)]
pub struct RangeEntry<T: RangeType> {
    pub min: T,
    pub max: T,
    pub res: T,
}

impl<T: RangeType> RangeEntry<T> {
    pub const fn new(min: T, max: T, res: T) -> Self {
        Self { min, max, res }
    }
    pub const fn new_fixed(fixed: T) -> Self {
        Self {
            min: fixed,
            max: fixed,
            res: T::ZERO,
        }
    }

    fn write<W: embedded_io::Write>(&self, mut buf: W) -> core::result::Result<usize, W::Error> {
        buf.write_all(self.min.to_le_bytes().as_ref())?;
        buf.write_all(self.max.to_le_bytes().as_ref())?;
        buf.write_all(self.res.to_le_bytes().as_ref())?;
        Ok((T::ZERO.count_zeros() as usize / 8) * 3)
    }
}

// The spec guarantees that ranges do not overlap, so compare by min is correct.
impl<T: RangeType + PartialOrd> PartialOrd for RangeEntry<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl<T: RangeType + PartialOrd> Ord for RangeEntry<T> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.min.cmp(&other.min)
    }
}
/// Represent Isochronous feedback as integer and fractional parts, internally as 16.16.
///
/// Provides conversions to and from the different USB formats.
#[derive(Copy, Clone)]
pub struct UsbIsochronousFeedback(u32);

impl UsbIsochronousFeedback {
    pub fn new(v: u32) -> Self {
        Self(v)
    }
    /// Accepts all u16 values, saturating the output depending on format
    pub fn new_frac(int: u16, frac: u16) -> Self {
        Self(((int as u32) << 16) | frac as u32)
    }
    /// Convert float to fixed point
    pub fn new_float(rate: f32) -> Self {
        let fb = (rate * 65536.0 + 0.5) as u32;
        Self(fb)
    }
    pub fn parts(&self) -> (u16, u16) {
        ((self.0 >> 16) as u16, (self.0 & 0xffff) as u16)
    }
    /// Serialize into a u32 in 16.16 representation for USB HS
    pub fn to_u32_12_13(&self) -> u32 {
        self.0
    }
    /// Serialize into a u32 in 10.14 representation for USB FS
    pub fn to_u32_10_14(&self) -> u32 {
        (self.0 + 2) >> 2
    }
    /// Serialize into 16.16 little endian byte array for USB HS
    pub fn to_bytes_12_13(&self) -> [u8; 4] {
        self.to_u32_12_13().to_le_bytes()
    }
    /// Serialize into 10.14 little endian byte array for USB FS
    pub fn to_bytes_10_14(&self) -> [u8; 3] {
        let bytes = self.to_u32_10_14().to_le_bytes();
        [bytes[0], bytes[1], bytes[2]]
    }
}

/// A trait for implementing USB Audio Class 2 devices
///
/// Contains callback methods which will be called by the class driver.
pub trait AudioHandler<'a, B: UsbBus> {
    /// Called when audio data is received from the host. `ep` is ready for
    /// `ep.read()`.
    fn audio_data_rx(&mut self, ep: &Endpoint<'a, B, endpoint::Out>);

    /// Called when we must produce audio data for the host. `ep` is ready for
    /// `ep.write()`.
    fn audio_data_tx(&mut self, ep: &Endpoint<'a, B, endpoint::In>);

    /// Called after a previously submitted audio IN packet completes.
    fn audio_data_tx_complete(&mut self) {}

    /// Called when a previously submitted audio IN packet misses its service
    /// interval. The class will request a replacement packet immediately after
    /// this callback returns.
    fn audio_data_tx_incomplete(&mut self) {}

    /// Called when it's time to send an isochronous feedback update. Should
    /// return the correct feedback payload. Feedback always runs at 1ms (in
    /// this implementation), and will be passed the nominal frame size.
    ///
    /// Required for isochronous asynchronous mode to work properly. If None is
    /// returned, no IN packet will be emitted at feedback time.
    fn feedback(&mut self, nominal: UsbIsochronousFeedback) -> Option<UsbIsochronousFeedback>;

    /// Called when the alternate setting of `terminal`'s interface is changed,
    /// before the `AudioStream` is updated. This is how the host signals start/stop
    /// of audio streaming. We do not expect audio frames when alt setting is 0.
    ///
    /// The implementation will also call `alternate_setting_changed(_, 0)` when
    /// the host disconnects / resets.
    fn alternate_setting_changed(&mut self, terminal: UsbDirection, alt_setting: u8);
}

/// A trait for implementing Sampling Frequency Control for USB Audio Clock Sources
/// ref: USB Audio Class Specification 2.0 5.2.5.1.1
///
/// Contains callback methods which will be called by the class driver.
///
/// Unimplemented callbacks should return `Err(UsbAudioClassError::NotImplemented)`. Other
/// errors will panic (the underlying callbacks are not fallible). If you need to handle errors,
/// you should use the callback to infalliably signal another task.
pub trait ClockSource {
    const CLOCK_TYPE: ClockType;
    const SOF_SYNC: bool;
    /// Called when the host or class needs the current sample rate. Returns the
    /// sample rate in Hz. It should be cheap and infallible as it gets called in every cycle
    /// of the feedback loop. Use clock validity to signal to the host if the clock is not usable.
    ///
    /// Should never return 0 as it may be used in divides in the feedback loop
    /// and that would cause a hard fault.
    fn sample_rate(&self) -> u32;
    /// Called when the host requests to set the sample rate. Not necessarily called at all startups,
    /// so alt_setting should start/stop the clock. Not required for 'fixed' clocks.
    #[allow(unused_variables)]
    fn set_sample_rate(
        &mut self,
        sample_rate: u32,
    ) -> core::result::Result<(), UsbAudioClassError> {
        Err(UsbAudioClassError::NotImplemented)
    }
    /// Called when the host requests to get the clock validity. Returns `true`
    /// if the clock is stable and on frequency.
    fn clock_validity(&self) -> core::result::Result<bool, UsbAudioClassError> {
        Err(UsbAudioClassError::NotImplemented)
    }
    /// Called during descriptor construction to describe if the clock validity can be read (write is not valid).
    ///
    /// By default will call `get_clock_validity` to determine if the clock validity can be read.
    fn clock_validity_access(&self) -> core::result::Result<bool, UsbAudioClassError> {
        match self.clock_validity() {
            Ok(_) => Ok(true),
            Err(UsbAudioClassError::NotImplemented) => Ok(false),
            Err(err) => Err(err),
        }
    }

    /// Called during allocation and when the hosts makes a RANGE request for the clock source.
    ///
    /// Returns a slice of possible sample rates.
    ///
    /// Rates must meet the invariants in the specification:
    ///   * The subranges must be ordered in ascendingorder
    ///   * Individual subranges cannot overlap
    ///   * If a subrange consists of only a single value, the corresponding triplet must contain that value for both
    ///     its MIN and MAX subattribute and the RES subattribute must be set to zero
    ///
    /// ref: USB Audio Class Specification 2.0 5.2.1 & 5.2.3.3
    fn sample_rates(&self) -> core::result::Result<&[RangeEntry<u32>], UsbAudioClassError>;

    /// Build the ClockSource descriptor. It is not intended to override this
    /// method, but provided so you can.
    ///
    /// Assumes access control based on clock type. Internal fixed/variable are read only,
    /// external and internal programmable are programmable.
    fn clock_configuration_descriptor(
        &self,
        id: u8,
        string: Option<StringIndex>,
    ) -> usb_device::Result<descriptors::ClockSource> {
        let frequency_access = match Self::CLOCK_TYPE {
            ClockType::InternalFixed | ClockType::InternalVariable => AccessControl::ReadOnly,
            ClockType::External | ClockType::InternalProgrammable => AccessControl::Programmable,
        };
        let validity_access = match self.clock_validity_access() {
            Ok(true) => AccessControl::ReadOnly,
            Ok(false) | Err(UsbAudioClassError::NotImplemented) => AccessControl::NotPresent,
            _ => return Err(UsbError::Unsupported),
        };

        let cs = descriptors::ClockSource {
            id,
            clock_type: Self::CLOCK_TYPE,
            sof_sync: Self::SOF_SYNC,
            frequency_access,
            validity_access,
            assoc_terminal: 0,
            string,
        };
        Ok(cs)
    }
}

// This trait is needed since we specialize on D
trait TerminalConfigurationDescriptors {
    fn get_configuration_descriptors(&self) -> (InputTerminal, OutputTerminal);
}

pub struct TerminalConfigBuilder<D: EndpointDirection> {
    base_id: Option<u8>,
    clock_source_id: Option<u8>,
    num_channels: Option<u8>,
    format: Option<FormatType1>,
    terminal_type: Option<TerminalType>,
    channel_config: Option<ChannelConfig>,
    sync_type: Option<IsochronousSynchronizationType>,
    lock_delay: Option<LockDelay>,
    string: Option<StringIndex>,
    _direction: PhantomData<D>,
}

// Global builder initialization defaults
impl<D: EndpointDirection> TerminalConfigBuilder<D> {
    fn new() -> Self {
        Self {
            base_id: None,
            clock_source_id: None,
            num_channels: None,
            format: None,
            terminal_type: None,
            channel_config: None,
            sync_type: None,
            lock_delay: None,
            string: None,
            _direction: PhantomData,
        }
    }

    pub fn base_id(mut self, id: u8) -> Self {
        self.base_id = Some(id);
        self
    }

    pub fn clock_source_id(mut self, id: u8) -> Self {
        self.clock_source_id = Some(id);
        self
    }

    pub fn num_channels(mut self, channels: u8) -> Self {
        self.num_channels = Some(channels);
        self
    }

    pub fn format(mut self, format: FormatType1) -> Self {
        self.format = Some(format);
        self
    }

    pub fn terminal_type(mut self, t: TerminalType) -> Self {
        self.terminal_type = Some(t);
        self
    }

    pub fn channel_config(mut self, config: ChannelConfig) -> Self {
        self.channel_config = Some(config);
        self
    }

    pub fn sync_type(mut self, sync: IsochronousSynchronizationType) -> Self {
        self.sync_type = Some(sync);
        self
    }

    pub fn lock_delay(mut self, delay: LockDelay) -> Self {
        self.lock_delay = Some(delay);
        self
    }

    pub fn string(mut self, index: StringIndex) -> Self {
        self.string = Some(index);
        self
    }

    pub fn build(self) -> TerminalConfig<D> {
        TerminalConfig {
            base_id: self.base_id.expect("base_id is required"),
            clock_source_id: self.clock_source_id.unwrap_or(1),
            num_channels: self.num_channels.unwrap_or(2),
            format: self.format.unwrap_or(FormatType1 {
                bytes_per_sample: 4,
                bit_resolution: 32,
            }),
            terminal_type: self.terminal_type.unwrap_or(TerminalType::ExtUndefined),
            channel_config: self
                .channel_config
                .unwrap_or(ChannelConfig::default_chans(self.num_channels.unwrap_or(2))),
            sync_type: IsochronousSynchronizationType::Asynchronous,
            lock_delay: LockDelay::Undefined(0),
            string: self.string, // Implicitly handles None or Option mapping
            _direction: PhantomData,
        }
    }
}

pub struct TerminalConfig<D: EndpointDirection> {
    /// USB terminal in the D direction will have this id, audio terminal will have this id + 1
    base_id: u8,
    clock_source_id: u8,
    num_channels: u8,
    format: FormatType1,
    terminal_type: TerminalType,
    channel_config: ChannelConfig,
    sync_type: IsochronousSynchronizationType,
    lock_delay: LockDelay,
    string: Option<StringIndex>,
    _direction: PhantomData<D>,
}

impl<D: EndpointDirection> TerminalConfig<D> {
    #[allow(clippy::too_many_arguments)]
    fn new(
        base_id: u8,
        clock_source_id: u8,
        num_channels: u8,
        format: FormatType1,
        terminal_type: TerminalType,
        channel_config: ChannelConfig,
        sync_type: IsochronousSynchronizationType,
        lock_delay: LockDelay,
        string: Option<StringIndex>,
    ) -> Self {
        TerminalConfig {
            base_id,
            clock_source_id,
            num_channels,
            format,
            terminal_type,
            channel_config,
            sync_type,
            lock_delay,
            string,
            _direction: PhantomData,
        }
    }
    // Bytes per audio frame
    pub fn bytes_per_frame(&self) -> u32 {
        self.format.bytes_per_sample as u32 * self.num_channels as u32
    }

    pub fn builder() -> TerminalConfigBuilder<D> {
        TerminalConfigBuilder::new()
    }
}
impl TerminalConfigurationDescriptors for TerminalConfig<Out> {
    fn get_configuration_descriptors(&self) -> (InputTerminal, OutputTerminal) {
        let input_terminal = InputTerminal {
            id: self.base_id,
            terminal_type: TerminalType::UsbStreaming,
            assoc_terminal: 0,
            clock_source: self.clock_source_id,
            num_channels: self.num_channels,
            channel_config: self.channel_config,
            channel_names: 0, // not supported
            copy_protect_control: AccessControl::NotPresent,
            connector_control: AccessControl::NotPresent,
            overload_control: AccessControl::NotPresent,
            cluster_control: AccessControl::NotPresent,
            underflow_control: AccessControl::NotPresent,
            overflow_control: AccessControl::NotPresent,
            phantom_power_control: AccessControl::NotPresent,
            string: self.string,
        };
        let output_terminal = OutputTerminal {
            id: self.base_id + 1,
            terminal_type: self.terminal_type,
            assoc_terminal: 0,
            source_id: self.base_id,
            clock_source: self.clock_source_id,
            copy_protect_control: AccessControl::NotPresent,
            connector_control: AccessControl::NotPresent,
            overload_control: AccessControl::NotPresent,
            underflow_control: AccessControl::NotPresent,
            overflow_control: AccessControl::NotPresent,
            string: self.string,
        };
        (input_terminal, output_terminal)
    }
    // fn get_interface_descriptor(&self, id: InterfaceIndex) )
}

impl TerminalConfigurationDescriptors for TerminalConfig<In> {
    fn get_configuration_descriptors(&self) -> (InputTerminal, OutputTerminal) {
        let output_terminal = OutputTerminal {
            id: self.base_id,
            terminal_type: TerminalType::UsbStreaming,
            assoc_terminal: 0,
            source_id: self.base_id + 1,
            clock_source: self.clock_source_id,
            copy_protect_control: AccessControl::NotPresent,
            connector_control: AccessControl::NotPresent,
            overload_control: AccessControl::NotPresent,
            underflow_control: AccessControl::NotPresent,
            overflow_control: AccessControl::NotPresent,
            string: self.string,
        };
        let input_terminal = InputTerminal {
            id: self.base_id + 1,
            terminal_type: self.terminal_type,
            assoc_terminal: 0,
            clock_source: self.clock_source_id,
            num_channels: self.num_channels,
            channel_config: self.channel_config,
            channel_names: 0,
            copy_protect_control: AccessControl::NotPresent,
            connector_control: AccessControl::NotPresent,
            cluster_control: AccessControl::NotPresent,
            overload_control: AccessControl::NotPresent,
            underflow_control: AccessControl::NotPresent,
            overflow_control: AccessControl::NotPresent,
            phantom_power_control: AccessControl::NotPresent,
            string: self.string,
        };
        (input_terminal, output_terminal)
    }
}

#[derive(Copy, Clone, Debug)]
pub enum UsbSpeed {
    Low, // Not supported for audio
    Full,
    High,
    Super,
}

/// Configuration and references to the Audio Class descriptors
///
/// Supports one clock source, optionally one input terminal and optionally one output terminal.
/// An optional set of additional descriptors can be provided, but must be handled by the user.
///
/// The two Terminal descriptors will be built per their TerminalConfig
///
/// Unit IDs will be fixed as follows:
/// * Clock Source: 1
/// * USB Streaming Input: 2
/// * Output Terminal: 3
/// * USB Streaming Output: 4
/// * Input Terminal: 5
/// * User provided descriptors: 6+
///
/// A single Clock Source is always required, but a fully custom descriptor set can be built by only providing
/// the Clock Source and additional descriptors, if the Terminal descriptors are inappropriate.
///
pub struct UsbAudioClassConfig<'a, B: UsbBus, AU: AudioHandler<'a, B> + ClockSource> {
    pub speed: UsbSpeed,
    pub device_category: FunctionCode,
    pub audio_impl: &'a mut AU,
    pub input_config: Option<TerminalConfig<In>>,
    pub output_config: Option<TerminalConfig<Out>>,
    pub additional_descriptors: Option<&'a [AudioClassDescriptor]>,
    _bus: PhantomData<B>,
}

impl<'a, B: UsbBus, AU: AudioHandler<'a, B> + ClockSource> UsbAudioClassConfig<'a, B, AU> {
    pub fn new(speed: UsbSpeed, device_category: FunctionCode, audio_impl: &'a mut AU) -> Self {
        Self {
            speed,
            device_category,
            audio_impl,
            input_config: None,
            output_config: None,
            additional_descriptors: None,
            _bus: PhantomData,
        }
    }
    pub fn with_input_config(mut self, input_config: TerminalConfig<In>) -> Self {
        self.input_config = Some(input_config);
        self
    }
    pub fn with_output_config(mut self, output_config: TerminalConfig<Out>) -> Self {
        self.output_config = Some(output_config);
        self
    }
    pub fn with_additional_descriptors(
        mut self,
        additional_descriptors: &'a [AudioClassDescriptor],
    ) -> Self {
        self.additional_descriptors = Some(additional_descriptors);
        self
    }

    /// Allocate the various USB IDs, and build the class implementation
    pub fn build(self, alloc: &'a UsbBusAllocator<B>) -> Result<UsbAudioClass<'a, B, AU>> {
        let speed = self.speed;
        let (interval, fb_interval, audio_rate) = match speed {
            UsbSpeed::Full => (1, 1, 1000),
            UsbSpeed::High | UsbSpeed::Super => (1, 4, 8000), //
            UsbSpeed::Low => return Err(Error::InvalidSpeed),
        };
        let max_rate = self
            .audio_impl
            .sample_rates()
            .unwrap()
            .iter()
            .max()
            .unwrap()
            .max;
        let control_iface = alloc.interface();

        let nominal_fb = UsbIsochronousFeedback::new_float(
            self.audio_impl.sample_rate().to_f32().unwrap() / audio_rate.to_f32().unwrap(),
        );

        let mut ac = UsbAudioClass {
            control_iface,
            audio_impl: self.audio_impl,
            output: None,
            input: None,
            feedback: None,
            additional_descriptors: self.additional_descriptors,
            device_category: self.device_category,
            in_iface: 0,
            out_iface: 0,
            in_ep: 0,
            out_ep: 0,
            fb_ep: 0,
            speed,
            nominal_fb,
            audio_rate,
        };

        if let Some(config) = self.output_config {
            let interface = alloc.interface();
            let endpoint = alloc.isochronous(
                config.sync_type,
                IsochronousUsageType::Data,
                ((max_rate.div_ceil(audio_rate) + 1) * config.bytes_per_frame()) as u16, // headroom of 1 sample for rate control
                interval,
            );
            let feedback_ep = alloc.isochronous(
                IsochronousSynchronizationType::NoSynchronization,
                IsochronousUsageType::Feedback,
                4,
                fb_interval,
            );
            let alt_setting = DEFAULT_ALTERNATE_SETTING;
            ac.out_iface = interface.into();
            ac.out_ep = endpoint.address().index();
            ac.fb_ep = feedback_ep.address().index();
            ac.output = Some(AudioStream {
                config,
                interface,
                endpoint,
                alt_setting,
            });
            ac.feedback = Some(feedback_ep);
        }

        if let Some(config) = self.input_config {
            let interface = alloc.interface();
            // Match the known-working XCore407I composite endpoint layout:
            // UVC camera IN = EP1 (0x81), UAC2 audio IN = EP2 (0x82).
            let endpoint = alloc.alloc::<In>(
                Some(EndpointAddress::from(0x82)),
                EndpointType::Isochronous {
                    synchronization: config.sync_type,
                    usage: IsochronousUsageType::Data,
                },
                ((max_rate.div_ceil(audio_rate) + 1) * config.bytes_per_frame()) as u16,
                interval,
            ).expect("UAC2 audio IN endpoint EP2 allocation failed");
            let alt_setting = DEFAULT_ALTERNATE_SETTING;
            ac.in_iface = interface.into();
            ac.in_ep = endpoint.address().index();

            ac.input = Some(AudioStream {
                config,
                interface,
                endpoint,
                alt_setting,
            });
        }

        Ok(ac)
    }
}

/// USB audio errors, including possible USB Stack errors
#[derive(Debug)]
pub enum Error {
    InvalidValue,
    BandwidthExceeded,
    StreamNotInitialized,
    InvalidSpeed,
    UsbError(usb_device::UsbError),
}

impl From<UsbError> for Error {
    fn from(err: UsbError) -> Self {
        Error::UsbError(err)
    }
}
type Result<T> = core::result::Result<T, Error>;

struct AudioStream<'a, B: UsbBus, D: EndpointDirection> {
    // UsbStreaming terminal ID
    config: TerminalConfig<D>,
    interface: InterfaceNumber,
    endpoint: Endpoint<'a, B, D>,
    alt_setting: u8,
}

impl<'a, B: UsbBus, D: EndpointDirection> AudioStream<'a, B, D> {
    fn write_interface_descriptors(&self, writer: &mut DescriptorWriter) -> usb_device::Result<()> {
        debug!(
            "  AudioStream<{}>.write_interface_descriptors",
            core::any::type_name::<D>()
        );
        // UAC2 4.9.1 Standard AS Interface Descriptor
        //   zero bandwidth configuration per 3.16.2
        //
        // Whenever an AudioStreaming interface requires an isochronous data
        // endpoint, it must at least provide the default Alternate Setting
        // (Alternate Setting 0) with zero bandwidth requirements (no
        // isochronous data endpoint defined) and an additional Alternate
        // Setting that contains the actual isochronous data endpoint.
        debug!("writer.interface AudioStreaming");
        writer.interface(
            self.interface,
            USB_CLASS_AUDIO,
            InterfaceSubclass::AudioStreaming as u8,
            InterfaceProtocol::Version2 as u8,
        )?;
        // UAC2 4.9.1 Standard AS Interface Descriptor
        //   live data configuration
        debug!("writer.interface_alt AudioStreaming");
        writer.interface_alt(
            self.interface,
            1,
            USB_CLASS_AUDIO,
            InterfaceSubclass::AudioStreaming as u8,
            InterfaceProtocol::Version2 as u8,
            None,
        )?;

        // UAC2 4.9.2 Class-Specific AS Interface Descriptor
        let as_general = AudioStreamingInterface {
            terminal_id: self.config.base_id, // Always the USB streaming terminal id
            active_alt_setting: AccessControl::NotPresent,
            valid_alt_settings: AccessControl::NotPresent,
            format_type: FormatType::Type1,
            format_bitmap: Type1FormatBitmap::Pcm, // only PCM is supported
            num_channels: self.config.num_channels,
            channel_config: self.config.channel_config,
            string: self.config.string,
        };
        as_general.write_descriptor(writer)?;

        // UAC2 4.9.3 Class-Specific AS Format Type Descriptor
        self.config.format.write_descriptor(writer)?;

        Ok(())
    }

    fn write_endpoint_descriptors(&self, writer: &mut DescriptorWriter) -> usb_device::Result<()> {
        // UAC2 4.10.1.1 Standard AS Isochronous Audio Data Endpoint Descriptor
        debug!(
            "  AudioStream<{}>.write_endpoint_descriptors",
            core::any::type_name::<D>()
        );
        debug!("writer.endpoint");
        writer.endpoint(&self.endpoint)?;
        // UAC2 4.10.1.2 Class-Specific AS Isochronous Audio Data Endpoint Descriptor
        let cs_ep = AudioStreamingEndpoint {
            max_packets_only: false,
            pitch_control: AccessControl::NotPresent,
            overrun_control: AccessControl::NotPresent,
            underrun_control: AccessControl::NotPresent,
            lock_delay: self.config.lock_delay,
        };
        cs_ep.write_descriptor(writer)?;
        Ok(())
    }
}

pub struct UsbAudioClass<'a, B: UsbBus, AU: AudioHandler<'a, B> + ClockSource> {
    control_iface: InterfaceNumber,
    audio_impl: &'a mut AU,
    output: Option<AudioStream<'a, B, Out>>,
    input: Option<AudioStream<'a, B, In>>,
    feedback: Option<Endpoint<'a, B, In>>,
    additional_descriptors: Option<&'a [AudioClassDescriptor]>,
    device_category: FunctionCode,
    in_iface: u8,
    out_iface: u8,
    in_ep: usize,
    out_ep: usize,
    fb_ep: usize,
    speed: UsbSpeed,
    nominal_fb: UsbIsochronousFeedback,
    audio_rate: u32, // audio packet rate in hz
}

impl<'a, B: UsbBus, AU: AudioHandler<'a, B> + ClockSource> UsbClass<B>
    for UsbAudioClass<'a, B, AU>
{
    fn get_configuration_descriptors(
        &self,
        writer: &mut DescriptorWriter<'_>,
    ) -> usb_device::Result<()> {
        info!("  AudioClass::get_configuration_descriptors");
        // Control + 0-2 streaming
        let n_interfaces = 1 + (self.input.is_some() as u8) + (self.output.is_some() as u8);

        debug!("writer.iad()");
        // UAC2 4.6 Interface Association Descriptor
        writer.iad(
            self.control_iface,
            n_interfaces,
            USB_CLASS_AUDIO,
            InterfaceSubclass::Undefined as u8,
            FunctionProtocol::Version2 as u8,
            None,
        )?;

        debug!("writer.interface()");
        // UAC2 4.7 Standard AC Interface Descriptor
        writer.interface(
            self.control_iface,
            USB_CLASS_AUDIO,
            InterfaceSubclass::AudioControl as u8,
            InterfaceProtocol::Version2 as u8,
        )?;

        // BUILD CONFIGURATION DESCRIPTORS //
        let mut total_length: u16 = 9; // HEADER
        let clock_desc = self.audio_impl.clock_configuration_descriptor(1, None)?;
        total_length += clock_desc.size() as u16;
        let output_descs = match &self.output {
            Some(stream) => {
                let descs = stream.config.get_configuration_descriptors();
                total_length += descs.0.size() as u16 + descs.1.size() as u16;
                Some(descs)
            }
            None => None,
        };
        let input_descs = match &self.input {
            Some(stream) => {
                let descs = stream.config.get_configuration_descriptors();
                total_length += descs.0.size() as u16 + descs.1.size() as u16;
                Some(descs)
            }
            None => None,
        };
        let additional_descs = match &self.additional_descriptors {
            Some(descs) => {
                total_length += descs.iter().map(|desc| desc.size() as u16).sum::<u16>();
                Some(descs)
            }
            None => None,
        };
        debug!(
            "have output: {}, have input: {}, have additional: {}, total length: {}",
            output_descs.is_some(),
            input_descs.is_some(),
            additional_descs.is_some(),
            total_length
        );

        // UAC2 4.7.2 Class-specific AC Interface Descriptor
        let ac_header: [u8; 7] = [
            ClassSpecificACInterfaceDescriptorSubtype::Header as u8,
            0,                                  // bcdADC[0]
            2,                                  // bcdADC[1]
            self.device_category as u8,         // bCategory
            (total_length & 0xff) as u8,        // wTotalLength LSB
            ((total_length >> 8) & 0xff) as u8, // wTotalLength MSB
            0,                                  // bmControls
        ];
        debug!("writer.write (AC header)");
        writer.write(ClassSpecificDescriptorType::Interface as u8, &ac_header)?;

        // UAC2 4.7.2.1 Clock Source Descriptor
        clock_desc.write_descriptor(writer)?;

        // UAC2 4.7.2.4 & 4.7.2.5 Input & Output Terminal Descriptors
        if let Some((a, b)) = output_descs {
            a.write_descriptor(writer)?;
            b.write_descriptor(writer)?;
        }
        if let Some((a, b)) = input_descs {
            a.write_descriptor(writer)?;
            b.write_descriptor(writer)?;
        }

        // UAC2 4.7
        if let Some(descs) = additional_descs {
            for desc in descs.iter() {
                desc.write_descriptor(writer)?;
            }
        }

        // UAC2 4.9 & 2.4.10
        if let Some(stream) = &self.input {
            stream.write_interface_descriptors(writer)?;
            stream.write_endpoint_descriptors(writer)?;
        }
        if let Some(stream) = &self.output {
            stream.write_interface_descriptors(writer)?;
            stream.write_endpoint_descriptors(writer)?;
        }
        // UAC2 4.9.2.1 Feedback Endpoint Descriptor
        //   Should always be present if an OUT endpoint is present
        if let Some(feedback) = &self.feedback {
            debug!("writer.endpoint (feedback)");
            writer.endpoint(feedback)?;
        }

        Ok(())
    }
    fn control_out(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();
        match req.request_type {
            RequestType::Standard => self.standard_request_out(xfer),
            RequestType::Class => self.class_request_out(xfer),
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }
    fn control_in(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();
        match req.request_type {
            RequestType::Standard => self.standard_request_in(xfer),
            RequestType::Class => self.class_request_in(xfer),
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }
    fn endpoint_out(&mut self, addr: EndpointAddress) {
        debug!("EP {} out data", addr);
        if addr.index() == self.out_ep {
            self.audio_impl
                .audio_data_rx(&self.output.as_ref().unwrap().endpoint);
        } else {
            debug!("  unexpected OUT on {}", addr);
        }
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        debug!("EP {} IN complete", addr);
        if let Some(_fb_ep) = self.feedback.as_ref()
            && addr.index() == self.fb_ep
        {
            self.emit_feedback();
        } else if let Some(in_ep) = self.input.as_ref()
            && addr.index() == self.in_ep
        {
            self.audio_impl.audio_data_tx_complete();
            self.audio_impl.audio_data_tx(&in_ep.endpoint);
        } else {
            debug!("  unexpected IN on {}", addr);
        }
    }

    fn endpoint_in_incomplete(&mut self, addr: EndpointAddress) {
        // A missed HS microframe must not permanently stop a capture stream.
        // The DWC2 bus driver has stopped the missed transfer before dispatching
        // this callback, so the handler can queue the next packet.
        if let Some(in_ep) = self.input.as_ref()
            && addr.index() == self.in_ep
        {
            self.audio_impl.audio_data_tx_incomplete();
            self.audio_impl.audio_data_tx(&in_ep.endpoint);
        }
    }

    fn poll(&mut self) {
        debug!("poll");
        // no streaming in alt 0
        if self.output.as_ref().is_none_or(|o| o.alt_setting != 0)
            || self.input.as_ref().is_none_or(|i| i.alt_setting != 0)
        {
            return;
        }
        loop {
            if let Some(o) = self.output.as_ref() {
                let mut buf = [0; 1024];
                match o.endpoint.read(&mut buf) {
                    Ok(len) if len > 0 => {
                        debug!("EP OUT data {:?}", len);
                    }
                    Ok(_) => {
                        debug!("EP OUT empty");
                        break;
                    }
                    Err(UsbError::WouldBlock) => break,
                    Err(_err) => {
                        debug!("EP OUT error {:?}", _err);
                    }
                }
            }
        }
    }

    fn reset(&mut self) {
        info!("usb reset");
        // A bus reset returns every AudioStreaming interface to its implicit
        // zero-bandwidth alternate setting. Keep the software state in sync so
        // a later SET_INTERFACE(1) restarts the capture pipeline.
        if let Some(input) = self.input.as_mut() {
            input.alt_setting = 0;
        }
        if let Some(output) = self.output.as_mut() {
            output.alt_setting = 0;
        }
        self.audio_impl
            .alternate_setting_changed(UsbDirection::In, 0);
        self.audio_impl
            .alternate_setting_changed(UsbDirection::Out, 0);
    }
}

impl<'a, B: UsbBus, AU: AudioHandler<'a, B> + ClockSource> UsbAudioClass<'a, B, AU> {
    /// Expose the owned AudioHandler implementation
    pub fn handler(&mut self) -> &mut AU {
        self.audio_impl
    }
    fn emit_feedback(&mut self) {
        if let Some(fb_ep) = self.feedback.as_ref() {
            if let Some(fb) = self.audio_impl.feedback(self.nominal_fb) {
                debug!("  emitting feedback IN {:08x}", fb.to_u32_12_13());
                let r = match self.speed {
                    UsbSpeed::Low | UsbSpeed::Full => fb_ep.write(&fb.to_bytes_10_14()),
                    UsbSpeed::High | UsbSpeed::Super => fb_ep.write(&fb.to_bytes_12_13()),
                };
                if let Err(_e) = r {
                    warn!("  feedback IN failed {:?}", _e);
                }
            } else {
                debug!("  feedback callback returned None");
            }
        }
    }
    fn standard_request_out(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();
        match (req.recipient, req.request) {
            (Recipient::Interface, Request::SET_INTERFACE) => self.set_alt_interface(xfer),
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }

    fn standard_request_in(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();
        match (req.recipient, req.request) {
            (Recipient::Interface, Request::GET_INTERFACE) => self.get_alt_interface(xfer),
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }

    fn class_request_out(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();
        match (req.recipient, req.request.into()) {
            (Recipient::Interface, ClassSpecificRequest::Cur) => self.set_interface_cur(xfer),
            (Recipient::Interface, ClassSpecificRequest::Range) => {}
            (Recipient::Endpoint, ClassSpecificRequest::Cur) => self.set_endpoint_cur(xfer),
            (Recipient::Endpoint, ClassSpecificRequest::Range) => {}
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }

    fn class_request_in(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();

        match (req.recipient, req.request.into()) {
            (Recipient::Interface, ClassSpecificRequest::Cur) => self.get_interface_cur(xfer),
            (Recipient::Interface, ClassSpecificRequest::Range) => self.get_interface_range(xfer),
            (Recipient::Endpoint, ClassSpecificRequest::Cur) => self.get_endpoint_cur(xfer),
            (Recipient::Endpoint, ClassSpecificRequest::Range) => self.get_endpoint_range(xfer),
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }

    fn set_alt_interface(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();
        let iface = req.index as u8;
        let alt_setting = req.value as u8;

        debug!("  SET_ALT_INTERFACE {} {}", iface, alt_setting);

        // This implementation defines only alt 0 (zero bandwidth) and alt 1
        // (the configured PCM stream).
        if alt_setting > 1 {
            debug!("   unsupported alternate setting {}", alt_setting);
            return;
        }

        // The AudioControl interface has only alternate setting zero.
        if iface == self.control_iface.into() {
            if alt_setting == 0 {
                xfer.accept().ok();
            }
            return;
        }

        if let Some(input) = &mut self.input
            && iface == self.in_iface
        {
            let old_alt = input.alt_setting;
            if old_alt != alt_setting {
                self.audio_impl
                    .alternate_setting_changed(UsbDirection::In, alt_setting);
                input.alt_setting = alt_setting;
                if alt_setting == 1 {
                    // Start the capture IN cycle.
                    self.audio_impl.audio_data_tx(&input.endpoint);
                }
            }
            // SET_INTERFACE(0) is valid even when alt zero is already active.
            xfer.accept().ok();
        } else if self.output.is_some() && iface == self.out_iface {
            let old_alt = self.output.as_ref().unwrap().alt_setting;
            if old_alt != alt_setting {
                self.audio_impl
                    .alternate_setting_changed(UsbDirection::Out, alt_setting);
                self.output.as_mut().unwrap().alt_setting = alt_setting;
                if alt_setting == 1 {
                    // Start the playback feedback IN cycle.
                    self.emit_feedback();
                }
            }
            xfer.accept().ok();
        } else {
            debug!(
                "   not handled (control: {}, in: {}, out: {}, got: {}).",
                u8::from(self.control_iface), self.in_iface, self.out_iface, iface
            );
        }
    }
    fn get_alt_interface(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();
        let iface = req.index as u8;
        debug!("  GET_ALT_INTERFACE {}", iface);
        if iface == self.control_iface.into() {
            xfer.accept_with(&[0]).ok();
            return;
        }
        if let Some(input) = self.input.as_ref()
            && iface == self.in_iface
        {
            xfer.accept_with(&[input.alt_setting]).ok();
            return;
        } else if let Some(output) = self.output.as_ref()
            && iface == self.out_iface
        {
            xfer.accept_with(&[output.alt_setting]).ok();
            return;
        }
        debug!("   Unimplemented.");
    }
    fn get_interface_cur(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();
        let entity = (req.index >> 8) as u8;
        let interface = (req.index & 0xff) as u8;
        let control = (req.value >> 8) as u8;
        let channel = (req.value & 0xff) as u8;

        debug!(
            "  GET_INTERFACE_CUR entity: {} iface: {} control: {} channel: {}",
            entity, interface, control, channel
        );
        if interface == self.control_iface.into() {
            return self.get_control_interface_cur(xfer, entity, channel, control);
        }
        debug!("   Unimpleneted.");
    }
    fn set_interface_cur(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();
        let entity = (req.index >> 8) as u8;
        let interface = (req.index & 0xff) as u8;
        let control = (req.value >> 8) as u8;
        let channel = (req.value & 0xff) as u8;

        debug!(
            "  SET_INTERFACE_CUR entity: {} iface: {} control: {} channel: {}",
            entity, interface, control, channel
        );

        if interface == self.control_iface.into() {
            return self.set_control_interface_cur(xfer, entity, channel, control);
        } else if interface == self.in_iface {
            return self.set_streaming_interface_cur(
                xfer,
                UsbDirection::In,
                entity,
                channel,
                control,
            );
        } else if interface == self.out_iface {
            return self.set_streaming_interface_cur(
                xfer,
                UsbDirection::Out,
                entity,
                channel,
                control,
            );
        }
        debug!("   Unimplemented.");
    }

    fn get_control_interface_cur(
        &mut self,
        xfer: ControlIn<B>,
        entity: u8,
        channel: u8,
        control: u8,
    ) {
        if entity == 1 {
            return self.get_clock_cur(xfer, channel, control);
        }
        debug!("   Unimplemented.");
    }

    fn set_control_interface_cur(
        &mut self,
        xfer: ControlOut<B>,
        entity: u8,
        channel: u8,
        control: u8,
    ) {
        if entity == 1 {
            return self.set_clock_cur(xfer, channel, control);
        }
        debug!("   Unimplemented.");
    }

    fn set_streaming_interface_cur(
        &mut self,
        _xfer: ControlOut<B>,
        _direction: UsbDirection,
        _entity: u8,
        _channel: u8,
        _control: u8,
    ) {
        debug!("   Unimplemented.");
    }

    fn get_endpoint_cur(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();
        let _entity = (req.index >> 8) as u8;
        let _interface = (req.index & 0xff) as u8;
        let _control = (req.value >> 8) as u8;
        let _channel = (req.value & 0xff) as u8;

        debug!("   Unimplemented.");
    }
    fn get_endpoint_range(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();
        let _entity = (req.index >> 8) as u8;
        let _interface = (req.index & 0xff) as u8;
        let _control = (req.value >> 8) as u8;
        let _channel = (req.value & 0xff) as u8;

        debug!("   Unimplemented.");
    }

    fn set_endpoint_cur(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();
        let _entity = (req.index >> 8) as u8;
        let _interface = (req.index & 0xff) as u8;
        let _control = (req.value >> 8) as u8;
        let _channel = (req.value & 0xff) as u8;

        debug!("   Unimplemented.");
    }

    fn get_interface_range(&mut self, xfer: ControlIn<B>) {
        let req = xfer.request();

        let entity = (req.index >> 8) as u8;
        let interface = (req.index & 0xff) as u8;
        let control = (req.value >> 8) as u8;
        let channel = (req.value & 0xff) as u8;

        debug!(
            "  GET_INTERFACE_RANGE entity: {} iface: {} control: {} channel: {}",
            entity, interface, control, channel
        );
        if interface == self.control_iface.into() {
            return self.get_control_interface_range(xfer, entity, channel, control);
        }
        debug!("   Unimplemented.");
    }

    fn get_control_interface_range(
        &mut self,
        xfer: ControlIn<B>,
        entity: u8,
        channel: u8,
        control: u8,
    ) {
        match entity {
            1 => self.get_clock_range(xfer, channel, control), // clock source
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }
    fn get_clock_cur(&mut self, xfer: ControlIn<B>, channel: u8, control: u8) {
        match control.into() {
            ClockSourceControlSelector::SamFreqControl => {
                debug!("   SamplingFreqControl");
                if channel != 0 {
                    error!(
                        "   Invalid channel {} for SamplingFreqControl GET CUR. Ignoring.",
                        channel
                    );
                }
                xfer.accept(|mut buf| {
                    let rate = self.audio_impl.sample_rate();

                    debug!("    {}", rate);
                    buf.write_u32::<LittleEndian>(rate)
                        .map_err(|_e| UsbError::BufferOverflow)?;
                    Ok(4)
                })
                .ok();
            }
            ClockSourceControlSelector::ClockValidControl => {
                debug!("   ClockValidControl");
                if channel != 0 {
                    error!(
                        "   Invalid channel {} for ClockValidControl GET CUR. Ignoring.",
                        channel
                    );
                }
                xfer.accept(|mut buf| match self.audio_impl.clock_validity() {
                    Ok(valid) => {
                        debug!("    {}", valid);
                        buf.write_u8(valid as u8)
                            .map_err(|_e| UsbError::BufferOverflow)
                            .ok();
                        Ok(1)
                    }
                    Err(_e) => Err(UsbError::InvalidState),
                })
                .ok();
            }
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }
    fn set_clock_cur(&mut self, xfer: ControlOut<B>, channel: u8, control: u8) {
        match control.into() {
            ClockSourceControlSelector::SamFreqControl => {
                debug!("   SamplingFreqControl");
                if channel != 0 {
                    error!(
                        "   Invalid channel {} for SamplingFreqControl GET CUR. Ignoring.",
                        channel
                    );
                }
                match xfer.data().read_u32::<LittleEndian>() {
                    Ok(rate) => {
                        debug!("   SET SamplingFreqControl CUR {}", rate);
                        self.audio_impl.set_sample_rate(rate).ok();
                        self.nominal_fb = UsbIsochronousFeedback::new_float(
                            rate.to_f32().unwrap() / self.audio_rate.to_f32().unwrap(),
                        );
                        xfer.accept().ok();
                    }
                    Err(_e) => {
                        error!("   SET SamplingFreqControl CUR ERROR BAD DATA");
                    }
                }
            }
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }
    fn get_clock_range(&mut self, xfer: ControlIn<B>, channel: u8, control: u8) {
        match control.into() {
            ClockSourceControlSelector::SamFreqControl => {
                debug!("   SamplingFreqControl");
                if channel != 0 {
                    error!(
                        "   Invalid channel {} for SamplingFreqControl GET RANGE. Ignoring.",
                        channel
                    );
                }
                let req_len = xfer.request().length;
                xfer.accept(|mut buf| match self.audio_impl.sample_rates() {
                    Ok(rates) => {
                        debug!("    wLength = {} buf_len = {}", req_len, buf.len());
                        buf.write_u16::<LittleEndian>(rates.len() as u16)
                            .map_err(|_e| UsbError::BufferOverflow)?;
                        let mut written = 2usize;
                        if req_len > 2 {
                            for rate in rates {
                                written += rate
                                    .write(&mut buf)
                                    .map_err(|_e| UsbError::BufferOverflow)?
                            }
                        }
                        Ok(written)
                    }
                    Err(_) => Err(UsbError::InvalidState),
                })
                .ok();
            }
            _ => {
                debug!("   Unimplemented.");
            }
        }
    }
    pub fn read(&mut self, buf: &mut [u8]) -> usb_device::Result<usize> {
        match self.output.as_mut().unwrap().endpoint.read(buf) {
            Ok(len) => {
                debug!("NO CB read {} bytes", len);
                Ok(len)
            }
            Err(UsbError::WouldBlock) => Err(UsbError::WouldBlock),
            Err(e) => {
                error!("read error");
                Err(e)
            }
        }
    }
    pub fn max_read_packet_size(&self) -> core::result::Result<u16, UsbAudioClassError> {
        match self.output.as_ref() {
            Some(output) => Ok(output.endpoint.max_packet_size()),
            None => Err(UsbAudioClassError::Other),
        }
    }
}

/// Set up a `UsbDeviceBuilder` with the device IDs for Usb Audio Class 2.0
pub fn builder<'a, B: UsbBus>(
    alloc: &'a UsbBusAllocator<B>,
    vid_pid: UsbVidPid,
) -> UsbDeviceBuilder<'a, B> {
    UsbDeviceBuilder::new(alloc, vid_pid)
        .composite_with_iads() // required by UAC2 4.6
        .device_class(0xef) // required by UAC2 4.2
        .device_sub_class(0x02) // required by UAC2 4.2
        .device_protocol(0x01) // required by UAC2 4.2
}
