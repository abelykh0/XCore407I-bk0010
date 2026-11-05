use crate::constants::*;
use crate::debug;
use crate::error;
use crate::{constants::ClassSpecificACInterfaceDescriptorSubtype, cursor::Cursor};

use byteorder_embedded_io::{LittleEndian, WriteBytesExt};
use modular_bitfield::prelude::*;
use usb_device::UsbError;
use usb_device::{bus::StringIndex, descriptor::DescriptorWriter};

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ClockType {
    External = 0,
    InternalFixed = 1,
    InternalVariable = 2,
    InternalProgrammable = 3,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Specifier)]
#[bits = 2]
pub enum AccessControl {
    NotPresent = 0,
    ReadOnly = 1,
    Programmable = 3,
}

#[bitfield(bits = 32)]
#[repr(u32)]
#[derive(Copy, Clone, Debug)]
pub struct ChannelConfig {
    pub front_left: bool,
    pub front_right: bool,
    pub front_center: bool,
    pub low_frequency_effects: bool,
    pub back_left: bool,
    pub back_right: bool,
    pub front_left_of_center: bool,
    pub front_right_of_center: bool,
    pub back_center: bool,
    pub side_left: bool,
    pub side_right: bool,
    pub top_center: bool,
    pub top_front_left: bool,
    pub top_front_center: bool,
    pub top_front_right: bool,
    pub top_back_left: bool,
    pub top_back_center: bool,
    pub top_back_right: bool,
    pub top_front_left_of_center: bool,
    pub top_front_right_of_center: bool,
    pub left_row_low_frequency_effects: bool,
    pub right_row_low_frequency_effects: bool,
    pub top_side_left: bool,
    pub top_side_right: bool,
    pub bottom_center: bool,
    pub back_left_of_center: bool,
    pub back_right_of_center: bool,
    #[skip]
    __: B4,
    raw_data: bool,
}

impl ChannelConfig {
    /// Creates a new ChannelConfig with a default channel configuration based on the number of channels.
    ///
    /// Supports:
    /// - 1 channel: Front Left
    /// - 2 channels: Front Left, Front Right
    /// - 4 channels: Front Left, Front Right, Back Left, Back Right
    /// - 6 channels: Front Left, Front Right, Back Left, Back Right, Front Center, LFE
    /// - 8 channels: Front Left, Front Right, Back Left, Back Right, Front Center, Side Left, Side Right, LFE
    pub fn default_chans(n: u8) -> Self {
        match n {
            1 => ChannelConfig::new().with_front_left(true),
            2 => ChannelConfig::new()
                .with_front_left(true)
                .with_front_right(true),
            4 => ChannelConfig::new()
                .with_front_left(true)
                .with_front_right(true)
                .with_back_left(true)
                .with_back_right(true),
            6 => ChannelConfig::new()
                .with_front_left(true)
                .with_front_center(true)
                .with_front_right(true)
                .with_back_left(true)
                .with_back_right(true)
                .with_low_frequency_effects(true),
            8 => ChannelConfig::new()
                .with_front_left(true)
                .with_front_center(true)
                .with_front_right(true)
                .with_back_left(true)
                .with_back_right(true)
                .with_front_center(true)
                .with_side_left(true)
                .with_side_right(true)
                .with_low_frequency_effects(true),
            _ => panic!("Unsupported number of channels"),
        }
    }
}

#[derive(Copy, Clone)]
#[bitfield(bits = 32)]
pub struct FeatureControls {
    mute: AccessControl,
    volume: AccessControl,
    bass: AccessControl,
    mid: AccessControl,
    treble: AccessControl,
    graphic_eq: AccessControl,
    agc: AccessControl,
    delay: AccessControl,
    bass_boost: AccessControl,
    loudness: AccessControl,
    input_gain: AccessControl,
    input_gain_pad: AccessControl,
    phase_inverter: AccessControl,
    underflow: AccessControl,
    overflow: AccessControl,
    hpf: AccessControl,
}

pub trait Descriptor {
    /// The maximum possible size of the descriptor, including the length and descriptor type bytes. For creation of Sized buffers.
    const MAX_SIZE: usize;
    /// The bDescriptorType of the descriptor
    fn descriptor_type(&self) -> u8;
    /// The actual size of the descriptor, including the length and descriptor type bytes.
    fn size(&self) -> u8;
    /// The payload of the descriptor, not including the bLength and bDescriptorType bytes
    fn write_payload<T: embedded_io::Write>(&self, writer: &mut T) -> Result<(), T::Error>;
    /// Write the descriptor to the provided buffer. Includes the length and descriptor type bytes. The default implementation defers to write_payload and self.size().
    fn write<T: embedded_io::Write>(&self, writer: &mut T) -> Result<(), T::Error> {
        writer.write_u8(self.size())?;
        writer.write_u8(self.descriptor_type())?;
        self.write_payload(writer)?;
        Ok(())
    }
    /// Write the descriptor using the provided usb_device DescriptorWriter. The default implementation defers to write_payload.
    fn write_descriptor<'w, 'd>(
        &self,
        writer: &'w mut DescriptorWriter<'d>,
    ) -> Result<(), usb_device::UsbError> {
        debug!("writer.write {}", core::any::type_name_of_val(self));
        writer.write_with(self.descriptor_type(), |buf| {
            let mut cur = Cursor::new(buf);
            if let Err(e) = self.write_payload(&mut cur) {
                error!("Write payload err {}", defmt::Display2Format(&e));
                Err(e.into())
            } else {
                Ok(cur.position())
            }
        })
    }
}

#[derive(Clone)]
pub struct ClockSource {
    pub id: u8,
    pub clock_type: ClockType,
    pub sof_sync: bool,
    pub frequency_access: AccessControl,
    pub validity_access: AccessControl,
    pub assoc_terminal: u8,
    pub string: Option<StringIndex>,
}

impl ClockSource {
    fn bm_attributes(&self) -> u8 {
        self.clock_type as u8 | if self.sof_sync { 4 } else { 0 }
    }
    fn bm_controls(&self) -> u8 {
        ((self.validity_access as u8) << 2) | self.frequency_access as u8
    }
}

impl Descriptor for ClockSource {
    const MAX_SIZE: usize = 8;
    fn descriptor_type(&self) -> u8 {
        ClassSpecificDescriptorType::Interface as u8
    }

    fn size(&self) -> u8 {
        Self::MAX_SIZE as u8
    }

    fn write_payload<T: embedded_io::Write>(&self, writer: &mut T) -> Result<(), T::Error> {
        writer.write_u8(ClassSpecificACInterfaceDescriptorSubtype::ClockSource as u8)?; // bDescriptorSubtype
        writer.write_u8(self.id)?; // bClockId
        writer.write_u8(self.bm_attributes())?; // bmAttributes
        writer.write_u8(self.bm_controls())?; // bmControls
        writer.write_u8(self.assoc_terminal)?; // bAssocTerminal
        writer.write_u8(self.string.map_or(0, u8::from))?; // iClockSource
        Ok(())
    }
}

#[derive(Clone)]
pub struct InputTerminal {
    pub id: u8,
    pub terminal_type: TerminalType,
    pub assoc_terminal: u8,
    pub clock_source: u8,
    pub num_channels: u8,
    pub channel_config: ChannelConfig,
    pub channel_names: u8,
    pub copy_protect_control: AccessControl,
    pub connector_control: AccessControl,
    pub overload_control: AccessControl,
    pub cluster_control: AccessControl,
    pub underflow_control: AccessControl,
    pub overflow_control: AccessControl,
    pub phantom_power_control: AccessControl,
    pub string: Option<StringIndex>,
}

impl Descriptor for InputTerminal {
    const MAX_SIZE: usize = 17;
    fn descriptor_type(&self) -> u8 {
        ClassSpecificDescriptorType::Interface as u8
    }
    fn size(&self) -> u8 {
        Self::MAX_SIZE as u8
    }
    fn write_payload<T: embedded_io::Write>(&self, writer: &mut T) -> Result<(), T::Error> {
        writer.write_u8(ClassSpecificACInterfaceDescriptorSubtype::InputTerminal as u8)?; // bDescriptorSubtype
        writer.write_u8(self.id)?; // bTerminalID
        writer.write_u16::<LittleEndian>(self.terminal_type as u16)?; // wTerminalType
        writer.write_u8(self.assoc_terminal)?; // bAssocTerminal
        writer.write_u8(self.clock_source)?; // bCSourceID
        writer.write_u8(self.num_channels)?; // bNrChannels
        writer.write(&self.channel_config.into_bytes())?;
        writer.write_u8(self.channel_names)?;
        writer.write_u8(
            self.copy_protect_control as u8
                | ((self.connector_control as u8) << 2)
                | ((self.overload_control as u8) << 4)
                | ((self.cluster_control as u8) << 6),
        )?;
        writer.write_u8(
            self.underflow_control as u8
                | ((self.overflow_control as u8) << 2)
                | ((self.phantom_power_control as u8) << 4),
        )?;
        writer.write_u8(self.string.map_or(0, u8::from))?;
        Ok(())
    }
}

#[derive(Clone)]
pub struct OutputTerminal {
    pub id: u8,
    pub terminal_type: TerminalType,
    pub assoc_terminal: u8,
    pub source_id: u8,
    pub clock_source: u8,
    pub copy_protect_control: AccessControl,
    pub connector_control: AccessControl,
    pub overload_control: AccessControl,
    pub underflow_control: AccessControl,
    pub overflow_control: AccessControl,
    pub string: Option<StringIndex>,
}

impl Descriptor for OutputTerminal {
    const MAX_SIZE: usize = 12;
    fn descriptor_type(&self) -> u8 {
        ClassSpecificDescriptorType::Interface as u8
    }
    fn size(&self) -> u8 {
        Self::MAX_SIZE as u8
    }
    fn write_payload<T: embedded_io::Write>(&self, writer: &mut T) -> Result<(), T::Error> {
        writer.write_u8(ClassSpecificACInterfaceDescriptorSubtype::OutputTerminal as u8)?; // bDescriptorSubtype
        writer.write_u8(self.id)?; // bTerminalID
        writer.write_u16::<LittleEndian>(self.terminal_type as u16)?; // wTerminalType
        writer.write_u8(self.assoc_terminal)?; // bAssocTerminal
        writer.write_u8(self.source_id)?; // bSourceID
        writer.write_u8(self.clock_source)?; // bCSourceID
        writer.write_u8(
            self.copy_protect_control as u8
                | ((self.connector_control as u8) << 2)
                | ((self.overload_control as u8) << 4)
                | ((self.underflow_control as u8) << 6),
        )?;
        writer.write_u8(self.overflow_control as u8)?;
        writer.write_u8(self.string.map_or(0, u8::from))?; // iTerminal
        Ok(())
    }
}

#[derive(Clone)]
pub enum Terminal {
    Input(InputTerminal),
    Output(OutputTerminal),
}

impl Descriptor for Terminal {
    const MAX_SIZE: usize = if InputTerminal::MAX_SIZE > OutputTerminal::MAX_SIZE {
        InputTerminal::MAX_SIZE
    } else {
        OutputTerminal::MAX_SIZE
    };
    fn descriptor_type(&self) -> u8 {
        ClassSpecificDescriptorType::Interface as u8
    }
    fn size(&self) -> u8 {
        match self {
            Self::Input(t) => t.size(),
            Self::Output(t) => t.size(),
        }
    }
    fn write_payload<T: embedded_io::Write>(&self, writer: &mut T) -> Result<(), T::Error> {
        match self {
            Self::Input(t) => t.write_payload(writer),
            Self::Output(t) => t.write_payload(writer),
        }
    }
}

#[derive(Clone, Debug)]
pub struct ClockSelector<const MAX_SOURCES: usize> {
    pub id: u8,
    pub n_sources: u8,
    pub sources: [u8; MAX_SOURCES], // baCSourceID[]
    pub selector_access: AccessControl,
    pub string: u8, // iClockSelector
}

impl<const N: usize> Descriptor for ClockSelector<N> {
    const MAX_SIZE: usize = 7 + N;
    fn descriptor_type(&self) -> u8 {
        ClassSpecificDescriptorType::Interface as u8
    }
    fn size(&self) -> u8 {
        7 + self.n_sources
    }
    fn write_payload<T: embedded_io::Write>(&self, writer: &mut T) -> Result<(), T::Error> {
        writer.write_u8(ClassSpecificACInterfaceDescriptorSubtype::ClockSelector as u8)?; // bDescriptorSubtype
        writer.write_u8(self.id)?; // bClockID
        writer.write_u8(self.n_sources)?; // bNrInPins
        writer.write(&self.sources[0..(self.n_sources as usize)])?;
        writer.write_u8(self.selector_access as u8)?; // bmControls (CX_CLOCK_SELECTOR)
        writer.write_u8(self.string)?; // iClockSelector (last byte)
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct ClockMultiplier {
    pub id: u8,
    pub source_id: u8,
    pub numerator_access: AccessControl,
    pub denominator_access: AccessControl,
    pub string: u8, // iClockMultiplier
}

impl ClockMultiplier {
    fn bm_controls(&self) -> u8 {
        (self.numerator_access as u8) | ((self.denominator_access as u8) << 2)
    }
    fn write_payload<T: embedded_io::Write>(&self, writer: &mut T) -> Result<usize, T::Error> {
        writer.write_u8(ClassSpecificACInterfaceDescriptorSubtype::ClockMultiplier as u8)?; // bDescriptorSubtype
        writer.write_u8(self.id)?; // bClockID
        writer.write_u8(self.source_id)?; // bCSourceID
        writer.write_u8(self.bm_controls())?; // bmControls
        writer.write_u8(self.string)?; // iClockMultiplier
        Ok(self.size() as usize)
    }
}

impl Descriptor for ClockMultiplier {
    const MAX_SIZE: usize = 7;
    fn descriptor_type(&self) -> u8 {
        ClassSpecificDescriptorType::Interface as u8
    }
    fn size(&self) -> u8 {
        Self::MAX_SIZE as u8
    }
    fn write_payload<T: embedded_io::Write>(&self, writer: &mut T) -> Result<(), T::Error> {
        writer.write_u8(ClassSpecificACInterfaceDescriptorSubtype::ClockMultiplier as u8)?; // bDescriptorSubtype
        writer.write_u8(self.id)?; // bClockID
        writer.write_u8(self.source_id)?; // bCSourceID
        writer.write_u8(self.bm_controls())?; // bmControls
        writer.write_u8(self.string)?; // iClockMultiplier
        Ok(())
    }
}
// Feature Unit's size depends on the number of channels in its source node, which we don't have a way
// to look up with the current API. Ignore and leave unimplemented for now.
//
// pub struct FeatureUnit<const MAX_CONTROLS: usize> {
//     pub id: u8,
//     pub source_id: u8,
//     pub controls: [FeatureControls; MAX_CONTROLS], // bmaControls[] (N == channels + 1)
//     pub n_controls: u8,
//     pub string: u8, // iFeature
// }

// impl<const N: usize> Descriptor for FeatureUnit<N> {
//     const MAX_SIZE: usize = 6 + 4 * N;
//     fn size(&self) -> u8 {
//         6 + 4 * (self.n_controls as u8)
//     }

//     fn write(&self, buf: &mut [u8]) -> Result<usize, embedded_io::ErrorKind> {
//         let mut cur = Cursor::new(buf);
//         cur.write_u8(self.size())?; // bLength
//         cur.write_u8(ClassSpecificDescriptorType::Interface as u8)?; // bDescriptorType
//         cur.write_u8(ClassSpecificACInterfaceDescriptorSubtype::FeatureUnit as u8)?; // bDescriptorSubtype
//         cur.write_u8(self.id)?; // bUnitID
//         cur.write_u8(self.source_id)?; // bSourceID

//         // bmaControls[i] as LE u32
//         for v in self.controls.iter() {
//             cur.write(&v.bytes)?;
//         }

//         cur.write_u8(self.string)?; // iFeature
//         assert_eq!(cur.position(), self.size() as usize);
//         Ok(cur.position())
//     }
// }

// This implementation is not correct due to how pins/channels/mixer channels are computed
// it needs to know more about the rest of the devices
// pub struct MixerUnit<const MAX_SOURCES: usize, const MAX_CONTROLS: usize> {
//     pub id: u8,
//     pub sources: [u8; MAX_SOURCES], // baSourceID[]
//     pub n_sources: u8,
//     pub num_channels: u8,                   // bNrChannels
//     pub channel_config: ChannelConfig,      // bmChannelConfig (u32)
//     pub channel_names: u8,                  // iChannelNames
//     pub mixer_controls: [u8; MAX_CONTROLS], // bmMixerControls[]
//     pub n_controls: u8,
//     pub cluster_control: AccessControl,
//     pub underflow_control: AccessControl,
//     pub overflow_control: AccessControl,
//     pub latency_control: AccessControl,
//     pub string: u8, // iMixer
// }

// impl<const MS: usize, const MC: usize> MixerUnit<MS, MC> {
//     fn bm_controls(&self) -> u8 {
//         (self.cluster_control as u8)
//             | ((self.underflow_control as u8) << 2)
//             | ((self.overflow_control as u8) << 4)
//             | ((self.latency_control as u8) << 6)
//     }
// }

// impl<const MS: usize, const MC: usize> Descriptor for MixerUnit<MS, MC> {
//     const MAX_SIZE: usize = 13 + MS + MC;
//     fn size(&self) -> u8 {
//         13 + self.n_sources as u8 + self.n_controls as u8
//     }

//     fn write(&self, buf: &mut [u8]) -> Result<usize, embedded_io::ErrorKind> {
//         let mut cur = Cursor::new(buf);
//         cur.write_u8(self.size())?; // bLength
//         cur.write_u8(ClassSpecificDescriptorType::Interface as u8)?; // bDescriptorType
//         cur.write_u8(ClassSpecificACInterfaceDescriptorSubtype::MixerUnit as u8)?; // bDescriptorSubtype
//         cur.write_u8(self.id)?; // bUnitID
//         cur.write_u8(self.n_sources as u8)?; // bNrInPins
//         cur.write(&self.sources[0..(self.n_sources as usize)])?; // baSourceID[]

//         cur.write_u8(self.num_channels)?; // bNrChannels
//         cur.write(&self.channel_config.bytes)?; // bmChannelConfig (already u32 LE)

//         cur.write_u8(self.channel_names)?; // iChannelNames

//         cur.write(&self.mixer_controls[0..(self.n_controls as usize)])?; // bmMixerControls[]

//         cur.write_u8(self.bm_controls())?; // bmControls
//         cur.write_u8(self.string)?; // iMixer
//         assert_eq!(cur.position(), self.size() as usize);
//         Ok(cur.position())
//     }
// }

#[derive(Clone, Debug)]
pub struct SelectorUnit<const MAX_SOURCES: usize> {
    pub id: u8,
    pub sources: [u8; MAX_SOURCES], // baSourceID[]
    pub n_sources: u8,
    pub selector_control: AccessControl,
    pub string: u8, // iSelector
}

impl<const N: usize> SelectorUnit<N> {
    fn bm_controls(&self) -> u8 {
        self.selector_control as u8
    }
}

impl<const N: usize> Descriptor for SelectorUnit<N> {
    const MAX_SIZE: usize = 7 + N;
    fn descriptor_type(&self) -> u8 {
        ClassSpecificDescriptorType::Interface as u8
    }
    fn size(&self) -> u8 {
        7 + self.n_sources
    }
    fn write_payload<T: embedded_io::Write>(&self, writer: &mut T) -> Result<(), T::Error> {
        writer.write_u8(ClassSpecificACInterfaceDescriptorSubtype::SelectorUnit as u8)?; // bDescriptorSubtype
        writer.write_u8(self.id)?; // bUnitID
        writer.write_u8(self.n_sources)?; // bNrInPins
        writer.write(&self.sources[0..(self.n_sources as usize)])?;
        writer.write_u8(self.bm_controls())?; // bmControls
        writer.write_u8(self.string)?; // iSelector
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct ProcessingUnit<const MAX_SOURCES: usize> {
    pub id: u8,
    pub process_type: u16,          // wProcessType
    pub sources: [u8; MAX_SOURCES], // baSourceID[]
    pub num_channels: u8,           // bNrChannels
    pub n_sources: u8,
    pub channel_config: ChannelConfig, // bmChannelConfig
    pub channel_names: u8,             // iChannelNames
    pub enable_control: AccessControl,
    pub mode_select_control: AccessControl,
    pub cluster_control: AccessControl,
    pub underflow_control: AccessControl,
    pub overflow_control: AccessControl,
    pub latency_control: AccessControl,
    pub string: u8, // iProcessing
}

impl<const N: usize> ProcessingUnit<N> {
    fn bm_controls(&self) -> u16 {
        (self.enable_control as u16)
            | ((self.mode_select_control as u16) << 2)
            | ((self.cluster_control as u16) << 4)
            | ((self.underflow_control as u16) << 6)
            | ((self.overflow_control as u16) << 8)
            | ((self.latency_control as u16) << 10)
    }
}

impl<const N: usize> Descriptor for ProcessingUnit<N> {
    const MAX_SIZE: usize = 17 + N;
    fn descriptor_type(&self) -> u8 {
        ClassSpecificDescriptorType::Interface as u8
    }
    fn size(&self) -> u8 {
        17 + self.n_sources
    }
    fn write_payload<T: embedded_io::Write>(&self, writer: &mut T) -> Result<(), T::Error> {
        writer.write_u8(ClassSpecificACInterfaceDescriptorSubtype::ProcessingUnit as u8)?; // bDescriptorSubtype
        writer.write_u8(self.id)?; // bUnitID
        writer.write_u16::<LittleEndian>(self.process_type)?; // wProcessType
        writer.write_u8(self.n_sources)?; // bNrInPins
        writer.write(&self.sources[0..(self.n_sources as usize)])?; // baSourceID[]
        writer.write_u8(self.num_channels)?; // bNrChannels
        writer.write(&self.channel_config.into_bytes())?; // bmChannelConfig (already u32 LE)
        writer.write_u8(self.channel_names)?; // iChannelNames
        writer.write_u16::<LittleEndian>(self.bm_controls())?; // bmControls (PU is 2 bytes in UAC2)
        writer.write_u8(self.string)?; // iProcessing
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct ExtensionUnit<const MAX_SOURCES: usize> {
    pub id: u8,
    pub extension_code: u16,        // wExtensionCode
    pub sources: [u8; MAX_SOURCES], // baSourceID[]
    pub n_sources: u8,
    pub num_channels: u8,              // bNrChannels
    pub channel_config: ChannelConfig, // bmChannelConfig
    pub channel_names: u8,             // iChannelNames
    pub enable_control: AccessControl,
    pub cluster_control: AccessControl,
    pub underflow_control: AccessControl,
    pub overflow_control: AccessControl,
    pub string: u8, // iExtension
}

impl<const N: usize> ExtensionUnit<N> {
    fn bm_controls(&self) -> u8 {
        (self.enable_control as u8)
            | ((self.cluster_control as u8) << 2)
            | ((self.underflow_control as u8) << 4)
            | ((self.overflow_control as u8) << 6)
    }
}

impl<const N: usize> Descriptor for ExtensionUnit<N> {
    const MAX_SIZE: usize = 16 + N;
    fn descriptor_type(&self) -> u8 {
        ClassSpecificDescriptorType::Interface as u8
    }
    fn size(&self) -> u8 {
        16 + self.n_sources
    }
    fn write_payload<T: embedded_io::Write>(&self, writer: &mut T) -> Result<(), T::Error> {
        writer.write_u8(ClassSpecificACInterfaceDescriptorSubtype::ExtensionUnit as u8)?; // bDescriptorSubtype
        writer.write_u8(self.id)?; // bUnitID
        writer.write_u16::<LittleEndian>(self.extension_code)?; // wExtensionCode
        writer.write_u8(self.n_sources)?; // bNrInPins
        writer.write(&self.sources[0..(self.n_sources as usize)])?; // baSourceID[]
        writer.write_u8(self.num_channels)?; // bNrChannels
        writer.write(&self.channel_config.into_bytes())?; // bmChannelConfig
        writer.write_u8(self.channel_names)?; // iChannelNames
        writer.write_u8(self.bm_controls())?; // bmControls (XU is 1 byte in UAC2)
        writer.write_u8(self.string)?; // iExtension
        Ok(())
    }
}
// Effect unit is also variable based on its source node, leave unimplemented for now.
//
// pub struct EffectUnit<const MAX_CONTROLS: usize> {
//     pub id: u8,
//     pub effect_type: u16,              // wEffectType
//     pub source_id: u8,                 // bSourceID
//     pub controls: [u32; MAX_CONTROLS], // bmaControls[] (N == channels + 1)
//     pub n_controls: u8,
//     pub string: u8, // iEffect
// }

// impl<const N: usize> Descriptor for EffectUnit<N> {
//     const MAX_SIZE: usize = 16 + 4 * N;
//     fn size(&self) -> u8 {
//         16 + 4 * self.n_controls
//     }

//     fn write(&self, buf: &mut [u8]) -> Result<usize, embedded_io::ErrorKind> {
//         let mut cur = Cursor::new(buf);
//         cur.write_u8(self.size())?; // bLength
//         cur.write_u8(ClassSpecificDescriptorType::Interface as u8)?; // bDescriptorType
//         cur.write_u8(ClassSpecificACInterfaceDescriptorSubtype::EffectUnit as u8)?; // bDescriptorSubtype
//         cur.write_u8(self.id)?; // bUnitID
//         cur.write_u16::<LittleEndian>(self.effect_type)?; // wEffectType
//         cur.write_u8(self.source_id)?; // bSourceID
//         for v in self.controls.iter() {
//             cur.write_u32::<LittleEndian>(*v)?;
//         }
//         cur.write_u8(self.string)?; // iEffect
//         assert_eq!(cur.position(), self.size() as usize);
//         Ok(cur.position())
//     }
// }

#[derive(Clone)]
/// Enum covering basic sized audio class descriptors for building the Class-Specific
/// AC Interface descriptor. Dynamically sized descriptors are not supported yet.
pub enum AudioClassDescriptor {
    ClockSource(ClockSource),
    ClockMultiplier(ClockMultiplier),
    InputTerminal(InputTerminal),
    OutputTerminal(OutputTerminal),
}

impl AudioClassDescriptor {
    pub fn size(&self) -> u8 {
        match self {
            AudioClassDescriptor::ClockSource(cs) => cs.size(),
            AudioClassDescriptor::ClockMultiplier(cm) => cm.size(),
            AudioClassDescriptor::InputTerminal(it) => it.size(),
            AudioClassDescriptor::OutputTerminal(ot) => ot.size(),
        }
    }
    pub fn write<T: embedded_io::Write>(&self, writer: &mut T) -> Result<(), T::Error> {
        match self {
            AudioClassDescriptor::ClockSource(cs) => cs.write(writer),
            AudioClassDescriptor::ClockMultiplier(cm) => cm.write(writer),
            AudioClassDescriptor::InputTerminal(it) => it.write(writer),
            AudioClassDescriptor::OutputTerminal(ot) => ot.write(writer),
        }
    }
    pub fn write_descriptor(&self, writer: &mut DescriptorWriter) -> Result<(), UsbError> {
        match self {
            AudioClassDescriptor::ClockSource(cs) => cs.write_descriptor(writer),
            AudioClassDescriptor::ClockMultiplier(cm) => cm.write_descriptor(writer),
            AudioClassDescriptor::InputTerminal(it) => it.write_descriptor(writer),
            AudioClassDescriptor::OutputTerminal(ot) => ot.write_descriptor(writer),
        }
    }
}

impl From<ClockSource> for AudioClassDescriptor {
    fn from(cs: ClockSource) -> Self {
        AudioClassDescriptor::ClockSource(cs)
    }
}

impl From<ClockMultiplier> for AudioClassDescriptor {
    fn from(cm: ClockMultiplier) -> Self {
        AudioClassDescriptor::ClockMultiplier(cm)
    }
}

impl From<InputTerminal> for AudioClassDescriptor {
    fn from(it: InputTerminal) -> Self {
        AudioClassDescriptor::InputTerminal(it)
    }
}
impl From<OutputTerminal> for AudioClassDescriptor {
    fn from(ot: OutputTerminal) -> Self {
        AudioClassDescriptor::OutputTerminal(ot)
    }
}
impl From<Terminal> for AudioClassDescriptor {
    fn from(t: Terminal) -> Self {
        match t {
            Terminal::Input(it) => AudioClassDescriptor::InputTerminal(it),
            Terminal::Output(ot) => AudioClassDescriptor::OutputTerminal(ot),
        }
    }
}

pub struct AudioClassInterfaceDescriptor<const NUM_DESCRIPTORS: usize> {
    inner: [AudioClassDescriptor; NUM_DESCRIPTORS],
    category: FunctionCode,
}

impl<const N: usize> AudioClassInterfaceDescriptor<N> {
    pub fn new(inner: [AudioClassDescriptor; N], category: FunctionCode) -> Self {
        Self { inner, category }
    }
    /// Total length of the interface descriptor and all its associated descriptors.
    /// wTotalLength in the Class-Specific AC Interface Header
    pub fn total_length(&self) -> u16 {
        9 + self
            .inner
            .iter()
            .map(|desc| desc.size() as u16)
            .sum::<u16>()
    }
    fn write_header<T: embedded_io::Write>(&self, writer: &mut T) -> Result<(), T::Error> {
        let total_length = self.total_length();
        writer.write_u8(9)?; // bLength
        writer.write_u8(ClassSpecificDescriptorType::Interface as u8)?; // bDescriptorType
        writer.write_u8(ClassSpecificACInterfaceDescriptorSubtype::Header as u8)?; // bDescriptorSubtype
        writer.write_u8(0)?; // bcdADC msd
        writer.write_u8(2)?; // bcdADC lsd
        writer.write_u8(self.category as u8)?; // bCategory
        writer.write_u16::<LittleEndian>(total_length)?; // wTotalLength
        writer.write_u8(0)?; // bmControls
        Ok(())
    }
    pub fn write<T: embedded_io::Write>(&self, writer: &mut T) -> Result<(), T::Error> {
        self.write_header(writer)?;
        for desc in &self.inner {
            desc.write(writer)?;
        }
        Ok(())
    }
}

/// USB Device Class Definition for Audio Data Formats Type I Format Type Descriptor
pub enum SamplingFrequencySet<'a> {
    Discrete(&'a [u32]),
    Continuous(u32, u32),
}

impl<'a> SamplingFrequencySet<'a> {
    pub fn size(&self) -> u8 {
        match self {
            SamplingFrequencySet::Discrete(freqs) => freqs.len() as u8 * 3,
            SamplingFrequencySet::Continuous(_, _) => 6,
        }
    }
}

pub struct FormatType1 {
    pub bytes_per_sample: u8, // bSubframeSize
    pub bit_resolution: u8,   // bBitResolution
}

impl Descriptor for FormatType1 {
    const MAX_SIZE: usize = 6;
    fn descriptor_type(&self) -> u8 {
        ClassSpecificDescriptorType::Interface as u8
    }
    fn size(&self) -> u8 {
        6
    }
    fn write_payload<T: embedded_io::Write>(&self, writer: &mut T) -> Result<(), T::Error> {
        writer.write_u8(ClassSpecificASInterfaceDescriptorSubtype::FormatType as u8)?;
        writer.write_u8(FormatType::Type1 as u8)?; // bFormatType
        writer.write_u8(self.bytes_per_sample)?; // bSubslotSize
        writer.write_u8(self.bit_resolution)?; // bBitResolution
        Ok(())
    }
}

#[repr(u32)]
#[derive(Clone, Copy, Debug)]
pub enum Type1FormatBitmap {
    Pcm = (1 << 0),
    Pcm8 = (1 << 1),
    IeeeFloat = (1 << 2),
    Alaw = (1 << 3),
    Mulaw = (1 << 4),
    Raw = (1 << 31),
}

pub struct AudioStreamingInterface {
    pub terminal_id: u8,
    pub active_alt_setting: AccessControl,
    pub valid_alt_settings: AccessControl,
    /// Only type 1 format is supported
    pub format_type: FormatType,
    pub format_bitmap: Type1FormatBitmap,
    pub num_channels: u8,
    pub channel_config: ChannelConfig,
    pub string: Option<StringIndex>,
}

impl AudioStreamingInterface {
    fn bm_controls(&self) -> u8 {
        self.active_alt_setting as u8 | ((self.valid_alt_settings as u8) << 2)
    }
}

impl Descriptor for AudioStreamingInterface {
    const MAX_SIZE: usize = 16;
    fn descriptor_type(&self) -> u8 {
        ClassSpecificDescriptorType::Interface as u8
    }
    fn size(&self) -> u8 {
        Self::MAX_SIZE as u8
    }
    fn write_payload<T: embedded_io::Write>(&self, writer: &mut T) -> Result<(), T::Error> {
        writer.write_u8(ClassSpecificASInterfaceDescriptorSubtype::General as u8)?;
        writer.write_u8(self.terminal_id)?;
        writer.write_u8(self.bm_controls())?;
        writer.write_u8(self.format_type as u8)?;
        writer.write_u32::<LittleEndian>(self.format_bitmap as u32)?;
        writer.write_u8(self.num_channels)?;
        writer.write(&self.channel_config.bytes)?;
        writer.write_u8(self.string.map_or(0, u8::from))?;
        Ok(())
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LockDelay {
    Undefined(u16),
    Milliseconds(u16),
    DecodedSamples(u16),
}
impl LockDelay {
    pub fn units(&self) -> u8 {
        match self {
            LockDelay::Undefined(_) => 0,
            LockDelay::Milliseconds(_) => 1,
            LockDelay::DecodedSamples(_) => 2,
        }
    }
    pub fn value(&self) -> u16 {
        match self {
            LockDelay::Undefined(v) => *v,
            LockDelay::Milliseconds(v) => *v,
            LockDelay::DecodedSamples(v) => *v,
        }
    }
}

pub struct AudioStreamingEndpoint {
    pub max_packets_only: bool,
    pub pitch_control: AccessControl,
    pub overrun_control: AccessControl,
    pub underrun_control: AccessControl,
    pub lock_delay: LockDelay,
}

impl AudioStreamingEndpoint {
    fn bm_controls(&self) -> u8 {
        (self.pitch_control as u8)
            | (self.overrun_control as u8) << 2
            | (self.underrun_control as u8) << 4
    }
}

impl Descriptor for AudioStreamingEndpoint {
    const MAX_SIZE: usize = 8;
    fn descriptor_type(&self) -> u8 {
        ClassSpecificDescriptorType::Endpoint as u8
    }
    fn size(&self) -> u8 {
        Self::MAX_SIZE as u8
    }
    fn write_payload<T: embedded_io::Write>(
        &self,
        writer: &mut T,
    ) -> core::result::Result<(), T::Error> {
        writer.write_u8(ClassSpecificEndpointDescriptorSubtype::General as u8)?; // bDescriptorSubtype
        writer.write_u8((self.max_packets_only as u8) << 7)?; // bmAttributes
        writer.write_u8(self.bm_controls())?; // bmControls
        writer.write_u8(self.lock_delay.units())?; // bLockDelayUnits
        writer.write_u16::<LittleEndian>(self.lock_delay.value())?; // wLockDelay
        Ok(())
    }
}

#[cfg(test)]
extern crate std;
#[cfg(test)]
mod tests {
    use super::*;
    use crate::cursor::Cursor;
    use std::{print, println};
    #[test]
    fn test_clock_source() {
        let string = Some(unsafe { core::mem::transmute(10u8) });
        let cs = ClockSource {
            id: 8,
            clock_type: ClockType::InternalFixed,
            sof_sync: false,
            frequency_access: AccessControl::ReadOnly,
            validity_access: AccessControl::ReadOnly,
            assoc_terminal: 6,
            string,
        };
        let mut buf = [0u8; ClockSource::MAX_SIZE];
        let mut cur = Cursor::new(&mut buf[..]);
        let len = cs.size();
        cs.write(&mut cur).unwrap();
        let descriptor = &buf[..len as usize];
        assert_eq!(
            descriptor,
            &[
                8,    // bLength
                0x24, // CS_INTERFACE
                0x0a, // CLOCK_SOURCE
                8,    // bClockId
                1,    // bmAttributes
                5,    // bmControls
                6,    // bAssocTerminal
                10,   // iClockSource
            ]
        )
    }
    #[test]
    fn test_clock_selector() {
        let cs = ClockSelector {
            id: 8,
            n_sources: 3,
            sources: [2, 3, 4, 0],
            selector_access: AccessControl::Programmable,
            string: 10,
        };
        let mut buf = [0u8; ClockSelector::<4>::MAX_SIZE];
        let mut cur = Cursor::new(&mut buf[..]);
        let len = cs.size();
        cs.write(&mut cur).unwrap();
        let descriptor = &buf[..len as usize];
        assert_eq!(
            descriptor,
            &[
                10,   // bLength
                0x24, // CS_INTERFACE
                0x0b, // CLOCK_SELECTOR
                8,    // bClockId
                3,    // bNrInPins
                2,    // baCSourceId
                3, 4, 3,  // bmControls
                10, // iClockSelector
            ]
        )
    }
    #[test]
    fn test_clock_multiplier() {
        let cm = ClockMultiplier {
            id: 8,
            source_id: 10,
            numerator_access: AccessControl::Programmable,
            denominator_access: AccessControl::ReadOnly,
            string: 20,
        };
        let mut buf = [0u8; ClockMultiplier::MAX_SIZE];
        let mut cur = Cursor::new(&mut buf[..]);
        let len = cm.size();

        cm.write(&mut cur).unwrap();
        let descriptor = &buf[0..len as usize];
        assert_eq!(
            descriptor,
            &[
                7,    // bLength
                0x24, // CS_INTERFACE
                0x0c, // CLOCK_MULTIPLIER
                8,    // bClockId
                10,   // bCSourceId
                7,    // bmControls
                20,   // iClockMultiplier
            ]
        )
    }
    #[test]
    fn test_input_terminal() {
        let it = InputTerminal {
            id: 8,
            terminal_type: TerminalType::InMicrophone,
            assoc_terminal: 10,
            clock_source: 12,
            num_channels: 2,
            channel_config: ChannelConfig::new()
                .with_front_left(true)
                .with_front_right(true),
            channel_names: 14,
            copy_protect_control: AccessControl::NotPresent,
            connector_control: AccessControl::Programmable,
            overload_control: AccessControl::ReadOnly,
            cluster_control: AccessControl::Programmable,
            underflow_control: AccessControl::ReadOnly,
            overflow_control: AccessControl::ReadOnly,
            phantom_power_control: AccessControl::NotPresent,
            string: Some(unsafe { core::mem::transmute(20u8) }),
        };
        let mut buf = [0u8; InputTerminal::MAX_SIZE];
        let mut cur = Cursor::new(&mut buf[..]);
        let len = it.size();

        it.write(&mut cur).unwrap();
        let descriptor = &buf[..len as usize];
        assert_eq!(
            descriptor,
            &[
                17,   // bLength
                0x24, // CS_INTERFACE
                0x02, // INPUT_TERMINAL
                8,
                0x01, // wTerminalType (u16)
                0x02,
                10, // bAssocTerminal
                12, // bCSourceId
                2,  // bNrChannels
                3,  // bmChannelConfig (u32)
                0,
                0,
                0,
                14,                             // iChannelNames
                (3 << 2) | (1 << 4) | (3 << 6), // bmControls (u16)
                (1 << 0) | (1 << 2),
                20, // iTerminal
            ]
        )
    }
    #[test]
    fn test_output_terminal() {
        let ot = OutputTerminal {
            id: 8,
            terminal_type: TerminalType::OutSpeaker,
            assoc_terminal: 10,
            source_id: 11,
            clock_source: 12,
            copy_protect_control: AccessControl::NotPresent,
            connector_control: AccessControl::Programmable,
            overload_control: AccessControl::ReadOnly,
            underflow_control: AccessControl::ReadOnly,
            overflow_control: AccessControl::ReadOnly,
            string: Some(unsafe { core::mem::transmute(20u8) }),
        };
        let mut buf = [0u8; OutputTerminal::MAX_SIZE];
        let mut cur = Cursor::new(&mut buf[..]);
        let len = ot.size();

        ot.write(&mut cur).unwrap();
        let descriptor = &buf[..len as usize];
        assert_eq!(
            descriptor,
            &[
                12,   // bLength
                0x24, // CS_INTERFACE
                0x03, // OUTPUT_TERMINAL
                8,    // bTerminalId
                01,   // wTerminalType (u16)
                03,
                10,                             // bAssocTerminal
                11,                             // bSourceId
                12,                             // bCSourceId
                (3 << 2) | (1 << 4) | (1 << 6), // bmControls (u16)
                1,
                20, // iTerminal
            ]
        )
    }
    #[test]
    fn test_selector_unit() {
        let su = SelectorUnit {
            id: 8,
            n_sources: 3,
            sources: [2, 3, 4, 0],
            selector_control: AccessControl::Programmable,
            string: 20,
        };
        let mut buf = [0u8; SelectorUnit::<4>::MAX_SIZE];
        let mut cur = Cursor::new(&mut buf[..]);
        let len = su.size();

        su.write(&mut cur).unwrap();
        let descriptor = &buf[..len as usize];
        assert_eq!(
            descriptor,
            &[
                10,   // bLength
                0x24, // CS_INTERFACE
                0x05, // SELECTOR_UNIT
                8,    // bUnitId
                3,    // bNrInPins
                2,    // baSourceId[3]
                3, 4, 3,  // bmControls
                20, // iSelector
            ]
        )
    }

    /// Write a minimal PCAP file containing a single synthetic USB control
    /// transfer that returns the provided descriptor bytes.
    ///
    /// The resulting file can be opened in Wireshark and expanded under:
    /// USB → URB → Descriptors
    ///
    /// Note: this is slop
    pub fn write_usb_descriptor_pcap(
        path: &str,
        descriptor_type: u8, // e.g. 0x02 = Configuration
        descriptor_index: u8,
        descriptor_bytes: &[u8],
    ) -> std::io::Result<()> {
        use std::fs::File;
        use std::io::{self, Write};
        use std::time::{SystemTime, UNIX_EPOCH};
        use std::vec::Vec;

        const DLT_USBPCAP: u32 = 249;

        // URB function codes (Windows). 0x0008 is URB_FUNCTION_CONTROL_TRANSFER.
        const URB_FUNCTION_CONTROL_TRANSFER: u16 = 0x0008;

        // USBPcap transfer types
        const USBPCAP_TRANSFER_CONTROL: u8 = 2;

        // USBPcap control stages
        const USBPCAP_CONTROL_STAGE_SETUP: u8 = 0;
        const USBPCAP_CONTROL_STAGE_COMPLETE: u8 = 3;

        let mut f = File::create(path)?;

        // PCAP GLOBAL HEADER (little endian)
        f.write_all(&[
            0xd4,
            0xc3,
            0xb2,
            0xa1, // magic
            0x02,
            0x00, // version major
            0x04,
            0x00, // version minor
            0x00,
            0x00,
            0x00,
            0x00, // thiszone
            0x00,
            0x00,
            0x00,
            0x00, // sigfigs
            0xff,
            0xff,
            0x00,
            0x00, // snaplen
            (DLT_USBPCAP & 0xff) as u8,
            ((DLT_USBPCAP >> 8) & 0xff) as u8,
            ((DLT_USBPCAP >> 16) & 0xff) as u8,
            ((DLT_USBPCAP >> 24) & 0xff) as u8,
        ])?;

        let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
        let ts_sec = now.as_secs() as u32;
        let ts_usec = now.subsec_micros();

        // USB SETUP packet for GET_DESCRIPTOR
        let w_value = ((descriptor_type as u16) << 8) | (descriptor_index as u16);
        let w_length = descriptor_bytes.len() as u16;

        let setup_pkt = [
            0x80, // bmRequestType: device->host, standard, device
            0x06, // bRequest: GET_DESCRIPTOR
            (w_value & 0xff) as u8,
            (w_value >> 8) as u8,
            0x00,
            0x00, // wIndex
            (w_length & 0xff) as u8,
            (w_length >> 8) as u8,
        ];

        // Helper to write one USBPcap packet record
        fn write_usbpcap_record(
            f: &mut File,
            ts_sec: u32,
            ts_usec: u32,
            // USBPcap base header fields
            irp_id: u64,
            status: u32,
            function: u16,
            info: u8,
            bus: u16,
            device: u16,
            endpoint: u8,
            transfer: u8,
            stage: u8,
            payload: &[u8],
        ) -> io::Result<()> {
            // USBPCAP_BUFFER_PACKET_HEADER is packed(1) and begins with USHORT headerLen.
            // For control transfers, header is: base header + stage byte.
            // base header size = 2+8+4+2+1+2+2+1+1+4 = 27 bytes
            // + stage = 1 => 28 bytes total headerLen
            let header_len: u16 = 28;
            let data_len: u32 = payload.len() as u32;

            let mut pkt = Vec::with_capacity(header_len as usize + payload.len());

            // --- USBPCAP_BUFFER_PACKET_HEADER ---
            pkt.extend_from_slice(&header_len.to_le_bytes()); // USHORT headerLen
            pkt.extend_from_slice(&irp_id.to_le_bytes()); // UINT64 irpId
            pkt.extend_from_slice(&status.to_le_bytes()); // USBD_STATUS status (UINT32)
            pkt.extend_from_slice(&function.to_le_bytes()); // USHORT function
            pkt.push(info); // UCHAR info
            pkt.extend_from_slice(&bus.to_le_bytes()); // USHORT bus
            pkt.extend_from_slice(&device.to_le_bytes()); // USHORT device
            pkt.push(endpoint); // UCHAR endpoint (MSB = IN)
            pkt.push(transfer); // UCHAR transfer (2 = control)
            pkt.extend_from_slice(&data_len.to_le_bytes()); // UINT32 dataLength

            // --- USBPCAP_BUFFER_CONTROL_HEADER extra field ---
            pkt.push(stage); // UCHAR stage

            debug_assert_eq!(pkt.len(), header_len as usize);

            // --- transfer data ---
            pkt.extend_from_slice(payload);

            // PCAP per-record header
            let incl_len = pkt.len() as u32;
            f.write_all(&ts_sec.to_le_bytes())?;
            f.write_all(&ts_usec.to_le_bytes())?;
            f.write_all(&incl_len.to_le_bytes())?;
            f.write_all(&incl_len.to_le_bytes())?;
            f.write_all(&pkt)?;
            Ok(())
        }

        // Use consistent IDs so Wireshark can correlate them.
        let irp_id: u64 = 0x1111_2222_3333_4444;
        let bus: u16 = 1;
        let device: u16 = 1;

        // 1) SETUP stage (host -> device), carries SETUP bytes as payload.
        write_usbpcap_record(
            &mut f,
            ts_sec,
            ts_usec,
            irp_id,
            0, // status
            URB_FUNCTION_CONTROL_TRANSFER,
            0x00, // info: FDO->PDO (request)
            bus,
            device,
            0x00, // EP0 OUT
            USBPCAP_TRANSFER_CONTROL,
            USBPCAP_CONTROL_STAGE_SETUP,
            &setup_pkt, // payload = 8-byte setup
        )?;

        let mut config_descriptor = Vec::with_capacity(9 + 8 + 9 + descriptor_bytes.len());
        // Config Descriptor
        config_descriptor.push(0x09); // bLength
        config_descriptor.push(0x02); // bDescriptorType = CONFIGURATION
        config_descriptor.write_all(&(9 + 8 + 9 + descriptor_bytes.len() as u16).to_le_bytes())?;
        config_descriptor.push(0x01); // bNumInterfaces
        config_descriptor.push(0x01); // bConfigurationValue
        config_descriptor.push(0x00); // iConfiguration
        config_descriptor.push(0xC0); // bmAttributes = self-powered
        config_descriptor.push(0x32); // bMaxPower = 100mA

        // Interface Association Descriptor
        config_descriptor.push(0x08); // bLength
        config_descriptor.push(0x0B); // bDescriptorType = INTERFACE ASSOCIATION
        config_descriptor.push(0x00); // bFirstInterface
        config_descriptor.push(0x01); // bInterfaceCount
        config_descriptor.push(0x01); // bFunctionClass = AUDIO
        config_descriptor.push(0x03); // bFunctionSubClass = AUDIO_STREAMING
        config_descriptor.push(0x00); // bFunctionProtocol = NONE
        config_descriptor.push(0x00); // iFunction

        // 'Standard' Audio Class Interface Descriptor
        config_descriptor.push(0x09); // bLength
        config_descriptor.push(0x04); // bDescriptorType = INTERFACE
        config_descriptor.push(0x00); // bInterfaceNumber
        config_descriptor.push(0x00); // bAlternateSetting
        config_descriptor.push(0x02); // bNumEndpoints
        config_descriptor.push(0x01); // bInterfaceClass = AUDIO
        config_descriptor.push(0x01); // bInterfaceSubClass = AUDIO_CONTROL
        config_descriptor.push(0x20); // bInterfaceProtocol = NONE
        config_descriptor.push(0x00); // iInterface

        config_descriptor.write_all(descriptor_bytes)?;

        // 2) COMPLETE stage (device -> host), carries descriptor bytes as payload.
        write_usbpcap_record(
            &mut f,
            ts_sec,
            ts_usec.wrapping_add(1), // tiny delta
            irp_id,
            0, // status
            URB_FUNCTION_CONTROL_TRANSFER,
            0x01, // info: PDO->FDO (response)
            bus,
            device,
            0x80, // EP0 IN
            USBPCAP_TRANSFER_CONTROL,
            USBPCAP_CONTROL_STAGE_COMPLETE,
            &config_descriptor, // payload = IN data (descriptors)
        )?;
        Ok(())
    }

    #[test]
    fn test_ac_interface() {
        let descriptors: [AudioClassDescriptor; _] = [
            ClockSource {
                id: 1,
                clock_type: ClockType::InternalFixed,
                sof_sync: false,
                frequency_access: AccessControl::NotPresent,
                validity_access: AccessControl::NotPresent,
                assoc_terminal: 0,
                string: None,
            }
            .into(),
            InputTerminal {
                id: 2,
                terminal_type: TerminalType::UsbStreaming,
                assoc_terminal: 0,
                clock_source: 1,
                num_channels: 2,
                channel_config: ChannelConfig::default_chans(2),
                channel_names: 0,
                copy_protect_control: AccessControl::NotPresent,
                connector_control: AccessControl::NotPresent,
                overload_control: AccessControl::NotPresent,
                cluster_control: AccessControl::NotPresent,
                underflow_control: AccessControl::NotPresent,
                overflow_control: AccessControl::NotPresent,
                phantom_power_control: AccessControl::NotPresent,
                string: None,
            }
            .into(),
            OutputTerminal {
                id: 3,
                terminal_type: TerminalType::OutUndefined,
                assoc_terminal: 0,
                source_id: 2,
                clock_source: 1,
                copy_protect_control: AccessControl::NotPresent,
                connector_control: AccessControl::NotPresent,
                overload_control: AccessControl::NotPresent,
                underflow_control: AccessControl::NotPresent,
                overflow_control: AccessControl::NotPresent,
                string: None,
            }
            .into(),
            OutputTerminal {
                id: 4,
                source_id: 5,
                terminal_type: TerminalType::UsbStreaming,
                assoc_terminal: 0,
                clock_source: 1,
                copy_protect_control: AccessControl::NotPresent,
                connector_control: AccessControl::NotPresent,
                overload_control: AccessControl::NotPresent,
                underflow_control: AccessControl::NotPresent,
                overflow_control: AccessControl::NotPresent,
                string: None,
            }
            .into(),
            InputTerminal {
                id: 5,
                terminal_type: TerminalType::InUndefined,
                assoc_terminal: 0,
                clock_source: 1,
                num_channels: 2,
                channel_config: ChannelConfig::default_chans(2),
                channel_names: 0,
                copy_protect_control: AccessControl::NotPresent,
                connector_control: AccessControl::NotPresent,
                overload_control: AccessControl::NotPresent,
                cluster_control: AccessControl::NotPresent,
                underflow_control: AccessControl::NotPresent,
                overflow_control: AccessControl::NotPresent,
                phantom_power_control: AccessControl::NotPresent,
                string: None,
            }
            .into(),
        ];
        let ac = AudioClassInterfaceDescriptor::new(descriptors, FunctionCode::Undefined);
        let mut buf = [0u8; 1024];
        let len = {
            let mut cur = Cursor::new(&mut buf[..]);
            ac.write(&mut cur).unwrap();
            cur.position()
        };
        let bytes = &buf[..len];
        for (i, b) in bytes.iter().enumerate() {
            if i.is_multiple_of(16) {
                println!();
            }
            print!("{:02x} ", b);
        }
        println!();
        write_usb_descriptor_pcap("./uac2.pcap", 0x02, 0, bytes).unwrap();
    }

    #[test]
    fn test_format_type1() {
        let format = FormatType1 {
            bytes_per_sample: 4,
            bit_resolution: 24,
        };
        let mut buf = [0u8; FormatType1::MAX_SIZE];
        let len = {
            let mut cur = Cursor::new(&mut buf[..]);
            format.write(&mut cur).unwrap();
            cur.position()
        };
        let descriptor = &buf[..len];
        assert_eq!(
            descriptor,
            &[
                6,    //bLength
                0x24, // CS_INTERFACE
                0x02, // FORMAT_TYPE
                0x01, // FORMAT_TYPE_I
                4,    // bSubframeSize
                24,   // bBitResolution
            ]
        );
    }
    fn test_as_interface_desc() {
        let intf = AudioStreamingInterface {
            terminal_id: 2,
            active_alt_setting: AccessControl::Programmable,
            valid_alt_settings: AccessControl::ReadOnly,
            format_type: FormatType::Type1,
            format_bitmap: Type1FormatBitmap::Pcm,
            num_channels: 2,
            channel_config: ChannelConfig::default_chans(2),
            string: None,
        };
        let mut buf = [0u8; AudioStreamingInterface::MAX_SIZE];
        let len = {
            let mut cur = Cursor::new(&mut buf[..]);
            intf.write(&mut cur).unwrap();
            cur.position()
        };
        let descriptor = &buf[..len];
        assert_eq!(
            descriptor,
            &[
                16,
                0x24,         // CS_INTERFACE
                0x01,         // AS_GENERAL
                2,            // bTerminalLink
                3 | (1 << 2), // bmControls
                1,            // bFormatType
                1,            // bmFormats[0]
                0,            // bmFormats[1]
                0,            // bmFormats[2]
                0,            // bmFormats[3]
                2,            // bNrChannels
                3,            // bmChannelConfig[0]
                0,            // bmChannelConfig[1]
                0,            // bmChannelConfig[2]
                0,            // bmChannelConfig[3]
                0             // iChannelNames
            ]
        );
    }

    #[test]
    fn test_as_endpoint_desc() {
        let ep = AudioStreamingEndpoint {
            max_packets_only: true,
            pitch_control: AccessControl::NotPresent,
            overrun_control: AccessControl::ReadOnly,
            underrun_control: AccessControl::Programmable,
            lock_delay: LockDelay::DecodedSamples(1024),
        };

        let mut buf = [0u8; AudioStreamingEndpoint::MAX_SIZE];
        let len = {
            let mut cur = Cursor::new(&mut buf[..]);
            ep.write(&mut cur).unwrap();
            cur.position()
        };
        let descriptor = &buf[..len];
        assert_eq!(
            descriptor,
            &[
                8,
                0x25,                    // CS_ENDPOINT
                0x01,                    // EP_GENERAL
                0x80,                    // bmAttributes
                0 | (1 << 2) | (3 << 4), // bmControls
                2,                       // bLockDelayUnits
                (1024u16 & 0xff) as u8,  // bLockDelay[0]
                (1024u16 >> 8) as u8,    // bLockDelay[1]
            ]
        );
    }
}
