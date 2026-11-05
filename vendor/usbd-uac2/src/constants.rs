pub const USB_CLASS_AUDIO: u8 = 0x1;
pub const HEADER: u8 = 0x1;

/// A.2 Audio Function Subclass Codes
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum FunctionSubclass {
    Undefined = 0,
}

/// A.3 Audio Function Protocol Codes
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum FunctionProtocol {
    Undefined = 0,
    Version2 = 0x20,
}

/// A.5 Audio Interface Subclass Codes
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum InterfaceSubclass {
    Undefined = 0,
    AudioControl = 1,
    AudioStreaming = 2,
    MidiStreaming = 3,
}

/// A.6 Audio Interface Protocol Codes
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum InterfaceProtocol {
    Undefined = 0,
    Version2 = 0x20,
}

/// A.7 Audio Function Category Codes
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum FunctionCode {
    Undefined = 0,
    DesktopSpeaker = 1,
    HomeTheater = 2,
    Microphone = 3,
    Headset = 4,
    Telephone = 5,
    Converter = 6,
    SoundRecorder = 7,
    IoBox = 8,
    MusicalInstrument = 9,
    ProAudio = 0xa,
    AudioVideo = 0xb,
    ControlPanel = 0xc,
    Other = 0xff,
}

/// A.8 Audio Class-Specific Descriptor Types
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ClassSpecificDescriptorType {
    Undefined = 0x20,
    Device = 0x21,
    Configuration = 0x22,
    String = 0x23,
    Interface = 0x24,
    Endpoint = 0x25,
}

/// A.9 Audio Class-Specific AC Interface Descriptor Subtypes
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ClassSpecificACInterfaceDescriptorSubtype {
    Undefined = 0,
    Header = 0x01,
    InputTerminal = 0x02,
    OutputTerminal = 0x03,
    MixerUnit = 0x04,
    SelectorUnit = 0x05,
    FeatureUnit = 0x06,
    EffectUnit = 0x07,
    ProcessingUnit = 0x08,
    ExtensionUnit = 0x09,
    ClockSource = 0x0A,
    ClockSelector = 0x0B,
    ClockMultiplier = 0x0C,
    SampleRateConverter = 0x0D,
}

/// A.10 Audio Class-Specific AS Interface Descriptor Subtypes
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ClassSpecificASInterfaceDescriptorSubtype {
    Undefined = 0,
    General = 1,
    FormatType = 2,
    Encoder = 3,
    Decoder = 4,
}

/// A.11 Effect Unit Effect Types
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum EffectUnitEffectType {
    Undefined = 0,
    ParamEqSection = 1,
    Reverb = 2,
    ModDelay = 3,
    DynRangeComp = 4,
}

/// A.12 Processing Unit Process Types
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ProcessingUnitProcessType {
    Undefined = 0,
    UpDownMix = 1,
    DolbyPrologic = 2,
    StereoExtender = 3,
}

/// A.13 Audio Class-Specific Endpoint Descriptor Subtypes
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ClassSpecificEndpointDescriptorSubtype {
    Undefined = 0,
    General = 1,
}

/// A.14 Audio Class-Specific Request Codes
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ClassSpecificRequest {
    Undefined = 0,
    Cur = 1,
    Range = 2,
    Mem = 3,
}

impl From<u8> for ClassSpecificRequest {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Undefined,
            1 => Self::Cur,
            2 => Self::Range,
            3 => Self::Mem,
            _ => Self::Undefined,
        }
    }
}

/// A.15 Encoder Type Codes
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum EncoderType {
    Undefined = 0,
    Other = 1,
    Mpeg = 2,
    Ac3 = 3,
    Wma = 4,
    Dts = 5,
}
/// A.16 Decoder Type Codes
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum DecoderType {
    Undefined = 0,
    Other = 1,
    Mpeg = 2,
    Ac3 = 3,
    Wma = 4,
    Dts = 5,
}

/// A.17.1 Clock Source Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ClockSourceControlSelector {
    Undefined = 0,
    SamFreqControl = 1,
    ClockValidControl = 2,
}

impl From<u8> for ClockSourceControlSelector {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Undefined,
            1 => Self::SamFreqControl,
            2 => Self::ClockValidControl,
            _ => Self::Undefined,
        }
    }
}

/// A.17.2 Clock Selector Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ClockSelectorControlSelector {
    ControlUndefined = 0,
    ClockSelectorControl = 1,
}

/// A.17.3 Clock Multiplier Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ClockMultiplierControlSelector {
    Undefined = 0,
    NumeratorControl = 1,
    DenominatorControl = 2,
}

/// A.17.4 Terminal Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum TerminalControlSelector {
    Undefined = 0,
    CopyProtect = 1,
    Connector = 2,
    Overload = 3,
    Cluster = 4,
    Underflow = 5,
    Overflow = 6,
    Latency = 7,
    PhantomPower = 8,
}

/// A.17.5 Mixer Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MixerControlSelector {
    Undefined = 0,
    Mixer = 1,
    Cluster = 2,
    Underflow = 3,
    Overflow = 4,
    Latency = 5,
}

/// A.17.6 Selector Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SelectorControlSelector {
    Undefined = 0,
    Selector = 1,
    Latency = 2,
}

/// A.17.7 Feature Unit Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum FeatureUnitControlSelector {
    Undefined = 0,
    Mute = 0x01,
    Volume = 0x02,
    Bass = 0x03,
    Mid = 0x04,
    Treble = 0x05,
    GraphicEqualizer = 0x06,
    AutomaticGain = 0x07,
    Delay = 0x08,
    BassBoost = 0x09,
    Loudness = 0x0A,
    InputGain = 0x0B,
    InputGainPad = 0x0C,
    PhaseInverter = 0x0D,
    Underflow = 0x0E,
    Overflow = 0x0F,
    Latency = 0x10,
    HighpassFilter = 0x11,
}
/// A.17.8.1 Parametric Equalizer Section Effect Unit Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ParametricEqSectionEffectUnitControlSelector {
    Undefined = 0,
    Enable = 1,
    CenterFreq = 2,
    QFactor = 3,
    Gain = 4,
    Underflow = 5,
    Overflow = 6,
    Latency = 7,
}

/// A.17.8.2 Reverberation Effect Unit Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ReverbEffectUnitControlSelector {
    Undefined = 0,
    Enable = 1,
    Type = 2,
    Level = 3,
    Time = 4,
    Feedback = 5,
    PreDelay = 6,
    Density = 7,
    HiFreqRolloff = 8,
    Underflow = 9,
    Overflow = 0xa,
    Latency = 0xb,
}

/// A.17.8.3 Modulation Delay Effect Unit Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ModDelayEffectUnitControlSelector {
    Undefined = 0,
    Enable = 1,
    Balance = 2,
    Rate = 3,
    Depth = 4,
    Time = 5,
    Feedback = 6,
    Underflow = 7,
    Overflow = 8,
    Latency = 9,
}

/// A.17.8.4 Dynamic Range Compressor Effect Unit Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum DynamicRangeCompressorEffectUnitControlSelector {
    Undefined = 0,
    Enable = 1,
    CompressionRate = 2,
    MaxAmplitude = 3,
    Threshold = 4,
    AttackTime = 5,
    ReleaseTime = 6,
    Underflow = 7,
    Overflow = 8,
    Latency = 9,
}

/// A.17.9.1 Up/Down-mix Processing Unit Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum UpDownMixProcessingUnitControlSelector {
    Undefined = 0,
    Enable = 1,
    ModeSelect = 2,
    Cluster = 3,
    Underflow = 4,
    Overflow = 5,
    Latency = 6,
}

/// A.17.9.2 Dolby Prologic Processing Unit Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum DolbyProcessingUnitControlSelectors {
    Undefined = 0,
    Enable = 1,
    ModeSelect = 2,
    Cluster = 3,
    Underflow = 4,
    Overflow = 5,
    Latency = 6,
}

/// A.17.9.3 Stereo Extender Processing Unit Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum StereoExtenderProcessingUnitControlSelector {
    Undefined = 0,
    Enable = 1,
    Width = 2,
    Underflow = 3,
    Overflow = 4,
    Latency = 5,
}

/// A.17.10 Extension Unit Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ExtensionUnitControlSelector {
    Undefined = 0,
    Enable = 1,
    Cluster = 2,
    Underflow = 3,
    Overflow = 4,
    Latency = 5,
}

/// A.17.11 AudioStreaming Interface Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AudioStreamingInterfaceControlSelector {
    Undefined = 0,
    ActAltSetting = 1,
    ValAltSetting = 2,
    AudioDataFormat = 3,
}

/// A.17.12 Encoder Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum EncoderControlSelector {
    Undefined = 0,
    BitRate = 0x01,
    Quality = 0x02,
    Vbr = 0x03,
    Type = 0x04,
    Underflow = 0x05,
    Overflow = 0x06,
    EncoderError = 0x07,
    Param1 = 0x08,
    Param2 = 0x09,
    Param3 = 0x0A,
    Param4 = 0x0B,
    Param5 = 0x0C,
    Param6 = 0x0D,
    Param7 = 0x0E,
    Param8 = 0x0F,
}

/// A.17.13.1 MPEG Decoder Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MpegDecoderControlSelector {
    Undefined = 0,
    DualChannel = 1,
    SecondStereo = 2,
    Multilingual = 3,
    DynRange = 4,
    Scaling = 5,
    HiloScaling = 6,
    Underflow = 7,
    Overflow = 8,
    DecoderError = 9,
}

/// A.17.13.2 AC-3 Decoder Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Ac3DecoderControlSelector {
    Undefined = 0,
    Mode = 1,
    DynRange = 2,
    Scaling = 3,
    HiloScaling = 4,
    Underflow = 5,
    Overflow = 6,
    DecoderError = 7,
}

/// A.17.13.3 WMA Decoder Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum WmaDecoderControlSelector {
    Undefined = 0,
    Underflow = 1,
    Overflow = 2,
    DecoderError = 3,
}

/// A.17.13.4 DTS Decoder Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum DtsDecoderControlSelector {
    Undefined = 0,
    Underflow = 1,
    Overflow = 2,
    DecoderError = 3,
}

/// A.17.14 Endpoint Control Selectors
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum EndpointControlSelector {
    Undefined = 0,
    Pitch = 1,
    DataOverrun = 2,
    DataUnderrun = 3,
}

/// Universal Serial Bus Device Class Definition for Terminal Types
#[repr(u16)]
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum TerminalType {
    // USB Terminal Types
    UsbUndefined = 0x0100,
    UsbStreaming = 0x0101,
    UsbVendor = 0x01ff,

    // Input Terminal Types
    InUndefined = 0x0200,
    InMicrophone = 0x0201,
    InDesktopMicrophone = 0x0202,
    InPersonalMicrophone = 0x0203,
    InOmniDirectionalMicrophone = 0x0204,
    InMicrophoneArray = 0x0205,
    InProcessingMicrophoneArray = 0x0206,

    // Output Terminal Types
    OutUndefined = 0x0300,
    OutSpeaker = 0x0301,
    OutHeadphones = 0x0302,
    OutHeadMountedDisplayAudio = 0x0303,
    OutDesktopSpeaker = 0x0304,
    OutRoomSpeaker = 0x0305,
    OutCommunicationSpeaker = 0x0306,
    OutLowFrequencyEffectsSpeaker = 0x0307,

    // External Terminal Types
    ExtUndefined = 0x0600,
    ExtAnalogConnector = 0x0601,
    ExtDigitalAudioInterface = 0x0602,
    ExtLineConnector = 0x0603,
    ExtLegacyAudioConnector = 0x0604,
    ExtSpdifConnector = 0x0605,
    Ext1394DaStream = 0x0606,
    Ext1394DvStreamSoundtrack = 0x0607,
}

#[repr(u16)]
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AudioDataFormatType1 {
    Undefined = 0,
    Pcm = 1,
    Pcm8 = 2,
    IeeeFloat = 3,
    Alaw = 4,
    Mulaw = 5,
}

#[repr(u16)]
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AudioDataFormatType2 {
    Undefined = 0x1000,
    Mpeg = 0x1001,
    Ac3 = 0x1002,
}

#[repr(u16)]
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AudioDataFormatType3 {
    Undefined = 0x2000,
    Ac3 = 0x2001,
    Mpeg1Layer1 = 0x2002,
    Mpeg1Layer23OrMpeg2NoExt = 0x2003,
    Mpeg2Ext = 0x2004,
    Mpeg2Layer1Ls = 0x2005,
    Mpeg2Layer23Ls = 0x2006,
}
#[repr(u8)]
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum FormatType {
    Undefined = 0,
    Type1 = 1,
    Type2 = 2,
    Type3 = 3,
}
