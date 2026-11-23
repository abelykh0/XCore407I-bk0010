//! USB high-speed device implementation.
//!
//! The generated TinyUSB FFI bindings remain at the crate root because they
//! are shared with the independent OTG_FS host controller.

pub mod audio;
pub mod cdc;
pub mod descriptors;
pub mod video;

// Preserve concise device-level paths for the application layer.
pub use self::audio::{bk_audio, uac2 as audio_uac2};
pub use self::cdc::{console as cdc_console, zmodem as cdc_zmodem};
pub use self::descriptors as tusb_descriptors;
pub use self::video::{uvc, uvc_nv12};
