// src/log.rs (or log/mod.rs)
#[allow(unused_imports)]
#[cfg(feature = "defmt")]
pub use defmt::{debug, error, info, trace, warn};

#[cfg(not(feature = "defmt"))]
mod no_defmt {
    #[macro_export]
    macro_rules! trace {
        ($($t:tt)*) => {
            ()
        };
    }
    #[macro_export]
    macro_rules! debug {
        ($($t:tt)*) => {
            ()
        };
    }
    #[macro_export]
    macro_rules! info {
        ($($t:tt)*) => {
            ()
        };
    }
    #[macro_export]
    macro_rules! warn {
        ($($t:tt)*) => {
            ()
        };
    }
    #[macro_export]
    macro_rules! error {
        ($($t:tt)*) => {
            ()
        };
    }
}

#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
pub use no_defmt::*;
