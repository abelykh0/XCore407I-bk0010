#![no_std]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#[cfg(feature = "tinyrlibc")]
extern crate tinyrlibc;
include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
