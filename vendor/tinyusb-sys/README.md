[![crates.io](https://img.shields.io/crates/d/tinyusb-sys.svg)](https://crates.io/crates/tinyusb-sys)
[![crates.io](https://img.shields.io/crates/v/tinyusb-sys.svg)](https://crates.io/crates/tinyusb-sys)

TinyUSB Rust Bindings
=====================

[TinyUSB](https://github.com/hathach/tinyusb) is an open-source cross-platform
USB Host/Device stack for embedded system, designed to be memory-safe with no
dynamic allocation and thread-safe with all interrupt events are deferred then
handled in the non-ISR task function.

This crate provides Rust bindings for the library.

Declare the dependency as such, for example to operate as a mass storage device
on an MCU from the STM32F1 family:

```toml
[dependencies]
tinyusb-sys = { version = "0.1", features = ["device", "msc", "stm32f1"] }
```

## Feature definitions

To use the crate, you must pick one mode (host or device), at least one USB
class, and an MCU family.

### Mode

- `host`
- `device`

### Host features

- `msc`
- `cdc`
- `vendor`
- `hub`
- `hid`
- `midi`

### Device features

- `bth`
- `video`
- `ecm_rndis`
- `dfu`
- `midi`
- `ncm`
- `dfu_runtime`
- `hid`
- `vendor`
- `usbtmc`
- `audio`
- `msc`
- `cdc`

### MCU family

See the feature list in `Cargo.toml` for supported MCU families.
