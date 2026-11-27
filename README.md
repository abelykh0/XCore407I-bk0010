# БК-0010 Emulator

[БК-0010](https://en.wikipedia.org/wiki/Elektronika_BK) emulator for the Waveshare [XCore407I](https://www.waveshare.com/wiki/XCore407I) board (STM32F407IGT6, 168 MHz).

- **Video:** USB UVC, uncompressed NV12, 25 fps.
- **Audio:** USB Line In.
- **Input:** USB keyboard connected to the board's host port.
- **Storage:** Samsung K9F1G08U0E 1-Gbit raw NAND.

[Video demo](https://www.youtube.com/watch?v=IPjBy6mAeOw)

## Summary

The board uses an STM32F407IGT6 with 192 KB SRAM, a USB3300 high-speed PHY,
and onboard Samsung K9F1G08U0E 1-Gbit raw NAND.

Video is generated and streamed without storing a complete frame in RAM.
The firmware coordinates USB video, audio, keyboard input, and emulator timing.

## Implementation

### Video

The camera image is rendered as uncompressed NV12.
Rows are generated incrementally from the emulator's canvas; a complete video
frame is never buffered in RAM. Two small SRAM buffers decouple rendering in
the main loop from high-speed USB isochronous transfers. The stream runs at
25 fps over TinyUSB's high-speed device port.

### Audio

The BK-0010 speaker output is captured as timestamped level transitions from
the emulator. A main-loop worker converts those edges into 24 kHz, mono,
signed 16-bit PCM in a ring buffer. TinyUSB's UAC2 isochronous IN path sends
the already-rendered samples to the host.

### NAND storage

Dhara is the only raw-NAND backend. It manages physical blocks, bad blocks,
and wear levelling; STM32 FSMC Hamming ECC protects page data. LittleFS uses
700 logical erase blocks (87.5 MiB).

Use CDC console to format NAND and upload files.

### CDC console and ZMODEM

The USB CDC interface provides a serial console for firmware commands and
ZMODEM file transfers. To enter CDC console mode hold F10 on the keyboard at startup. 
In Tera Term, enable these settings in `teraterm.ini`:

```ini
; Clear serial port buffer when port opening
ClearComBuffOnOpen=off
; ZMODEM auto receive
ZmodemAuto=on
```

`ClearComBuffOnOpen=off` preserves the startup console banner when the port
opens. `ZmodemAuto=on` enables automatic ZMODEM receive handling.

### Ethernet HTTP file server

Ethernet is available in CDC mode. The STM32F407's Ethernet MAC uses the
onboard DP83848 PHY over RMII. The network interface obtains its IPv4
configuration from DHCP; use the CDC console's `IPCONFIG` command to see the
assigned address and link status.

Open `http://<board-ip>/` in a browser to access the LittleFS file server.
The page lists files stored at the filesystem root and provides browser-based
file upload:

- The directory listing is loaded in pages, so large directories do not need
  to be returned in one response.
- Uploads use HTTP POST requests with 2 KiB chunks. The server keeps the
  destination file open across chunks and writes each chunk at its specified
  offset.
- Root-level filenames may contain 1–64 ASCII letters, digits, dots,
  underscores, and hyphens. Directory paths are not accepted.
- JPEG (`.jpg`, `.jpeg`) and PNG (`.png`) files are linked from the listing
  and served inline by the browser. Other files remain available for listing
  and upload; they are not exposed as downloadable links by this page.

The HTTP server and its buffers are active only in CDC mode and use the same
LittleFS volume as the CDC console and ZMODEM transfers.

## RTC registers

The STM32 calendar is exposed as read-only Unix milliseconds (low 32 bits):
  
| Octal address | Register |
| --- | --- |
| `177720` | `RTC_MILLIS_L` — bits 0–15 |
| `177722` | `RTC_MILLIS_H` — bits 16–31 |

Read the low word first; it captures a snapshot used by the following high-word
read. The value wraps every 49.7 days. RTC resolution is approximately 3.9 ms.

## Third-party libraries

The firmware uses the following third-party libraries and support packages.
Vendored sources are kept in the repository; Rust crate versions are pinned
by `Cargo.lock`.

### Firmware libraries

- **[cortex-m](https://github.com/rust-embedded/cortex-m)** and
  **[cortex-m-rt](https://github.com/rust-embedded/cortex-m-rt)** —
  Cortex-M core support, critical sections, and runtime/startup.
- **[stm32f4](https://github.com/stm32-rs/stm32-rs)** — STM32F4
  peripheral access crate (PAC).
- **[stm32f4xx-hal](https://github.com/stm32-rs/stm32f4xx-hal)** —
  STM32F4 hardware abstraction layer.
- **[libm 0.2](https://github.com/rust-lang/libm)** — portable math routines
  for `no_std`.
- **[panic-halt](https://github.com/korken89/panic-halt)** — panic
  handler that halts the target.
- **[TinyUSB](https://github.com/hathach/tinyusb)** — USB device and
  host stack; used for UVC video, UAC2 audio, CDC serial, and USB host input.
  Its C sources are compiled directly from `vendor/tinyusb`.
- **[smoltcp](https://github.com/smoltcp-rs/smoltcp)** — allocation-free
  TCP/IP network stack used by the Ethernet interface, DHCP client, and TCP
  sockets for the HTTP file server.
- **CMSIS Core and STM32F4 device headers** — ARM core and ST device
  definitions used to compile TinyUSB for the STM32F407.

- **[Dhara](https://github.com/dlbeer/dhara)** — NAND flash translation layer
  for raw NAND management, including bad-block handling, garbage collection,
  and wear levelling. Vendored under `vendor/dhara`.
- **[littlefs](https://github.com/littlefs-project/littlefs)** —
  embedded filesystem implementation. Accessed through the vendored
  **littlefs2** Rust API and **littlefs2-sys** bindings under
  `vendor/littlefs2` and `vendor/littlefs2-sys`; these copies include
  project-specific integration and modifications.
- **[zmodem2](https://codeberg.org/jarkko/zmodem2)** — ZMODEM file
  transfer protocol implementation. Vendored under `vendor/zmodem2`.

### Build dependencies

- **[cc](https://github.com/rust-lang/cc-rs)** — compiles the BK CPU,
  Dhara adapter, Dhara, and TinyUSB C sources for the target.
- **[bindgen](https://github.com/rust-lang/rust-bindgen)** — generates
  Rust FFI bindings for the TinyUSB API.
- **[glob](https://github.com/rust-lang/glob)** — recursively discovers
  TinyUSB C source files during the build.

The project also contains the local `stm32f4-crc32` crate, which provides
STM32F4 hardware CRC32 access for the firmware and ZMODEM integration. It is
project code rather than an external dependency. Transitive Rust dependencies
are recorded in `Cargo.lock`.

## Build and flash

```powershell
cargo build --release --target thumbv7em-none-eabihf
probe-rs download --chip STM32F407IGTx target/thumbv7em-none-eabihf/release/xcore407i_blinker
```

Optional CPU timing diagnostics:

```powershell
cargo build --release --target thumbv7em-none-eabihf --features cpu-cycle-diag
```

## TODO

- Investigate keyboard startup detection.
- Improve music playback performance.
- Complete keyboard support (СУ).
- БК-0011 support.
