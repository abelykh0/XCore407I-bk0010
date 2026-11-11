# БК-0010 Emulator

## Project Description

A **[БК-0010](https://en.wikipedia.org/wiki/Elektronika_BK) (Elektronika BK) emulator** running on the [XCore407I](https://www.waveshare.com/wiki/XCore407I) development board, based on the STM32F407IGT6 microcontroller. The project explores how far a resource-constrained microcontroller can be pushed to emulate a vintage computer while interfacing with a modern PC through standard multimedia and USB interfaces.

The project uses an unusual approach to multimedia I/O:

- **Video:** Streamed directly to the PC as a USB Video Class (UVC) camera at **25 fps**, using uncompressed NV12 video.
- **Audio:** Transmitted to the PC through a USB Line In audio device.
- **Input:** A physical USB keyboard connected to a separate USB port, using the STM32's USB Host functionality.
- **Storage:** Reads files from a LittleFS filesystem stored on the onboard K9F1G08U0E 1-Gbit (128 MB) NAND Flash. Files can be launched directly from the filesystem. Adding files to LittleFS is outside the scope of this project.

[Video](https://www.youtube.com/watch?v=IPjBy6mAeOw)

## Hardware

The XCore407I board provides the hardware required for the emulator:

- **STM32F407IGT6:** ARM Cortex-M4 microcontroller running at 168 MHz, with 192 KB SRAM and 1 MB Flash.
- **USB3300:** External USB High-Speed PHY, providing the physical interface for USB video and audio.
- **K9F1G08U0E:** 1-Gbit NAND Flash (128 MB) for LittleFS storage.
- **USB interfaces:** USB High-Speed device connection for UVC video and USB Line In audio, and a separate USB Full-Speed Host connection for the keyboard.

## Technical Challenges

- **RAM constraints:** The STM32F407 has only 192 KB of SRAM, insufficient to store a complete 512×512 NV12 video frame. Video must therefore be generated and streamed on the fly, without a full-frame buffer.
- **USB complexity:** Coordinating UVC video streaming and USB Line In audio over the High-Speed USB device connection, while handling keyboard input through a separate Full-Speed USB Host connection.
- **Performance:** Maintaining stable 25 fps video alongside continuous audio requires careful interrupt prioritization, synchronization, and CPU-load management.
- **NAND Flash integration:** Implementing reliable LittleFS access on raw NAND Flash, including handling its block-oriented storage characteristics.

## Emulator RTC I/O

The emulator exposes the STM32F407 calendar as unsigned 32-bit Unix time
(seconds since 1970-01-01 00:00:00 UTC) through two read-only BK I/O words:

| Octal address | Register | Value |
| --- | --- | --- |
| `177720` | `RTC_TIME_L` | Bits 0..15 |
| `177722` | `RTC_TIME_H` | Bits 16..31 |

Read `177720` first. That access captures the complete 32-bit timestamp;
the subsequent read of `177722` returns the high word from the same snapshot.
The snapshot is refreshed by the next read of `177720`. Byte reads follow
the same low-word-first rule. The registers are read-only.

## Commands to Build and Flash

```powershell
cargo build --release --target thumbv7em-none-eabihf
cargo build --release --features cpu-cycle-diag
probe-rs download --chip STM32F407IGTx target/thumbv7em-none-eabihf/release/xcore407i_blinker
```

## Not implemented / TODO

- Solve mystery with keyboard not working at start
- Improve performance when playing music
- Full keyboard support (СУ, АР2)
- Ability to save and delete files
- БК-0011 support
