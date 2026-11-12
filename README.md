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

The emulator exposes the STM32F407 calendar as the **low 32 bits of Unix
milliseconds** (milliseconds since 1970-01-01 00:00:00 UTC) through two
read-only BK I/O words:

| Octal address | Register | Value |
| --- | --- | --- |
| `177720` | `RTC_MILLIS_L` | Bits 0..15 |
| `177722` | `RTC_MILLIS_H` | Bits 16..31 |

Read `177720` first. That access captures the complete 32-bit millisecond
value; the subsequent read of `177722` returns the high word from the same
snapshot. The snapshot is refreshed by the next read of `177720`. Byte reads
follow the same low-word-first rule. The registers are read-only.

The 32-bit value wraps every 49.7 days. Subtracting two readings gives the
elapsed milliseconds correctly across a wrap for intervals shorter than that.
The RTC is configured for 256 subsecond ticks per second, so the timestamp
resolution is approximately 3.9 ms (not 1 ms).

## Dhara-backed LittleFS

Dhara is the only NAND storage backend. It manages all physical blocks,
including bad-block handling and wear levelling, while the existing STM32 FSMC
Hamming ECC protects page data. LittleFS sees 700 logical erase blocks
(87.5 MiB). A private Dhara sector outside the LittleFS range identifies the
formatted volume.

The NAND has already been provisioned for this layout. There is no alternate
raw-NAND filesystem backend and no firmware feature for erasing or reformatting
the entire NAND. If the Dhara journal or volume marker is missing or invalid,
initialization fails safely and leaves NAND unchanged.

## LittleFS Formatting and NAND Provisioning

If LittleFS cannot be mounted but Dhara initializes successfully, the file
browser offers a two-step, explicit confirmation to format the LittleFS
volume. Formatting deletes all files in LittleFS but preserves Dhara's
journal and the private volume marker. After formatting, reopen the file
browser with F2.

This operation formats **only the existing LittleFS volume**. It does not
initialize or format raw NAND, create a Dhara journal, or recover a missing
Dhara volume.

On a new or unprovisioned NAND, opening the file browser detects the
missing Dhara journal and offers a separate two-step **NAND initialization**
confirmation. Confirming it erases every usable physical NAND block (factory
bad blocks are preserved), creates a fresh Dhara journal and private volume
marker, then formats and verifies an empty LittleFS volume. This destroys all
pre-existing data on the NAND and must only be used when that data can be lost.

The initialization prompt is not automatic: declining it leaves NAND
unchanged. A valid existing Dhara journal and marker are resumed normally;
they are never reinitialized just because LittleFS is corrupt. In that case,
use the separate LittleFS-only format confirmation above.

## Commands to Build and Flash

```powershell
cargo build --release --target thumbv7em-none-eabihf
cargo build --release --target thumbv7em-none-eabihf --features cpu-cycle-diag
probe-rs download --chip STM32F407IGTx target/thumbv7em-none-eabihf/release/xcore407i_blinker
```

## Not implemented / TODO

- Solve mystery with keyboard not working at start
- Improve performance when playing music
- Full keyboard support (СУ, АР2)
- Ability to save and delete files
- БК-0011 support
