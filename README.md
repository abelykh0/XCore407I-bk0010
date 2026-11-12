# БК-0010 Emulator

БК-0010 emulator for the Waveshare XCore407I board (STM32F407IGT6, 168 MHz).

- **Video:** USB UVC, uncompressed NV12, 25 fps.
- **Audio:** USB Line In.
- **Input:** USB keyboard connected to the board's host port.
- **Storage:** onboard 128 MB raw NAND, managed by Dhara and LittleFS.

[Video demo](https://www.youtube.com/watch?v=IPjBy6mAeOw)

## Hardware

The board uses an STM32F407IGT6 with 192 KB SRAM, a USB3300 high-speed PHY,
and a K9F1G08U0E 1-Gbit NAND chip.

Video is generated and streamed without storing a complete frame in RAM.
The firmware coordinates USB video, audio, keyboard input, and emulator timing.

## RTC registers

The STM32 calendar is exposed as read-only Unix milliseconds (low 32 bits):

| Octal address | Register |
| --- | --- |
| `177720` | `RTC_MILLIS_L` — bits 0–15 |
| `177722` | `RTC_MILLIS_H` — bits 16–31 |

Read the low word first; it captures a snapshot used by the following high-word
read. The value wraps every 49.7 days. RTC resolution is approximately 3.9 ms.

## NAND storage

Dhara is the only raw-NAND backend. It handles physical blocks, bad blocks,
and wear levelling; STM32 FSMC Hamming ECC protects page data. LittleFS uses
700 logical erase blocks (87.5 MiB).

If Dhara is valid but LittleFS cannot mount, the file browser offers a
two-step confirmation to format LittleFS. This deletes its files but preserves
the Dhara journal.

**Ctrl+Alt+F12** opens a two-step NAND reinitialization confirmation at any
time. Confirming erases all usable NAND blocks, creates a new Dhara volume,
and formats an empty LittleFS filesystem. Factory bad blocks are preserved.
All existing NAND data is lost. Cancel at either step to leave NAND unchanged.

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
- Complete keyboard support (СУ, АР2).
- Add file deletion and improve file management.
- БК-0011 support.
