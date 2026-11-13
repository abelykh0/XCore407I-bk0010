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

The camera image is rendered as uncompressed NV12 and packetized as UVC 1.1.
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

If Dhara is valid but LittleFS cannot mount, the file browser offers a
two-step confirmation to format LittleFS. This deletes its files but preserves
the Dhara journal.

The manual **Ctrl+Alt+F12** NAND reinitialization shortcut is compiled only
with the `destructive-nand-init` feature. It opens a two-step confirmation;
confirming erases usable NAND blocks, creates a Dhara volume, and formats an
empty LittleFS filesystem. Factory bad blocks are preserved. All NAND data
is lost. Without this feature, the shortcut is unavailable.

## RTC registers

The STM32 calendar is exposed as read-only Unix milliseconds (low 32 bits):

| Octal address | Register |
| --- | --- |
| `177720` | `RTC_MILLIS_L` — bits 0–15 |
| `177722` | `RTC_MILLIS_H` — bits 16–31 |

Read the low word first; it captures a snapshot used by the following high-word
read. The value wraps every 49.7 days. RTC resolution is approximately 3.9 ms.

## Third-party libraries

- **TinyUSB 0.21.0** — USB device and host stack, including UVC transport and
  UAC2 audio.
- **Dhara** — raw-NAND flash translation layer.
- **LittleFS 2** (through `littlefs2` / `littlefs2-sys`) — filesystem.
- **STM32F4 PAC and HAL** (`stm32f4`, `stm32f4xx-hal`) — STM32F407 register
  access and hardware abstraction.
- **Cortex-M support** (`cortex-m`, `cortex-m-rt`) — Cortex-M runtime and
  interrupt support.
- **CMSIS Core and STM32F4 device headers** — processor and device definitions
  used when compiling TinyUSB.
- **Build dependencies:** `cc`, `bindgen`, and `glob` — compile C sources,
  generate Rust bindings, and enumerate TinyUSB sources.

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
