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
- Complete keyboard support (СУ).
- БК-0011 support.
