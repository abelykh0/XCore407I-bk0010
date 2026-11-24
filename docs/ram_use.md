# RAM allocation by operating mode

This document combines a source-level inventory with a linker-map snapshot. Source-level values describe declared allocations; linker-map values describe the actual linked release image and include compiler retention and linker alignment. Runtime stack high-water usage is not measured by the map.

STM32F407 memory: 128 KiB main SRAM (DMA-accessible) and 64 KiB CCM SRAM (not Ethernet-DMA-accessible).

## Linker-map snapshot

The uploaded `firmware.map` was generated from a release build for `thumbv7em-none-eabihf`. It confirms the following static allocation:

| Region / section | Start | End (exclusive) | Size | Capacity / remaining |
|---|---:|---:|---:|---:|
| Flash image (vector table, text, rodata, data load image) | `0x08000000` | `0x0805370C` | 341,772 B (333.76 KiB) | 1 MiB / 706,804 B free |
| Main SRAM `.data` | `0x20000000` | `0x20001454` | 5,204 B | |
| Main SRAM `.bss` | `0x20001458` | `0x2001C364` | 110,348 B | |
| Main SRAM static total | `0x20000000` | `0x2001C364` | 115,552 B (112.84 KiB) | 128 KiB / 15,516 B free |
| CCM `.ccmram` | `0x10000000` | `0x1000FF00` | 65,280 B (63.75 KiB) | 64 KiB / 256 B free |

**RAM usage:** 180,832 bytes statically allocated: 115,552 B in main SRAM and 65,280 B in CCM. This leaves 15,516 B in main SRAM and 256 B in CCM for runtime use. The STM32F407 supports a CCM stack, but its placement depends on the linker configuration. CCM is not DMA-accessible.

Flash section sizes: `.vector_table` 392 B, `.text` 280,956 B, `.rodata` 55,208 B, and the `.data` load image 5,204 B. The linked image ends at `0x0805370C`; this includes linker alignment.

CCM is nearly full. Its `.ccmram` output section contains:
- `VOLUME_MARKER_BUFFER`: 2,048 B at `0x10000000`.
- `DEMO_CANVAS_TAIL`: 46,848 B at `0x10000800`.
- `RamBuffer`: 16,384 B at `0x1000BF00`.

These allocations total 65,280 B, leaving 256 B in the 64 KiB CCM region. CCM is not accessible to Ethernet DMA.

The map confirms `DEMO_CANVAS` is 84,224 B in main SRAM, `DEMO_CANVAS_TAIL` is 46,848 B in CCM, and together they form the 131,072-byte logical canvas backing store. It also confirms `VIDEO_BUFFERS` is 4,608 B, `ROW_BUFFERS` is 1,024 B, `NAND_PAGE_SCRATCH` is 2,112 B, and `LITTLEFS_MOUNT_ALLOCATION` is 4,568 B.

These are link-time static figures for this particular build. They do not include runtime stack depth, transient call frames, or any runtime allocation not represented as a static linker section. Rebuild the map after changes that affect features, source, compiler, or linker configuration.

## Shared backing regions and mode-dependent reuse

| Allocation | Region | Size | Emulator mode | CDC mode |
|---|---|---:|---|---|
| DEMO_CANVAS | Main SRAM, .bss | 84,224 B reserved | First physical segment of the logical BK canvas backing store; during audio diagnostics this is part of the history ring | CDC receiver + Ethernet DMA ring + HTTP buffers + alternate stack |
| DEMO_CANVAS_TAIL | CCM, .ccmram.canvas | 46,848 B reserved | Second segment of the logical BK canvas; also holds audio history during capture | Reused for CPU-only smoltcp socket metadata + HTTP TCP TX buffer |
| Combined | Main SRAM + CCM | 131,072 B reserved | One logical 512 × 256-byte BK canvas backing store split across two memory regions; not a full 512 × 512 NV12 frame | Same physical allocation reused, not allocated again |

Derived from camera.rs: CANVAS_BYTES = 512 × 256 = 131,072; CANVAS_TAIL_BYTES = 46,848; CANVAS_MAIN_BYTES = 84,224.

## Common allocations (both modes)

These allocations are used by both emulator/audio and CDC modes. Their combined size is shown in each mode table as **Common**; details are listed here once.

| Block | Region | Size | Purpose |
|---|---|---:|---|
| VIDEO_BUFFERS[2] | Main SRAM, .bss | 4,608 B | Two UVC transport buffers, each 768 × 3 = 2,304 B |
| ROW_BUFFER | Main SRAM, .bss | 512 B | One 512-byte row-conversion scratch buffer |
| Dhara C page buffer | Main SRAM, .bss | 2,048 B | NAND page buffer used by the Dhara storage layer |
| LittleFS mount allocation | Main SRAM, .bss | 4,568 B | Read cache (2,048 B), program cache (2,048 B), lookahead buffer (32 B), and remaining filesystem/configuration storage and alignment |

LittleFS `metadata_max` is configured to **8 KiB**. This is a limit on metadata-pair size, not an 8 KiB RAM buffer, so it does not add 8 KiB to the static RAM allocation shown above.

## Emulator / audio (main mode)

| Block | Region | Size | Purpose |
|---|---|---:|---|
| Common | Main SRAM, .bss | 11,736 B | See Common allocations section |
| DEMO_CANVAS | Main SRAM | 84,224 B reserved | First segment of the shared BK canvas backing store |
| DEMO_CANVAS_TAIL | CCM | 46,848 B reserved | Second segment of the same canvas backing store; together these are one logical 128 KiB allocation, not a complete UVC frame |
| RamBuffer | CCM, .ccmram | 16,384 B | BK-0010 emulated RAM |
| BK audio edge queue arrays | Main SRAM, .bss | 2,560 B | 512 32-bit timestamps + 512 8-bit levels for the live audio queue |
| BK audio edge history | Existing canvas (main SRAM + CCM) | 0 B additional | Up to 16,384 eight-byte timestamp/level/drop records; temporarily replaces the canvas while capturing music-only programs |
| F2 browser file table | Main SRAM, .bss | 990 B | 30 entries × 33 B (`BrowserFile` path + length); shared with CDC mode |
| USB host/device state | Various | Not measured | TinyUSB state, descriptors, endpoint buffers, host state and class state |
| Emulator/audio/keyboard state | Various | Not measured | Other C and Rust globals, queues and state |

The common allocations are listed above. The shared F2 browser table holds 30 root-level `.bin` paths (990 B); its 32 display rows comprise a title, 30 file entries, and controls. BK audio edge history reuses the existing 128 KiB canvas, adding no static RAM. It retains up to 16,384 eight-byte records (timestamps, level, and dropped flag), equivalent to 131,072 bytes of storage. The canvas spans main SRAM and CCM; UVC images are generated and packetized on the fly, without retaining a complete 512 × 512 NV12 frame.

## CDC mode — UVC static page + CDC + Ethernet HTTP

CDC mode reuses the canvas allocation: `DEMO_CANVAS` holds shared ZMODEM receiver/send storage, the DMA ring, HTTP buffers, and the 8 KiB alternate stack reservation; `DEMO_CANVAS_TAIL` holds CPU-only socket metadata and the TCP TX buffer.

| Block | Region | Size | Purpose |
|---|---|---:|---|
| Common | Main SRAM, .bss | 11,736 B | See Common allocations section |
| Shared ZMODEM storage | DEMO_CANVAS, offset 0 | 4,096 B | Receiver state during uploads; send staging uses the first 1 KiB during downloads |
| CDC alternate stack reservation | DEMO_CANVAS, top 8,960 B | 8,960 B | Includes the 8 KiB alternate stack and its additional reserved space |
| ZMODEM send staging | Inside shared ZMODEM storage | 1,024 B | Reuses the same canvas allocation; no separate send buffer in `State` |
| Alternate filesystem stack (accounting note) | Inside DEMO_CANVAS | 8,960 B reserved | Same alternate-stack reservation; do not count twice |
| Ethernet DMA ring | DEMO_CANVAS, offset 4,096 | 4,656 B | Two RX descriptors + buffers and one TX descriptor + buffer; DMA-accessible main SRAM |
| Ethernet socket storage | DEMO_CANVAS_TAIL, CCM, offset 0 | Build-dependent | Two smoltcp SocketStorage entries; CPU-only metadata |
| HTTP TCP TX | DEMO_CANVAS_TAIL, immediately after socket storage | Remaining tail, about 46 KiB | smoltcp TCP transmit buffer; CPU-accessed, not DMA |
| HTTP TCP RX | DEMO_CANVAS, offset 8,752 | 4,096 B | smoltcp TCP receive buffer |
| HTTP body | DEMO_CANVAS, offset 12,848 | 62,416 B | Upload chunk and generated response body scratch |
| NAND_PAGE_SCRATCH | Main SRAM, .bss | 2,112 B | 2,048-byte page + 64-byte spare |

Ethernet and HTTP buffers are carved from the canvas. The 4 KiB ZMODEM storage is shared between receiver state and send staging; only one transfer direction is active at a time. The DMA ring is in DMA-accessible main SRAM; HTTP RX (4 KiB) and body storage (62,416 B) fill the space before the alternate-stack reservation. CCM holds socket metadata and the TCP TX buffer. Directory responses support 128 entries; upload and browser chunks remain 2 KiB. HTTP header/name scratch arrays are locals on the alternate stack.

## Source-verifiable allocation subtotals

| Item | Size |
|---|---:|
| Large canvas reservation (main + CCM) | 131,072 B |
| UVC transport double buffer | 4,608 B |
| Row scratch | 512 B |
| HTTP RX + body in main SRAM | 66,512 B |
| Ethernet RX + TX packet buffers | 4,608 B |
| Ethernet DMA descriptors | 48 B |
| NAND page scratch | 2,112 B |
| Dhara C page buffer | 2,048 B |
| BK emulator RAM | 16,384 B |
| BK audio edge queue arrays | 2,560 B |
| BK audio edge history (canvas reuse) | 0 B additional |
| F2 browser file table (30 entries) | 990 B |

These values are not summed into per-mode totals: some are mutually exclusive, some are embedded in reused canvas regions, and others omit build-dependent state, USB allocations, alignment, and stack usage.

## Implications for the RAM reallocation

In emulator mode, DEMO_CANVAS and DEMO_CANVAS_TAIL are two physical segments of one logical BK canvas backing store. The UVC frame is packetized/generated on the fly; the firmware does not hold a complete 512 × 512 NV12 frame in RAM. In CDC mode, that same 131,072-byte backing allocation is reused. DEMO_CANVAS holds, in order, shared ZMODEM storage (4,096 B), DMA ring (4,656 B), HTTP RX/body storage (66,512 B), and alternate-stack reservation (8,960 B). DEMO_CANVAS_TAIL holds CPU-only smoltcp socket metadata followed by the TCP TX buffer, which consumes the remaining CCM tail. Compile-time assertions verify the main-SRAM HTTP arena ends exactly at the reserved alternate-stack boundary and that the socket metadata plus TCP TX fit in CCM.

The linker map now provides the static RAM baseline for this release build: 115,552 B in main SRAM and 65,280 B in CCM. It does not establish runtime stack high-water usage or, by itself, identify the cause of a USB enumeration failure.