# RAM allocation by operating mode

This is a source-level inventory for the ethernet branch. Values are allocation sizes from declarations, not final ELF section totals. Alignment, linker placement, compiler retention, USB library state, and stack reservations affect actual usage.

STM32F407 memory: 128 KiB main SRAM (DMA-accessible) and 64 KiB CCM SRAM (not Ethernet-DMA-accessible).

## Shared backing regions and mode-dependent reuse

| Allocation | Region | Size | Emulator mode | CDC mode |
|---|---|---:|---|---|
| DEMO_CANVAS | Main SRAM, .bss | 84,224 B reserved | First physical segment of the logical BK canvas backing store | CDC receiver + Ethernet DMA ring + HTTP buffers + alternate stack |
| DEMO_CANVAS_TAIL | CCM, .ccmram.canvas | 46,848 B reserved | Second physical segment of the same logical BK canvas backing store | Reused for CPU-only smoltcp socket metadata + HTTP TCP TX buffer |
| Combined | Main SRAM + CCM | 131,072 B reserved | One logical 512 × 256-byte BK canvas backing store split across two memory regions; not a full 512 × 512 NV12 frame | Same physical allocation reused, not allocated again |

Derived from camera.rs: CANVAS_BYTES = 512 × 256 = 131,072; CANVAS_TAIL_BYTES = 46,848; CANVAS_MAIN_BYTES = 84,224.

## Emulator / audio (main mode)

| Block | Region | Size | Purpose |
|---|---|---:|---|
| DEMO_CANVAS | Main SRAM | 84,224 B reserved | First segment of the shared BK canvas backing store |
| DEMO_CANVAS_TAIL | CCM | 46,848 B reserved | Second segment of the same canvas backing store; together these are one logical 128 KiB allocation, not a complete UVC frame |
| VIDEO_BUFFERS[2] | Main SRAM, .bss | 4,608 B | Two UVC transport buffers, each 768 × 3 = 2,304 B |
| ROW_BUFFERS | Main SRAM, .bss | 1,024 B | Two 512-byte row-conversion scratch buffers |
| RamBuffer | CCM, .ccmram | 16,384 B | BK-0010 emulated RAM |
| BK audio edge arrays | Main SRAM, .bss | 2,560 B | 512 32-bit cycle timestamps + 512 8-bit levels |
| USB host/device state | Various | Not measured | TinyUSB state, descriptors, endpoint buffers, host state and class state |
| Emulator/audio/keyboard state | Various | Not measured | Other C and Rust globals, queues and state |

The audio edge arrays are declared in csrc/bk/platform.c. The 2,560-byte figure excludes other audio state and alignment. DEMO_CANVAS and DEMO_CANVAS_TAIL are two physical segments of the same logical canvas backing store; the split reflects the available main-SRAM and CCM regions. The UVC image is generated and packetized from this backing store on the fly. The firmware does not allocate or retain a complete 512 × 512 NV12 frame (384 KiB) in RAM.

## CDC mode — UVC static page + CDC + Ethernet HTTP

CDC mode does not run the emulator. It reuses the same physical backing-store allocation. DEMO_CANVAS holds the receiver, DMA ring, HTTP buffers and alternate stack in disjoint ranges. DEMO_CANVAS_TAIL holds CPU-only smoltcp socket metadata.

| Block | Region | Size | Purpose |
|---|---|---:|---|
| CDC receiver | DEMO_CANVAS, offset 0 | 4,096 B | Heapless ZMODEM receive storage |
| CDC alternate stack | DEMO_CANVAS, top 8,192 B | 8,192 B | Stack grows down from the end of the main canvas block |
| CDC receiver (accounting note) | Inside DEMO_CANVAS | 4,096 B claimed | Same receiver allocation; do not count twice |
| Alternate filesystem stack (accounting note) | Inside DEMO_CANVAS | 8,192 B reserved | Same alternate stack allocation; do not count twice |
| Ethernet DMA ring | DEMO_CANVAS, offset 4,096 | 4,656 B | Two RX descriptors + buffers and one TX descriptor + buffer; DMA-accessible main SRAM |
| Ethernet socket storage | DEMO_CANVAS_TAIL, CCM, offset 0 | Build-dependent | Two smoltcp SocketStorage entries; CPU-only metadata |
| HTTP TCP TX | DEMO_CANVAS_TAIL, immediately after socket storage | Remaining tail, about 46 KiB | smoltcp TCP transmit buffer; CPU-accessed, not DMA |
| HTTP TCP RX | DEMO_CANVAS, offset 8,752 | 4,096 B | smoltcp TCP receive buffer |
| HTTP body | DEMO_CANVAS, offset 12,848 | 63,184 B | Upload chunk and generated response body scratch |
| VIDEO_BUFFERS[2] | Main SRAM, .bss | 4,608 B | UVC transport buffers for low-rate PC CONNECT static page |
| ROW_BUFFERS | Main SRAM, .bss | 1,024 B | Camera row-conversion scratch |
| NAND_PAGE_SCRATCH | Main SRAM, .bss | 2,112 B | 2,048-byte page + 64-byte spare |
| LittleFS mount allocation | Static storage | Build-dependent | MaybeUninit<LittleFsAllocation>; exact size requires ELF/map or size_of |
| Dhara C page buffer | Main SRAM, .bss | 2,048 B | One NAND page in csrc/dhara_adapter.c |
| ZMODEM send buffer | State/stack | 1,024 B | Embedded in CDC ZMODEM state; receiver payload is the separate 4 KiB canvas allocation |

All Ethernet packet, descriptor, socket and HTTP buffers are carved from the reusable canvas backing store. The DMA ring remains in DMA-accessible main SRAM. HTTP RX and body storage use the entire 28,624-byte gap between the DMA ring and alternate stack: 4 KiB RX and 24,528 B body. The TCP TX buffer was moved into CCM immediately after the socket metadata and now uses the remaining tail (about 46 KiB). The directory response page size is 128 entries instead of 3. The upload chunk size and browser chunk size remain 2 KiB. No Ethernet/TCP/HTTP buffers remain as independent .bss statics. The small HTTP response-header and upload-name scratch arrays are automatic locals on the CDC alternate stack.

## Source-verifiable allocation subtotals

| Item | Size |
|---|---:|
| Large canvas reservation (main + CCM) | 131,072 B |
| UVC transport double buffer | 4,608 B |
| Row scratch | 1,024 B |
| HTTP RX + body in main SRAM | 28,624 B |
| Ethernet RX + TX packet buffers | 4,608 B |
| Ethernet DMA descriptors | 48 B |
| NAND page scratch | 2,112 B |
| Dhara C page buffer | 2,048 B |
| BK emulator RAM | 16,384 B |
| BK audio edge arrays | 2,560 B |

These values are not summed into per-mode totals: some are mutually exclusive, some are embedded in reused canvas regions, and others omit build-dependent state, USB allocations, alignment, and stack usage.

## Implications for the RAM reallocation

In emulator mode, DEMO_CANVAS and DEMO_CANVAS_TAIL are two physical segments of one logical BK canvas backing store. The UVC frame is packetized/generated on the fly; the firmware does not hold a complete 512 × 512 NV12 frame in RAM. In CDC mode, that same 131,072-byte backing allocation is reused. DEMO_CANVAS holds, in order, the receiver (4,096 B), DMA ring (4,656 B), HTTP RX/body storage (67,280 B), and alternate stack (8,192 B). DEMO_CANVAS_TAIL holds CPU-only smoltcp socket metadata followed by the TCP TX buffer, which consumes the remaining CCM tail. Compile-time assertions verify the main-SRAM HTTP arena ends exactly at the reserved 8 KiB stack boundary and that the socket metadata plus TCP TX fit in CCM.

To explain the USB enumeration regression conclusively, inspect the actual ELF/map output for .data, .bss, .ccmram, USB endpoint buffers, and stack reservations. This source inventory alone does not establish remaining free SRAM or prove the root cause.