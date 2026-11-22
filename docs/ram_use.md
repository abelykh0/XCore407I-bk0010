# RAM allocation by operating mode

This is a source-level inventory for the ethernet branch. Values are allocation sizes from declarations, not final ELF section totals. Alignment, linker placement, compiler retention, USB library state, and stack reservations affect actual usage.

STM32F407 memory: 128 KiB main SRAM (DMA-accessible) and 64 KiB CCM SRAM (not Ethernet-DMA-accessible).

## Shared backing regions and mode-dependent reuse

| Allocation | Region | Size | Emulator mode | CDC mode |
|---|---|---:|---|---|
| DEMO_CANVAS | Main SRAM, .bss | 84,224 B reserved | Shared backing storage; not all bytes are active framebuffer data | CDC receiver + Ethernet DMA ring + alternate stack; 28,624 B gap remains between allocations |
| DEMO_CANVAS_TAIL | CCM, .ccmram.canvas | 46,848 B reserved | Canvas-tail storage, partly unused or repurposed depending on BK display layout | CPU-only smoltcp socket storage and HTTP buffers |
| Combined | Main SRAM + CCM | 131,072 B reserved | Backing-store capacity; not necessarily all live video pixels | Same physical allocation reused, not allocated again |

Derived from camera.rs: CANVAS_BYTES = 512 × 256 = 131,072; CANVAS_TAIL_BYTES = 46,848; CANVAS_MAIN_BYTES = 84,224.

## Emulator / audio (main mode)

| Block | Region | Size | Purpose |
|---|---|---:|---|
| DEMO_CANVAS | Main SRAM | 84,224 B reserved | Shared backing region; only the active BK display / extended-RAM portions are used; the full allocation is not a continuously live framebuffer |
| DEMO_CANVAS_TAIL | CCM | 46,848 B reserved | Canvas-tail storage is not necessarily active in this mode; portions are unused or repurposed according to the BK display layout |
| VIDEO_BUFFERS[2] | Main SRAM, .bss | 4,608 B | Two UVC transport buffers, each 768 × 3 = 2,304 B |
| ROW_BUFFERS | Main SRAM, .bss | 1,024 B | Two 512-byte row-conversion scratch buffers |
| RamBuffer | CCM, .ccmram | 16,384 B | BK-0010 emulated RAM |
| BK audio edge arrays | Main SRAM, .bss | 2,560 B | 512 32-bit cycle timestamps + 512 8-bit levels |
| USB host/device state | Various | Not measured | TinyUSB state, descriptors, endpoint buffers, host state and class state |
| Emulator/audio/keyboard state | Various | Not measured | Other C and Rust globals, queues and state |

The audio edge arrays are declared in csrc/bk/platform.c. The 2,560-byte figure excludes other audio state and alignment. The two canvas figures above are linker-reserved capacities, not proof that every byte is actively used by the emulator. In particular, the backing store is shared with the BK extended-memory layout; the active framebuffer extent depends on the selected BK display mode. Do not count the entire 131,072-byte reservation as live emulator video data.

## CDC mode — UVC static page + CDC + Ethernet HTTP

CDC mode does not run the emulator. It reuses the same canvas regions above. The receiver, Ethernet DMA ring, and alternate stack occupy disjoint ranges in DEMO_CANVAS; the CCM tail is reused for CPU-only TCP and HTTP storage.

| Block | Region | Size | Purpose |
|---|---|---:|---|
| CDC receiver | DEMO_CANVAS, offset 0 | 4,096 B | Heapless ZMODEM receive storage |
| Ethernet DMA ring | DEMO_CANVAS, offset 4,096 | 4,656 B | Two RX descriptors + buffers and one TX descriptor + buffer; DMA-accessible main SRAM |
| CDC alternate stack | DEMO_CANVAS, top 46,848 B | 46,848 B | Stack grows down from the end of the main canvas block |
| Ethernet socket storage | DEMO_CANVAS_TAIL, CCM | Build-dependent | Two smoltcp SocketStorage entries |
| HTTP TCP RX | DEMO_CANVAS_TAIL, CCM | 768 B | smoltcp TCP receive buffer |
| HTTP TCP TX | DEMO_CANVAS_TAIL, CCM | 2,304 B | smoltcp TCP transmit buffer |
| HTTP body | DEMO_CANVAS_TAIL, CCM | 768 B | Upload chunk and generated response body scratch |
| VIDEO_BUFFERS[2] | Main SRAM, .bss | 4,608 B | UVC transport buffers for low-rate PC CONNECT static page |
| ROW_BUFFERS | Main SRAM, .bss | 1,024 B | Camera row-conversion scratch |
| CDC receiver | Inside DEMO_CANVAS | 4,096 B claimed | Heapless ZMODEM receive storage; do not count twice |
| Alternate filesystem stack | Inside DEMO_CANVAS | 46,848 B reserved | Stack; do not count twice |
| NAND_PAGE_SCRATCH | Main SRAM, .bss | 2,112 B | 2,048-byte page + 64-byte spare |
| LittleFS mount allocation | Static storage | Build-dependent | MaybeUninit<LittleFsAllocation>; exact size requires ELF/map or size_of |
| Dhara C page buffer | Main SRAM, .bss | 2,048 B | One NAND page in csrc/dhara_adapter.c |
| ZMODEM send buffer | State/stack | 1,024 B | Embedded in CDC ZMODEM state; receiver payload is the separate 4 KiB canvas allocation |

All Ethernet packet, descriptor, socket, and HTTP buffers are now carved from the two reusable canvas regions. The DMA ring occupies 4,656 bytes in DEMO_CANVAS, after the 4 KiB CDC receiver. The TCP socket storage and 3,840 bytes of HTTP RX/TX/body buffers occupy DEMO_CANVAS_TAIL in CCM. No Ethernet/TCP/HTTP packet buffers remain as independent .bss statics. The small HTTP response-header and upload-name scratch arrays are automatic locals on the CDC alternate stack, which is also inside DEMO_CANVAS.

## Source-verifiable allocation subtotals

| Item | Size |
|---|---:|
| Large canvas reservation (main + CCM) | 131,072 B |
| UVC transport double buffer | 4,608 B |
| Row scratch | 1,024 B |
| HTTP RX + TX + body | 3,840 B |
| Ethernet RX + TX packet buffers | 4,608 B |
| Ethernet DMA descriptors | 48 B |
| NAND page scratch | 2,112 B |
| Dhara C page buffer | 2,048 B |
| BK emulator RAM | 16,384 B |
| BK audio edge arrays | 2,560 B |

These values are not summed into per-mode totals: some are mutually exclusive, some are embedded in reused canvas regions, and others omit build-dependent state, USB allocations, alignment, and stack usage.

## Implications for the RAM reallocation

The CDC layout now partitions the two regions by hardware access and lifetime. DEMO_CANVAS holds the 4 KiB receiver at offset 0, the 4,656-byte DMA ring immediately after it, and the 46,848-byte alternate stack at the top. A compile-time assertion verifies that the receiver and DMA ring do not overlap the stack reservation. DEMO_CANVAS_TAIL holds the CPU-only smoltcp socket storage and HTTP buffers. The stack was moved out of CCM before using the tail for HTTP storage, preventing the stack from overwriting live TCP buffers. Ethernet DMA descriptors and packet buffers remain in DMA-accessible main SRAM.

To explain the USB enumeration regression conclusively, inspect the actual ELF/map output for .data, .bss, .ccmram, USB endpoint buffers, and stack reservations. This source inventory alone does not establish remaining free SRAM or prove the root cause.