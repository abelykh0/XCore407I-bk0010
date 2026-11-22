# RAM allocation by operating mode

This is a source-level inventory for the ethernet branch. Values are allocation sizes from declarations, not final ELF section totals. Alignment, linker placement, compiler retention, USB library state, and stack reservations affect actual usage.

STM32F407 memory: 128 KiB main SRAM (DMA-accessible) and 64 KiB CCM SRAM (not Ethernet-DMA-accessible).

## Shared large canvas regions

| Allocation | Region | Size | Emulator mode | CDC mode |
|---|---|---:|---|---|
| DEMO_CANVAS | Main SRAM, .bss | 84,224 B | First canvas segment | Reused as CDC receiver backing region; currently 4,096 B is claimed, remainder unused by receiver |
| DEMO_CANVAS_TAIL | CCM, .ccmram.canvas | 46,848 B | Second canvas segment | Reused as alternate stack for synchronous filesystem operations |
| Combined | Main SRAM + CCM | 131,072 B | Full 512 × 256 canvas | Same physical allocation reused, not allocated again |

Derived from camera.rs: CANVAS_BYTES = 512 × 256 = 131,072; CANVAS_TAIL_BYTES = 46,848; CANVAS_MAIN_BYTES = 84,224.

## Mode 1 — emulator + UVC + audio + keyboard host

| Block | Region | Size | Purpose |
|---|---|---:|---|
| DEMO_CANVAS | Main SRAM | 84,224 B | Emulator video canvas, first segment |
| DEMO_CANVAS_TAIL | CCM | 46,848 B | Emulator video canvas, second segment |
| VIDEO_BUFFERS[2] | Main SRAM, .bss | 4,608 B | Two UVC transport buffers, each 768 × 3 = 2,304 B |
| ROW_BUFFERS | Main SRAM, .bss | 1,024 B | Two 512-byte row-conversion scratch buffers |
| RamBuffer | CCM, .ccmram | 16,384 B | BK-0010 emulated RAM |
| BK audio edge arrays | Main SRAM, .bss | 2,560 B | 512 32-bit cycle timestamps + 512 8-bit levels |
| USB host/device state | Various | Not measured | TinyUSB state, descriptors, endpoint buffers, host state and class state |
| Emulator/audio/keyboard state | Various | Not measured | Other C and Rust globals, queues and state |

The audio edge arrays are declared in csrc/bk/platform.c. The 2,560-byte figure excludes other audio state and alignment.

## Mode 2 — CDC + UVC static page + Ethernet HTTP

CDC mode does not run the emulator. It reuses the same canvas regions above. The receiver allocation is inside DEMO_CANVAS, not an additional allocation. The alternate stack occupies DEMO_CANVAS_TAIL.

| Block | Region | Size | Purpose |
|---|---|---:|---|
| DEMO_CANVAS (reused) | Main SRAM | 84,224 B reserved | CDC receiver claims 4,096 B; remaining area is available for other mode-specific CPU-only buffers |
| DEMO_CANVAS_TAIL (reused) | CCM | 46,848 B reserved | Alternate filesystem stack |
| VIDEO_BUFFERS[2] | Main SRAM, .bss | 4,608 B | UVC transport buffers for low-rate PC CONNECT static page |
| ROW_BUFFERS | Main SRAM, .bss | 1,024 B | Camera row-conversion scratch |
| HTTP TCP RX | Main SRAM, .bss | 768 B | smoltcp TCP receive buffer |
| HTTP TCP TX | Main SRAM, .bss | 2,304 B | smoltcp TCP transmit buffer |
| HTTP body | Main SRAM, .bss | 768 B | Upload chunk and generated response body scratch |
| Ethernet RX packet buffers | Main SRAM, .bss | 3,072 B | Two DMA buffers × 1,536 B |
| Ethernet TX packet buffer | Main SRAM, .bss | 1,536 B | One DMA buffer |
| Ethernet descriptors | Main SRAM, .bss | 48 B | Two RX + one TX descriptors, 16 B each |
| Ethernet socket storage | Main SRAM, .bss | Build-dependent | Two smoltcp SocketStorage entries |
| CDC receiver | Inside DEMO_CANVAS | 4,096 B claimed | Heapless ZMODEM receive storage; do not count twice |
| Alternate filesystem stack | Inside DEMO_CANVAS_TAIL | 46,848 B reserved | Stack; do not count twice |
| NAND_PAGE_SCRATCH | Main SRAM, .bss | 2,112 B | 2,048-byte page + 64-byte spare |
| LittleFS mount allocation | Static storage | Build-dependent | MaybeUninit<LittleFsAllocation>; exact size requires ELF/map or size_of |
| Dhara C page buffer | Main SRAM, .bss | 2,048 B | One NAND page in csrc/dhara_adapter.c |
| ZMODEM send buffer | State/stack | 1,024 B | Embedded in CDC ZMODEM state; receiver payload is the separate 4 KiB canvas allocation |

Current HTTP buffers total 3,840 bytes (768 + 2,304 + 768) and are independent .bss statics. They have not yet been relocated into the reusable canvas blocks. Ethernet DMA descriptors and packet buffers must remain in DMA-accessible main SRAM, not CCM.

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

The CDC layout should place mode-specific CPU-only allocations (especially HTTP RX/TX/body buffers) inside the two existing canvas regions instead of adding independent .bss allocations. The current 4 KiB CDC receiver uses only a small portion of the 84,224-byte main canvas block, so the unused portion is a candidate for HTTP storage. The CCM tail is occupied by the alternate stack during filesystem work; do not place live HTTP buffers there without explicitly managing stack overlap and lifetime. Keep Ethernet DMA descriptors and packet buffers in main SRAM.

To explain the USB enumeration regression conclusively, inspect the actual ELF/map output for .data, .bss, .ccmram, USB endpoint buffers, and stack reservations. This source inventory alone does not establish remaining free SRAM or prove the root cause.