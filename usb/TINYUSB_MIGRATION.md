# TinyUSB migration — experimental branch

This work is isolated on `experiment/tinyusb-device-migration`. Do not merge into
`main` until the board-level tests below pass.

## Required controller topology

- Root port 0: STM32F407 OTG_FS, internal FS PHY, USB host, HID keyboard.
- Root port 1: STM32F407 OTG_HS, external USB3300 ULPI PHY, USB device.
- Device functions: UVC camera and UAC2 audio.
- HS UVC endpoint: 768 bytes per transaction, three transactions per microframe.
  Its descriptor uses `wMaxPacketSize = 768 | (2 << 11)`.
- HS audio IN: 48 kHz, mono, PCM16, 12 bytes per microframe.

## Patches already staged

1. `dcd_dwc2.c`
   - Decode the HS endpoint transaction multiplier from `wMaxPacketSize[12:11]`.
   - Program `DIEPTSIZ.MC` for isochronous IN transfers using the actual
     packet count in this microframe. For a full UVC batch this is
     `XFRSIZ=2304`, `PKTCNT=3`, `MC=3`.
   - Reapply MC when an incomplete isochronous IN transfer is retried.
   - Reject ISO IN submissions that exceed the endpoint's advertised
     transactions-per-microframe, including the FIFO submission API.
   - Initialize EP0's implicit one-transaction limit after bus reset.
   - Reserve two max-packet slots in the HS ISO IN TX FIFO. RM0090 permits
     the third packet to be refilled after the first has gone onto the bus.
     The allocator still checks the shared 1024-word (4096-byte) DWC2 FIFO
     budget, including RX FIFO and DMA endpoint-info storage.

2. `video_device.c`
   - Compare the negotiated UVC payload size against the full HS isochronous
     service-interval capacity (MPS multiplied by the descriptor multiplier),
     not just the 768-byte per-transaction MPS.

3. `usb/tusb_config.h`
   - Defines port 0 as FS host and port 1 as HS device.
   - Enables internal DMA for the HS device controller.
   - Sets the UVC class staging buffer to 2304 bytes and the UAC2 IN staging
     buffer to 12 bytes; EP0 staging is 512 bytes.
   - Sets four-byte DMA alignment and compile-time guards against staging
     buffers larger than 4096 bytes.

4. The TinyUSB C sources are now compiled directly from the root `build.rs`, with
   bindings generated from `tusb.h` into `src/tusb.rs`. The `tinyusb-sys`
   Rust wrapper dependency has been removed from `Cargo.toml`; the checked-in
   TinyUSB 0.21.0 source remains in place.
5. Existing Rust-side DMA buffers are documented/guarded:
   - UVC producer buffers: two separate 2304-byte, 4-byte-aligned buffers.
   - UAC2 source packet: 12 bytes in a 4-byte-aligned persistent object.
   - EP0 allocator storage: 256-byte OUT and 64-byte IN regions.

## Important distinction: FIFO RAM vs DMA buffers

The STM32F407 OTG_HS has 1024 words (4096 bytes) of shared DWC2 FIFO RAM.
That is not the same memory as the SRAM buffers read by internal DMA.
The DWC2 FIFO allocator accounts for TX, RX, and DMA endpoint-info regions
and rejects allocations that exceed the hardware FIFO depth. All actual DMA
buffers must additionally be in DMA-accessible SRAM, not CCM, and remain
unchanged until transfer completion.

## Required validation before removing the old stack

1. Build the new direct TinyUSB integration. The source/config/build wiring is
   in place, but a successful compile has not yet been verified.
2. Initialize OTG_FS as host and OTG_HS as device simultaneously.
3. Verify the HS UVC descriptor advertises 768 x 3 and that DWC2 registers show
   `XFRSIZ=2304`, `PKTCNT=3`, `MC=3` for a full video microframe.
4. Verify the FIFO allocation succeeds within the 1024-word HS FIFO RAM.
5. Verify DMA buffer addresses are in SRAM (not CCM), 4-byte aligned, and no
   individual DMA transfer buffer exceeds 4096 bytes.
6. Only then port UVC/UAC2 class callbacks and remove the remaining three old Rust crates.

No hardware test or complete class migration has been claimed yet.
