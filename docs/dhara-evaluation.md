# Dhara evaluation (experimental)

This branch evaluates [Dhara](https://github.com/dlbeer/dhara), a small NAND
flash translation layer, as the physical-management layer beneath LittleFS.

## Safety boundary

The `dhara-eval` Cargo feature currently **only compiles the vendored Dhara
library**. No Dhara instance is created, no NAND callbacks are connected, and
no Dhara API is called at runtime. The existing LittleFS path remains active.

Do not call `dhara_map_resume()`, `dhara_map_clear()`, or any write/erase
operation against the board's current NAND volume during this evaluation.
Dhara uses its own on-flash journal format. It cannot adopt the existing raw
LittleFS block layout; a future migration would require an explicit backup and
restore procedure.

The firmware now records the factory-good physical blocks remaining after
LittleFS's 1,008-block mapping and the two mapping-metadata blocks. Under
`dhara-eval`, `NandStorage::dhara_test_blocks()` exposes only this isolated
tail region. The list is read-only inventory at this stage: Dhara is not yet
connected, and no blocks in this region are erased or programmed by the new
code. The region contains at most 14 blocks (1.75 MiB); factory-bad blocks
reduce that count.

## Hardware fit

The current Samsung K9F1G08U0E driver reports:

- 2,048-byte data pages
- 64 pages per 128 KiB erase block
- 1,024 physical erase blocks
- OOB bytes 0..1 reserved for factory bad-block markers
- OOB bytes 2..15 used by the current four-chunk FSMC Hamming ECC scheme

Dhara requires a power-of-two page count per block, so this device fits:
`log2_page_size = 11`, `log2_ppb = 6`. Dhara does not consume OOB bytes.
Its NAND backend must preserve the factory marker positions and perform
ECC-corrected reads. The current ECC implementation can be reused for full
page reads; a partial-read callback must still ECC-check the page sufficiently
to detect uncorrectable errors.

## Proposed layering

```text
LittleFS
  Storage adapter (2 KiB page reads/writes; 128 KiB erase -> 64 trims)
Dhara map (logical page -> physical NAND page)
Dhara NAND callbacks
  Existing FSMC command / ECC implementation
Samsung K9F1G08U0E
```

The adapter is structurally feasible because LittleFS's block size is exactly
64 Dhara sectors at 2 KiB each. A LittleFS erase should trim the corresponding
64 logical sectors; it must not directly erase a physical block. Dhara then
owns placement, garbage collection, bad-block retirement, and wear leveling.

## Questions before runtime integration

1. Implement and test `is_bad`, `mark_bad`, `erase`, `prog`, `is_free`,
   `read`, and `copy`. Failed program/erase must propagate as bad-block
   errors. Dhara requires sequential page programming and no page reprogramming.
2. Preserve the Samsung factory marker convention in OOB bytes 0..1. Confirm
   the grown-bad-block marking method against the K9F1G08U0E datasheet.
3. Ensure every Dhara read, including metadata reads and copies, returns
   corrected data or an explicit uncorrectable-ECC error. Do not let partial
   reads bypass ECC validation.
4. Measure usable capacity and write amplification for several GC ratios.
   Dhara reserves capacity for garbage collection and retry safety.
5. Run power-cut tests during page program, erase, checkpoint, garbage
   collection, and LittleFS erase/trim sequences.
6. Treat migration as an explicit destructive format or backup/restore.
   Never automatically initialize Dhara on the existing LittleFS volume.

## Vendored revision

Upstream: `dlbeer/dhara`
Commit: `0e083ded62b911dc1eca1ee99176342541af95c5`
License: permissive ISC-style license, retained in `vendor/dhara/LICENSE`.

Only the core `dhara/` sources and required public headers are vendored.
The upstream ECC implementations are not included because the STM32 FSMC
hardware ECC path is already present in this project.
