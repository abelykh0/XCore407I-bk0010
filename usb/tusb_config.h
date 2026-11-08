#ifndef XCORE407I_TUSB_CONFIG_H
#define XCORE407I_TUSB_CONFIG_H

// STM32F407 has two DWC2 root ports. TinyUSB maps port 0 to OTG_FS and
// port 1 to OTG_HS (external USB3300 ULPI PHY).
#define CFG_TUSB_MCU OPT_MCU_STM32F4
#define CFG_TUSB_OS OPT_OS_NONE
#define CFG_TUSB_DEBUG 0

#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_HOST | OPT_MODE_FULL_SPEED)
#define CFG_TUSB_RHPORT1_MODE (OPT_MODE_DEVICE | OPT_MODE_HIGH_SPEED)
#define CFG_TUH_ENABLED 1
#define CFG_TUD_ENABLED 1
#define CFG_TUH_MAX_SPEED OPT_MODE_FULL_SPEED
#define CFG_TUD_MAX_SPEED OPT_MODE_HIGH_SPEED

// DMA buffers must reside in normal SRAM (0x20000000 region), never CCM
// (0x10000000), and be at least word-aligned. Leave the section unspecified;
// the STM32F407 linker places ordinary static data in DMA-accessible SRAM.
#define CFG_TUD_MEM_ALIGN __attribute__((aligned(4)))
#define CFG_TUH_MEM_ALIGN __attribute__((aligned(4)))

// HS device DMA is mandatory for the UVC data path. FS host HID bandwidth is
// low; keep its controller configuration independently selectable.
#define CFG_TUD_DWC2_DMA_ENABLE 1
// Reserve two max-packet slots for a three-transaction HS ISO IN endpoint.
#define CFG_TUD_DWC2_ISO_IN_FIFO_MULTIPLIER 2
// This DWC2 port implements the ISO endpoint preallocate/activate API.
#define TUP_DCD_EDPT_ISO_ALLOC
#define CFG_TUH_DWC2_DMA_ENABLE 0

#define CFG_TUD_ENDPOINT0_SIZE 64
#define CFG_TUD_ENDPOINT0_BUFSIZE 512

// The STM32F407 OTG_HS has 1024 words (4096 bytes) of shared DWC2 FIFO RAM.
// TinyUSB's DWC2 allocator accounts for RX FIFO, DMA endpoint-info words, EP0,
// audio TX and video TX. For 3 x 768-byte UVC transactions it reserves 2 x
// MPS (1536 bytes) in the video TX FIFO; the DWC2 can refill the third packet
// after the first packet is sent. This is distinct from SRAM DMA buffers.
//
// UVC is implemented by the application's XCore UVC class driver.
// Do not also register TinyUSB's built-in video class driver or allocate
// its unused streaming staging buffer.
#define CFG_TUD_VIDEO 0
#define CFG_TUD_VIDEO_STREAMING 0
#define CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE 2304

// Device audio class is enabled during the UAC2 port; its function-specific
// packet/FIFO sizing will be set alongside the migrated audio descriptors.
// Use TinyUSB's built-in UAC2 class driver. UVC remains application-defined.
#define CFG_TUD_AUDIO 1
#define CFG_TUD_AUDIO_ENABLE_EP_IN 1
#define CFG_TUD_AUDIO_ENABLE_EP_OUT 0
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX 14
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ 96
#define CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL 0

// USB FS host: keyboard only.
#define CFG_TUH_DEVICE_MAX 1
#define CFG_TUH_ENUMERATION_BUFSIZE 256
#define CFG_TUH_HID 4

// Fail the build if a future class/configuration change exceeds the F407's
// per-buffer USB-HS DMA budget. This is per DMA buffer, not aggregate SRAM.
#if CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE > 4096
#error "UVC endpoint DMA staging buffer exceeds 4 KiB"
#endif
#if CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX > 4096
#error "UAC2 endpoint DMA staging buffer exceeds 4 KiB"
#endif
#if CFG_TUD_ENDPOINT0_BUFSIZE > 4096
#error "EP0 staging buffer exceeds 4 KiB"
#endif

#endif
