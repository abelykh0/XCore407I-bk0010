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
#define CFG_TUH_DWC2_DMA_ENABLE 0

#define CFG_TUD_ENDPOINT0_SIZE 64
#define CFG_TUD_ENDPOINT0_BUFSIZE 512

// The UVC endpoint carries exactly 3 x 768-byte transactions per microframe.
// TinyUSB's class staging buffer must hold one complete 2304-byte transfer,
// not a full video frame. Keep every individual USB DMA buffer <= 4096 bytes.
#define CFG_TUD_VIDEO 1
#define CFG_TUD_VIDEO_STREAMING 1
#define CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE 2304

// Device audio class is enabled during the UAC2 port; its function-specific
// packet/FIFO sizing will be set alongside the migrated audio descriptors.
#define CFG_TUD_AUDIO 1
#define CFG_TUD_AUDIO_ENABLE_EP_IN 1
#define CFG_TUD_AUDIO_ENABLE_EP_OUT 0
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX 12
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ 96
#define CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL 0

// USB FS host: keyboard only.
#define CFG_TUH_DEVICE_MAX 1
#define CFG_TUH_ENUMERATION_BUFSIZE 256
#define CFG_TUH_HID 4

#endif
