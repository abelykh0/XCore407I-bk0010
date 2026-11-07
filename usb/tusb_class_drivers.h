// SPDX-License-Identifier: MIT
//
// Private bridge between TinyUSB's USBD class-driver ABI and Rust.
//
// Keep usbd_class_driver_t in C: it is a TinyUSB-private structure and must
// not become part of the Rust-facing ABI.
#ifndef XCORE_TUSB_CLASS_DRIVERS_H
#define XCORE_TUSB_CLASS_DRIVERS_H

#include <stdbool.h>
#include <stdint.h>

uint16_t xcore_tud_uvc_open(uint8_t rhport, const void *interface_desc, uint16_t max_len);
bool xcore_tud_uvc_control_xfer(uint8_t rhport, uint8_t stage, const void *request);
bool xcore_tud_uvc_xfer(uint8_t rhport, uint8_t ep_addr, int32_t result, uint32_t xferred_bytes);

uint16_t xcore_tud_audio_open(uint8_t rhport, const void *interface_desc, uint16_t max_len);
bool xcore_tud_audio_control_xfer(uint8_t rhport, uint8_t stage, const void *request);
bool xcore_tud_audio_xfer(uint8_t rhport, uint8_t ep_addr, int32_t result, uint32_t xferred_bytes);

#endif
