// SPDX-License-Identifier: MIT
//
// Private bridge between TinyUSB's USBD class-driver ABI and Rust.
#ifndef XCORE_TUSB_CLASS_DRIVERS_H
#define XCORE_TUSB_CLASS_DRIVERS_H

#include <stdbool.h>
#include <stdint.h>

uint16_t xcore_tud_uvc_open(uint8_t rhport, const void *interface_desc, uint16_t max_len);
void xcore_tud_uvc_reset(uint8_t rhport);
void xcore_tud_uvc_task(uint8_t rhport);
bool xcore_tud_uvc_control_xfer(uint8_t rhport, uint8_t stage, const void *request);
bool xcore_tud_uvc_xfer(uint8_t rhport, uint8_t ep_addr, int32_t result, uint32_t xferred_bytes);

bool xcore_tud_control_xfer(uint8_t rhport, const void *request, void *buffer, uint16_t length);
bool xcore_tud_control_status(uint8_t rhport, const void *request);
uint8_t *xcore_usbd_control_buffer(void);
bool xcore_usbd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t packet_size);
bool xcore_usbd_edpt_iso_activate(uint8_t rhport, const void *endpoint_desc);
void xcore_usbd_edpt_close(uint8_t rhport, uint8_t ep_addr);
bool xcore_usbd_edpt_claim(uint8_t rhport, uint8_t ep_addr);
bool xcore_usbd_edpt_release(uint8_t rhport, uint8_t ep_addr);
bool xcore_usbd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t length);
bool xcore_usbd_edpt_ready(uint8_t rhport, uint8_t ep_addr);

uint16_t xcore_tud_audio_open(uint8_t rhport, const void *interface_desc, uint16_t max_len);
bool xcore_tud_audio_control_xfer(uint8_t rhport, uint8_t stage, const void *request);
bool xcore_tud_audio_xfer(uint8_t rhport, uint8_t ep_addr, int32_t result, uint32_t xferred_bytes);

#endif
