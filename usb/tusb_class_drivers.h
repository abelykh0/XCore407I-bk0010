// SPDX-License-Identifier: MIT
//
// Private bridge between TinyUSB's USBD class-driver ABI and Rust.
#ifndef XCORE_TUSB_CLASS_DRIVERS_H
#define XCORE_TUSB_CLASS_DRIVERS_H

#include <stdbool.h>
#include <stdint.h>
#include "tusb.h"

uint16_t xcore_tud_uvc_open(uint8_t rhport, const void *interface_desc, uint16_t max_len);
const void *xcore_tud_uvc_endpoint_descriptor(void);
bool xcore_tud_uvc_get_interface(uint8_t interface, uint8_t *alternate);
bool xcore_tud_uvc_set_interface(uint8_t alternate);
bool xcore_tud_uvc_start_stream(uint8_t rhport);
bool xcore_tud_uvc_class_control_in(uint8_t request_type, uint8_t request_code,
                                    uint16_t value, uint16_t index, uint16_t length,
                                    const uint8_t **data, uint16_t *data_length);
bool xcore_tud_uvc_class_control_out(uint8_t request_type, uint8_t request_code,
                                     uint16_t value, uint16_t index, uint16_t length,
                                     const uint8_t *data);
void xcore_tud_uvc_reset(uint8_t rhport);
void xcore_tud_uvc_task(uint8_t rhport);
bool xcore_tud_uvc_xfer(uint8_t rhport, uint8_t ep_addr, int32_t result, uint32_t xferred_bytes);
bool xcore_tud_uvc_xfer_isr(uint8_t rhport, uint8_t ep_addr, int32_t result, uint32_t xferred_bytes);

bool xcore_usbd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t packet_size);
bool xcore_usbd_edpt_iso_activate(uint8_t rhport, const void *endpoint_desc);
void xcore_usbd_edpt_close(uint8_t rhport, uint8_t ep_addr);
bool xcore_usbd_edpt_claim(uint8_t rhport, uint8_t ep_addr);
bool xcore_usbd_edpt_release(uint8_t rhport, uint8_t ep_addr);
bool xcore_usbd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t length);
bool xcore_usbd_edpt_xfer_isr(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t length);
bool xcore_usbd_edpt_ready(uint8_t rhport, uint8_t ep_addr);

bool xcore_tud_audio_set_interface(uint8_t interface, uint8_t alternate);
uint16_t xcore_tud_audio_fill_packet(uint8_t *buffer, uint16_t capacity);
void xcore_tud_audio_record_packet_timing(uint32_t total_cycles, uint32_t fill_cycles, uint32_t write_cycles);
void xcore_tud_audio_stop(void);

#endif
