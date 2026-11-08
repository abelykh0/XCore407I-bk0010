// SPDX-License-Identifier: MIT
//
// TinyUSB application class-driver registration.
//
// UVC is implemented as a custom class driver to preserve the three-transaction
// high-speed ISO path. UAC2 remains non-claiming until its full driver is ready.

#include "tusb.h"
#include "device/usbd_pvt.h"
#include "tusb_class_drivers.h"

static void xcore_driver_init(void) {
}

static bool xcore_driver_deinit(void) {
  return true;
}

static void xcore_driver_reset(uint8_t rhport) {
  (void) rhport;
}

static void xcore_uvc_reset(uint8_t rhport) {
  xcore_tud_uvc_reset(rhport);
}

static bool xcore_uvc_control(uint8_t rhport, uint8_t stage,
                              tusb_control_request_t const *request) {
  return xcore_tud_uvc_control_xfer(rhport, stage, request);
}

static bool xcore_uvc_xfer(uint8_t rhport, uint8_t ep_addr,
                           xfer_result_t result, uint32_t xferred_bytes) {
  return xcore_tud_uvc_xfer(rhport, ep_addr, (int32_t) result, xferred_bytes);
}

static uint16_t xcore_uvc_open(uint8_t rhport,
                               tusb_desc_interface_t const *desc_intf,
                               uint16_t max_len) {
  return xcore_tud_uvc_open(rhport, desc_intf, max_len);
}

static bool xcore_audio_control(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request) {
  return xcore_tud_audio_control_xfer(rhport, stage, request);
}

static bool xcore_audio_xfer(uint8_t rhport, uint8_t ep_addr,
                             xfer_result_t result, uint32_t xferred_bytes) {
  return xcore_tud_audio_xfer(rhport, ep_addr, (int32_t) result, xferred_bytes);
}

static void xcore_audio_reset(uint8_t rhport) {
  xcore_tud_audio_reset(rhport);
}

static uint16_t xcore_audio_open(uint8_t rhport,
                                 tusb_desc_interface_t const *desc_intf,
                                 uint16_t max_len) {
  return xcore_tud_audio_open(rhport, desc_intf, max_len);
}

static usbd_class_driver_t const xcore_class_drivers[] = {
  {
    .name = "XCore UVC",
    .init = xcore_driver_init,
    .deinit = xcore_driver_deinit,
    .reset = xcore_uvc_reset,
    .open = xcore_uvc_open,
    .control_xfer_cb = xcore_uvc_control,
    .xfer_cb = xcore_uvc_xfer,
    .xfer_isr = NULL,
    .sof = NULL,
  },
  {
    .name = "XCore UAC2",
    .init = xcore_driver_init,
    .deinit = xcore_driver_deinit,
    .reset = xcore_audio_reset,
    .open = xcore_audio_open,
    .control_xfer_cb = xcore_audio_control,
    .xfer_cb = xcore_audio_xfer,
    .xfer_isr = NULL,
    .sof = NULL,
  },
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
  *driver_count = (uint8_t) (sizeof(xcore_class_drivers) / sizeof(xcore_class_drivers[0]));
  return xcore_class_drivers;
}

bool xcore_tud_control_xfer(uint8_t rhport, const void *request,
                            void *buffer, uint16_t length) {
  return tud_control_xfer(rhport, (tusb_control_request_t const *) request,
                           buffer, length);
}

bool xcore_tud_control_status(uint8_t rhport, const void *request) {
  return tud_control_status(rhport, (tusb_control_request_t const *) request);
}

uint8_t *xcore_usbd_control_buffer(void) {
  return usbd_get_ctrl_buf();
}

bool xcore_usbd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr,
                               uint16_t packet_size) {
  return usbd_edpt_iso_alloc(rhport, ep_addr, packet_size);
}

bool xcore_usbd_edpt_iso_activate(uint8_t rhport, const void *endpoint_desc) {
  return usbd_edpt_iso_activate(rhport,
      (tusb_desc_endpoint_t const *) endpoint_desc);
}

void xcore_usbd_edpt_close(uint8_t rhport, uint8_t ep_addr) {
  usbd_edpt_close(rhport, ep_addr);
}

bool xcore_usbd_edpt_claim(uint8_t rhport, uint8_t ep_addr) {
  return usbd_edpt_claim(rhport, ep_addr);
}

bool xcore_usbd_edpt_release(uint8_t rhport, uint8_t ep_addr) {
  return usbd_edpt_release(rhport, ep_addr);
}

bool xcore_usbd_edpt_xfer(uint8_t rhport, uint8_t ep_addr,
                          uint8_t *buffer, uint16_t length) {
  return usbd_edpt_xfer(rhport, ep_addr, buffer, length, false);
}

bool xcore_usbd_edpt_ready(uint8_t rhport, uint8_t ep_addr) {
  return usbd_edpt_ready(rhport, ep_addr);
}
