// SPDX-License-Identifier: MIT
//
// TinyUSB application class-driver registration.
//
// This is deliberately an inert registration scaffold: each open callback
// returns zero, so TinyUSB continues to its built-in class drivers. It lets
// us validate the private C ABI and Rust callback boundary without changing
// the working HS device path. Do not enable ownership until the Rust drivers
// implement descriptor parsing, endpoint activation and control transfers.

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
    .reset = xcore_driver_reset,
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
    .reset = xcore_driver_reset,
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
