// SPDX-License-Identifier: MIT
//
// TinyUSB application class-driver registration.
//
// UVC remains application-defined for its three-transaction high-bandwidth
// endpoint. UAC2 is delegated to TinyUSB's built-in audio class driver.

#include "tusb.h"
#include "device/usbd_pvt.h"
#include "tusb_class_drivers.h"


static void xcore_driver_init(void) {
}

static bool xcore_driver_deinit(void) {
  return true;
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

static bool xcore_uvc_xfer_isr(uint8_t rhport, uint8_t ep_addr,
                               xfer_result_t result, uint32_t xferred_bytes) {
  return xcore_tud_uvc_xfer_isr(rhport, ep_addr, (int32_t) result, xferred_bytes);
}

static uint16_t xcore_uvc_open(uint8_t rhport,
                               tusb_desc_interface_t const *desc_intf,
                               uint16_t max_len) {
  return xcore_tud_uvc_open(rhport, desc_intf, max_len);
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
    .xfer_isr = xcore_uvc_xfer_isr,
    .sof = NULL,
  },
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
  *driver_count = (uint8_t) (sizeof(xcore_class_drivers) / sizeof(xcore_class_drivers[0]));
  return xcore_class_drivers;
}



#if CFG_TUD_AUDIO
// The clock source is fixed at 48 kHz. Keep these responses byte-for-byte
// equivalent to the former custom UAC2 implementation.
bool tud_audio_get_req_entity_cb(uint8_t rhport,
                                 tusb_control_request_t const *request) {
  static const uint8_t frequency_cur[4] = {0x80, 0xbb, 0x00, 0x00};
  static const uint8_t frequency_range[14] = {
    1, 0,                         // one subrange
    0x80, 0xbb, 0x00, 0x00,       // minimum: 48000 Hz
    0x80, 0xbb, 0x00, 0x00,       // maximum: 48000 Hz
    0x00, 0x00, 0x00, 0x00        // resolution: fixed
  };
  static const uint8_t clock_valid = 1;

  const uint8_t entity = TU_U16_HIGH(request->wIndex);
  const uint8_t selector = TU_U16_HIGH(request->wValue);
  const uint8_t channel = TU_U16_LOW(request->wValue);
  if (entity != 1 || channel != 0) {
    return false;
  }

  if (request->bRequest == 0x01 && selector == 0x01) { // GET_CUR, sampling frequency
    const uint16_t length = request->wLength < sizeof(frequency_cur)
                              ? request->wLength : sizeof(frequency_cur);
    return tud_control_xfer(rhport, request, (void *) frequency_cur, length);
  }
  if (request->bRequest == 0x01 && selector == 0x02) { // GET_CUR, clock validity
    const uint16_t length = request->wLength < sizeof(clock_valid)
                              ? request->wLength : sizeof(clock_valid);
    return tud_control_xfer(rhport, request, (void *) &clock_valid, length);
  }
  if (request->bRequest == 0x02 && selector == 0x01) { // GET_RANGE, sampling frequency
    const uint16_t length = request->wLength < sizeof(frequency_range)
                              ? request->wLength : sizeof(frequency_range);
    return tud_control_xfer(rhport, request, (void *) frequency_range, length);
  }
  return false;
}

// TinyUSB owns SET_INTERFACE and endpoint activation. The application only
// starts/stops the BK audio source after the class driver changes the AS alt.
bool tud_audio_set_itf_cb(uint8_t rhport,
                          tusb_control_request_t const *request) {
  (void) rhport;
  return xcore_tud_audio_set_interface(TU_U16_LOW(request->wIndex),
                                       TU_U16_LOW(request->wValue));
}

// Called from the audio endpoint completion ISR. Fill TinyUSB's FIFO; its
// own driver copies the queued samples into its DMA-capable linear buffer.
bool tud_audio_tx_done_isr(uint8_t rhport, uint16_t n_bytes_sent,
                           uint8_t func_id, uint8_t ep_in,
                           uint8_t cur_alt_setting) {
  (void) rhport;
  (void) n_bytes_sent;
  if (func_id != 0 || ep_in != 0x82) {
    return false;
  }
  if (cur_alt_setting == 0) {
    return true;
  }

  uint8_t packet[12] __attribute__((aligned(4)));
  const uint16_t length = xcore_tud_audio_fill_packet(packet, sizeof(packet));
  if (length == 0) {
    return true;
  }
  return tud_audio_n_write(func_id, packet, length) == length;
}

// Stop capture on bus reset, disconnect, or configuration teardown.
void tud_umount_cb(void) {
  xcore_tud_audio_stop();
}
#endif

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

bool xcore_usbd_edpt_xfer_isr(uint8_t rhport, uint8_t ep_addr,
                              uint8_t *buffer, uint16_t length) {
  return usbd_edpt_xfer(rhport, ep_addr, buffer, length, true);
}

bool xcore_usbd_edpt_ready(uint8_t rhport, uint8_t ep_addr) {
  return usbd_edpt_ready(rhport, ep_addr);
}
