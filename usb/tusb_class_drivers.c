// SPDX-License-Identifier: MIT
//
// TinyUSB application class-driver registration.
//
// UVC remains application-defined for its three-transaction high-bandwidth
// endpoint. UAC2 is delegated to TinyUSB's built-in audio class driver.

#include "tusb.h"
#include "class/video/video.h"
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
  if (request == NULL) {
    return false;
  }

  const uint8_t request_type = request->bmRequestType;
  const uint8_t request_code = request->bRequest;
  const uint16_t value = request->wValue;
  const uint16_t index = request->wIndex;
  const uint16_t length = request->wLength;
  const uint8_t interface = (uint8_t) index;

  if (((request_type >> 5) & 0x03) == TUSB_REQ_TYPE_STANDARD &&
      (request_type & 0x1f) == TUSB_REQ_RCPT_INTERFACE) {
    if (stage != CONTROL_STAGE_SETUP) {
      return true;
    }

    if (request_code == TUSB_REQ_GET_INTERFACE) {
      uint8_t alternate = 0;
      if (!xcore_tud_uvc_get_interface(interface, &alternate)) {
        return false;
      }
      return tud_control_xfer(rhport, request, &alternate, sizeof(alternate));
    }

    if (request_code == TUSB_REQ_SET_INTERFACE) {
      if (length != 0 || (value >> 8) != 0) {
        return false;
      }

      if (interface == 0) {
        if ((uint8_t) value != 0) {
          return false;
        }
        return tud_control_status(rhport, request);
      }

      if (interface != 1 || (uint8_t) value > 1) {
        return false;
      }

      const uint8_t alternate = (uint8_t) value;
      if (alternate == 1 &&
          !xcore_usbd_edpt_iso_activate(rhport, xcore_tud_uvc_endpoint_descriptor())) {
        return false;
      }

      if (!xcore_tud_uvc_set_interface(alternate)) {
        if (alternate == 1) {
          xcore_usbd_edpt_close(rhport, 0x81);
        }
        return false;
      }

      if (alternate == 0) {
        xcore_usbd_edpt_close(rhport, 0x81);
      } else if (!xcore_tud_uvc_start_stream(rhport)) {
        return false;
      }

      return tud_control_status(rhport, request);
    }

    return false;
  }

  if (((request_type >> 5) & 0x03) != TUSB_REQ_TYPE_CLASS) {
    return false;
  }

  if (stage == CONTROL_STAGE_SETUP) {
    if ((request_type & TUSB_DIR_IN_MASK) != 0) {
      const uint8_t *response = NULL;
      uint16_t response_length = 0;
      if (!xcore_tud_uvc_class_control_in(request_type, request_code, value,
                                         index, length, &response,
                                         &response_length)) {
        return false;
      }
      return tud_control_xfer(rhport, request, (void *) response, response_length);
    }

    if (request_code != VIDEO_REQUEST_SET_CUR || length > 34) {
      return false;
    }
    return tud_control_xfer(rhport, request, usbd_get_ctrl_buf(), length);
  }

  if (stage == CONTROL_STAGE_DATA &&
      (request_type & TUSB_DIR_IN_MASK) == 0) {
    if (length > 34) {
      return false;
    }
    return xcore_tud_uvc_class_control_out(request_type, request_code, value,
                                           index, length, usbd_get_ctrl_buf());
  }

  return true;
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
// The clock source is fixed at 24 kHz.
bool tud_audio_get_req_entity_cb(uint8_t rhport,
                                 tusb_control_request_t const *request) {
  static const uint8_t frequency_cur[4] = {0xc0, 0x5d, 0x00, 0x00};
  static const uint8_t frequency_range[14] = {
    1, 0,                         // one subrange
    0xc0, 0x5d, 0x00, 0x00,       // minimum: 24000 Hz
    0xc0, 0x5d, 0x00, 0x00,       // maximum: 24000 Hz
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
  const uint8_t interface = TU_U16_LOW(request->wIndex);
  const uint8_t alternate = TU_U16_LOW(request->wValue);
  if (!xcore_tud_audio_set_interface(interface, alternate)) {
    return false;
  }

  // The initial ZLP is followed by the normal endpoint completion callback.
  // Keep PCM consumption exclusively in that ISR callback.
  return true;
}

// Stop the old stream before audiod replaces/closes its active endpoint.
// If the host switches alt 1 -> alt 1, the final SET_INTERFACE callback
// starts the new stream again after TinyUSB has reactivated the endpoint.
bool tud_audio_set_itf_close_ep_cb(uint8_t rhport,
                                   tusb_control_request_t const *request) {
  (void) rhport;
  return xcore_tud_audio_set_interface(TU_U16_LOW(request->wIndex), 0);
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

  uint8_t packet[6] __attribute__((aligned(4)));
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
