// SPDX-License-Identifier: MIT
//
// TinyUSB UAC2 application callbacks. TinyUSB retains ownership of the
// built-in audio class driver; these callbacks bridge it to the BK audio source.

#include "tusb.h"
#include "audio_uac2.h"

#if CFG_TUD_AUDIO
// The clock rate is supplied by the shared Rust AUDIO_KHZ constant.
extern uint32_t xcore_audio_sample_rate_hz(void);
#if defined(XCORE_AUDIO_UNDERRUN_DIAG)
extern void xcore_tud_audio_note_write_failure(void);
#endif

bool tud_audio_get_req_entity_cb(uint8_t rhport,
                                 tusb_control_request_t const *request) {
  static uint8_t frequency_cur[4];
  static uint8_t frequency_range[14] = {
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
  };
  const uint32_t sample_rate = xcore_audio_sample_rate_hz();
  for (uint8_t i = 0; i < 4; i++) {
    const uint8_t byte = (uint8_t) (sample_rate >> (8 * i));
    frequency_cur[i] = byte;
    frequency_range[2 + i] = byte;
    frequency_range[6 + i] = byte;
  }
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

  uint8_t packet[10] __attribute__((aligned(4)));
  const uint16_t length = xcore_tud_audio_fill_packet(packet, sizeof(packet));
  if (length == 0) {
    return true;
  }
  const uint16_t written = tud_audio_n_write(func_id, packet, length);
#if defined(XCORE_AUDIO_UNDERRUN_DIAG)
  if (written != length) {
    xcore_tud_audio_note_write_failure();
  }
#endif
  return written == length;
}

// Stop capture on bus reset, disconnect, or configuration teardown.
void tud_umount_cb(void) {
  xcore_tud_audio_stop();
}
#endif

// Referenced by the UVC registration object to force this callback object
// out of the static archive. TinyUSB's weak callback defaults otherwise
// satisfy its own references before this object is extracted.
void xcore_audio_uac2_link_anchor(void) {
}
