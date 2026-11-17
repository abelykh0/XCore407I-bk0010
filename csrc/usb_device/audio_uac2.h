// SPDX-License-Identifier: MIT
//
// Private bridge between TinyUSB's UAC2 callbacks and the Rust audio source.
#ifndef XCORE_AUDIO_UAC2_H
#define XCORE_AUDIO_UAC2_H

#include <stdbool.h>
#include <stdint.h>

uint32_t xcore_audio_sample_rate_hz(void);
bool xcore_tud_audio_set_interface(uint8_t interface, uint8_t alternate);
uint16_t xcore_tud_audio_fill_packet(uint8_t *buffer, uint16_t capacity);
void xcore_tud_audio_record_packet_timing(uint32_t total_cycles, uint32_t fill_cycles, uint32_t write_cycles);
void xcore_tud_audio_stop(void);

#endif
