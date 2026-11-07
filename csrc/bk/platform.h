#ifndef BK_PLATFORM_H
#define BK_PLATFORM_H

#include <stdint.h>
#include "defines.h"

#define BK_RAM_SIZE       0x4000u
#define BK_MONITOR_BASE   0x8000u
#define BK_MONITOR_SIZE   0x2000u
#define BK_BASIC_BASE     0xA000u
#define BK_BASIC_SIZE     0x5F80u

extern uint8_t RamBuffer[BK_RAM_SIZE];
extern pdp_regs pdp;
extern flag_t bkmodel;
extern flag_t io_stop_happened;
extern unsigned short last_branch;

extern uint8_t port0177660;
extern uint8_t port0177662;
extern uint16_t port0177716;
extern uint16_t port0177664;
extern uint16_t port0177714;

uint8_t bk_audio_start(void);
void bk_audio_stop(void);
uint8_t bk_audio_pop_edge(uint32_t *cycles, uint8_t *level);

void bk_platform_init(const uint8_t *monitor_rom,
                      const uint8_t *basic_rom,
                      uint32_t basic_size_bytes);
void bk_reset(void);
void bk_set_pc(uint16_t start);
void bk_request_stop_irq(void);
int32_t bk_loop(void);
uint32_t bk_run_batch(uint32_t instruction_budget);
uint64_t bk_get_ticks(void);
uint32_t bk_get_instruction_count(void);
uint32_t bk_get_frame_count(void);
void sendInput(uint8_t *buffer, uint8_t size);

#endif
