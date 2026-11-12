/*
 * BK-0010 platform layer.
 *
 * The PDP-11 instruction implementation remains in the original C sources.
 * This file supplies the machine-specific memory map and CPU lifecycle.
 */
#include <stdint.h>

#include <stddef.h>
#include "platform.h"


// Emulator RTC extension: expose the low 32 bits of Unix milliseconds.
// Reading the low word captures a snapshot; the high word uses that snapshot.
extern uint32_t bk_rtc_unix_millis(void);
static uint32_t bk_rtc_latched_millis;
static uint16_t bk_rtc_read_time_low(void)
{
    bk_rtc_latched_millis = bk_rtc_unix_millis();
    return (uint16_t)bk_rtc_latched_millis;
}

static uint16_t bk_rtc_read_time_high(void)
{
    return (uint16_t)(bk_rtc_latched_millis >> 16);
}


#define RAM_AVAILABLE 0x4000u
#define MONITOR_BASE  0x8000u
#define MONITOR_SIZE  0x2000u
#define BASIC_BASE    0xA000u
#define VIDEO_BASE    0x4000u
#define VIDEO_END     0x7FFFu
#define IO_BASE       0xFF80u

uint8_t RamBuffer[RAM_AVAILABLE] __attribute__((section(".ccmram"), aligned(4)));
pdp_regs pdp;
flag_t bkmodel = 0;
flag_t io_stop_happened = 0;
unsigned short last_branch = 0;

uint8_t  port0177660 = 0x40;    // Keyboard Status Register
uint8_t  port0177662 = 0;       // Keyboard Data / Code Register
uint16_t port0177714 = 0;       // Parallel Port / Peripheral Device Register
uint16_t port0177716 = 0x0040;  // System Port Register / Control Register (bit 6 - speaker)
uint16_t port0177664 = 01330;   // Screen Scroll / Offset Register

// Audio output transition queue. The emulator is the single producer
// (thread context); the USB Audio endpoint callback consumes transitions.
#define BK_AUDIO_EDGE_CAPACITY 512u
#define BK_AUDIO_EDGE_MASK (BK_AUDIO_EDGE_CAPACITY - 1u)
static volatile uint16_t bk_audio_edge_head;
static volatile uint16_t bk_audio_edge_tail;
static volatile uint8_t bk_audio_enabled;
static uint8_t bk_audio_level;
static uint32_t bk_audio_edge_cycles[BK_AUDIO_EDGE_CAPACITY];
static uint8_t bk_audio_edge_levels[BK_AUDIO_EDGE_CAPACITY];

static void bk_audio_record_level(uint8_t level)
{
    level = level ? 1u : 0u;
    if (level == bk_audio_level) return;

    uint16_t head = bk_audio_edge_head;
    uint16_t tail = bk_audio_edge_tail;
    if ((uint16_t)(head - tail) >= BK_AUDIO_EDGE_CAPACITY) {
        // Preserve the last successfully queued level so a later write can
        // retry recording the current speaker state after space becomes free.
        return;
    }

    bk_audio_level = level;
    uint16_t slot = head & BK_AUDIO_EDGE_MASK;
    // Timestamp in the emulator's PDP-cycle domain, not the MCU clock domain.
    bk_audio_edge_cycles[slot] = bk_timing_cycles();
    bk_audio_edge_levels[slot] = level;
    __asm volatile("dmb" ::: "memory");
    bk_audio_edge_head = (uint16_t)(head + 1u);
}

uint8_t bk_audio_start(void)
{
    // Called from the USB ISR; the emulator producer is paused while this runs.
    bk_audio_edge_tail = bk_audio_edge_head;
    bk_audio_level = (port0177716 & 0x0040u) ? 1u : 0u;
    __asm volatile("dmb" ::: "memory");
    bk_audio_enabled = 1u;
    return bk_audio_level;
}

void bk_audio_stop(void)
{
    bk_audio_enabled = 0u;
    __asm volatile("dmb" ::: "memory");
}

uint8_t bk_audio_pop_edge(uint32_t *cycles, uint8_t *level)
{
    uint16_t tail = bk_audio_edge_tail;
    if (tail == bk_audio_edge_head || cycles == NULL || level == NULL) {
        return 0u;
    }

    __asm volatile("dmb" ::: "memory");
    uint16_t slot = tail & BK_AUDIO_EDGE_MASK;
    *cycles = bk_audio_edge_cycles[slot];
    *level = bk_audio_edge_levels[slot];
    __asm volatile("dmb" ::: "memory");
    bk_audio_edge_tail = (uint16_t)(tail + 1u);
    return 1u;
}

// STOP key requests the independent hardware IRQ1 (priority 5, vector 000004).
static volatile uint8_t stop_irq_pending = 0;

static const uint8_t *monitor_rom;
static const uint8_t *basic_rom;
static uint32_t basic_rom_size;

extern int bk_video_read_byte(uint32_t addr, uint8_t mode, uint8_t *value);
extern int bk_video_write_byte(uint32_t addr, uint8_t mode, uint8_t value);
extern void bk_video_mode_changed(uint8_t mode);
extern void bk_video_shift_changed(uint8_t shift);
extern int ev_init(void);

void bk_platform_init(const uint8_t *monitor, const uint8_t *basic,
                      uint32_t basic_size_bytes)
{
    monitor_rom = monitor;
    basic_rom = basic;
    basic_rom_size = basic_size_bytes > BK_BASIC_SIZE ? BK_BASIC_SIZE : basic_size_bytes;
    port0177664 = 01330;
    bk_reset();
}

void bk_set_pc(uint16_t start)
{
    pdp.regs[PC] = start;
}

void bk_reset(void)
{
    unsigned i;
    for (i = 0; i < 8; ++i) {
        pdp.regs[i] = 0;
    }
    pdp.psw = 0;
    pdp.ir = 0;
    pdp.ea_addr = 0;
    pdp.total = 0;
    bk_timing_reset();
    pdp.look_time = 0;
    pdp.regs[PC] = MONITOR_BASE;

    for (i = 0; i < RAM_AVAILABLE; i += 4) {
        RamBuffer[i] = 0xFF;
        RamBuffer[i + 1] = 0x00;
        RamBuffer[i + 2] = 0xFF;
        RamBuffer[i + 3] = 0x00;
    }

    port0177660 = 0x40;
    port0177662 = 0;
    port0177716 = 0x0040;
    port0177714 = 0;
    bk_rtc_latched_millis = 0;
    bk_audio_enabled = 0;
    bk_audio_level = 0;
    bk_audio_edge_head = 0;
    bk_audio_edge_tail = 0;
    stop_irq_pending = 0;
    io_stop_happened = 0;
    last_branch = 0;
    ev_init();
    bk_video_mode_changed(RamBuffer[0x0020]);
    bk_video_shift_changed((uint8_t)port0177664);
}

int ll_byte(pdp_regs *p, c_addr addr, d_byte *byte)
{
    (void)p;
    if (byte == NULL || addr > 0xFFFFu) return BUS_ERROR;

    if (addr >= IO_BASE) {
        switch (addr) {
        case 0177660: *byte = port0177660; break;
        case 0177662:
            *byte = port0177662;
            port0177660 &= (uint8_t)~0x80u;
            break;
        case 0177664: *byte = (uint8_t)port0177664; break;
        case 0177665: *byte = (uint8_t)(port0177664 >> 8); break;
        case 0177714: *byte = (uint8_t)port0177714; break;
        case 0177715: *byte = (uint8_t)(port0177714 >> 8); break;
        case 0177716: *byte = (uint8_t)port0177716; break;
        case 0177717: *byte = (uint8_t)(port0177716 >> 8); break;
        case 0177720: *byte = (uint8_t)bk_rtc_read_time_low(); break;
        case 0177721: *byte = (uint8_t)(bk_rtc_latched_millis >> 8); break;
        case 0177722: *byte = (uint8_t)bk_rtc_read_time_high(); break;
        case 0177723: *byte = (uint8_t)(bk_rtc_latched_millis >> 24); break;
        default: *byte = 0; break;
        }
    } else if (addr >= BASIC_BASE) {
        uint32_t offset = addr - BASIC_BASE;
        *byte = (basic_rom != NULL && offset < basic_rom_size) ? basic_rom[offset] : 0;
    } else if (addr >= MONITOR_BASE) {
        uint32_t offset = addr - MONITOR_BASE;
        *byte = (monitor_rom != NULL && offset < MONITOR_SIZE) ? monitor_rom[offset] : 0;
    } else if (addr >= VIDEO_BASE) {
        return bk_video_read_byte(addr, RamBuffer[0x0020], byte);
    } else {
        *byte = RamBuffer[addr];
    }
    return OK;
}

int ll_word(pdp_regs *p, c_addr addr, d_word *word)
{
    if (word == NULL) return BUS_ERROR;
    if (addr & 1u) {
        d_byte lo, hi;
        int result = ll_byte(p, addr, &lo);
        if (result != OK) return result;
        result = ll_byte(p, addr + 1u, &hi);
        if (result != OK) return result;
        *word = (d_word)((uint16_t)lo | ((uint16_t)hi << 8));
        return OK;
    }

    if (addr >= IO_BASE) {
        switch (addr) {
        case 0177660: *word = port0177660; break;
        case 0177662:
            *word = port0177662;
            port0177660 &= (uint8_t)~0x80u;
            break;
        case 0177664: *word = port0177664; break;
        case 0177714: *word = port0177714; break;
        case 0177716: *word = port0177716; break;
        case 0177720: *word = bk_rtc_read_time_low(); break;
        case 0177722: *word = bk_rtc_read_time_high(); break;
        default: *word = 0; break;
        }
        return OK;
    }
    if (addr >= BASIC_BASE) {
        uint32_t offset = addr - BASIC_BASE;
        uint8_t lo = 0, hi = 0;
        if (basic_rom != NULL && offset < basic_rom_size) lo = basic_rom[offset];
        if (basic_rom != NULL && offset + 1u < basic_rom_size) hi = basic_rom[offset + 1u];
        *word = (d_word)((uint16_t)lo | ((uint16_t)hi << 8));
        return OK;
    }
    if (addr >= MONITOR_BASE) {
        uint32_t offset = addr - MONITOR_BASE;
        uint8_t lo = (monitor_rom != NULL && offset < MONITOR_SIZE) ? monitor_rom[offset] : 0;
        uint8_t hi = (monitor_rom != NULL && offset + 1u < MONITOR_SIZE) ? monitor_rom[offset + 1u] : 0;
        *word = (d_word)((uint16_t)lo | ((uint16_t)hi << 8));
        return OK;
    }
    if (addr >= VIDEO_BASE) {
        uint8_t lo, hi;
        int result = bk_video_read_byte(addr, RamBuffer[0x0020], &lo);
        if (result != OK) return result;
        result = bk_video_read_byte(addr + 1u, RamBuffer[0x0020], &hi);
        if (result != OK) return result;
        *word = (d_word)((uint16_t)lo | ((uint16_t)hi << 8));
        return OK;
    }

    *word = (d_word)((uint16_t)RamBuffer[addr] |
                     ((uint16_t)RamBuffer[addr + 1u] << 8));
    return OK;
}

int sl_byte(pdp_regs *p, c_addr addr, d_byte value)
{
    (void)p;
    if (addr > 0xFFFFu) return BUS_ERROR;

    if (addr >= IO_BASE) {
        switch (addr) {
        case 0177660:
            port0177660 = (uint8_t)((port0177660 & (uint8_t)~0x40u) | (value & 0x40u));
            break;
        case 0177662: port0177662 = value; break;
        case 0177663: port0177662 = (uint8_t)((port0177662 & 0x00FFu) | ((uint16_t)value << 8)); break;
        case 0177664:
            port0177664 = (uint16_t)((port0177664 & 0xFF00u) | value);
            bk_video_shift_changed((uint8_t)port0177664);
            break;
        case 0177665:
            port0177664 = (uint16_t)((port0177664 & 0x00FFu) | ((uint16_t)value << 8));
            bk_video_shift_changed((uint8_t)port0177664);
            break;
        case 0177714:
            port0177714 = (uint16_t)((port0177714 & 0xFF00u) | value);
            break;
        case 0177715:
            port0177714 = (uint16_t)((port0177714 & 0x00FFu) | ((uint16_t)value << 8));
            break;
        case 0177716:
            port0177716 = (uint16_t)((port0177716 & 0xFF00u) | value);
            if (bk_audio_enabled) {
                bk_audio_record_level((uint8_t)(port0177716 & 0x0040u));
            }
            break;
        case 0177717:
            port0177716 = (uint16_t)((port0177716 & 0x00FFu) | ((uint16_t)value << 8));
            break;
        default: break;
        }
    } else if (addr >= BASIC_BASE || addr >= MONITOR_BASE) {
        /* ROM is read-only. */
    } else if (addr >= VIDEO_BASE) {
        return bk_video_write_byte(addr, RamBuffer[0x0020], value);
    } else {
        RamBuffer[addr] = value;
        if (addr == 0x0020u) bk_video_mode_changed(value);
    }
    return OK;
}

int sl_word(pdp_regs *p, c_addr addr, d_word value)
{
    if (addr & 1u) {
        int result = sl_byte(p, addr, (d_byte)value);
        if (result != OK) return result;
        result = sl_byte(p, addr + 1u, (d_byte)(value >> 8));
        if (result != OK) return result;
        return ODD_ADDRESS;
    }

    if (addr >= IO_BASE) {
        switch (addr) {
        case 0177660:
            port0177660 = (uint8_t)((port0177660 & (uint8_t)~0x40u) | (value & 0x40u));
            break;
        case 0177664:
            port0177664 = value & 01377;
            bk_video_shift_changed((uint8_t)port0177664);
            break;
        case 0177714:
            port0177714 = value;
            break;
        case 0177716:
            port0177716 = value;
            if (bk_audio_enabled) {
                bk_audio_record_level((uint8_t)(port0177716 & 0x0040u));
            }
            break;
        default: break;
        }
    } else if (addr >= BASIC_BASE || addr >= MONITOR_BASE) {
        /* ROM is read-only. */
    } else if (addr >= VIDEO_BASE) {
        int result = bk_video_write_byte(addr, RamBuffer[0x0020], (d_byte)value);
        if (result != OK) return result;
        return bk_video_write_byte(addr + 1u, RamBuffer[0x0020], (d_byte)(value >> 8));
    } else {
        RamBuffer[addr] = (uint8_t)value;
        RamBuffer[addr + 1u] = (uint8_t)(value >> 8);
        if (addr == 0x0020u) bk_video_mode_changed((uint8_t)value);
    }
    return OK;
}

void q_reset(void)
{
}

// Called by the host-keyboard adapter for PC Pause/F12. This is not a
// character input: set the BK STOP status bit and latch the independent IRQ1.
void bk_request_stop_irq(void)
{
    port0177716 |= 0x0004u;
    stop_irq_pending = 1;
}

/* Original BK keyboard input path: wait for the previous character's
 * interrupt to be acknowledged, then schedule the next TTY interrupt. */
static int bk_tty_finish(d_word info)
{
    (void)info;
    return service(060);
}

uint8_t bk_try_send_input_byte(uint8_t symbol)
{
    if (port0177660 & 0x80u) return 0u;
    if (symbol == 0x0Du) symbol = 0x0Au;
    port0177660 |= 0x80u;
    port0177662 = symbol & 0x7Fu;
    (void)ev_register(TTY_PRI, bk_tty_finish, 0, symbol);
    return 1u;
}

void sendInput(uint8_t *buffer, uint8_t size)
{
    uint8_t i;
    if (buffer == NULL) return;
    for (i = 0; i < size; ++i) {
        while (!bk_try_send_input_byte(buffer[i])) bk_loop();
    }
}

int32_t bk_loop(void)
{
    pdp_regs *p = &pdp;
    int result, result2 = OK, rtt = 0;
    int32_t return_value = 0;

    // STOP is hardware IRQ1 at priority 5 (vector 000004), independent
    // of the normal keyboard TTY interrupt enable/mask (priority 4).
    if (stop_irq_pending && ((p->psw >> 5) & 7) < 5) {
        stop_irq_pending = 0;
        (void)service(004);
        p->total++;
        return return_value;
    }

    result = ll_word(p, p->regs[PC], &p->ir);
    p->regs[PC] += 2;
    if (result == OK) {
        // Account the PDP instruction's emulated cycle cost before executing it.
        timing(p);
        result = itab[p->ir >> 6].func(p);
    }

    if (result != OK) {
        switch (result) {
        case BUS_ERROR: bk_timing_add_cycles(64u); break;
        case ODD_ADDRESS: result2 = service(04); break;
        case CPU_ILLEGAL: result2 = service(010); break;
        case CPU_BPT: result2 = service(014); break;
        case CPU_EMT: result2 = service(030); break;
        case CPU_TRAP: result2 = service(034); break;
        case CPU_IOT: result2 = service(020); break;
        case CPU_WAIT: in_wait_instr = 1; result2 = OK; break;
        case CPU_RTT: rtt = 1; result2 = OK; break;
        case CPU_HALT:
            io_stop_happened = 4;
            result2 = service(004);
            break;
        default: result2 = OK; break;
        }
        if (result2 != OK) {
            ll_word(p, 0177716, &p->regs[PC]);
            p->regs[PC] &= 0177400;
        }
    }

    if ((p->psw & 020) && rtt == 0) {
        if (service(014) != OK) {
            ll_word(p, 0177716, &p->regs[PC]);
            p->regs[PC] &= 0177400;
            p->regs[SP] = 01000;
        }
    }
    p->total++;

    int priority = (p->psw >> 5) & 7;
    if (pending_interrupts && priority != 7) ev_fire(priority);
    return return_value;
}
