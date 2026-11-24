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
uint16_t port0177716 = 0x0040;  // SEL1 output register (bit 6 - speaker)
static volatile uint8_t keyboard_key_pressed;
static uint8_t sel1_write_flag;
static uint8_t keyboard_irq_info;

static void bk_schedule_keyboard_irq(void);
static int bk_tty_finish(d_word info);

/*
 * BK-0010 system timer (177706/177710/177712).
 * DWT CYCCNT is a 168 MHz wall-clock source. One base timer tick is
 * 42.9 us (7207.2 core clocks); the timer is independent of PDP execution.
 */
#define BK_TIMER_DWT_CYCCNT ((volatile uint32_t *)0xE0001004u)
#define BK_TIMER_BASE_TICK_SCALED 72072u

static uint16_t bk_timer_preset;
static uint16_t bk_timer_counter;
static uint8_t bk_timer_control;
static uint8_t bk_timer_expiry;
static uint8_t bk_timer_expiry_warmup = 1u;
static uint32_t bk_timer_last_dwt;
static uint64_t bk_timer_phase;

static void bk_timer_reset(void)
{
    bk_timer_preset = 0;
    bk_timer_counter = 0;
    bk_timer_control = 0;
    bk_timer_expiry = 0;
    bk_timer_expiry_warmup = 1u;
    bk_timer_phase = 0;
    bk_timer_last_dwt = *BK_TIMER_DWT_CYCCNT;
}

void bk_timer_update(void)
{
    if ((bk_timer_control & 0x10u) == 0u ||
        (bk_timer_control & 0x01u) != 0u) {
        bk_timer_phase = 0;
        return;
    }

    uint32_t now = *BK_TIMER_DWT_CYCCNT;
    uint32_t elapsed_cycles = now - bk_timer_last_dwt;
    bk_timer_last_dwt = now;

    uint32_t prescaler = 1u;
    if (bk_timer_control & 0x20u) prescaler *= 4u;
    if (bk_timer_control & 0x40u) prescaler *= 16u;

    const uint64_t tick_period = (uint64_t)BK_TIMER_BASE_TICK_SCALED * prescaler;
    uint64_t elapsed_scaled = (uint64_t)elapsed_cycles * 10u;
    if (elapsed_scaled < tick_period - bk_timer_phase) {
        bk_timer_phase += elapsed_scaled;
        return;
    }

    bk_timer_phase += elapsed_scaled;
    uint64_t ticks = bk_timer_phase / tick_period;
    bk_timer_phase %= tick_period;

    if (bk_timer_control & 0x02u) {
        /* Continuous mode: wrap through 177777 without reload or expiry. */
        bk_timer_counter = (uint16_t)(bk_timer_counter - (uint16_t)ticks);
        return;
    }

    uint32_t until_zero = bk_timer_counter ? bk_timer_counter : 65536u;
    if (ticks < until_zero) {
        bk_timer_counter = (uint16_t)(bk_timer_counter - (uint16_t)ticks);
        return;
    }

    ticks -= until_zero;
    uint64_t rollovers = 1u;
    if (bk_timer_control & 0x08u) {
        /* One-shot: stop at zero after the first completed interval. */
        bk_timer_counter = 0;
        bk_timer_control &= (uint8_t)~0x10u;
        bk_timer_phase = 0;
    } else {
        uint32_t interval = bk_timer_preset ? bk_timer_preset : 65536u;
        rollovers += ticks / interval;
        ticks %= interval;
        bk_timer_counter = (uint16_t)(bk_timer_preset - (uint16_t)ticks);
    }

    /* The first expiry transition after power-on is not reported by BK-0010. */
    if ((bk_timer_control & 0x04u) && !(bk_timer_control & 0x02u)) {
        if (bk_timer_expiry_warmup) {
            if (rollovers >= 2u) bk_timer_expiry = 1u;
            bk_timer_expiry_warmup = 0u;
        } else {
            bk_timer_expiry = 1u;
        }
    }
}

static void bk_timer_write_control(uint8_t value)
{
    bk_timer_update();
    bk_timer_control = value & 0x7Fu;
    if ((value & 0x80u) == 0u) bk_timer_expiry = 0u;
    /* A control write loads the preset; bit 4 then enables countdown. */
    bk_timer_counter = bk_timer_preset;
    bk_timer_phase = 0;
    bk_timer_last_dwt = *BK_TIMER_DWT_CYCCNT;
}
uint16_t port0177664 = 01330;   // Screen Scroll / Offset Register

// Audio output transition queue. The emulator is the single producer.
// The PCM renderer and USB-side BLEP stage are independent consumers of the
// same timestamped edges; the oldest consumer cursor controls slot reuse.
#define BK_AUDIO_EDGE_CAPACITY 512u
#define BK_AUDIO_EDGE_MASK (BK_AUDIO_EDGE_CAPACITY - 1u)
// The complete 128 KiB video canvas is reused as a compact audio history
// while the music-only program is running. Each entry occupies eight bytes.
#define BK_AUDIO_DIAG_CAPACITY 16384u
#define BK_AUDIO_DIAG_MASK (BK_AUDIO_DIAG_CAPACITY - 1u)
static uint32_t bk_audio_diag_head;
static uint32_t bk_audio_diag_dropped;
static uint8_t bk_audio_diag_recording;

extern void bk_audio_diag_begin(void);
extern void bk_audio_diag_freeze(void);
extern void bk_audio_diag_store(uint32_t slot, uint32_t cycles,
                                uint8_t level, uint8_t dropped);
static volatile uint16_t bk_audio_edge_head;
static volatile uint16_t bk_audio_edge_render_tail;
static volatile uint16_t bk_audio_edge_blep_tail;
static volatile uint8_t bk_audio_enabled;
static uint8_t bk_audio_level;
static uint32_t bk_audio_edge_cycles[BK_AUDIO_EDGE_CAPACITY];
static uint8_t bk_audio_edge_levels[BK_AUDIO_EDGE_CAPACITY];

static uint16_t bk_audio_oldest_tail(void)
{
    return (int16_t)(bk_audio_edge_render_tail - bk_audio_edge_blep_tail) <= 0
        ? bk_audio_edge_render_tail : bk_audio_edge_blep_tail;
}

static void bk_audio_record_level(uint8_t level)
{
    level = level ? 1u : 0u;
    if (level == bk_audio_level) return;

    uint32_t cycles = bk_timing_cycles();
    uint16_t head = bk_audio_edge_head;
    uint8_t dropped =
        (uint16_t)(head - bk_audio_oldest_tail()) >= BK_AUDIO_EDGE_CAPACITY;

    if (bk_audio_diag_recording) {
        uint32_t diag_slot = bk_audio_diag_head & BK_AUDIO_DIAG_MASK;
        bk_audio_diag_store(diag_slot, cycles, level, dropped);
        bk_audio_diag_head++;
        if (dropped) bk_audio_diag_dropped++;
    }

    if (dropped) return;

    bk_audio_level = level;
    uint16_t slot = head & BK_AUDIO_EDGE_MASK;
    bk_audio_edge_cycles[slot] = cycles;
    bk_audio_edge_levels[slot] = level;
    __asm volatile("dmb" ::: "memory");
}

uint32_t bk_audio_diag_count(void)
{
    return bk_audio_diag_head < BK_AUDIO_DIAG_CAPACITY
        ? bk_audio_diag_head : BK_AUDIO_DIAG_CAPACITY;
}

void bk_audio_diag_freeze(void)
{
    bk_audio_diag_recording = 0u;
    __asm volatile("dmb" ::: "memory");
}

uint32_t bk_audio_diag_dropped_count(void)
{
    return bk_audio_diag_dropped;
}



uint8_t bk_audio_start(void)
{
    bk_audio_edge_render_tail = bk_audio_edge_head;
    bk_audio_edge_blep_tail = bk_audio_edge_head;
    bk_audio_diag_head = 0;
    bk_audio_diag_dropped = 0;
    bk_audio_diag_recording = 1u;
    bk_audio_diag_begin();
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

static uint8_t bk_audio_pop_edge_from(volatile uint16_t *tail_ptr, uint32_t *cycles, uint8_t *level)
{
    uint16_t tail = *tail_ptr;
    if (tail == bk_audio_edge_head || cycles == NULL || level == NULL) {
        return 0u;
    }

    __asm volatile("dmb" ::: "memory");
    uint16_t slot = tail & BK_AUDIO_EDGE_MASK;
    *cycles = bk_audio_edge_cycles[slot];
    *level = bk_audio_edge_levels[slot];
    __asm volatile("dmb" ::: "memory");
    *tail_ptr = (uint16_t)(tail + 1u);
    return 1u;
}

uint8_t bk_audio_pop_edge(uint32_t *cycles, uint8_t *level)
{
    return bk_audio_pop_edge_from(&bk_audio_edge_render_tail, cycles, level);
}

uint8_t bk_audio_pop_blep_edge(uint32_t *cycles, uint8_t *level)
{
    return bk_audio_pop_edge_from(&bk_audio_edge_blep_tail, cycles, level);
}

// STOP key requests the independent hardware IRQ1 (priority 5, vector 000004).
static volatile uint8_t stop_irq_pending = 0;

static const uint8_t *monitor_rom;
static const uint8_t *basic_rom;
static uint32_t basic_rom_size;

extern int bk_video_read_byte(uint32_t addr, uint8_t mode, uint8_t *value);
extern int bk_video_write_byte(uint32_t addr, uint8_t mode, uint8_t value);
extern void bk_video_mode_changed(uint8_t mode);
extern void bk_video_shift_changed(uint16_t shift);
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

// Apply the short BIN header exactly as the BK monitor does: the header
// is received into system cells 264/266 before the payload is transferred.
// A payload that overlaps those cells must therefore overwrite them.
int32_t bk_load_bin_prepare(uint16_t load_addr, uint16_t data_len)
{
    const uint32_t memory_end = 0100000u;
    if ((uint32_t)load_addr + data_len > memory_end) {
        return -1;
    }

    // F3 full-memory images span base RAM and the reclaimed 12 KiB
    // extended program-RAM window. Select that map before streaming the
    // payload so writes at 0x4000..0x6FFF reach hidden RAM, not video RAM.
    if (load_addr == 0x0200u && data_len == 0x6E00u) {
        port0177664 &= (uint16_t)~0x0200u;
        bk_video_shift_changed(port0177664);
    }

    if (sl_word(&pdp, 0000264, load_addr) != OK ||
        sl_word(&pdp, 0000266, data_len) != OK) {
        return -1;
    }
    return 0;
}

// Stream a .bin payload into emulated memory without persistent loader state.
int32_t bk_load_bin_write(uint16_t load_addr, uint32_t offset,
                          const uint8_t *data, size_t length)
{
    const uint32_t memory_end = 0100000u;
    const uint32_t start = (uint32_t)load_addr;

    if (data == NULL && length != 0) return -1;
    if (start >= memory_end || offset > memory_end - start ||
        length > memory_end - start - offset) {
        return -1;
    }

    for (size_t i = 0; i < length; ++i) {
        const c_addr addr = start + offset + (uint32_t)i;
        if (sl_byte(&pdp, addr, data[i]) != OK) {
            return -1;
        }
    }
    return 0;
}

// Returns 1 when execution is started, 0 for a full-screen image dump,
// and -1 on a memory access error.
int32_t bk_load_bin_finish(uint16_t load_addr, uint16_t data_len)
{
    const uint32_t memory_end = 0100000u;
    if ((uint32_t)load_addr + data_len > memory_end) {
        return -1;
    }

    // Header system cells were populated by bk_load_bin_prepare() before
    // payload transfer, matching the monitor's receive order.

    // A complete screen dump updates video memory but leaves the monitor active.
    if (load_addr == 0040000 && data_len == 0040000) {
        return 0;
    }

    // The BK-0010 monitor starts a program with CALL (R5) at 100376.
    // Its return address is therefore 100400. The monitor enters this call
    // with SP=1000; emulate the JSR PC,(R5) stack effect explicitly.
    pdp.regs[SP] = 001000;
    pdp.regs[SP] = (uint16_t)(pdp.regs[SP] - 2u);
    if (sl_word(&pdp, pdp.regs[SP], 0100400) != OK) {
        return -1;
    }
    pdp.regs[PC] = load_addr;

    // MOV 264,R5 in the monitor sets N/Z and clears V; CALL (R5) preserves
    // the condition codes. Preserve C as MOV does.
    pdp.psw &= CC_C;
    if (load_addr & 0100000u) {
        pdp.psw |= CC_N;
    }
    if (load_addr == 0) {
        pdp.psw |= CC_Z;
    }
    return 1;
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
    port0177664 = 01330;
    port0177716 = 0x0040;
    keyboard_key_pressed = 0;
    sel1_write_flag = 0;
    keyboard_irq_info = 0;
    bk_timer_reset();
    port0177714 = 0;
    bk_rtc_latched_millis = 0;
    bk_audio_enabled = 0;
    bk_audio_level = 0;
    bk_audio_edge_head = 0;
    bk_audio_edge_render_tail = 0;
    bk_audio_edge_blep_tail = 0;
    bk_audio_diag_head = 0;
    bk_audio_diag_dropped = 0;
    stop_irq_pending = 0;
    io_stop_happened = 0;
    last_branch = 0;
    in_wait_instr = 0;
    ev_init();
    bk_video_mode_changed(RamBuffer[0x0020]);
    bk_video_shift_changed(port0177664);
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
        case 0177706: bk_timer_update(); *byte = (uint8_t)bk_timer_preset; break;
        case 0177707: bk_timer_update(); *byte = (uint8_t)(bk_timer_preset >> 8); break;
        case 0177710: bk_timer_update(); *byte = (uint8_t)bk_timer_counter; break;
        case 0177711: bk_timer_update(); *byte = (uint8_t)(bk_timer_counter >> 8); break;
        case 0177712:
            bk_timer_update();
            *byte = (uint8_t)(bk_timer_control | (bk_timer_expiry ? 0x80u : 0u));
            break;
        case 0177713: *byte = 0xFFu; break;
        case 0177665: *byte = (uint8_t)(port0177664 >> 8); break;
        case 0177714: *byte = (uint8_t)port0177714; break;
        case 0177715: *byte = (uint8_t)(port0177714 >> 8); break;
        case 0177716:
            *byte = (uint8_t)((port0177716 & (uint16_t)~0x0044u) |
                (keyboard_key_pressed ? 0u : 0x0040u) |
                (sel1_write_flag ? 0x0004u : 0u));
            sel1_write_flag = 0;
            break;
        case 0177717:
            *byte = (uint8_t)(port0177716 >> 8);
            sel1_write_flag = 0;
            break;
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
        case 0177706: bk_timer_update(); *word = bk_timer_preset; break;
        case 0177710: bk_timer_update(); *word = bk_timer_counter; break;
        case 0177712:
            bk_timer_update();
            *word = (uint16_t)(0xFF00u | bk_timer_control | (bk_timer_expiry ? 0x80u : 0u));
            break;
        case 0177714: *word = port0177714; break;
        case 0177716:
            *word = (uint16_t)((port0177716 & (uint16_t)~0x0044u) |
                (keyboard_key_pressed ? 0u : 0x0040u) |
                (sel1_write_flag ? 0x0004u : 0u));
            sel1_write_flag = 0;
            break;
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
            if ((port0177660 & 0x40u) == 0u) bk_schedule_keyboard_irq();
            break;
        case 0177662: port0177662 = value; break;
        case 0177663: port0177662 = (uint8_t)((port0177662 & 0x00FFu) | ((uint16_t)value << 8)); break;
        case 0177706:
            bk_timer_update();
            bk_timer_preset = (uint16_t)((bk_timer_preset & 0xFF00u) | value);
            break;
        case 0177707:
            bk_timer_update();
            bk_timer_preset = (uint16_t)((bk_timer_preset & 0x00FFu) | ((uint16_t)value << 8));
            break;
        case 0177710:
        case 0177711:
            /* Countdown register is read-only. */
            break;
        case 0177712:
            bk_timer_write_control(value);
            break;
        case 0177713:
            /* Only the low byte of control/status is implemented. */
            break;
        case 0177664:
            port0177664 = (uint16_t)((port0177664 & 0xFF00u) | value);
            bk_video_shift_changed(port0177664);
            break;
        case 0177665:
            port0177664 = (uint16_t)((port0177664 & 0x00FFu) | ((uint16_t)(value & 0x02u) << 8));
            bk_video_shift_changed(port0177664);
            break;
        case 0177714:
            port0177714 = (uint16_t)((port0177714 & 0xFF00u) | value);
            break;
        case 0177715:
            port0177714 = (uint16_t)((port0177714 & 0x00FFu) | ((uint16_t)value << 8));
            break;
        case 0177716:
            port0177716 = (uint16_t)((port0177716 & 0xFF00u) | value);
            sel1_write_flag = 1;
            if (bk_audio_enabled) {
                bk_audio_record_level((uint8_t)(port0177716 & 0x0040u));
            }
            break;
        case 0177717:
            port0177716 = (uint16_t)((port0177716 & 0x00FFu) | ((uint16_t)value << 8));
            sel1_write_flag = 1;
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
            if ((port0177660 & 0x40u) == 0u) bk_schedule_keyboard_irq();
            break;
        case 0177706:
            bk_timer_update();
            bk_timer_preset = value;
            break;
        case 0177710:
            /* Countdown register is read-only. */
            break;
        case 0177712:
            bk_timer_write_control((uint8_t)value);
            break;
        case 0177664:
            port0177664 = value & 01377;
            bk_video_shift_changed(port0177664);
            break;
        case 0177714:
            port0177714 = value;
            break;
        case 0177716:
            port0177716 = value;
            sel1_write_flag = 1;
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
void bk_set_keyboard_pressed(uint8_t pressed)
{
    keyboard_key_pressed = pressed ? 1u : 0u;
}

void bk_request_stop_irq(void)
{
    stop_irq_pending = 1;
}

/* Schedule a keyboard interrupt only when the peripheral interrupt mask
 * is clear. Data-ready remains set while masked, allowing polling; clearing
 * the mask later makes the still-pending character interrupt eligible. */
static void bk_schedule_keyboard_irq(void)
{
    if ((port0177660 & 0xC0u) == 0x80u) {
        (void)ev_register(TTY_PRI, bk_tty_finish, 0, keyboard_irq_info);
    }
}

/* Original BK keyboard input path: wait for the previous character's
 * interrupt to be acknowledged, then schedule the next TTY interrupt. */
static int bk_tty_finish(d_word info)
{
    // The event may have been queued before software masked keyboard IRQs.
    if ((port0177660 & 0xC0u) != 0x80u) return OK;
    // Bit 7 is an adapter-side АР2 marker, not part of 177662.
    return service((info & 0x80u) ? 0274 : 060);
}

uint8_t bk_try_send_input_byte(uint8_t symbol)
{
    if (port0177660 & 0x80u) return 0u;
    if ((symbol & 0x7Fu) == 0x0Du) symbol = (uint8_t)((symbol & 0x80u) | 0x0Au);
    port0177660 |= 0x80u;
    port0177662 = symbol & 0x7Fu;
    // Preserve the modifier marker until the queued interrupt is serviced.
    keyboard_irq_info = symbol;
    bk_schedule_keyboard_irq();
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
