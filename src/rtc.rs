
// ! STM32F407 RTC initialization and cached 24-hour clock display.
//
// Register access goes through the STM32F407 PAC. The RTC remains backed by
// LSE and the backup-domain calendar is only initialized on first use.

use core::sync::atomic::{AtomicU32, Ordering};
use stm32f4::stm32f407::{PWR, RCC, RTC};

static DISPLAY_HHMMSS: AtomicU32 = AtomicU32::new(0);

/// Start the LSE-backed RTC and initialize it to 00:00:00 only if its calendar
/// has never been initialized. Existing backup-domain time is preserved.
pub fn initialize(rcc: &RCC, pwr: &PWR, rtc: &RTC) -> bool {

    // Enable the PWR peripheral clock.
    rcc.apb1enr().modify(|_, w| w.pwren().set_bit());

    // Enable access to the RTC backup domain.
    pwr.cr().modify(|_, w| w.dbp().set_bit());

    // Initialize the backup-domain calendar only once.
    if !rtc.isr().read().inits().bit_is_set() {
        let clock_source = rcc.bdcr().read().rtcsel().bits();

        // Do not change an already-selected non-LSE clock source.
        if clock_source != 0 && clock_source != 1 {
            return false;
        }

        // Enable LSE.
        rcc.bdcr().modify(|_, w| w.lseon().set_bit());

        let mut timeout = 100_000_000u32;
        while !rcc.bdcr().read().lserdy().bit_is_set() && timeout != 0 {
            timeout -= 1;
        }

        if !rcc.bdcr().read().lserdy().bit_is_set() {
            return false;
        }

        // Select LSE as RTC clock source and enable RTC.
        rcc.bdcr().modify(|_, w| {
            w.rtcsel().lse().rtcen().set_bit()
        });

        // Unlock RTC write protection.
        write_protection_key(rtc, 0xca);
        write_protection_key(rtc, 0x53);

        // Enter RTC initialization mode.
        rtc.isr().modify(|_, w| w.init().set_bit());

        let mut timeout = 1_000_000u32;
        while !rtc.isr().read().initf().bit_is_set() && timeout != 0 {
            timeout -= 1;
        }

        if !rtc.isr().read().initf().bit_is_set() {
            write_protection_key(rtc, 0xff);
            return false;
        }

        // Configure prescalers for a 1 Hz calendar clock.
        // LSE = 32768 Hz:
        // 32768 / (127 + 1) / (255 + 1) = 1 Hz.
        //
        // RM0090 requires two separate writes, synchronous first.
        rtc.prer().modify(|_, w| unsafe {
            w.prediv_s().bits(255)
        });

        rtc.prer().modify(|_, w| unsafe {
            w.prediv_a().bits(127)
        });

        // Use 24-hour format.
        rtc.cr().modify(|_, w| w.fmt().clear_bit());

        // Initialize time to midnight.
        rtc.tr().write(|w| unsafe {
            w.bits(0)
        });

        // Initialize date to 2000-01-01 (Saturday).
        // RTC weekday encoding: Monday = 1, Saturday = 6.
        rtc.dr().write(|w| unsafe {
            w.bits((6 << 13) | (1 << 8) | 1)
        });

        // Leave initialization mode and restart the calendar.
        rtc.isr().modify(|_, w| w.init().clear_bit());

        // Lock RTC write protection.
        write_protection_key(rtc, 0xff);
    }

    // Synchronize RTC shadow registers after CPU reset.
    rtc.isr().modify(|_, w| w.rsf().clear_bit());

    let mut timeout = 1_000_000u32;
    while !rtc.isr().read().rsf().bit_is_set() && timeout != 0 {
        timeout -= 1;
    }

    if !rtc.isr().read().rsf().bit_is_set() {
        return false;
    }

    refresh_display_time(rtc);
    true
}

/// Write the RTC write-protection key.
#[inline]
fn write_protection_key(
    rtc: &stm32f4::stm32f407::rtc::RegisterBlock,
    key: u8,
) {
    // SAFETY: The value is one of the documented RTC write-protection keys.
    rtc.wpr().write(|w| unsafe {
        w.bits(key as u32)
    });
}

/// Refresh the overlay's cached time. Call this about once per second.
pub fn refresh_display_time(rtc: &RTC) {

    let time = rtc.tr().read().bits();

    // Reading the date after the time completes the shadow-register sequence.
    let _date = rtc.dr().read().bits();

    let Some(hours) = decode_bcd((time >> 16) & 0x3f) else {
        return;
    };
    let Some(minutes) = decode_bcd((time >> 8) & 0x7f) else {
        return;
    };
    let Some(seconds) = decode_bcd(time & 0x7f) else {
        return;
    };

    if hours >= 24 || minutes >= 60 || seconds >= 60 {
        return;
    }

    DISPLAY_HHMMSS.store(
        (hours as u32) * 10_000 + (minutes as u32) * 100 + seconds as u32,
        Ordering::Relaxed,
    );
}

/// Return the cached time with one blank character on each side.
pub(crate) fn display_time() -> [u8; 10] {
    let time = DISPLAY_HHMMSS.load(Ordering::Relaxed);
    let hours = (time / 10_000) as u8;
    let minutes = ((time / 100) % 100) as u8;
    let seconds = (time % 100) as u8;

    [
        b' ',
        b'0' + hours / 10,
        b'0' + hours % 10,
        b':',
        b'0' + minutes / 10,
        b'0' + minutes % 10,
        b':',
        b'0' + seconds / 10,
        b'0' + seconds % 10,
        b' ',
    ]
}

fn decode_bcd(value: u32) -> Option<u8> {
    let units = (value & 0x0f) as u8;
    let tens = ((value >> 4) & 0x0f) as u8;

    if units > 9 || tens > 9 {
        return None;
    }

    Some(tens * 10 + units)
}
