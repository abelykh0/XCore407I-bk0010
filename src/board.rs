use cortex_m::peripheral::{scb::SystemHandler, NVIC, SCB};
use stm32f4::stm32f407::interrupt;
use stm32f4xx_hal::gpio::{GpioExt, Speed};
use stm32f4xx_hal::pac;

const SYSTICK_PRIORITY: u8 = 1;
pub(crate) static TICKS: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

// TinyUSB's no-RTOS time source. SysTick increments TICKS once per millisecond.
#[unsafe(no_mangle)]
pub extern "C" fn tusb_time_millis_api() -> u32 {
    use core::sync::atomic::Ordering;
    TICKS.load(Ordering::Relaxed)
}

// DWC2 STM32 clock setup reads this CMSIS system-clock symbol.
#[unsafe(no_mangle)]
pub static SystemCoreClock: u32 = 168_000_000;

// C BK platform callback: return the low 32 bits of Unix milliseconds (UTC).
#[unsafe(no_mangle)]
pub extern "C" fn bk_rtc_unix_millis() -> u32 {
    let rtc = unsafe { &*pac::RTC::ptr() };
    xcore407i_blinker::rtc::unix_time_millis(rtc)
}

/// Set a simple preemption-priority grouping and the system-handler priorities.
/// STM32F407 implements four priority bits, stored in the upper nibble.
pub(crate) fn configure_exception_priorities(scb: &mut SCB) {
    const SCB_AIRCR: *mut u32 = 0xE000_ED0C as *mut u32;

    unsafe {
        // PRIGROUP=0: all implemented priority bits are preemption priority.
        let aircr = core::ptr::read_volatile(SCB_AIRCR);
        core::ptr::write_volatile(SCB_AIRCR, (aircr & !(7 << 8)) | (0x5FA << 16));

        // USB controller interrupts preempt the emulator loop. SysTick
        // remains enabled at priority 1 for TinyUSB's millisecond timebase.
        scb.set_priority(SystemHandler::SysTick, SYSTICK_PRIORITY << 4);
        // OTG_HS is unmasked after successful TinyUSB device initialization.
        NVIC::mask(interrupt::OTG_HS);
        NVIC::unpend(interrupt::OTG_HS);
    }
}

#[cfg(feature = "cpu-cycle-diag")]
#[inline]
pub(crate) fn read_dwt_cycle_count() -> u32 {
    // ARM DWT_CYCCNT at 0xE000_1004; 32-bit wrap is handled by wrapping_sub.
    unsafe { core::ptr::read_volatile(0xE000_1004 as *const u32) }
}

#[cfg(feature = "cpu-cycle-diag")]
pub(crate) fn enable_cycle_counter(core: &mut cortex_m::Peripherals) {
    // DWT access is provided by cortex-m; global trace must be enabled first.
    core.DCB.enable_trace();
    core.DWT.set_cycle_count(0);
    core.DWT.enable_cycle_counter();
}
 
pub(crate) fn system_clock_168mhz(rcc: &pac::RCC, pwr: &pac::PWR, flash: &pac::FLASH) {
    // The board has the 8 MHz HSE crystal used by the original firmware.
    rcc.apb1enr().modify(|_, w| w.pwren().set_bit());
    pwr.cr().modify(|_, w| w.vos().set_bit()); // scale 1

    // Enable instruction/data caches, prefetch, and five wait states.
    flash.acr().write(|w| unsafe { w.bits((1 << 10) | (1 << 9) | (1 << 8) | 5) });

    rcc.cr().modify(|_, w| w.hseon().set_bit());
    while !rcc.cr().read().hserdy().bit_is_set() {}

    // PLLM=4, PLLN=168, PLLP=/2, PLL source=HSE, PLLQ=7.
    rcc.pllcfgr().write(|w| unsafe {
        w.bits(4 | (168 << 6) | (1 << 22) | (7 << 24))
    });

    rcc.cr().modify(|_, w| w.pllon().set_bit());
    while !rcc.cr().read().pllrdy().bit_is_set() {}

    // HCLK=/1, APB1=/4, APB2=/2, then switch SYSCLK to PLL.
    rcc.cfgr().modify(|_, w| {
        w.hpre().div1()
            .ppre1().div4()
            .ppre2().div2()
            .sw().pll()
    });

    while rcc.cfgr().read().sws().bits() != 2 {}
}

/// Configure the Ethernet MDIO/MDC and USB3300 ULPI pin muxes.
///
/// The HAL owns the GPIO register layout and alternate-function mapping. The
/// pins are intentionally dropped after configuration; their hardware state
/// remains configured for the peripheral.
pub(crate) fn configure_board_pins(
    rcc: &mut pac::RCC,
    gpioa: pac::GPIOA,
    gpiob: pac::GPIOB,
    gpioc: pac::GPIOC,
    gpiod: pac::GPIOD,
    gpioe: pac::GPIOE,
    gpioh: pac::GPIOH,
    gpioi: pac::GPIOI,
) {
    let a = gpioa.split(rcc);
    let b = gpiob.split(rcc);
    let c = gpioc.split(rcc);
    let d = gpiod.split(rcc);
    let e = gpioe.split(rcc);
    let h = gpioh.split(rcc);
    let i = gpioi.split(rcc);

    // XCore407I NAND flash uses FSMC AF12. CLE=A17 (PD12), ALE=A16 (PD11),
    // NCE2=PD7, NWAIT=PD6, NWE=PD5, NOE=PD4, D0..D3=PD14/15/0/1,
    // and D4..D7=PE7..PE10.
    let _nand_pins = (
        d.pd0.into_alternate::<12>().speed(Speed::VeryHigh),
        d.pd1.into_alternate::<12>().speed(Speed::VeryHigh),
        d.pd4.into_alternate::<12>().speed(Speed::VeryHigh),
        d.pd5.into_alternate::<12>().speed(Speed::VeryHigh),
        d.pd6.into_alternate::<12>().speed(Speed::VeryHigh),
        d.pd7.into_alternate::<12>().speed(Speed::VeryHigh),
        d.pd11.into_alternate::<12>().speed(Speed::VeryHigh),
        d.pd12.into_alternate::<12>().speed(Speed::VeryHigh),
        d.pd14.into_alternate::<12>().speed(Speed::VeryHigh),
        d.pd15.into_alternate::<12>().speed(Speed::VeryHigh),
        e.pe7.into_alternate::<12>().speed(Speed::VeryHigh),
        e.pe8.into_alternate::<12>().speed(Speed::VeryHigh),
        e.pe9.into_alternate::<12>().speed(Speed::VeryHigh),
        e.pe10.into_alternate::<12>().speed(Speed::VeryHigh),
    );

    // USB FS host data pins use the internal PHY (AF10).
    let _usb_fs_pins = (
        a.pa11.into_alternate::<10>().speed(Speed::VeryHigh), // OTG_FS_DM
        a.pa12.into_alternate::<10>().speed(Speed::VeryHigh), // OTG_FS_DP
    );

    // XCore407I schematic: PC1 is FS_VBUSEN / OTG_PWR_OUT. The FS port's
    // MIC2025/75 power switch is enabled by driving EN low. PC1 is also
    // routed as Ethernet MDC on this board, so Ethernet MDIO management is
    // intentionally disabled while USB FS host VBUS is enabled.
    unsafe {
        // Use the PAC's GPIOC peripheral pointer instead of duplicating its
        // STM32F407 peripheral base address.
        let gpioc = pac::GPIOC::ptr().cast::<u8>() as *mut u8;
        let bsrr = gpioc.add(0x18).cast::<u32>();
        let moder = gpioc.cast::<u32>();
        let otyper = gpioc.add(0x04).cast::<u32>();
        let ospeedr = gpioc.add(0x08).cast::<u32>();
        let pupdr = gpioc.add(0x0c).cast::<u32>();

        // Drive low before switching the pin from its reset input state.
        core::ptr::write_volatile(bsrr, 1 << (1 + 16));
        core::ptr::write_volatile(moder, (core::ptr::read_volatile(moder) & !(0b11 << 2)) | (0b01 << 2));
        core::ptr::write_volatile(otyper, core::ptr::read_volatile(otyper) & !(1 << 1));
        core::ptr::write_volatile(ospeedr, core::ptr::read_volatile(ospeedr) & !(0b11 << 2));
        core::ptr::write_volatile(pupdr, core::ptr::read_volatile(pupdr) & !(0b11 << 2));
    }

    // USB3300 ULPI: all data/control pins use AF10 except PA4 SOF (AF12).
    // The original firmware selected very-high GPIO speed and no pull resistors.
    let _ulpi_pins = (
        a.pa3.into_alternate::<10>().speed(Speed::VeryHigh),  // D0
        a.pa5.into_alternate::<10>().speed(Speed::VeryHigh),  // CLK
        a.pa4.into_alternate::<12>().speed(Speed::VeryHigh),  // SOF
        b.pb0.into_alternate::<10>().speed(Speed::VeryHigh),  // D1
        b.pb1.into_alternate::<10>().speed(Speed::VeryHigh),  // D2
        b.pb10.into_alternate::<10>().speed(Speed::VeryHigh), // D3
        b.pb11.into_alternate::<10>().speed(Speed::VeryHigh), // D4
        b.pb12.into_alternate::<10>().speed(Speed::VeryHigh), // D5
        b.pb13.into_alternate::<10>().speed(Speed::VeryHigh), // D6
        b.pb5.into_alternate::<10>().speed(Speed::VeryHigh),  // D7
        c.pc0.into_alternate::<10>().speed(Speed::VeryHigh),  // STP
        h.ph4.into_alternate::<10>().speed(Speed::VeryHigh),  // NXT
        i.pi11.into_alternate::<10>().speed(Speed::VeryHigh), // DIR
    );
}
