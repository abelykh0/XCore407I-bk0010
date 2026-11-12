use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use cortex_m::peripheral::NVIC;
use stm32f4::stm32f407::interrupt;
use stm32f4xx_hal::pac;
use xcore407i_blinker::tusb;

const TIM3_PRIORITY: u8 = 3;
pub(crate) const USB_FS_RETRY_INTERVAL_MS: u32 = 500;
pub(crate) static USB_FS_STATUS: AtomicU32 = AtomicU32::new(0);
pub(crate) static USB_FS_READY: AtomicBool = AtomicBool::new(false);
pub(crate) static USB_FS_RETRY_AT: AtomicU32 = AtomicU32::new(0);
pub(crate) static USB_HOST_TASK_PENDING: AtomicBool = AtomicBool::new(false);

#[unsafe(no_mangle)]
pub extern "C" fn tuh_enum_descriptor_device_cb(_dev_addr: u8, _desc: *const tusb::tusb_desc_device_t) {
    USB_FS_STATUS.store(7, Ordering::Release);
}

#[unsafe(no_mangle)]
pub extern "C" fn tuh_enum_descriptor_configuration_cb(_dev_addr: u8, _cfg_index: u8, _desc: *const tusb::tusb_desc_configuration_t) -> bool {
    USB_FS_STATUS.store(8, Ordering::Release);
    true
}

#[unsafe(no_mangle)]
pub extern "C" fn tuh_mount_cb(_dev_addr: u8) { USB_FS_STATUS.store(3, Ordering::Release); }

#[unsafe(no_mangle)]
pub extern "C" fn tuh_umount_cb(_dev_addr: u8) { USB_FS_STATUS.store(5, Ordering::Release); }
 
pub(crate) fn try_init_usb_fs_host(rcc: &pac::RCC, nvic: &mut NVIC) -> bool {
    unsafe {
        nvic.set_priority(interrupt::OTG_FS, 0x3 << 4);
        NVIC::unpend(interrupt::OTG_FS);
        NVIC::mask(interrupt::OTG_FS);
    }

    // Enable the peripheral clock, then reset only OTG_FS. OTG_HS is
    // independent and carries the TinyUSB composite UVC/UAC2 device.
    rcc.ahb2enr().modify(|_, w| w.otgfsen().set_bit());
    let _ = rcc.ahb2enr().read().bits();

    rcc.ahb2rstr().modify(|_, w| w.otgfsrst().set_bit());
    for _ in 0..16 {
        unsafe { core::arch::asm!("nop"); }
    }
    rcc.ahb2rstr().modify(|_, w| w.otgfsrst().clear_bit());
    let _ = rcc.ahb2rstr().read().bits();

    let rh_init = tusb::tusb_rhport_init_t {
        role: tusb::tusb_role_t::TUSB_ROLE_HOST,
        speed: tusb::tusb_speed_t::TUSB_SPEED_FULL,
    };

    if unsafe { tusb::tuh_rhport_init(0, &rh_init) } {
        USB_FS_STATUS.store(1, Ordering::Release);
        USB_FS_READY.store(true, Ordering::Release);
        USB_HOST_TASK_PENDING.store(false, Ordering::Release);
        unsafe {
            NVIC::unpend(interrupt::OTG_FS);
            NVIC::unmask(interrupt::OTG_FS);
        }
        true
    } else {
        USB_FS_STATUS.store(2, Ordering::Release);
        USB_FS_READY.store(false, Ordering::Release);
        USB_HOST_TASK_PENDING.store(false, Ordering::Release);
        NVIC::unpend(interrupt::OTG_FS);
        NVIC::mask(interrupt::OTG_FS);
        false
    }
}

pub(crate) fn configure_usb_host_task_timer(
    rcc: &pac::RCC,
    tim3: &pac::TIM3,
    nvic: &mut NVIC,
) {
    // TIM3CLK = 84 MHz (SYSCLK=168 MHz, APB1=/4, timer clock doubled).
    // PSC=83 divides 84 MHz to 1 MHz; ARR=999 gives a 1 kHz update rate.

    rcc.apb1enr().modify(|_, w| w.tim3en().set_bit());

    rcc.apb1rstr().modify(|_, w| w.tim3rst().set_bit());
    rcc.apb1rstr().modify(|_, w| w.tim3rst().clear_bit());

    tim3.cr1().write(|w| w.cen().clear_bit());
    tim3.dier().write(|w| w.uie().clear_bit());
    tim3.psc().write(|w| unsafe { w.psc().bits(83) });
    tim3.arr().write(|w| unsafe { w.arr().bits(999) });
    tim3.egr().write(|w| w.ug().set_bit());
    tim3.sr().write(|w| w.uif().clear_bit());

    unsafe {
        nvic.set_priority(interrupt::TIM3, TIM3_PRIORITY << 4);
        NVIC::unpend(interrupt::TIM3);
        NVIC::unmask(interrupt::TIM3);
    }

    tim3.dier().write(|w| w.uie().set_bit());
    tim3.cr1().write(|w| w.cen().set_bit());
}
 
pub(crate) fn on_tim3_interrupt() {
    let tim3 = unsafe { &*pac::TIM3::ptr() };
    tim3.sr().write(|w| w.uif().clear_bit());
    USB_HOST_TASK_PENDING.store(true, Ordering::Release);
}
