#![no_std]
#![no_main]

mod app;
mod board;
mod file_browser;
mod keyboard;
mod usb_host;

use board::TICKS;
use core::sync::atomic::Ordering;
use cortex_m_rt::{entry, exception};
use panic_halt as _;
use stm32f4::stm32f407::interrupt;
use xcore407i_blinker::tusb;

#[entry]
fn main() -> ! {
    app::run()
}

#[interrupt]
#[allow(non_snake_case)]
fn ETH() {
    xcore407i_blinker::ethernet::on_dma_interrupt();
}

#[interrupt]
#[allow(non_snake_case)]
fn OTG_FS() {
    unsafe {
        tusb::hcd_int_handler(0, true);
    }
}

#[interrupt]
#[allow(non_snake_case)]
fn OTG_HS() {
    unsafe {
        tusb::tusb_int_handler(1, true);
    }
}

#[interrupt]
#[allow(non_snake_case)]
fn TIM3() {
    usb_host::on_tim3_interrupt();
}

#[exception]
#[allow(non_snake_case)]
fn SysTick() {
    TICKS.fetch_add(1, Ordering::Relaxed);
}
