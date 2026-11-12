#![no_std]
#![no_main]

mod board;
mod file_browser;
mod keyboard;
mod keyboard_map;
mod usb_host;

use core::ptr;
use core::sync::atomic::Ordering;
use cortex_m::peripheral::{syst::SystClkSource, NVIC};
use cortex_m_rt::{entry, exception};
use panic_halt as _;
use stm32f4::stm32f407::interrupt;
use stm32f4xx_hal::pac;
use xcore407i_blinker::camera::{clear_demo_canvas, show_usb_status, Nv12DemoImage};
#[cfg(feature = "dhara-eval")]
use xcore407i_blinker::camera::show_dhara_spare_inventory;
#[cfg(feature = "cpu-cycle-diag")]
use xcore407i_blinker::camera::show_pdp_cycles_per_second;
use xcore407i_blinker::uvc::prepare_next_video_batch;
use xcore407i_blinker::tusb;
use xcore407i_blinker::rtc;

use board::{
    configure_board_pins, configure_exception_priorities, system_clock_168mhz, TICKS,
};
#[cfg(feature = "cpu-cycle-diag")]
use board::{enable_cycle_counter, read_dwt_cycle_count};
use file_browser::{
    build_save_path, draw_file_browser, draw_save_dialog, list_root_files,
    load_tetris_from_path, save_file_exists, save_ram_to_path,
    BROWSER_PATHS, BROWSER_PATH_LENGTHS,
    FILE_BROWSER_ACTION, FILE_BROWSER_ACTIVE, FILE_BROWSER_OPEN, FILE_BROWSER_UP,
    FILE_BROWSER_DOWN, FILE_BROWSER_ENTER, FILE_BROWSER_ESCAPE,
    MAX_BROWSER_FILES, MAX_BROWSER_PATH,
    SAVE_DIALOG_IDLE, SAVE_DIALOG_INPUT, SAVE_DIALOG_STATE, SAVE_DIALOG_CHECK, SAVE_DIALOG_CONFIRM, SAVE_DIALOG_WRITE,
    SAVE_DIALOG_DIRTY, SAVE_DIALOG_RESULT, SAVE_OVERWRITE,
    SAVE_RESULT_CANCELLED, SAVE_RESULT_OK, SAVE_RESULT_ERROR,
};
use keyboard::{keyboard_queue_discard, service_keyboard_frame_pacing};
use usb_host::{
    configure_usb_host_task_timer, try_init_usb_fs_host, USB_FS_READY,
    USB_FS_RETRY_AT, USB_FS_RETRY_INTERVAL_MS, USB_HOST_TASK_PENDING,
};

const PDP_CYCLES_PER_SECOND: u32 = 3_010_000;

unsafe extern "C" {
    fn bk_platform_init(monitor_rom: *const u8, basic_rom: *const u8, basic_size_bytes: u32);
    fn bk_loop() -> i32;
    fn bk_timing_cycles() -> u32;
}

#[entry]
fn main() -> ! {
    let mut device = pac::Peripherals::take().unwrap();
    system_clock_168mhz(&device.RCC, &device.PWR, &device.FLASH);

    let mut core = cortex_m::Peripherals::take().unwrap();
    core.SCB.enable_fpu();

    rtc::initialize(&device.RCC, &device.PWR, &device.RTC);

    #[cfg(feature = "cpu-cycle-diag")]
    enable_cycle_counter(&mut core);

    // BK emulator initialization
    unsafe {
         bk_platform_init(
             xcore407i_blinker::rom::MONITOR_ROM.as_ptr(),
             xcore407i_blinker::rom::BASIC_ROM.as_ptr(),
             xcore407i_blinker::rom::BASIC_ROM.len() as u32,
         );
    }

    // initialize_demo_canvas();
    clear_demo_canvas(0x10);

    configure_board_pins(
        &mut device.RCC,
        device.GPIOA,
        device.GPIOB,
        device.GPIOC,
        device.GPIOD,
        device.GPIOE,
        device.GPIOH,
        device.GPIOI,
    );

    // Read only the factory markers to inventory blocks reserved for Dhara.
    // This does not mount LittleFS or program/erase any NAND block.
    #[cfg(feature = "dhara-eval")]
    let _ = xcore407i_blinker::nand_fs::NandStorage::scan_dhara_spare_inventory();
    #[cfg(feature = "dhara-eval-erase-spares")]
    let _ = xcore407i_blinker::nand_fs::erase_dhara_spare_blocks_once();
    #[cfg(feature = "dhara-eval")]
    let _ = xcore407i_blinker::nand_fs::verify_dhara_read_only_callbacks();

    // Non-destructive NAND identification and factory bad-block scan.
    // No blocks are erased or programmed.
    //nand_device::run_read_only_diagnostic();

    // Start the 1 ms timebase before TinyUSB initialization. TinyUSB's
    // no-RTOS port uses this clock during controller initialization.
    core.SYST.set_clock_source(SystClkSource::Core);
    core.SYST.set_reload(168_000 - 1); // 1 ms at 168 MHz
    core.SYST.clear_current();
    configure_exception_priorities(&mut core.SCB);
    core.SYST.enable_interrupt();
    core.SYST.enable_counter();

    // Prepare and initialize OTG_FS host. A failed attempt is retried from
    // the main loop after a short delay; OTG_HS is initialized separately as a TinyUSB device.
    unsafe {
        core.NVIC.set_priority(interrupt::OTG_FS, 0x3 << 4);
        NVIC::unpend(interrupt::OTG_FS);
        NVIC::mask(interrupt::OTG_FS);
    }
    if !try_init_usb_fs_host(&device.RCC, &mut core.NVIC) {
        USB_FS_RETRY_AT.store(
            TICKS.load(Ordering::Relaxed).wrapping_add(USB_FS_RETRY_INTERVAL_MS),
            Ordering::Release,
        );
    }

    // The main loop owns the renderer; TinyUSB UVC completion callbacks
    // consume prepared batches from the OTG_HS device interrupt.
    let mut video_renderer = Nv12DemoImage::try_new()
        .expect("NV12 renderer initialization failed");
    while prepare_next_video_batch(&mut video_renderer) {}

    // Initialize TinyUSB as the OTG_HS high-speed device. The FS host remains
    // independently initialized on rhport 0.
    device.RCC.ahb1enr().modify(|_, w| {
        w.otghsen().set_bit().otghsulpien().set_bit()
    });
    let _ = device.RCC.ahb1enr().read().bits();

    unsafe {
        core.NVIC.set_priority(interrupt::OTG_HS, 0);
        NVIC::unpend(interrupt::OTG_HS);
    }

    let hs_device_init = tusb::tusb_rhport_init_t {
        role: tusb::tusb_role_t::TUSB_ROLE_DEVICE,
        speed: tusb::tusb_speed_t::TUSB_SPEED_HIGH,
    };
    if !unsafe { tusb::tusb_rhport_init(1, &hs_device_init) } {
        panic!("TinyUSB OTG_HS device initialization failed");
    }

    // Service the DWC2 device controller from its real OTG_HS interrupt.
    // TIM2 polling is no longer used for USB device servicing.
    unsafe {
        core.NVIC.set_priority(interrupt::OTG_HS, 0);
        NVIC::unpend(interrupt::OTG_HS);
        NVIC::unmask(interrupt::OTG_HS);
    }

    // TIM3 schedules the TinyUSB FS host task at 1 kHz. The ISR only sets a
    // flag; tuh_task_ext() itself must run in thread context, never in an ISR.
    configure_usb_host_task_timer(
        &device.RCC,
        &device.TIM3,
        &mut core.NVIC,
    );

    #[cfg(feature = "nand-ecc-test")]
    let _nand_ecc_test_result =
        xcore407i_blinker::nand_fs::run_ecc_fault_injection_test();

    let browser_paths = unsafe { &mut *ptr::addr_of_mut!(BROWSER_PATHS) };
    let browser_path_lengths = unsafe { &mut *ptr::addr_of_mut!(BROWSER_PATH_LENGTHS) };
    let mut browser_count = 0usize;
    let mut browser_selected = 0usize;
    let mut browser_open = false;
    // Real-time emulator pacing uses the PDP cycle counter.
    let mut pacing_start_ms = TICKS.load(Ordering::Relaxed);
    let mut pacing_start_cycles = unsafe { bk_timing_cycles() };
    let mut save_dialog_was_active = false;
    #[cfg(feature = "dhara-eval")]
    let mut dhara_diag_last_ms = TICKS.load(Ordering::Relaxed).wrapping_sub(1_000);
    #[cfg(feature = "cpu-cycle-diag")]
    let mut diagnostic_last_dwt = read_dwt_cycle_count();
    #[cfg(feature = "cpu-cycle-diag")]
    let mut diagnostic_last_pdp = pacing_start_cycles;

    loop {
        // Retry OTG_FS initialization outside interrupt context. Keep the
        // IRQ masked until TinyUSB reports a successful root-hub init.
        if !USB_FS_READY.load(Ordering::Acquire) {
            let now = TICKS.load(Ordering::Relaxed);
            let retry_at = USB_FS_RETRY_AT.load(Ordering::Acquire);
            if now.wrapping_sub(retry_at) as i32 >= 0 {
                if !try_init_usb_fs_host(&device.RCC, &mut core.NVIC) {
                    USB_FS_RETRY_AT.store(
                        now.wrapping_add(USB_FS_RETRY_INTERVAL_MS),
                        Ordering::Release,
                    );
                }
            }
        }

        // Drain TinyUSB device control/bus events in thread context. ISO data
        // endpoints are re-armed directly by their class xfer_isr callbacks.
        unsafe {
            let _ = tusb::tud_task_ext(0, false);
            xcore407i_blinker::tusb_class_drivers::xcore_tud_uvc_task(1);
            xcore407i_blinker::tusb_class_drivers::xcore_tud_audio_render();
        }

        // Service the USB FS host in thread context. TIM3 only schedules this
        // work; the host task must never run from the timer ISR.
        if USB_FS_READY.load(Ordering::Acquire)
            && USB_HOST_TASK_PENDING.swap(false, Ordering::AcqRel)
        {
            unsafe { tusb::tuh_task_ext(0, false); }
        } else {
            USB_HOST_TASK_PENDING.store(false, Ordering::Release);
        }

        let browser_action = FILE_BROWSER_ACTION.swap(0, Ordering::AcqRel);

        let save_state = SAVE_DIALOG_STATE.load(Ordering::Acquire);
        if save_state == SAVE_DIALOG_CHECK {
            let mut path = [0u8; MAX_BROWSER_PATH];
            let path_len = unsafe { build_save_path(&mut path) };
            if path_len == 0 {
                SAVE_DIALOG_STATE.store(SAVE_DIALOG_IDLE, Ordering::Release);
                SAVE_DIALOG_RESULT.store(SAVE_RESULT_ERROR, Ordering::Release);
            } else {
                let exists = save_file_exists(&path[..path_len]);
                if exists {
                    SAVE_DIALOG_STATE.store(SAVE_DIALOG_CONFIRM, Ordering::Release);
                    SAVE_DIALOG_DIRTY.store(true, Ordering::Release);
                } else {
                    SAVE_OVERWRITE.store(false, Ordering::Release);
                    SAVE_DIALOG_STATE.store(SAVE_DIALOG_WRITE, Ordering::Release);
                }
            }
        }

        if SAVE_DIALOG_STATE.load(Ordering::Acquire) == SAVE_DIALOG_WRITE {
            let mut path = [0u8; MAX_BROWSER_PATH];
            let path_len = unsafe { build_save_path(&mut path) };
            let overwrite = SAVE_OVERWRITE.load(Ordering::Acquire);
            let saved = path_len != 0 && save_ram_to_path(&path[..path_len], overwrite).is_ok();
            SAVE_DIALOG_STATE.store(SAVE_DIALOG_IDLE, Ordering::Release);
            SAVE_DIALOG_RESULT.store(
                if saved { SAVE_RESULT_OK } else { SAVE_RESULT_ERROR },
                Ordering::Release,
            );
        }

        if SAVE_DIALOG_DIRTY.swap(false, Ordering::AcqRel) {
            let state = SAVE_DIALOG_STATE.load(Ordering::Acquire);
            if state == SAVE_DIALOG_INPUT || state == SAVE_DIALOG_CONFIRM {
                unsafe { draw_save_dialog(state); }
            }
        }

        let save_dialog_active = SAVE_DIALOG_STATE.load(Ordering::Acquire) != SAVE_DIALOG_IDLE;
        if save_dialog_active != save_dialog_was_active {
            pacing_start_ms = TICKS.load(Ordering::Relaxed);
            pacing_start_cycles = unsafe { bk_timing_cycles() };
            save_dialog_was_active = save_dialog_active;
        }

        match SAVE_DIALOG_RESULT.swap(0, Ordering::AcqRel) {
            SAVE_RESULT_CANCELLED => {
                clear_demo_canvas(0x10);
                show_usb_status(b"SAVE CANCELLED");
            }
            SAVE_RESULT_OK => {
                clear_demo_canvas(0x10);
                show_usb_status(b"RAM SAVED");
            }
            SAVE_RESULT_ERROR => {
                clear_demo_canvas(0x10);
                show_usb_status(b"RAM SAVE FAILED");
            }
            _ => {}
        }

        if browser_action == FILE_BROWSER_OPEN && !browser_open {
            keyboard_queue_discard();
            match list_root_files(&mut *browser_paths, &mut *browser_path_lengths) {
                Ok(count) => {
                    browser_count = count;
                    browser_selected = 0;
                    browser_open = true;
                    FILE_BROWSER_ACTIVE.store(true, Ordering::Release);
                    draw_file_browser(
                        &*browser_paths,
                        &*browser_path_lengths,
                        browser_count,
                        browser_selected,
                    );
                }
                Err(()) => {
                    browser_open = false;
                    FILE_BROWSER_ACTIVE.store(false, Ordering::Release);
                    show_usb_status(b"NAND FILE LIST FAILED");
                }
            }
        }

        if browser_open {
            let mut redraw = false;
            match browser_action {
                FILE_BROWSER_UP if browser_count != 0 && browser_selected > 0 => {
                    browser_selected -= 1;
                    redraw = true;
                }
                FILE_BROWSER_DOWN if browser_count != 0 && browser_selected + 1 < browser_count => {
                    browser_selected += 1;
                    redraw = true;
                }
                FILE_BROWSER_ENTER if browser_count != 0 => {
                    let path_len = browser_path_lengths[browser_selected] as usize;
                    let result = load_tetris_from_path(&browser_paths[browser_selected][..path_len]);
                    browser_open = false;
                    FILE_BROWSER_ACTIVE.store(false, Ordering::Release);
                    clear_demo_canvas(0x10);
                    if result.is_ok() {
                        show_usb_status(b"PROGRAM LOADED");
                    } else {
                        show_usb_status(b"LOAD FAILED - CHECK FILE FORMAT");
                    }
                }
                FILE_BROWSER_ESCAPE => {
                    browser_open = false;
                    FILE_BROWSER_ACTIVE.store(false, Ordering::Release);
                    clear_demo_canvas(0x10);
                    show_usb_status(b"FILE BROWSER CANCELLED");
                }
                _ => {}
            }
            if redraw {
                draw_file_browser(
                    &*browser_paths,
                    &*browser_path_lengths,
                    browser_count,
                    browser_selected,
                );
            }
        }

        // Replenish USB video buffers before emulator work. The producer
        // also runs between individual PDP instructions below, so a newly
        // freed DMA buffer is refilled promptly instead of waiting for a
        // complete emulator quantum to finish.
        while prepare_next_video_batch(&mut video_renderer) {}

        #[cfg(feature = "cpu-cycle-diag")]
        {
            // DWT provides an independent wall-time reference for the PDP rate.
            const DWT_CYCLES_PER_SECOND: u32 = 168_000_000;
            let now_dwt = read_dwt_cycle_count();
            let elapsed_dwt = now_dwt.wrapping_sub(diagnostic_last_dwt);
            if elapsed_dwt >= DWT_CYCLES_PER_SECOND {
                let now_pdp = unsafe { bk_timing_cycles() };
                let elapsed_pdp = now_pdp.wrapping_sub(diagnostic_last_pdp);
                let pdp_cycles_per_second =
                    (elapsed_pdp as u64 * DWT_CYCLES_PER_SECOND as u64
                        / elapsed_dwt as u64) as u32;

                show_pdp_cycles_per_second(pdp_cycles_per_second);
                diagnostic_last_dwt = now_dwt;
                diagnostic_last_pdp = now_pdp;
            }
        }

        #[cfg(feature = "dhara-eval")]
        {
            let now = TICKS.load(Ordering::Relaxed);
            if now.wrapping_sub(dhara_diag_last_ms) >= 1_000 {
                show_dhara_spare_inventory(
                    xcore407i_blinker::nand_fs::NandStorage::dhara_test_block_count_cached(),
                    xcore407i_blinker::nand_fs::dhara_read_only_test_cached(),
                    xcore407i_blinker::nand_fs::dhara_erase_status_cached(),
                );
                dhara_diag_last_ms = now;
            }
        }

        // Pace PDP execution against the 1 ms hardware timebase. This keeps
        // the emulated 3 MHz clock aligned with real time in both color and
        // monochrome modes. USB/audio/video servicing continues while ahead.
        if !browser_open && SAVE_DIALOG_STATE.load(Ordering::Acquire) == SAVE_DIALOG_IDLE {
            let now_ms = TICKS.load(Ordering::Relaxed);
            let elapsed_ms = now_ms.wrapping_sub(pacing_start_ms);
            let now_cycles = unsafe { bk_timing_cycles() };
            let elapsed_cycles = now_cycles.wrapping_sub(pacing_start_cycles);

            // Rebase every second so both wrapping counters remain unambiguous.
            if elapsed_ms >= 1_000 {
                pacing_start_ms = now_ms;
                pacing_start_cycles = now_cycles;
            } else {
                let target_cycles =
                    (elapsed_ms as u64 * PDP_CYCLES_PER_SECOND as u64) / 1_000;
                if (elapsed_cycles as u64) < target_cycles {
                    // Execute a small batch to amortize the pacing check. The
                    // batch is deliberately bounded so USB servicing remains
                    // responsive; any small cycle overshoot is corrected by
                    // the next iteration.
                    for _ in 0..16 {
                        unsafe { bk_loop(); }

                        // Refill buffers while the emulator runs, outside USB IRQs.
                        while prepare_next_video_batch(&mut video_renderer) {}
                    }
                    service_keyboard_frame_pacing();
                }
            }
        }
    }
}

#[interrupt]
fn OTG_FS() {
    unsafe { tusb::hcd_int_handler(0, true); }
}

#[interrupt]
fn OTG_HS() {
    unsafe { tusb::tusb_int_handler(1, true); }
}

#[interrupt]
fn TIM3() {
    usb_host::on_tim3_interrupt();
}

#[exception]
fn SysTick() {
    TICKS.fetch_add(1, Ordering::Relaxed);
}
