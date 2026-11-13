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
    FORMAT_STATE, FORMAT_ERROR, FORMAT_IDLE, FORMAT_CONFIRM, FORMAT_CONFIRM_FINAL,
    FORMAT_REQUEST, FORMAT_CANCELLED, FORMAT_DONE, FORMAT_FAILED,
    draw_format_confirmation,
    INIT_STATE, INIT_IDLE, INIT_CONFIRM, INIT_CONFIRM_FINAL, INIT_REQUEST,
    INIT_CANCELLED, INIT_DONE, INIT_FAILED, draw_init_confirmation,
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
    fn xcore_mtp_service_read_prepare() -> bool;
    fn xcore_mtp_service_read_submit();
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

    // The BK emulator is initialized only after the startup keyboard report
    // selects normal mode. MTP and keyboard-error modes never touch BK state.
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

    // TIM3 schedules the TinyUSB FS host task at 1 kHz. Start it before
    // selecting the boot mode so a keyboard can enumerate and report F10.
    configure_usb_host_task_timer(
        &device.RCC,
        &device.TIM3,
        &mut core.NVIC,
    );

    const KEYBOARD_DETECT_TIMEOUT_MS: u32 = 30_000;
    let keyboard_wait_started = TICKS.load(Ordering::Relaxed);
    while keyboard::boot_selection() == keyboard::BOOT_SELECTION_WAITING {
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

        if USB_FS_READY.load(Ordering::Acquire)
            && USB_HOST_TASK_PENDING.swap(false, Ordering::AcqRel)
        {
            unsafe { tusb::tuh_task_ext(0, false); }
        } else {
            USB_HOST_TASK_PENDING.store(false, Ordering::Release);
        }

        let now = TICKS.load(Ordering::Relaxed);
        if now.wrapping_sub(keyboard_wait_started) >= KEYBOARD_DETECT_TIMEOUT_MS {
            keyboard::set_boot_timeout();
            break;
        }
        cortex_m::asm::wfi();
    }

    let boot_mode = keyboard::boot_selection();
    let emulator_enabled = boot_mode == keyboard::BOOT_SELECTION_NORMAL;
    match boot_mode {
        keyboard::BOOT_SELECTION_NORMAL => {
            unsafe {
                bk_platform_init(
                    xcore407i_blinker::rom::MONITOR_ROM.as_ptr(),
                    xcore407i_blinker::rom::BASIC_ROM.as_ptr(),
                    xcore407i_blinker::rom::BASIC_ROM.len() as u32,
                );
            }
            keyboard::enable_emulator_input();
            clear_demo_canvas(0x10);
        }
        keyboard::BOOT_SELECTION_MTP => {
            // The canvas SRAM is reserved as the MTP transfer workspace in
            // this mode; the UVC renderer generates its status frame directly.
        }
        keyboard::BOOT_SELECTION_TIMEOUT => {
            clear_demo_canvas(0x10);
            show_usb_status(b"KEYBOARD NOT DETECTED");
        }
        _ => panic!("invalid startup mode"),
    }

    // The main loop owns the renderer; TinyUSB UVC completion callbacks
    // consume prepared batches from the OTG_HS device interrupt.
    let mut video_renderer = Nv12DemoImage::try_new()
        .expect("NV12 renderer initialization failed");
    if boot_mode == keyboard::BOOT_SELECTION_MTP {
        video_renderer.set_pc_connect_screen();
    }
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

    xcore407i_blinker::tusb_descriptors::set_mtp_mode(
        boot_mode == keyboard::BOOT_SELECTION_MTP,
    );

    let hs_device_init = tusb::tusb_rhport_init_t {
        role: tusb::tusb_role_t::TUSB_ROLE_DEVICE,
        speed: tusb::tusb_speed_t::TUSB_SPEED_HIGH,
    };
    if !unsafe { tusb::tusb_rhport_init(1, &hs_device_init) } {
        panic!("TinyUSB OTG_HS device initialization failed");
    }

    // Service the DWC2 device controller immediately after initialization.
    // In particular, do not perform NAND/LittleFS work before enabling the
    // IRQ: the host may already be issuing the first EP0 descriptor request.
    unsafe {
        core.NVIC.set_priority(interrupt::OTG_HS, 0);
        NVIC::unpend(interrupt::OTG_HS);
        NVIC::unmask(interrupt::OTG_HS);
    }

    let browser_paths = unsafe { &mut *ptr::addr_of_mut!(BROWSER_PATHS) };
    let browser_path_lengths = unsafe { &mut *ptr::addr_of_mut!(BROWSER_PATH_LENGTHS) };
    let mut browser_count = 0usize;
    let mut browser_selected = 0usize;
    let mut browser_open = false;
    // Real-time emulator pacing uses the PDP cycle counter.
    let mut pacing_start_ms = TICKS.load(Ordering::Relaxed);
    let mut pacing_start_cycles = if emulator_enabled {
        unsafe { bk_timing_cycles() }
    } else {
        0
    };
    let mut emulator_was_paused = false;
    let mut format_screen_state = FORMAT_IDLE;
    let mut init_screen_state = INIT_IDLE;
    #[cfg(feature = "cpu-cycle-diag")]
    let mut diagnostic_last_dwt = read_dwt_cycle_count();
    #[cfg(feature = "cpu-cycle-diag")]
    let mut diagnostic_last_pdp = if emulator_enabled { pacing_start_cycles } else { 0 };

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
            if emulator_enabled {
                xcore407i_blinker::tusb_class_drivers::xcore_tud_audio_render();
            }
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

        // MTP and startup-error modes keep USB/video alive but never run
        // emulator, keyboard-to-BK, save-dialog, or file-browser work.
        if !emulator_enabled {
            // MTP root enumeration is requested by OpenSession and must run
            // outside TinyUSB callbacks, after this iteration's USB service.
            if boot_mode == keyboard::BOOT_SELECTION_MTP {
                xcore407i_blinker::mtp_storage::service_background();
                // Read NAND in thread context with USB interrupts enabled. Only
                // the short endpoint submission is protected from OTG_HS ISR.
                if unsafe { xcore_mtp_service_read_prepare() } {
                    unsafe { NVIC::mask(interrupt::OTG_HS); }
                    unsafe { xcore_mtp_service_read_submit(); }
                    unsafe { NVIC::unmask(interrupt::OTG_HS); }
                }
            }
            // Produce at most one video batch per pass so MTP work and
            // TinyUSB servicing get another turn before the next render batch.
            let _ = prepare_next_video_batch(&mut video_renderer);
            continue;
        }

        let browser_action = FILE_BROWSER_ACTION.swap(0, Ordering::AcqRel);

        match INIT_STATE.load(Ordering::Acquire) {
            INIT_CONFIRM | INIT_CONFIRM_FINAL => {}
            INIT_CANCELLED => {
                INIT_STATE.store(INIT_IDLE, Ordering::Release);
                clear_demo_canvas(0x10);
                show_usb_status(b"NAND INIT CANCELLED");
            }
            INIT_REQUEST => {
                let result = xcore407i_blinker::nand_fs::initialize_nand_volume();
                if result.is_ok() {
                    INIT_STATE.store(INIT_DONE, Ordering::Release);
                } else {
                    INIT_STATE.store(INIT_FAILED, Ordering::Release);
                }
            }
            INIT_DONE => {
                INIT_STATE.store(INIT_IDLE, Ordering::Release);
                clear_demo_canvas(0x10);
                show_usb_status(b"NAND READY - EMPTY LFS");
            }
            INIT_FAILED => {
                INIT_STATE.store(INIT_IDLE, Ordering::Release);
                clear_demo_canvas(0x10);
                show_usb_status(b"NAND INIT FAILED");
            }
            _ => {}
        }

        let current_init_state = INIT_STATE.load(Ordering::Acquire);
        if current_init_state != init_screen_state {
            match current_init_state {
                INIT_CONFIRM => draw_init_confirmation(false),
                INIT_CONFIRM_FINAL => draw_init_confirmation(true),
                _ => {}
            }
            init_screen_state = current_init_state;
        }

        match FORMAT_STATE.load(Ordering::Acquire) {
            FORMAT_CONFIRM | FORMAT_CONFIRM_FINAL => {}
            FORMAT_CANCELLED => {
                FORMAT_STATE.store(FORMAT_IDLE, Ordering::Release);
                clear_demo_canvas(0x10);
                show_usb_status(b"FORMAT CANCELLED");
            }
            FORMAT_REQUEST => {
                let result = xcore407i_blinker::nand_fs::format_existing_littlefs();
                if result.is_ok() {
                    FORMAT_STATE.store(FORMAT_DONE, Ordering::Release);
                } else {
                    FORMAT_STATE.store(FORMAT_FAILED, Ordering::Release);
                }
            }
            FORMAT_DONE => {
                FORMAT_STATE.store(FORMAT_IDLE, Ordering::Release);
                clear_demo_canvas(0x10);
                show_usb_status(b"LITTLEFS FORMATTED - REOPEN F2");
            }
            FORMAT_FAILED => {
                FORMAT_STATE.store(FORMAT_IDLE, Ordering::Release);
                clear_demo_canvas(0x10);
                show_usb_status(b"LITTLEFS FORMAT FAILED");
            }
            _ => {}
        }

        let current_format_state = FORMAT_STATE.load(Ordering::Acquire);
        if current_format_state != format_screen_state {
            match current_format_state {
                FORMAT_CONFIRM => draw_format_confirmation(false, FORMAT_ERROR.load(Ordering::Acquire)),
                FORMAT_CONFIRM_FINAL => draw_format_confirmation(true, FORMAT_ERROR.load(Ordering::Acquire)),
                _ => {}
            }
            format_screen_state = current_format_state;
        }

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
                Err(error) => {
                    browser_open = false;
                    FILE_BROWSER_ACTIVE.store(false, Ordering::Release);
                    if matches!(
                        error,
                        xcore407i_blinker::nand_fs::FsAccessError::Init(
                            xcore407i_blinker::nand_fs::InitError::DharaNoJournal
                        )
                    ) {
                        INIT_STATE.store(INIT_CONFIRM, Ordering::Release);
                    } else if matches!(
                        error,
                        xcore407i_blinker::nand_fs::FsAccessError::Mount(_)
                    ) {
                        let detail = match error {
                            xcore407i_blinker::nand_fs::FsAccessError::Mount(code) => code,
                            _ => 0,
                        };
                        FORMAT_ERROR.store(detail, Ordering::Release);
                        FORMAT_STATE.store(FORMAT_CONFIRM, Ordering::Release);
                    } else {
                    let message: &[u8] = match error {
                        xcore407i_blinker::nand_fs::FsAccessError::Init(error) => {
                            match error {
                                xcore407i_blinker::nand_fs::InitError::Timeout =>
                                    b"NAND TIMEOUT",
                                xcore407i_blinker::nand_fs::InitError::WrongDevice =>
                                    b"NAND WRONG DEVICE",
                                xcore407i_blinker::nand_fs::InitError::MarkerRead =>
                                    b"NAND MARKER READ",
                                xcore407i_blinker::nand_fs::InitError::MountFailed =>
                                    b"NAND INIT MOUNT FAIL",
                                xcore407i_blinker::nand_fs::InitError::DharaNoJournal =>
                                    b"DHARA JOURNAL MISSING",
                                xcore407i_blinker::nand_fs::InitError::DharaCapacity =>
                                    b"DHARA CAPACITY LOW",
                                xcore407i_blinker::nand_fs::InitError::ProvisionFailed =>
                                    b"DHARA PROVISION FAILED",
                            }
                        },
                        xcore407i_blinker::nand_fs::FsAccessError::Mount(_) =>
                            b"LFS MOUNT FAILED",
                        xcore407i_blinker::nand_fs::FsAccessError::Operation =>
                            b"LFS ROOT READ FAILED",
                    };
                    show_usb_status(message);
                    }
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

        // Freeze PDP execution while any modal UI owns input. Rebase the
        // pacing clock on both entry and exit so the emulator never tries to
        // catch up for time spent in the file chooser or format recovery UI.
        let emulator_paused = browser_open
            || FILE_BROWSER_ACTIVE.load(Ordering::Acquire)
            || SAVE_DIALOG_STATE.load(Ordering::Acquire) != SAVE_DIALOG_IDLE
            || FORMAT_STATE.load(Ordering::Acquire) != FORMAT_IDLE
            || INIT_STATE.load(Ordering::Acquire) != INIT_IDLE;
        if emulator_paused != emulator_was_paused {
            pacing_start_ms = TICKS.load(Ordering::Relaxed);
            pacing_start_cycles = unsafe { bk_timing_cycles() };
            emulator_was_paused = emulator_paused;
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

        // Pace PDP execution against the 1 ms hardware timebase. This keeps
        // the emulated 3 MHz clock aligned with real time in both color and
        // monochrome modes. USB/audio/video servicing continues while ahead.
        if !emulator_paused {
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
