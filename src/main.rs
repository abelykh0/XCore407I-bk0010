#![no_std]
#![no_main]

mod board;
mod file_browser;
mod keyboard;
mod keyboard_map;
mod usb_host;

use core::ptr;
use core::sync::atomic::{AtomicPtr, Ordering};
use cortex_m::peripheral::{syst::SystClkSource, NVIC};
use cortex_m_rt::{entry, exception};
use panic_halt as _;
use stm32f4::stm32f407::interrupt;
use stm32f4xx_hal::pac;
use xcore407i_blinker::camera::{clear_demo_canvas, show_usb_status, show_file_browser_line, Nv12DemoImage};
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
    configure_usb_host_task_timer, restart_usb_fs_host, try_init_usb_fs_host, USB_FS_READY,
    USB_FS_RETRY_AT, USB_FS_RETRY_INTERVAL_MS, USB_HOST_TASK_PENDING, USB_FS_STATUS,
    USB_FS_LAST_ACTIVITY_TICK,
};

const PDP_CYCLES_PER_SECOND: u32 = 3_010_000;

static FILE_BROWSER_VIDEO_RENDERER: AtomicPtr<Nv12DemoImage> =
    AtomicPtr::new(ptr::null_mut());
static FILE_BROWSER_IO_SERVICE: fn() = service_file_browser_io;

// ECCSCAN runs in the CDC console's foreground. Keep the USB device stack
// serviced between NAND reads, but do not run the video renderer from this
// callback: renderer work can monopolize the scan between completed blocks.
fn service_cdc_scan_io() {
    unsafe {
        let _ = tusb::tud_task_ext(0, false);
        xcore407i_blinker::tusb_class_drivers::xcore_tud_uvc_task(1);
    }

    // ECCSCAN can run for a long time in the CDC foreground. Keep the
    // camera producer moving, but advance only one bounded batch per service
    // call so renderer work cannot monopolize the scan.
    let renderer = FILE_BROWSER_VIDEO_RENDERER.load(Ordering::Acquire);
    if !renderer.is_null() {
        unsafe {
            let _ = prepare_next_video_batch(&mut *renderer);
        }
    }
}

fn service_file_browser_io() {
    unsafe {
        let _ = tusb::tud_task_ext(0, false);
        xcore407i_blinker::tusb_class_drivers::xcore_tud_uvc_task(1);
    }
    if USB_FS_READY.load(Ordering::Acquire)
        && USB_HOST_TASK_PENDING.swap(false, Ordering::AcqRel)
    {
        unsafe { tusb::tuh_task_ext(0, false); }
    } else {
        USB_HOST_TASK_PENDING.store(false, Ordering::Release);
    }

    // A NAND wait hook must remain bounded. Prepare at most one video
    // batch per invocation; draining the renderer here can monopolize the
    // foreground while the NAND is busy and starve CDC/UVC servicing.
    let renderer = FILE_BROWSER_VIDEO_RENDERER.load(Ordering::Acquire);
    if !renderer.is_null() {
        unsafe {
            let _ = prepare_next_video_batch(&mut *renderer);
        }
    }
}

unsafe extern "C" {
    fn bk_platform_init(monitor_rom: *const u8, basic_rom: *const u8, basic_size_bytes: u32);
    fn bk_loop() -> i32;
    fn bk_timing_cycles() -> u32;
}


fn draw_keyboard_unavailable_screen() {
    fn decimal(mut value: u32, out: &mut [u8; 10]) -> &[u8] {
        let mut pos = out.len();
        if value == 0 {
            out[9] = b'0';
            return &out[9..];
        }
        while value != 0 && pos != 0 {
            pos -= 1;
            out[pos] = b'0' + (value % 10) as u8;
            value /= 10;
        }
        &out[pos..]
    }

    clear_demo_canvas(0x10); // blue background
    show_file_browser_line(3, b"KEYBOARD NOT AVAILABLE", false);
    show_file_browser_line(5, b"USB HOST DIAGNOSTICS", false);

    let (hid, boot, reports, last_len, rearm, protocol, unmounts) =
        keyboard::keyboard_diagnostics();
    show_file_browser_line(7, if USB_FS_READY.load(Ordering::Acquire) {
        b"OTG_FS HOST: READY" as &[u8]
    } else {
        b"OTG_FS HOST: NOT READY" as &[u8]
    }, false);

    let mut line = [b' '; 32];
    let mut number = [0u8; 10];
    line[..10].copy_from_slice(b"HOST EVENT");
    let digits = decimal(USB_FS_STATUS.load(Ordering::Acquire), &mut number);
    line[11..11 + digits.len()].copy_from_slice(digits);
    show_file_browser_line(9, &line, false);

    line.fill(b' ');
    line[..10].copy_from_slice(b"HID MOUNTS");
    let digits = decimal(hid, &mut number);
    line[11..11 + digits.len()].copy_from_slice(digits);
    show_file_browser_line(10, &line, false);

    line.fill(b' ');
    line[..14].copy_from_slice(b"BOOT KEYBOARDS");
    let digits = decimal(boot, &mut number);
    line[16..16 + digits.len()].copy_from_slice(digits);
    show_file_browser_line(11, &line, false);

    line.fill(b' ');
    line[..7].copy_from_slice(b"REPORTS");
    let digits = decimal(reports, &mut number);
    line[8..8 + digits.len()].copy_from_slice(digits);
    show_file_browser_line(12, &line, false);

    line.fill(b' ');
    line[..15].copy_from_slice(b"LAST REPORT LEN");
    let digits = decimal(last_len, &mut number);
    line[16..16 + digits.len()].copy_from_slice(digits);
    show_file_browser_line(13, &line, false);

    line.fill(b' ');
    line[..11].copy_from_slice(b"REARM FAILS");
    let digits = decimal(rearm, &mut number);
    line[12..12 + digits.len()].copy_from_slice(digits);
    show_file_browser_line(14, &line, false);

    line.fill(b' ');
    line[..12].copy_from_slice(b"HID IF PROTO");
    if protocol == 0xff {
        line[13..17].copy_from_slice(b"NONE");
    } else if protocol == 0xfe {
        line[13..27].copy_from_slice(b"SET PROTO FAIL");
    } else {
        let digits = decimal(protocol as u32, &mut number);
        line[13..13 + digits.len()].copy_from_slice(digits);
    }
    show_file_browser_line(15, &line, false);

    line.fill(b' ');
    line[..8].copy_from_slice(b"UNMOUNTS");
    let digits = decimal(unmounts, &mut number);
    line[9..9 + digits.len()].copy_from_slice(digits);
    show_file_browser_line(16, &line, false);

    line.fill(b' ');
    line[..13].copy_from_slice(b"HOST RESTARTS");
    let digits = decimal(usb_host::USB_FS_RESTART_COUNT.load(Ordering::Acquire), &mut number);
    line[14..14 + digits.len()].copy_from_slice(digits);
    show_file_browser_line(17, &line, false);

    show_file_browser_line(19, b"CHECK KEYBOARD USB CABLE/PORT", false);
    show_file_browser_line(20, b"USE USB BOOT-HID KEYBOARD", false);
    show_file_browser_line(22, b"AUTO USB HOST RETRY", false);
    show_file_browser_line(24, b"NO EMULATOR STARTED", false);
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

    // Emulator initialization is deferred until the first boot-keyboard
    // report selects normal mode. CDC mode never initializes BK or storage UI.

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

    // TIM3 schedules TinyUSB host-task processing outside interrupt context.
    configure_usb_host_task_timer(&device.RCC, &device.TIM3, &mut core.NVIC);

    // Prepare and initialize OTG_FS host. A failed attempt is retried during
    // the boot-selection window and later from the main loop; OTG_HS is
    // initialized separately as a TinyUSB device.
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

    // Show the keyboard diagnostic screen after 5 seconds without a
    // successful USB-host discovery event. Each successful stage refreshes
    // USB_FS_LAST_ACTIVITY_TICK; host servicing continues on the blue screen.
    // F10 selects CDC only before this timeout. A keyboard discovered later
    // starts the emulator directly, without another mode-selection step.
    const USB_DISCOVERY_INACTIVITY_TIMEOUT_MS: u32 = 5_000;
    let mut keyboard_host_restart_attempted = false;
    USB_FS_LAST_ACTIVITY_TICK.store(TICKS.load(Ordering::Acquire), Ordering::Release);
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

        // A valid keyboard report has already selected NORMAL or CDC.
        // Do not keep consuming the keyboard reports for the remainder of
        // the fallback window: initialize the selected mode immediately.
        if keyboard::boot_selection() != keyboard::BOOT_SELECTION_WAITING {
            break;
        }

        let now = TICKS.load(Ordering::Relaxed);
        let last_activity = USB_FS_LAST_ACTIVITY_TICK.load(Ordering::Acquire);
        if now.wrapping_sub(last_activity) >= USB_DISCOVERY_INACTIVITY_TIMEOUT_MS {
            if !keyboard_host_restart_attempted && USB_FS_READY.load(Ordering::Acquire) {
                // One recovery attempt before declaring the keyboard unavailable.
                // Keep the boot-selection loop active so F10 is still recognized
                // if the keyboard enumerates after the host restart.
                keyboard_host_restart_attempted = true;
                let _ = restart_usb_fs_host(&device.RCC, &device.TIM3, &mut core.NVIC);
                USB_FS_LAST_ACTIVITY_TICK.store(TICKS.load(Ordering::Acquire), Ordering::Release);
                continue;
            }

            // The single host restart did not produce a keyboard report within
            // the second discovery window. Select normal mode and show diagnostics.
            keyboard::set_boot_default_normal();
            break;
        }
    }

    let boot_mode = keyboard::boot_selection();
    let cdc_enabled = boot_mode == keyboard::BOOT_SELECTION_CDC;
    // A normal-mode timeout is not proof that a keyboard exists. Do not
    // start the emulator unless a boot-keyboard interface actually mounted.
    let mut emulator_enabled = boot_mode == keyboard::BOOT_SELECTION_NORMAL
        && keyboard::keyboard_is_mounted();
    let keyboard_unavailable =
        boot_mode == keyboard::BOOT_SELECTION_NORMAL && !emulator_enabled;

    if keyboard_unavailable {
        draw_keyboard_unavailable_screen();
    }

    if emulator_enabled {
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

    // The main loop owns the renderer; TinyUSB UVC completion callbacks
    // consume prepared batches from the OTG_HS device interrupt.
    let mut video_renderer = Nv12DemoImage::try_new()
        .expect("NV12 renderer initialization failed");
    if cdc_enabled {
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

    xcore407i_blinker::uvc::set_cdc_mode(cdc_enabled);

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

    if cdc_enabled {
        // F10 selects a device-only mode. Stop the FS host controller and
        // its scheduler permanently; no keyboard or other host traffic is
        // serviced after this point. OTG_HS remains the UVC+CDC device.
        NVIC::mask(interrupt::OTG_FS);
        NVIC::unpend(interrupt::OTG_FS);

        // Reset and gate OTG_FS after masking its IRQ. This makes the
        // keyboard host electrically inactive for the remainder of CDC mode.
        device.RCC.ahb2rstr().modify(|_, w| w.otgfsrst().set_bit());
        device.RCC.ahb2enr().modify(|_, w| w.otgfsen().clear_bit());
        device.TIM3.cr1().write(|w| w.cen().clear_bit());
        device.TIM3.dier().write(|w| w.uie().clear_bit());
        USB_FS_READY.store(false, Ordering::Release);
        USB_HOST_TASK_PENDING.store(false, Ordering::Release);

        // Mount LittleFS for each CDC session. After formatting, the old
        // storage object is dropped and this loop mounts a fresh Dhara/LittleFS instance.
        // Keep the ZMODEM state on the CCM alternate stack: it contains the reusable
        // LittleFS FileAllocation and must not consume the small main stack.
        let mut just_formatted = false;
        loop {
            let mount_result: Result<bool, xcore407i_blinker::nand_fs::FsAccessError> =
                unsafe {
                    xcore407i_blinker::nand_fs::with_cdc_alternate_stack(|| {
                        let mut zmodem_state = xcore407i_blinker::cdc_zmodem::State::new();
                        let mount_result = xcore407i_blinker::nand_fs::with_mounted_detailed(|fs| {
                            if just_formatted {
                                xcore407i_blinker::cdc_console::report_format_status(
                                    b"Fresh LittleFS mount succeeded. CDC storage is ready.\r\n",
                                );
                                just_formatted = false;
                            }
                            loop {
                                unsafe {
                                    let _ = tusb::tud_task_ext(0, false);
                                    xcore407i_blinker::tusb_class_drivers::xcore_tud_uvc_task(1);
                                }
                                // Keep USB/UVC and the video producer alive inside synchronous
                                // NAND waits. The PC CONNECT renderer does not read the canvas.
                                FILE_BROWSER_VIDEO_RENDERER.store(&mut video_renderer, Ordering::Release);
                                xcore407i_blinker::nand_fs::set_io_service_hook(Some(&FILE_BROWSER_IO_SERVICE));
                                let format_requested = xcore407i_blinker::cdc_console::service(
                                    Some(fs),
                                    &mut zmodem_state,
                                    TICKS.load(Ordering::Relaxed),
                                    &mut || service_cdc_scan_io(),
                                );
                                xcore407i_blinker::nand_fs::set_io_service_hook(None);
                                FILE_BROWSER_VIDEO_RENDERER.store(ptr::null_mut(), Ordering::Release);
                                while prepare_next_video_batch(&mut video_renderer) {}
                                if format_requested {
                                    break Ok(true);
                                }
                            }
                        });

                        // Keep the state on the alternate stack in the storage-unavailable
                        // console too; do not construct another large state on the main stack.
                        if let Err(error) = mount_result {
                            xcore407i_blinker::cdc_console::report_storage_error(error);
                            loop {
                                unsafe {
                                    let _ = tusb::tud_task_ext(0, false);
                                    xcore407i_blinker::tusb_class_drivers::xcore_tud_uvc_task(1);
                                }
                                let requested = xcore407i_blinker::cdc_console::service(
                                    None,
                                    &mut zmodem_state,
                                    TICKS.load(Ordering::Relaxed),
                                    &mut || {
                                        unsafe {
                                            let _ = tusb::tud_task_ext(0, false);
                                            xcore407i_blinker::tusb_class_drivers::xcore_tud_uvc_task(1);
                                        }
                                        while prepare_next_video_batch(&mut video_renderer) {}
                                    },
                                );
                                while prepare_next_video_batch(&mut video_renderer) {}
                                if requested {
                                    break Ok(true);
                                }
                            }
                        } else {
                            mount_result
                        }
                    })
                };

        // The mounted-filesystem closure has returned, so LittleFS and its
        // Dhara storage borrow are dropped before the destructive format.
        let mut format_requested = matches!(&mount_result, Ok(true));

        if format_requested {
            xcore407i_blinker::cdc_console::report_format_status(
                b"Filesystem released. Formatting NAND; all files will be lost...\r\n",
            );
            let mut last_reported = 0u32;
            let format_result =
                xcore407i_blinker::nand_fs::initialize_nand_volume_with_progress(
                    |completed, total| {
                        unsafe {
                            let _ = tusb::tud_task_ext(0, false);
                            xcore407i_blinker::tusb_class_drivers::xcore_tud_uvc_task(1);
                        }
                        while prepare_next_video_batch(&mut video_renderer) {}
                        if completed == total || completed >= last_reported + 32 {
                            last_reported = completed;
                            xcore407i_blinker::cdc_console::report_format_status(b".");
                        }
                    },
                );
            match format_result {
                Ok(report) => {
                    xcore407i_blinker::cdc_console::report_format_complete(report);
                    just_formatted = true;
                    continue;
                }
                Err(error) => {
                    xcore407i_blinker::cdc_console::report_format_failure(error);
                }
            }
        }
        }
    }

    let browser_paths = unsafe { &mut *ptr::addr_of_mut!(BROWSER_PATHS) };
    let browser_path_lengths = unsafe { &mut *ptr::addr_of_mut!(BROWSER_PATH_LENGTHS) };
    let mut browser_count = 0usize;
    let mut browser_selected = 0usize;
    let mut browser_open = false;
    // Real-time emulator pacing uses the PDP cycle counter.
    let mut keyboard_diag_last = keyboard::keyboard_diagnostics();
    let mut keyboard_host_status_last = USB_FS_STATUS.load(Ordering::Acquire);
    let mut keyboard_host_ready_last = USB_FS_READY.load(Ordering::Acquire);
    let mut pacing_start_ms = TICKS.load(Ordering::Relaxed);
    let mut pacing_start_cycles = if emulator_enabled { unsafe { bk_timing_cycles() } } else { 0 };
    let mut emulator_was_paused = false;
    let mut format_screen_state = FORMAT_IDLE;
    let mut init_screen_state = INIT_IDLE;
    #[cfg(feature = "cpu-cycle-diag")]
    let mut diagnostic_last_dwt = read_dwt_cycle_count();
    #[cfg(feature = "cpu-cycle-diag")]
    let mut diagnostic_last_pdp = pacing_start_cycles;

    loop {
        // Keyboard recovery is limited to one restart during boot selection.
        // Once the diagnostic screen is shown, do not repeatedly reset the host;
        // leave it running so late enumeration can still be observed.

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

        if !emulator_enabled && !cdc_enabled {
            let diagnostics = keyboard::keyboard_diagnostics();
            let host_status = USB_FS_STATUS.load(Ordering::Acquire);
            let host_ready = USB_FS_READY.load(Ordering::Acquire);
            if diagnostics != keyboard_diag_last
                || host_status != keyboard_host_status_last
                || host_ready != keyboard_host_ready_last
            {
                draw_keyboard_unavailable_screen();
                keyboard_diag_last = diagnostics;
                keyboard_host_status_last = host_status;
                keyboard_host_ready_last = host_ready;
            }
        }

        // If a boot keyboard mounts after the initial selection window,
        // initialize the emulator immediately; no keypress or F10 decision
        // is required on the diagnostic screen.
        if !emulator_enabled && !cdc_enabled && keyboard::keyboard_is_mounted() {
            // After the blue screen, a mounted boot keyboard is sufficient:
            // do not wait for a keypress or interpret F10 as a mode command.
            unsafe {
                bk_platform_init(
                    xcore407i_blinker::rom::MONITOR_ROM.as_ptr(),
                    xcore407i_blinker::rom::BASIC_ROM.as_ptr(),
                    xcore407i_blinker::rom::BASIC_ROM.len() as u32,
                );
            }
            keyboard::enable_emulator_input();
            emulator_enabled = true;
            clear_demo_canvas(0x10);
            show_file_browser_line(3, b"USB KEYBOARD CONNECTED", false);
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
            show_file_browser_line(27, b"F3: BUILDING SAVE PATH", false);
            let mut path = [0u8; MAX_BROWSER_PATH];
            let path_len = unsafe { build_save_path(&mut path) };
            if path_len == 0 {
                show_file_browser_line(27, b"F3: INVALID SAVE PATH", false);
                SAVE_DIALOG_STATE.store(SAVE_DIALOG_IDLE, Ordering::Release);
                SAVE_DIALOG_RESULT.store(SAVE_RESULT_ERROR, Ordering::Release);
            } else {
                show_file_browser_line(27, b"F3: CHECKING FILE ON NAND", false);
                let exists = save_file_exists(&path[..path_len]);
                show_file_browser_line(27, b"F3: FILE CHECK COMPLETE", false);
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
            show_file_browser_line(27, b"F3: PREPARING NAND WRITE", false);
            let mut path = [0u8; MAX_BROWSER_PATH];
            let path_len = unsafe { build_save_path(&mut path) };
            let overwrite = SAVE_OVERWRITE.load(Ordering::Acquire);
            show_file_browser_line(27, b"F3: WRITING RAM TO NAND", false);
            let saved = path_len != 0 && save_ram_to_path(&path[..path_len], overwrite).is_ok();
            show_file_browser_line(
                27,
                if saved { b"F3: NAND WRITE COMPLETE" } else { b"F3: NAND WRITE FAILED" },
                false,
            );
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

            // Enter the browser visibly before any NAND/Dhara/LittleFS work.
            // If mounting stalls or faults, the user can distinguish that
            // from a failure to receive F2.
            clear_demo_canvas(0);
            show_file_browser_line(0, b"NAND FILES - SELECT PROGRAM", false);
            show_file_browser_line(3, b"OPENING NAND STORAGE...", true);
            show_file_browser_line(5, b"MOUNTING LITTLEFS", false);
            show_file_browser_line(27, b"PLEASE WAIT", false);


            // NAND/LittleFS operations are synchronous. Install a narrowly
            // scoped hook so USB and the UVC producer continue running even
            // inside NAND ready/FSMC wait loops and ECC page processing.
            FILE_BROWSER_VIDEO_RENDERER.store(&mut video_renderer, Ordering::Release);
            xcore407i_blinker::nand_fs::set_io_service_hook(Some(&FILE_BROWSER_IO_SERVICE));
            let listing_result = list_root_files(
                &mut *browser_paths,
                &mut *browser_path_lengths,
                |done, total| {
                    if done == u32::MAX {
                        show_file_browser_line(
                            27,
                            if total == 0 {
                                b"F2: LITTLEFS MOUNT" as &[u8]
                            } else {
                                b"F2: ROOT DIRECTORY"
                            },
                            false,
                        );
                    } else if total == 0 {
                        show_file_browser_line(27, b"F2: NAND INIT", false);
                    } else if done == 1 || done % 32 == 0 || done == total {
                        let percent = done.saturating_mul(100) / total;
                        let mut line = [b' '; 32];
                        line[..13].copy_from_slice(b"F2: NAND SCAN");
                        line[13] = b'0' + ((percent / 100) % 10) as u8;
                        line[14] = b'0' + ((percent / 10) % 10) as u8;
                        line[15] = b'0' + (percent % 10) as u8;
                        line[16] = b'%';
                        show_file_browser_line(27, &line, false);
                    }
                    service_file_browser_io();
                },
            );
            xcore407i_blinker::nand_fs::set_io_service_hook(None);
            FILE_BROWSER_VIDEO_RENDERER.store(ptr::null_mut(), Ordering::Release);

            match listing_result {
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
        let emulator_paused = !emulator_enabled
            || browser_open
            || FILE_BROWSER_ACTIVE.load(Ordering::Acquire)
            || SAVE_DIALOG_STATE.load(Ordering::Acquire) != SAVE_DIALOG_IDLE
            || FORMAT_STATE.load(Ordering::Acquire) != FORMAT_IDLE
            || INIT_STATE.load(Ordering::Acquire) != INIT_IDLE;
        if emulator_paused != emulator_was_paused {
            pacing_start_ms = TICKS.load(Ordering::Relaxed);
            pacing_start_cycles = if emulator_enabled { unsafe { bk_timing_cycles() } } else { 0 };
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
                if !emulator_enabled { continue; }
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
