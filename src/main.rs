#![no_std]
#![no_main]

mod board;
mod file_browser;
mod keyboard;
mod usb_host;

use core::ptr;
use core::sync::atomic::{AtomicPtr, Ordering};
use cortex_m::peripheral::{syst::SystClkSource, NVIC};
use cortex_m_rt::{entry, exception};
use panic_halt as _;
use stm32f4::stm32f407::interrupt;
use stm32f4xx_hal::pac;
use xcore407i_blinker::camera::{clear_demo_canvas, show_usb_status, show_file_browser_line, Nv12DemoImage};
use xcore407i_blinker::usb_device::uvc::prepare_next_video_batch;
use xcore407i_blinker::tusb;
use xcore407i_blinker::rtc;

use board::{
    configure_board_pins, configure_exception_priorities, system_clock_168mhz, TICKS,
};
use xcore407i_blinker::SYSTEM_CORE_CLOCK_HZ;
use board::enable_cycle_counter;
use file_browser::{
    draw_file_browser, load_tetris_from_path, BROWSER_FILES,
    FILE_BROWSER_ACTION, FILE_BROWSER_ACTIVE, FILE_BROWSER_OPEN, FILE_BROWSER_UP,
    FILE_BROWSER_DOWN, FILE_BROWSER_ENTER, FILE_BROWSER_ESCAPE,
    SAVE_DIALOG_IDLE, SAVE_DIALOG_STATE, reset_emulator_ui,
    FORMAT_STATE, FORMAT_ERROR, FORMAT_IDLE, FORMAT_CONFIRM, FORMAT_CONFIRM_FINAL,
    FORMAT_REQUEST, FORMAT_CANCELLED, FORMAT_DONE, FORMAT_FAILED,
    draw_format_confirmation,
    INIT_STATE, INIT_IDLE, INIT_CONFIRM, INIT_CONFIRM_FINAL, INIT_REQUEST,
    INIT_CANCELLED, INIT_DONE, INIT_FAILED, draw_init_confirmation,
};
use keyboard::{keyboard_queue_discard, service_keyboard_frame_pacing, service_keyboard_repeat};
use usb_host::{
    configure_usb_host_task_timer, restart_usb_fs_host, try_init_usb_fs_host, USB_FS_READY,
    USB_FS_RETRY_AT, USB_FS_RETRY_INTERVAL_MS, USB_HOST_TASK_PENDING, USB_FS_STATUS,
    USB_FS_LAST_ACTIVITY_TICK,
};

#[cfg(feature = "cpu-cycle-diag")]
use xcore407i_blinker::camera::show_pdp_cycles_per_second;
#[cfg(feature = "cpu-cycle-diag")]
use board::read_dwt_cycle_count;

const PDP_CYCLES_PER_SECOND: u32 = 3_010_000;

static FILE_BROWSER_VIDEO_RENDERER: AtomicPtr<Nv12DemoImage> =
    AtomicPtr::new(ptr::null_mut());
static FILE_BROWSER_IO_SERVICE: fn() = service_file_browser_io;

// ECCSCAN runs in the CDC console's foreground. Keep the USB device stack
// serviced between NAND reads, but do not run the video renderer from this
// callback: renderer work can monopolize the scan between completed blocks.
fn service_cdc_scan_io() {
    unsafe {
        let _ = tusb::tud_task_ext(1, false);
        xcore407i_blinker::usb_device::uvc_nv12::xcore_tud_uvc_task(1);
    }
    xcore407i_blinker::usb_device::cdc_console::service_connection();

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
        let _ = tusb::tud_task_ext(1, false);
        xcore407i_blinker::usb_device::uvc_nv12::xcore_tud_uvc_task(1);
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
    fn bk_reset();
    safe fn bk_timing_cycles() -> u32;
    safe fn bk_timer_update();
    safe fn bk_audio_diag_freeze();
    safe fn bk_audio_diag_end();
}


fn draw_keyboard_unavailable_screen() {
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
    let digits = xcore407i_blinker::format::u32_to_decimal(USB_FS_STATUS.load(Ordering::Acquire), &mut number);
    line[11..11 + digits.len()].copy_from_slice(digits);
    show_file_browser_line(9, &line, false);

    line.fill(b' ');
    line[..10].copy_from_slice(b"HID MOUNTS");
    let digits = xcore407i_blinker::format::u32_to_decimal(hid, &mut number);
    line[11..11 + digits.len()].copy_from_slice(digits);
    show_file_browser_line(10, &line, false);

    line.fill(b' ');
    line[..14].copy_from_slice(b"BOOT KEYBOARDS");
    let digits = xcore407i_blinker::format::u32_to_decimal(boot, &mut number);
    line[16..16 + digits.len()].copy_from_slice(digits);
    show_file_browser_line(11, &line, false);

    line.fill(b' ');
    line[..7].copy_from_slice(b"REPORTS");
    let digits = xcore407i_blinker::format::u32_to_decimal(reports, &mut number);
    line[8..8 + digits.len()].copy_from_slice(digits);
    show_file_browser_line(12, &line, false);

    line.fill(b' ');
    line[..15].copy_from_slice(b"LAST REPORT LEN");
    let digits = xcore407i_blinker::format::u32_to_decimal(last_len, &mut number);
    line[16..16 + digits.len()].copy_from_slice(digits);
    show_file_browser_line(13, &line, false);

    line.fill(b' ');
    line[..11].copy_from_slice(b"REARM FAILS");
    let digits = xcore407i_blinker::format::u32_to_decimal(rearm, &mut number);
    line[12..12 + digits.len()].copy_from_slice(digits);
    show_file_browser_line(14, &line, false);

    line.fill(b' ');
    line[..12].copy_from_slice(b"HID IF PROTO");
    if protocol == 0xff {
        line[13..17].copy_from_slice(b"NONE");
    } else if protocol == 0xfe {
        line[13..27].copy_from_slice(b"SET PROTO FAIL");
    } else {
        let digits = xcore407i_blinker::format::u32_to_decimal(protocol as u32, &mut number);
        line[13..13 + digits.len()].copy_from_slice(digits);
    }
    show_file_browser_line(15, &line, false);

    line.fill(b' ');
    line[..8].copy_from_slice(b"UNMOUNTS");
    let digits = xcore407i_blinker::format::u32_to_decimal(unmounts, &mut number);
    line[9..9 + digits.len()].copy_from_slice(digits);
    show_file_browser_line(16, &line, false);

    line.fill(b' ');
    line[..13].copy_from_slice(b"HOST RESTARTS");
    let digits = xcore407i_blinker::format::u32_to_decimal(usb_host::USB_FS_RESTART_COUNT.load(Ordering::Acquire), &mut number);
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

    // DWT CYCCNT provides the independent wall-clock source for the BK timer.
    enable_cycle_counter(&mut core);

    // Emulator initialization is deferred until the boot-keyboard startup
    // selection completes. F10 selects CDC; otherwise normal mode is used.

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
    core.SYST.set_reload(SYSTEM_CORE_CLOCK_HZ / 1_000 - 1); // 1 ms
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

    // Allow time for USB discovery, then give a mounted keyboard 100 ms
    // to report F10 before defaulting to normal mode.
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

        // Once a boot keyboard mounts, allow 100 ms for an F10 report.
        // If none arrives, select normal mode without waiting for another
        // report from the keyboard.
        keyboard::select_normal_after_mount_window();
        if keyboard::boot_selection() != keyboard::BOOT_SELECTION_WAITING {
            break;
        }

        let now = TICKS.load(Ordering::Relaxed);
        let last_activity = USB_FS_LAST_ACTIVITY_TICK.load(Ordering::Acquire);
        if now.wrapping_sub(last_activity) >= USB_DISCOVERY_INACTIVITY_TIMEOUT_MS {
            if !keyboard_host_restart_attempted
                && USB_FS_READY.load(Ordering::Acquire)
                && !keyboard::keyboard_is_mounted()
            {
                // One recovery attempt before declaring the keyboard unavailable.
                // Keep the boot-selection loop active so F10 is still recognized
                // if the keyboard enumerates after the host restart.
                keyboard_host_restart_attempted = true;
                let _ = restart_usb_fs_host(&device.RCC, &device.TIM3, &mut core.NVIC);
                USB_FS_LAST_ACTIVITY_TICK.store(TICKS.load(Ordering::Acquire), Ordering::Release);
                continue;
            }

            // The host restart did not discover a boot keyboard within the
            // second discovery window. Select normal mode and show diagnostics.
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

    xcore407i_blinker::usb_device::uvc::set_cdc_mode(cdc_enabled);

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

        // The boot keyboard is no longer needed. PC1 can now be reassigned
        // from FS VBUS enable to Ethernet MDC; USB HS remains active.
        board::configure_ethernet_pins();

        // Start the Ethernet MAC/DMA after the RMII pins and PHY management bus are ready.
        // The polled network stack provides DHCP and the embedded HTTP test page.
        let _ethernet_mac_dma_ready = xcore407i_blinker::ethernet::initialize();
        if _ethernet_mac_dma_ready {
            xcore407i_blinker::ethernet::enable_dma_interrupts();
            unsafe {
                core.NVIC.set_priority(interrupt::ETH, 0x2 << 4);
                NVIC::unpend(interrupt::ETH);
                NVIC::unmask(interrupt::ETH);
            }
        }
        let ethernet_socket_storage = xcore407i_blinker::ethernet::socket_storage();
        let mut ethernet_network = xcore407i_blinker::ethernet::EthernetNetwork::new(ethernet_socket_storage);

        // Mount LittleFS for each CDC session. After formatting, the old
        // storage object is dropped and this loop mounts a fresh Dhara/LittleFS instance.
        // Keep the ZMODEM state on the canvas-backed alternate stack: it contains
        // the reusable LittleFS FileAllocation and must not consume the small main stack.
        let mut just_formatted = false;
        loop {
            let mount_result: Result<xcore407i_blinker::usb_device::cdc_console::FormatRequest, xcore407i_blinker::nand_fs::FsAccessError> =
                unsafe {
                    xcore407i_blinker::nand_fs::with_cdc_alternate_stack(|| {
                        let mut zmodem_state = xcore407i_blinker::usb_device::cdc_zmodem::State::new();
                        // FileAllocation owns persistent LittleFS state; keep the HTTP
                        // upload allocation on the alternate stack, not in the small
                        // main-stack EthernetNetwork value.
                        let mut http_upload_state = xcore407i_blinker::ethernet::HttpUploadState::new();
                        let mut http_download_state = xcore407i_blinker::ethernet::HttpDownloadState::new();

                        // NAND initialization and the initial LittleFS mount happen
                        // before the filesystem service closure is entered. Keep the
                        // USB device and PC CONNECT video alive during those operations.
                        FILE_BROWSER_VIDEO_RENDERER.store(&mut video_renderer, Ordering::Release);
                        xcore407i_blinker::nand_fs::set_io_service_hook(Some(&FILE_BROWSER_IO_SERVICE));
                        let mount_result = xcore407i_blinker::nand_fs::with_mounted_detailed_with_progress(|fs| {
                            if just_formatted {
                                xcore407i_blinker::usb_device::cdc_console::report_format_status(
                                    b"Fresh LittleFS mount succeeded. CDC storage is ready.\r\n",
                                );
                                just_formatted = false;
                            }
                            loop {
                                let _ = tusb::tud_task_ext(1, false);
                                xcore407i_blinker::usb_device::uvc_nv12::xcore_tud_uvc_task(1);
                                // The HTTP directory listing reads LittleFS synchronously.
                                // Keep USB/UVC serviced during NAND page reads.
                                xcore407i_blinker::nand_fs::set_io_service_hook(Some(&FILE_BROWSER_IO_SERVICE));
                                ethernet_network.poll(TICKS.load(Ordering::Relaxed), fs, &mut http_upload_state, &mut http_download_state);
                                xcore407i_blinker::nand_fs::set_io_service_hook(None);
                                // Keep USB/UVC and the video producer alive inside synchronous
                                // NAND waits. The PC CONNECT renderer does not read the canvas.
                                FILE_BROWSER_VIDEO_RENDERER.store(&mut video_renderer, Ordering::Release);
                                xcore407i_blinker::nand_fs::set_io_service_hook(Some(&FILE_BROWSER_IO_SERVICE));
                                let format_requested = xcore407i_blinker::usb_device::cdc_console::service(
                                    Some(fs),
                                    &mut zmodem_state,
                                    TICKS.load(Ordering::Relaxed),
                                    &mut || service_cdc_scan_io(),
                                );
                                xcore407i_blinker::nand_fs::set_io_service_hook(None);
                                FILE_BROWSER_VIDEO_RENDERER.store(ptr::null_mut(), Ordering::Release);
                                while prepare_next_video_batch(&mut video_renderer) {}
                                if format_requested != xcore407i_blinker::usb_device::cdc_console::FormatRequest::None {
                                    break Ok(format_requested);
                                }
                            }
                        }, |_, _| service_cdc_scan_io());
                        xcore407i_blinker::nand_fs::set_io_service_hook(None);
                        FILE_BROWSER_VIDEO_RENDERER.store(ptr::null_mut(), Ordering::Release);

                        // Keep the state on the alternate stack in the storage-unavailable
                        // console too; do not construct another large state on the main stack.
                        if let Err(error) = mount_result {
                            xcore407i_blinker::usb_device::cdc_console::report_storage_error(error);
                            loop {
                                let _ = tusb::tud_task_ext(1, false);
                                xcore407i_blinker::usb_device::uvc_nv12::xcore_tud_uvc_task(1);
                                let requested = xcore407i_blinker::usb_device::cdc_console::service(
                                    None,
                                    &mut zmodem_state,
                                    TICKS.load(Ordering::Relaxed),
                                    &mut || {
                                        let _ = tusb::tud_task_ext(1, false);
                                        xcore407i_blinker::usb_device::uvc_nv12::xcore_tud_uvc_task(1);
                                        while prepare_next_video_batch(&mut video_renderer) {}
                                    },
                                );
                                while prepare_next_video_batch(&mut video_renderer) {}
                                if requested != xcore407i_blinker::usb_device::cdc_console::FormatRequest::None {
                                    break Ok(requested);
                                }
                            }
                        } else {
                            mount_result
                        }
                    })
                };

        // The mounted-filesystem closure has returned, so LittleFS and its
        // Dhara storage borrow are dropped before the destructive format.
        let format_request = match &mount_result { Ok(request) => *request, Err(_) => xcore407i_blinker::usb_device::cdc_console::FormatRequest::None };

        if format_request == xcore407i_blinker::usb_device::cdc_console::FormatRequest::LittleFs {
            xcore407i_blinker::usb_device::cdc_console::report_format_status(b"Filesystem released. Formatting LittleFS only; Dhara journal is preserved...\r\n");
            FILE_BROWSER_VIDEO_RENDERER.store(&mut video_renderer, Ordering::Release);
            xcore407i_blinker::nand_fs::set_io_service_hook(Some(&FILE_BROWSER_IO_SERVICE));
            let format_result = xcore407i_blinker::nand_fs::format_existing_littlefs();
            xcore407i_blinker::nand_fs::set_io_service_hook(None);
            FILE_BROWSER_VIDEO_RENDERER.store(ptr::null_mut(), Ordering::Release);
            match format_result {
                Ok(()) => {
                    xcore407i_blinker::usb_device::cdc_console::report_format_status(b"LittleFS-only format complete and verified.\r\n");
                    just_formatted = true;
                    continue;
                }
                Err(error) => xcore407i_blinker::usb_device::cdc_console::report_format_failure(error),
            }
        } else if format_request == xcore407i_blinker::usb_device::cdc_console::FormatRequest::WholeNand {
            xcore407i_blinker::usb_device::cdc_console::report_format_status(
                b"Filesystem released. Formatting NAND; all files will be lost...\r\n",
            );
            let mut last_reported = 0u32;
            let format_result =
                xcore407i_blinker::nand_fs::initialize_nand_volume_with_progress(
                    |completed, total| {
                        unsafe {
                            let _ = tusb::tud_task_ext(1, false);
                            xcore407i_blinker::usb_device::uvc_nv12::xcore_tud_uvc_task(1);
                        }
                        while prepare_next_video_batch(&mut video_renderer) {}
                        if completed == total || completed >= last_reported + 32 {
                            last_reported = completed;
                            xcore407i_blinker::usb_device::cdc_console::report_format_status(b".");
                        }
                    },
                );
            match format_result {
                Ok(report) => {
                    xcore407i_blinker::usb_device::cdc_console::report_format_complete(report);
                    just_formatted = true;
                    continue;
                }
                Err(error) => {
                    xcore407i_blinker::usb_device::cdc_console::report_format_failure(error);
                }
            }
        }
        }
    }

    let browser_files = unsafe { &mut *ptr::addr_of_mut!(BROWSER_FILES) };
    let mut browser_count = 0usize;
    let mut browser_selected = 0usize;
    let mut browser_open = false;
    // Real-time emulator pacing uses the PDP cycle counter.
    let mut keyboard_diag_last = keyboard::keyboard_diagnostics();
    let mut keyboard_host_status_last = USB_FS_STATUS.load(Ordering::Acquire);
    let mut keyboard_host_ready_last = USB_FS_READY.load(Ordering::Acquire);
    let mut pacing_start_ms = TICKS.load(Ordering::Relaxed);
    let mut pacing_start_cycles = if emulator_enabled { bk_timing_cycles() } else { 0 };
    #[cfg(feature = "cpu-cycle-diag")]
    let mut audio_diag_last_tick = pacing_start_ms;
    let mut emulator_was_paused = false;
    let mut format_screen_state = FORMAT_IDLE;
    let mut init_screen_state = INIT_IDLE;
    #[cfg(feature = "cpu-cycle-diag")]
    let mut diagnostic_last_dwt = read_dwt_cycle_count();
    #[cfg(feature = "cpu-cycle-diag")]
    let mut diagnostic_last_pdp = pacing_start_cycles;

    loop {
        // Advance the autonomous BK timer even while PDP execution is paused.
        bk_timer_update();

        // Typematic repeat is timer-driven and independent of HID report delivery.
        service_keyboard_repeat();

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
            let _ = tusb::tud_task_ext(1, false);
            xcore407i_blinker::usb_device::uvc_nv12::xcore_tud_uvc_task(1);
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

        if keyboard::take_reset_request() && emulator_enabled {
            // Reset the BK machine and every host-side state machine that can
            // outlive it. USB, camera transport, NAND contents and host HID
            // connectivity are intentionally left running.
            unsafe { bk_reset(); }
            xcore407i_blinker::usb_device::audio_uac2::reset_emulator_capture();
            keyboard::reset_frame_pacing();
            keyboard_queue_discard();
            reset_emulator_ui();
            browser_open = false;
            browser_count = 0;
            browser_selected = 0;
            format_screen_state = FORMAT_IDLE;
            init_screen_state = INIT_IDLE;
            let now_ms = TICKS.load(Ordering::Relaxed);
            pacing_start_ms = now_ms;
            pacing_start_cycles = bk_timing_cycles();
            emulator_was_paused = false;
            #[cfg(feature = "cpu-cycle-diag")]
            {
                diagnostic_last_dwt = read_dwt_cycle_count();
                diagnostic_last_pdp = pacing_start_cycles;
            }
            xcore407i_blinker::camera::reset_demo_canvas_state();
            clear_demo_canvas(0x10);
        }

        // Read after reset handling so a queued browser action cannot survive
        // an F4 reset and reopen the browser on the following loop iteration.
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

        file_browser::process_save_dialog();

        if keyboard::AUDIO_DIAG_SAVE_REQUESTED.swap(false, Ordering::AcqRel) {
            // Stop modifying the ring before reading it. Keep canvas ownership
            // until the export finishes so diagnostic overlays cannot corrupt it.
            unsafe { bk_audio_diag_freeze(); }
            let result = file_browser::save_audio_edge_diagnostics();
            unsafe { bk_audio_diag_end(); }
            clear_demo_canvas(0x10);
            show_usb_status(if result.is_ok() { b"AUDIO EDGES SAVED: /audio_edges.txt" } else { b"AUDIO EDGE SAVE FAILED" });
        }

        let opening_browser = browser_action == FILE_BROWSER_OPEN && !browser_open;
        if opening_browser {
            keyboard_queue_discard();
            FILE_BROWSER_VIDEO_RENDERER.store(&mut video_renderer, Ordering::Release);
            xcore407i_blinker::nand_fs::set_io_service_hook(Some(&FILE_BROWSER_IO_SERVICE));
        }
        file_browser::process_open_action(
            browser_action,
            &mut browser_open,
            &mut browser_count,
            &mut browser_selected,
            &mut *browser_files,
            service_file_browser_io,
        );
        if opening_browser {
            xcore407i_blinker::nand_fs::set_io_service_hook(None);
            FILE_BROWSER_VIDEO_RENDERER.store(ptr::null_mut(), Ordering::Release);
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
                    let file = &browser_files[browser_selected];
                    let path_len = file.len as usize;
                    let result = load_tetris_from_path(&file.path[..path_len]);
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
                    &*browser_files,
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
            pacing_start_cycles = if emulator_enabled { bk_timing_cycles() } else { 0 };
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
            const DWT_CYCLES_PER_SECOND: u32 = SYSTEM_CORE_CLOCK_HZ;
            let now_dwt = read_dwt_cycle_count();
            let elapsed_dwt = now_dwt.wrapping_sub(diagnostic_last_dwt);
            if elapsed_dwt >= DWT_CYCLES_PER_SECOND {
                if !emulator_enabled { continue; }
                let now_pdp = bk_timing_cycles();
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
            let now_cycles = bk_timing_cycles();
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

                        // A BK mode-register write requests a camera-mode change.
                        // Stop executing the emulator until the renderer reaches
                        // its next frame boundary and applies that mode. This
                        // prevents further video writes from using the old canvas
                        // layout while the requested mode is waiting.
                        if xcore407i_blinker::camera::camera_mode_change_pending()
                            && xcore407i_blinker::usb_device::uvc::streaming_active()
                        {
                            break;
                        }
                    }

                    // While streaming, keep the renderer fed but hold BK execution
                    // until it consumes the pending mode at a frame boundary.
                    // USB completion IRQs continue freeing DMA buffers meanwhile.
                    while xcore407i_blinker::camera::camera_mode_change_pending()
                        && xcore407i_blinker::usb_device::uvc::streaming_active()
                    {
                        while prepare_next_video_batch(&mut video_renderer) {}
                    }

                    // Convert queued BK speaker transitions into PCM in thread
                    // context. The USB audio completion callback only drains
                    // this ring and must never perform sample rendering itself.
                    xcore407i_blinker::usb_device::audio_uac2::xcore_tud_audio_render();

                    #[cfg(feature = "cpu-cycle-diag")]
                    {
                        let now = TICKS.load(Ordering::Relaxed);
                        if now.wrapping_sub(audio_diag_last_tick) >= 250 {
                            if !xcore407i_blinker::camera::audio_history_active() {
                                xcore407i_blinker::emulator::bk_audio::show_diagnostics();
                            }
                            audio_diag_last_tick = now;
                        }
                    }

                    service_keyboard_frame_pacing();
                }
            }
        }
    }
}

#[interrupt]
#[allow(non_snake_case)]
fn ETH() {
    xcore407i_blinker::ethernet::on_dma_interrupt();
}

#[interrupt]
#[allow(non_snake_case)]
fn OTG_FS() {
    unsafe { tusb::hcd_int_handler(0, true); }
}

#[interrupt]
#[allow(non_snake_case)]
fn OTG_HS() {
    unsafe { tusb::tusb_int_handler(1, true); }
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
