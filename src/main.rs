#![no_std]
#![no_main]

use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};
use cortex_m::peripheral::{scb::SystemHandler, syst::SystClkSource, NVIC, SCB};
use cortex_m_rt::{entry, exception};
use panic_halt as _;
use stm32f4xx_hal::gpio::{GpioExt, Speed};
use stm32f4::stm32f407::interrupt;
use stm32f4xx_hal::pac;
use xcore407i_blinker::camera::{clear_demo_canvas, request_camera_mode_toggle, show_file_browser_line, show_usb_status, Nv12DemoImage};
use xcore407i_blinker::uvc::prepare_next_video_batch;
use xcore407i_blinker::tusb;
use xcore407i_blinker::rtc;

const TIM3_PRIORITY: u8 = 3;
const SYSTICK_PRIORITY: u8 = 1;

static TICKS: AtomicU32 = AtomicU32::new(0);

// TinyUSB's no-RTOS time source. SysTick increments TICKS once per millisecond.
#[unsafe(no_mangle)]
pub extern "C" fn tusb_time_millis_api() -> u32 {
    TICKS.load(Ordering::Relaxed)
}

// DWC2 STM32 clock setup reads this CMSIS system-clock symbol.
// system_clock_168mhz() configures SYSCLK to 168 MHz before USB initialization.
#[unsafe(no_mangle)]
pub static SystemCoreClock: u32 = 168_000_000;

// C BK platform callback: return the STM32 RTC as Unix seconds (UTC).
// The C I/O layer latches this value when BK reads RTC_TIME_L.
#[unsafe(no_mangle)]
pub extern "C" fn bk_rtc_unix_time() -> u32 {
    let rtc = unsafe { &*pac::RTC::ptr() };
    rtc::unix_time(rtc)
}

const FILE_BROWSER_OPEN: u8 = 1;
const FILE_BROWSER_UP: u8 = 2;
const FILE_BROWSER_DOWN: u8 = 3;
const FILE_BROWSER_ENTER: u8 = 4;
const FILE_BROWSER_ESCAPE: u8 = 5;
const MAX_BROWSER_FILES: usize = 25;
const MAX_BROWSER_PATH: usize = 256;

// Keep the browser's path table in ordinary .bss, not on the limited stack.
static mut BROWSER_PATHS: [[u8; MAX_BROWSER_PATH]; MAX_BROWSER_FILES] =
    [[0; MAX_BROWSER_PATH]; MAX_BROWSER_FILES];
static mut BROWSER_PATH_LENGTHS: [u16; MAX_BROWSER_FILES] = [0; MAX_BROWSER_FILES];
static FILE_BROWSER_ACTIVE: AtomicBool = AtomicBool::new(false);
static FILE_BROWSER_ACTION: AtomicU8 = AtomicU8::new(0);
static USB_FS_STATUS: AtomicU32 = AtomicU32::new(0);
static USB_FS_READY: AtomicBool = AtomicBool::new(false);
static USB_FS_RETRY_AT: AtomicU32 = AtomicU32::new(0);
const USB_FS_RETRY_INTERVAL_MS: u32 = 500;
// TIM3 schedules TinyUSB host work; the actual task runs in thread context.
static USB_HOST_TASK_PENDING: AtomicBool = AtomicBool::new(false);

// HID boot-keyboard state. TinyUSB invokes HID callbacks from tuh_task_ext(),
// so these non-atomic key snapshots are only accessed serially there.
static mut PREVIOUS_KEYS: [u8; 6] = [0; 6];
static mut PREVIOUS_MODIFIERS: u8 = 0;
static KEYBOARD_LOCKS: AtomicU8 = AtomicU8::new(0);
static KEYBOARD_DEVICE: AtomicU8 = AtomicU8::new(0);
static KEYBOARD_INSTANCE: AtomicU8 = AtomicU8::new(0);
static mut KEYBOARD_LED_REPORT: u8 = 0;
// Characters decoded by TinyUSB are handed to the emulator from the main loop.
static mut KEYBOARD_INPUT_BUFFER: [u8; 32] = [0; 32];
static KEYBOARD_INPUT_LEN: AtomicU8 = AtomicU8::new(0);

unsafe extern "C" {
    fn bk_platform_init(monitor_rom: *const u8, basic_rom: *const u8, basic_size_bytes: u32);
    fn bk_loop() -> i32;
    fn sendInput(buffer: *mut u8, size: u8);
    fn bk_set_pc(start: u16);
    fn bk_request_stop_irq();
    static mut RamBuffer: [u8; 0x4000];
}

#[entry]
fn main() -> ! {
    let mut device = pac::Peripherals::take().unwrap();
    system_clock_168mhz(&device.RCC, &device.PWR, &device.FLASH);

    let mut core = cortex_m::Peripherals::take().unwrap();
    core.SCB.enable_fpu();

    rtc::initialize(&device.RCC, &device.PWR, &device.RTC);

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
    // consume already-prepared batches from the 16 kHz TIM2 polling ISR.
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

    // TinyUSB's DCD enables the OTG_HS NVIC line during initialization.
    // Keep it masked and service the DWC2 device interrupt status from TIM2,
    // matching the old stack's timer-driven 16 kHz polling model.
    NVIC::mask(interrupt::OTG_HS);
    NVIC::unpend(interrupt::OTG_HS);
    configure_usb_device_poll_timer(
        &device.RCC,
        &device.TIM2,
        &mut core.NVIC,
    );

    // TIM3 schedules the TinyUSB FS host task at 1 kHz. The ISR only sets a
    // flag; tuh_task_ext() itself must run in thread context, never in an ISR.
    configure_usb_host_task_timer(
        &device.RCC,
        &device.TIM3,
        &mut core.NVIC,
    );

    let browser_paths = unsafe { &mut *ptr::addr_of_mut!(BROWSER_PATHS) };
    let browser_path_lengths = unsafe { &mut *ptr::addr_of_mut!(BROWSER_PATH_LENGTHS) };
    let mut browser_count = 0usize;
    let mut browser_selected = 0usize;
    let mut browser_open = false;

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

        if browser_action == FILE_BROWSER_OPEN && !browser_open {
            KEYBOARD_INPUT_LEN.store(0, Ordering::Release);
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

        // Run one emulator iteration in thread context. Video batches are
        // rendered from the canvas below, never from the USB timer ISR.
        if !browser_open {
            unsafe { bk_loop(); }

            let input_len = KEYBOARD_INPUT_LEN.swap(0, Ordering::AcqRel);
            if input_len > 0 {
                let mut buffer = [0u8; 32];
                unsafe {
                    ptr::copy_nonoverlapping(
                        core::ptr::addr_of!(KEYBOARD_INPUT_BUFFER).cast::<u8>(),
                        buffer.as_mut_ptr(),
                        input_len as usize,
                    );
                    sendInput(buffer.as_mut_ptr(), input_len);
                }
            }
        }

        // Keep the two video buffers replenished outside interrupt context.
        // Do not sleep here: the emulator and USB FS host task need servicing.
        while prepare_next_video_batch(&mut video_renderer) {}
    }
}


/// Read up to 25 regular files from the LittleFS root directory.
/// Paths are retained as NUL-terminated ASCII strings for the later load.
fn list_root_files(
    paths: &mut [[u8; MAX_BROWSER_PATH]; MAX_BROWSER_FILES],
    path_lengths: &mut [u16; MAX_BROWSER_FILES],
) -> Result<usize, ()> {
    xcore407i_blinker::nand_fs::with_mounted(|fs| {
        fs.read_dir_and_then(&littlefs2::path!("/"), |directory| {
            let mut count = 0usize;
            for item in directory {
                let entry = item?;
                if !entry.file_type().is_file() {
                    continue;
                }

                let full_path = entry.path();
                let path_bytes = full_path.as_str_ref_with_trailing_nul().as_bytes();
                if path_bytes.len() < 3
                    || path_bytes.len() > MAX_BROWSER_PATH
                    || !path_bytes[..path_bytes.len() - 1]
                        .iter()
                        .all(|byte| byte.is_ascii_graphic() || *byte == b' ')
                {
                    continue;
                }

                // Only expose .bin files in the browser (case-insensitive).
                // The stored path is NUL-terminated and begins with '/'.
                let filename = &path_bytes[1..path_bytes.len() - 1];
                if filename.len() < 4
                    || !filename[filename.len() - 4..].eq_ignore_ascii_case(b".bin")
                {
                    continue;
                }

                paths[count][..path_bytes.len()].copy_from_slice(path_bytes);
                path_lengths[count] = path_bytes.len() as u16;
                count += 1;
                if count == MAX_BROWSER_FILES {
                    break;
                }
            }
            Ok(count)
        })
    })
    .map_err(|_| ())
}

fn draw_file_browser(
    paths: &[[u8; MAX_BROWSER_PATH]; MAX_BROWSER_FILES],
    path_lengths: &[u16; MAX_BROWSER_FILES],
    count: usize,
    selected: usize,
) {
    clear_demo_canvas(0);
    show_file_browser_line(0, b"NAND FILES - SELECT PROGRAM", false);

    for row in 0..MAX_BROWSER_FILES {
        let mut line = [b' '; 64];
        if row < count {
            line[0] = if row == selected { b'>' } else { b' ' };
            line[1] = b' ';
            let path_len = path_lengths[row] as usize;
            // Root paths have the form "/filename\\0".
            let name = &paths[row][1..path_len - 1];
            let copy_len = name.len().min(line.len() - 2);
            line[2..2 + copy_len].copy_from_slice(&name[..copy_len]);
        } else if row == 0 && count == 0 {
            line[..8].copy_from_slice(b"NO FILES");
        }
        show_file_browser_line(row + 1, &line, row == selected && count != 0);
    }

    show_file_browser_line(27, b"UP/DOWN MOVE  ENTER LOAD  ESC CANCEL", false);
}

/// Load a selected file containing the 4-byte Start/Size header and payload.
fn load_tetris_from_path(path_bytes: &[u8]) -> Result<(), ()> {
    let path = littlefs2::path::Path::from_bytes_with_nul(path_bytes).map_err(|_| ())?;
    let (start, _size) = xcore407i_blinker::nand_fs::with_mounted(|fs| {
        fs.open_file_and_then(path, |file| {
            let mut header = [0u8; 4];
            let mut header_read = 0usize;
            while header_read < header.len() {
                let count = file.read(&mut header[header_read..])?;
                if count == 0 {
                    return Err(littlefs2::io::Error::IO);
                }
                header_read += count;
            }

            let start = u16::from_le_bytes([header[0], header[1]]);
            let size = u16::from_le_bytes([header[2], header[3]]);
            if size == 0 || start as u32 + size as u32 > 0x4000 {
                return Err(littlefs2::io::Error::IO);
            }

            let destination = unsafe {
                core::slice::from_raw_parts_mut(
                    core::ptr::addr_of_mut!(RamBuffer).cast::<u8>().add(start as usize),
                    size as usize,
                )
            };
            let mut total = 0usize;
            while total < size as usize {
                let count = file.read(&mut destination[total..])?;
                if count == 0 {
                    return Err(littlefs2::io::Error::IO);
                }
                total += count;
            }
            Ok((start, size))
        })
    }).map_err(|_| ())?;

    unsafe { bk_set_pc(start); }
    Ok(())
}

#[interrupt]
fn OTG_FS() {
    unsafe { tusb::hcd_int_handler(0, true); }
}

#[interrupt]
fn OTG_HS() {
    unsafe { tusb::tusb_int_handler(1, true); }
}

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

#[unsafe(no_mangle)]
pub extern "C" fn tuh_hid_mount_cb(dev_addr: u8, instance: u8, _desc: *const u8, _len: u16) {
    // HID interface protocol 1 is a boot keyboard; ignore unrelated HID
    // interfaces rather than decoding a mouse/gamepad report as keystrokes.
    if unsafe { tusb::tuh_hid_interface_protocol(dev_addr, instance) } != 1 {
        return;
    }
    KEYBOARD_DEVICE.store(dev_addr, Ordering::Release);
    KEYBOARD_INSTANCE.store(instance, Ordering::Release);
    KEYBOARD_LOCKS.store(0, Ordering::Release);
    KEYBOARD_INPUT_LEN.store(0, Ordering::Release);
    unsafe {
        PREVIOUS_KEYS = [0; 6];
        PREVIOUS_MODIFIERS = 0;
        KEYBOARD_LED_REPORT = 0;
    }
    USB_FS_STATUS.store(4, Ordering::Release);

    // TinyUSB normally selects boot protocol for boot keyboards during
    // enumeration (CFG_TUH_HID_SET_PROTOCOL_ON_ENUM). Verify that before
    // interpreting reports as the standard 8-byte boot-keyboard packet.
    if unsafe { tusb::tuh_hid_get_protocol(dev_addr, instance) } != 0 {
        // Request boot protocol; the completion callback starts reception.
        if !unsafe { tusb::tuh_hid_set_protocol(dev_addr, instance, 0) } {
            USB_FS_STATUS.store(9, Ordering::Release);
        }
    } else {
        unsafe { tusb::tuh_hid_receive_report(dev_addr, instance); }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn tuh_hid_umount_cb(dev_addr: u8, instance: u8) {
    if KEYBOARD_DEVICE.load(Ordering::Acquire) == dev_addr
        && KEYBOARD_INSTANCE.load(Ordering::Acquire) == instance
    {
        KEYBOARD_DEVICE.store(0, Ordering::Release);
        KEYBOARD_INSTANCE.store(0, Ordering::Release);
        KEYBOARD_LOCKS.store(0, Ordering::Release);
        KEYBOARD_INPUT_LEN.store(0, Ordering::Release);
        unsafe {
            PREVIOUS_KEYS = [0; 6];
            PREVIOUS_MODIFIERS = 0;
        }
    }
    USB_FS_STATUS.store(5, Ordering::Release);
}

#[unsafe(no_mangle)]
pub extern "C" fn tuh_hid_set_protocol_complete_cb(dev_addr: u8, instance: u8, protocol: u8) {
    // Ignore late control-transfer completions from a keyboard that has
    // already disconnected or been replaced by another HID interface.
    if KEYBOARD_DEVICE.load(Ordering::Acquire) != dev_addr
        || KEYBOARD_INSTANCE.load(Ordering::Acquire) != instance
    {
        return;
    }

    if protocol == 0 {
        unsafe { tusb::tuh_hid_receive_report(dev_addr, instance); }
    } else {
        USB_FS_STATUS.store(9, Ordering::Release);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn tuh_hid_report_received_cb(
    dev_addr: u8,
    instance: u8,
    report: *const u8,
    len: u16,
) {
    if KEYBOARD_DEVICE.load(Ordering::Acquire) != dev_addr
        || KEYBOARD_INSTANCE.load(Ordering::Acquire) != instance
    {
        return;
    }

    if len >= 8 && !report.is_null() {
        let bytes = unsafe { core::slice::from_raw_parts(report, len as usize) };
        process_boot_keyboard_report(dev_addr, instance, bytes);
    }
    // Re-arm only the still-active keyboard. A disconnect/reconnect can
    // reuse device addresses, so stale callbacks must not target a new HID.
    if KEYBOARD_DEVICE.load(Ordering::Acquire) == dev_addr
        && KEYBOARD_INSTANCE.load(Ordering::Acquire) == instance
    {
        unsafe { tusb::tuh_hid_receive_report(dev_addr, instance); }
    }
}

/// Process an 8-byte USB HID boot-keyboard report:
/// [modifier, reserved, key1..key6].
fn process_boot_keyboard_report(dev_addr: u8, instance: u8, report: &[u8]) {
    if KEYBOARD_DEVICE.load(Ordering::Acquire) != dev_addr
        || KEYBOARD_INSTANCE.load(Ordering::Acquire) != instance
    {
        return;
    }

    let modifiers = report[0];
    let shift = modifiers & 0x22 != 0;
    let control = modifiers & 0x11 != 0;
    let gui = modifiers & 0x88 != 0;
    let control_or_alt = modifiers & 0xdd != 0;
    let previous_modifiers = unsafe { PREVIOUS_MODIFIERS };
    let control_pressed = control && previous_modifiers & 0x11 == 0;
    let gui_pressed = gui && previous_modifiers & 0x88 == 0;
    let locks_before = KEYBOARD_LOCKS.load(Ordering::Acquire);
    let mut locks = locks_before;
    let mut text = [b' '; 32];
    let mut used = 0usize;
    let mut emulator_input = [0u8; 32];
    let mut emulator_used = 0usize;

    let previous = unsafe { PREVIOUS_KEYS };
    let mut current = [0u8; 6];

    // AZBK standard modifier keys are emitted on the modifier's rising edge.
    if control_pressed {
        append_keyboard_char(&mut emulator_input, &mut emulator_used, 0x1c); // СУ
    }
    // АР2 is a modifier (Alt/AltGr), not a standalone BK key code.
    if gui_pressed {
        // A single Win key toggles the language. Read the live BK flag so
        // changes made by the emulator itself are respected:
        // bit 7 set = РУС, clear = ЛАТ.
        let language_flag = unsafe { RamBuffer[0x23] };
        let language_key = if language_flag & 0x80 != 0 {
            0x0f // Currently РУС: request ЛАТ.
        } else {
            0x0e // Currently ЛАТ: request РУС.
        };
        append_keyboard_char(&mut emulator_input, &mut emulator_used, language_key);
    }
    current.copy_from_slice(&report[2..8]);

    for &key in current.iter() {
        if key == 0 || previous.contains(&key) {
            continue;
        }

        // STOP is a hardware IRQ1, not a character sent through 177662.
        // Pause and F12 are equivalent host keys for the BK STOP signal.
        if key == 0x48 || key == 0x45 {
            unsafe { bk_request_stop_irq(); }
            continue;
        }

        // F1 requests a camera mode change. The camera applies it at the
        // next UVC frame boundary and clears the canvas in the new format.
        if key == 0x3a {
            request_camera_mode_toggle();
            continue;
        }

        // F2 opens the NAND file browser; directory access runs in main().
        if key == 0x3b {
            FILE_BROWSER_ACTIVE.store(true, Ordering::Release);
            FILE_BROWSER_ACTION.store(FILE_BROWSER_OPEN, Ordering::Release);
            continue;
        }

        // While the browser is active, consume navigation/selection keys and
        // prevent them from reaching the BK emulator.
        if FILE_BROWSER_ACTIVE.load(Ordering::Acquire) {
            let action = match key {
                0x52 => FILE_BROWSER_UP,    // Up Arrow
                0x51 => FILE_BROWSER_DOWN,  // Down Arrow
                0x28 => FILE_BROWSER_ENTER, // Enter
                0x29 => FILE_BROWSER_ESCAPE,// Escape
                _ => 0,
            };
            if action != 0 {
                FILE_BROWSER_ACTION.store(action, Ordering::Release);
            }
            continue;
        }

        // HID keyboard usages for the three toggle keys.
        match key {
            0x39 => locks ^= 0x02, // Caps Lock
            0x53 => locks ^= 0x01, // Num Lock
            0x47 => locks ^= 0x04, // Scroll Lock
            _ => {}
        }

        if let Some(ch) = key_to_ascii(key, shift, locks & 0x02 != 0, locks & 0x01 != 0, control_or_alt) {
            append_keyboard_char(&mut text, &mut used, ch);
            append_keyboard_char(&mut emulator_input, &mut emulator_used, ch);
        } else if let Some(label) = special_key_label(key, locks & 0x01 != 0) {
            if used != 0 { append_keyboard_char(&mut text, &mut used, b' '); }
            append_keyboard_text(&mut text, &mut used, label);
        }
    }

    unsafe {
        PREVIOUS_KEYS = current;
        PREVIOUS_MODIFIERS = modifiers;
    }

    if locks != locks_before {
        KEYBOARD_LOCKS.store(locks, Ordering::Release);
        // HID LED output bits: Num=bit0, Caps=bit1, Scroll=bit2.
        // SET_REPORT is asynchronous; the one-byte buffer is static and
        // remains valid until TinyUSB completes the control transfer.
        unsafe {
            KEYBOARD_LED_REPORT = locks & 0x07;
            let led_report = core::ptr::addr_of_mut!(KEYBOARD_LED_REPORT);
            let _ = tusb::tuh_hid_set_report(
                dev_addr, instance, 0, 2, led_report.cast(), 1,
            );
        }

        /*
        let event: &[u8] = match locks & 0x07 {
            0 => b"USB KEYBOARD: LOCKS OFF",
            1 => b"USB KEYBOARD: NUM LOCK",
            2 => b"USB KEYBOARD: CAPS LOCK",
            3 => b"USB KEYBOARD: CAPS + NUM LOCK",
            4 => b"USB KEYBOARD: SCROLL LOCK",
            5 => b"USB KEYBOARD: NUM + SCROLL LOCK",
            6 => b"USB KEYBOARD: CAPS + SCROLL LOCK",
            _ => b"USB KEYBOARD: ALL LOCKS",
        };
        show_usb_status(event);
        */
    }

    // Only decoded key bytes go to the emulator; display labels are not BK input.
    if emulator_used != 0 {
        unsafe {
            ptr::copy_nonoverlapping(
                emulator_input.as_ptr(),
                core::ptr::addr_of_mut!(KEYBOARD_INPUT_BUFFER).cast::<u8>(),
                emulator_used,
            );
        }
        KEYBOARD_INPUT_LEN.store(emulator_used as u8, Ordering::Release);
    }
}

fn append_keyboard_char(out: &mut [u8; 32], used: &mut usize, ch: u8) {
    if *used < out.len() {
        out[*used] = ch;
        *used += 1;
    }
}

fn append_keyboard_text(out: &mut [u8; 32], used: &mut usize, text: &[u8]) {
    for &ch in text {
        append_keyboard_char(out, used, ch);
    }
}

fn key_to_ascii(key: u8, shift: bool, caps: bool, num: bool, control_or_alt: bool) -> Option<u8> {
    // AZBK standard PC-to-BK special-key assignments.
    // Values are the BK control-key codes consumed by the emulator.
    match key {
        0x29 => return Some(0x1b), // Esc -> КТ
        0x49 => return Some(0x17), // Insert -> ВС
        0x4c => return Some(0x1d), // Delete -> ГРАФ
        0x3c => return Some(0x1e), // F3 -> УСТ
        0x4d => return Some(0x1f), // End -> ЗАП
        0x4a => return Some(0x14), // Home -> ТАБ
        0x3e => return Some(0x10), // F5 -> ИНД СУ
        0x3f => return Some(0x11), // F6 -> БЛОК РЕД
        0x40 => return Some(0x12), // F7 -> ШАГ
        0x41 => return Some(0x13), // F8 -> СБР
        0x4e => return Some(0x18), // Page Down -> СТИР
        // The keypad navigation usages act as arrows when Num Lock is off.
        0x4f => return Some(0x19), // Right
        0x50 => return Some(0x08), // Left
        0x51 => return Some(0x1b), // Down
        0x52 => return Some(0x1a), // Up
        0x5a if !num => return Some(0x1b), // Keypad Down
        0x5c if !num => return Some(0x08), // Keypad Left
        0x5e if !num => return Some(0x19), // Keypad Right
        0x60 if !num => return Some(0x1a), // Keypad Up
        _ => {}
    }

    if control_or_alt { return None; }

    match key {
        0x04..=0x1d => {
            let lower = b'a' + (key - 0x04);
            Some(if shift ^ caps { lower - 32 } else { lower })
        }
        0x1e..=0x27 => {
            const NORMAL: &[u8; 10] = b"1234567890";
            const SHIFTED: &[u8; 10] = b"!@#$%^&*()";
            let i = (key - 0x1e) as usize;
            Some(if shift { SHIFTED[i] } else { NORMAL[i] })
        }
        0x28 => Some(b'\n'), // BK OnKey maps Enter to 0x0A.
        0x2a => Some(0x18), // BK OnKey maps Backspace to 0x18.
        0x2c => Some(b' '),
        0x2d => Some(if shift { b'_' } else { b'-' }),
        0x2e => Some(if shift { b'+' } else { b'=' }),
        0x2f => Some(if shift { b'{' } else { b'[' }),
        0x30 => Some(if shift { b'}' } else { b']' }),
        0x31 => Some(if shift { b'|' } else { b'\\' }),
        0x33 => Some(if shift { b':' } else { b';' }),
        0x34 => Some(if shift { b'"' } else { b'\'' }),
        0x35 => Some(if shift { b'~' } else { b'`' }),
        0x36 => Some(if shift { b'<' } else { b',' }),
        0x37 => Some(if shift { b'>' } else { b'.' }),
        0x38 => Some(if shift { b'?' } else { b'/' }),
        0x54 => Some(b'/'),
        0x55 => Some(b'*'),
        0x56 => Some(b'-'),
        0x57 => Some(b'+'),
        0x58 => Some(b'\n'),
        0x59..=0x61 if num => Some(b'1' + (key - 0x59)),
        0x62 if num => Some(b'0'),
        0x63 if num => Some(b'.'),
        _ => None,
    }
}

fn special_key_label(key: u8, num: bool) -> Option<&'static [u8]> {
    match key {
        0x28 => Some(b"[ENTER]"),
        0x29 => Some(b"[ESC]"),
        0x2a => Some(b"[BACKSPACE]"),
        0x2b => Some(b"[TAB]"),
        0x39 => Some(b"[CAPS]"),
        0x46 => Some(b"[PRTSC]"),
        0x47 => Some(b"[SCROLL]"),
        0x48 => Some(b"[PAUSE]"),
        0x49 => Some(b"[INS]"),
        0x4a => Some(b"[HOME]"),
        0x4b => Some(b"[PGUP]"),
        0x4c => Some(b"[DEL]"),
        0x4d => Some(b"[END]"),
        0x4e => Some(b"[PGDN]"),
        0x4f => Some(b"[RIGHT]"),
        0x50 => Some(b"[LEFT]"),
        0x51 => Some(b"[DOWN]"),
        0x52 => Some(b"[UP]"),
        0x53 => Some(b"[NUM]"),
        0x59 if !num => Some(b"[KP-END]"),
        0x5a if !num => Some(b"[KP-DOWN]"),
        0x5b if !num => Some(b"[KP-PGDN]"),
        0x5c if !num => Some(b"[KP-LEFT]"),
        0x5d if !num => Some(b"[KP5]"),
        0x5e if !num => Some(b"[KP-RIGHT]"),
        0x5f if !num => Some(b"[KP-HOME]"),
        0x60 if !num => Some(b"[KP-UP]"),
        0x61 if !num => Some(b"[KP-PGUP]"),
        0x62 if !num => Some(b"[KP-INS]"),
        0x63 if !num => Some(b"[KP-DEL]"),
        0x3a => Some(b"[F1]"),
        0x3b => Some(b"[F2]"),
        0x3c => Some(b"[F3]"),
        0x3d => Some(b"[F4]"),
        0x3e => Some(b"[F5]"),
        0x3f => Some(b"[F6]"),
        0x40 => Some(b"[F7]"),
        0x41 => Some(b"[F8]"),
        0x42 => Some(b"[F9]"),
        0x43 => Some(b"[F10]"),
        0x44 => Some(b"[F11]"),
        0x45 => Some(b"[F12]"),
        0x65 => Some(b"[MENU]"),
        _ => None,
    }
}

/// Configure TIM3 as a 1 kHz scheduler tick for the TinyUSB FS host task.
///
/// TinyUSB's host task dispatches callbacks and is not ISR-safe. TIM3 therefore
/// only requests a task run; main() performs the work in thread context.

/// Reset and initialize the internal-FS-PHY OTG_FS controller as a TinyUSB
/// host. The IRQ remains masked on failure so an uninitialized controller
/// cannot vector into the host stack.
fn try_init_usb_fs_host(rcc: &pac::RCC, nvic: &mut NVIC) -> bool {
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

/// Configure TIM2 as a 16 kHz polling source for the TinyUSB OTG_HS device.
///
/// The OTG_HS NVIC line remains masked. Each timer interrupt invokes TinyUSB's
/// DCD interrupt handler directly, matching the old stack's timer-driven
/// servicing model while keeping tud_task_ext() in the main loop.
fn configure_usb_device_poll_timer(
    rcc: &pac::RCC,
    tim2: &pac::TIM2,
    nvic: &mut NVIC,
) {
    // TIM2CLK = 84 MHz. PSC=0 and ARR=5249 produce exactly 16 kHz.
    rcc.apb1enr().modify(|_, w| w.tim2en().set_bit());

    rcc.apb1rstr().modify(|_, w| w.tim2rst().set_bit());
    rcc.apb1rstr().modify(|_, w| w.tim2rst().clear_bit());

    tim2.cr1().write(|w| w.cen().clear_bit());
    tim2.dier().write(|w| w.uie().clear_bit());
    tim2.psc().write(|w| unsafe { w.psc().bits(0) });
    tim2.arr().write(|w| unsafe { w.arr().bits(5249) });
    tim2.egr().write(|w| w.ug().set_bit());
    tim2.sr().write(|w| w.uif().clear_bit());

    unsafe {
        nvic.set_priority(interrupt::TIM2, 0);
        NVIC::unpend(interrupt::TIM2);
        NVIC::unmask(interrupt::TIM2);
    }

    tim2.dier().write(|w| w.uie().set_bit());
    tim2.cr1().write(|w| w.cen().set_bit());
}

#[interrupt]
fn TIM2() {
    let tim2 = unsafe { &*pac::TIM2::ptr() };
    tim2.sr().write(|w| w.uif().clear_bit());

    // Process DWC2 device events in interrupt context, but defer TinyUSB's
    // device task and class-event dispatch to the main loop.
    unsafe { tusb::tusb_int_handler(1, true); }
}

fn configure_usb_host_task_timer(
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

#[interrupt]
fn TIM3() {
    let tim3 = unsafe { &*pac::TIM3::ptr() };
    tim3.sr().write(|w| w.uif().clear_bit());
    USB_HOST_TASK_PENDING.store(true, Ordering::Release);
}

/// Set a simple preemption-priority grouping and the system-handler priorities.
/// STM32F407 implements four priority bits, stored in the upper nibble.
fn configure_exception_priorities(scb: &mut SCB) {
    const SCB_AIRCR: *mut u32 = 0xE000_ED0C as *mut u32;

    unsafe {
        // PRIGROUP=0: all implemented priority bits are preemption priority.
        let aircr = SCB_AIRCR.read_volatile();
        SCB_AIRCR.write_volatile((aircr & !(7 << 8)) | (0x5FA << 16));

        // USB controller interrupts preempt the emulator loop. SysTick
        // remains enabled at priority 1 for TinyUSB's millisecond timebase.
        scb.set_priority(SystemHandler::SysTick, SYSTICK_PRIORITY << 4);

        // Keep OTG_HS masked until TinyUSB's DCD initialization enables it.
        NVIC::mask(interrupt::OTG_HS);
        NVIC::unpend(interrupt::OTG_HS);
    }
}

fn enable_cycle_counter(core: &mut cortex_m::Peripherals) {
    // DWT access is provided by cortex-m; global trace must be enabled first.
    core.DCB.enable_trace();
    core.DWT.set_cycle_count(0);
    core.DWT.enable_cycle_counter();
}

#[exception]
fn SysTick() {
    TICKS.fetch_add(1, Ordering::Relaxed);
}

fn system_clock_168mhz(rcc: &pac::RCC, pwr: &pac::PWR, flash: &pac::FLASH) {
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
fn configure_board_pins(
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
        bsrr.write_volatile(1 << (1 + 16));
        moder.write_volatile((moder.read_volatile() & !(0b11 << 2)) | (0b01 << 2));
        otyper.write_volatile(otyper.read_volatile() & !(1 << 1));
        ospeedr.write_volatile(ospeedr.read_volatile() & !(0b11 << 2));
        pupdr.write_volatile(pupdr.read_volatile() & !(0b11 << 2));
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

