use std::{env, fs, path::Path};

use cc::Build;

fn add_all_c_files_in_dir(build: &mut Build, path: impl AsRef<Path>) {
    let pattern = path.as_ref().join("**/*.c");
    for entry in glob::glob(pattern.to_str().expect("TinyUSB source path is not UTF-8"))
        .expect("Failed to enumerate TinyUSB C sources")
    {
        let source = entry.expect("Failed to read TinyUSB source path");
        build.file(source);
    }
}

// Keep bindgen's view of system include paths consistent with the target C
// compiler, especially for the Windows-hosted arm-none-eabi-gcc toolchain.
fn compiler_include_paths() -> Vec<String> {
    let mut command = Build::new().get_compiler().to_command();
    command
        .args(["-E", "-Wp,-v", "-xc", "-"])
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::piped());

    let mut child = command
        .spawn()
        .expect("Failed to run the target compiler to get include paths");
    drop(child.stdin.take());
    let output = child
        .wait_with_output()
        .expect("Failed to query target compiler include paths");

    if !output.status.success() {
        panic!(
            "Target compiler failed while querying include paths: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }

    String::from_utf8(output.stderr)
        .expect("Compiler include-path output was not UTF-8")
        .lines()
        .filter_map(|line| line.strip_prefix(' '))
        .map(|path| format!("-I{path}"))
        .collect()
}

fn main() {
    println!("cargo:rerun-if-changed=csrc/bk");
    println!("cargo:rerun-if-changed=usb/tusb_config.h");
    println!("cargo:rerun-if-changed=usb/tusb_class_drivers.c");
    println!("cargo:rerun-if-changed=usb/tusb_class_drivers.h");
    println!("cargo:rerun-if-changed=vendor/tinyusb-sys/tinyusb/src");
    println!("cargo:rerun-if-changed=vendor/dhara");
    println!("cargo:rerun-if-changed=csrc/dhara_adapter.c");

    let target = env::var("TARGET").expect("TARGET not set");

    // Existing BK emulator C implementation.
    let mut bk = Build::new();
    bk.target(&target)
        .include("csrc/bk")
        .flag("-std=gnu11")
        .flag("-ffunction-sections")
        .flag("-fdata-sections")
        .files([
            "csrc/bk/branch.c",
            "csrc/bk/double.c",
            "csrc/bk/ea.c",
            "csrc/bk/itab.c",
            "csrc/bk/service.c",
            "csrc/bk/single.c",
            "csrc/bk/timing.c",
            "csrc/bk/weird.c",
            "csrc/bk/platform.c",
        ])
        .compile("bk_cpu");

    // Compile upstream Dhara only when explicitly requested. The optional
    // LittleFS adapter is linked into the same archive as the Dhara core.
    if env::var_os("CARGO_FEATURE_DHARA_EVAL").is_some()
        || env::var_os("CARGO_FEATURE_DHARA_LITTLEFS").is_some()
    {
        let mut dhara = Build::new();
        dhara
            .target(&target)
            .flag("-std=gnu11")
            .flag("-ffunction-sections")
            .flag("-fdata-sections")
            .include("vendor/dhara/include")
            .files([
                "vendor/dhara/dhara/error.c",
                "vendor/dhara/dhara/journal.c",
                "vendor/dhara/dhara/map.c",
            ]);
        if env::var_os("CARGO_FEATURE_DHARA_LITTLEFS").is_some() {
            // Keep the adapter in the same archive as Dhara to avoid
            // static-library link-order dependencies between the two.
            dhara.file("csrc/dhara_adapter.c");
        }
        dhara.compile("dhara");
    }

    // Build TinyUSB directly from its upstream C sources. This removes the
    // tinyusb-sys Rust wrapper while retaining the existing checked-in
    // TinyUSB 0.21.0 source tree during the stack migration.
    let tinyusb_root = "vendor/tinyusb-sys/tinyusb";
    let cmsis_device = format!("{tinyusb_root}/hw/mcu/st/cmsis_device_f4/Include");
    let cmsis_core = "vendor/tinyusb-sys/../../../Drivers/CMSIS/Include";
    let cmsis_st_device = "vendor/tinyusb-sys/../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include";

    let mut tinyusb = Build::new();
    tinyusb
        .target(&target)
        .flag("-std=gnu11")
        .flag("-ffunction-sections")
        .flag("-fdata-sections")
        .include("usb")
        .file("usb/tusb_class_drivers.c")
        .include(format!("{tinyusb_root}/src"))
        .include(&cmsis_device)
        .include(cmsis_core)
        .include(cmsis_st_device)
        .define("STM32F407xx", None);
    add_all_c_files_in_dir(&mut tinyusb, format!("{tinyusb_root}/src"));
    tinyusb.compile("tinyusb");

    // Generate bindings for the application's small host/device API surface.
    // This is binding generation only; there is no tinyusb-sys crate wrapper.
    let out_dir = env::var("OUT_DIR").expect("OUT_DIR not set");
    let include_paths = compiler_include_paths();
    let bindings = bindgen::Builder::default()
        .header(format!("{tinyusb_root}/src/tusb.h"))
        .rustified_enum(".*")
        .derive_default(true)
        .layout_tests(false)
        .use_core()
        .formatter(bindgen::Formatter::Rustfmt)
        .ctypes_prefix("core::ffi")
        // TinyUSB's UVC probe/commit control contains nested packed anonymous
        // bitfield structs. bindgen 0.68 emits an invalid Rust repr(C, packed,
        // align) for one of those nested types. Keep the public named type as
        // an opaque ABI-sized type (so API declarations remain valid), while
        // preventing bindgen from generating the problematic nested layout.
        // The application transfers UVC control payloads as raw bytes.
        .opaque_type("video_probe_and_commit_control_t")
        // The nested helper is referenced by a generated union field, so it
        // must remain declared. Make it opaque rather than blocklisting it;
        // this preserves the Rust type name while avoiding its invalid layout.
        .opaque_type("video_probe_and_commit_control_t__bindgen_ty_1__bindgen_ty_1")
        .clang_args(&[
            "-target",
            &target,
            "-fvisibility=default",
            "-fshort-enums",
        ])
        .clang_arg("-Iusb")
        .clang_arg(format!("-I{tinyusb_root}/src"))
        .clang_arg(format!("-I{cmsis_device}"))
        .clang_arg(format!("-I{cmsis_core}"))
        .clang_arg(format!("-I{cmsis_st_device}"))
        .clang_arg("-DSTM32F407xx")
        .clang_args(&include_paths)
        .generate()
        .expect("Unable to generate TinyUSB bindings");

    let bindings_path = format!("{out_dir}/tusb_bindings.rs");
    bindings
        .write_to_file(&bindings_path)
        .expect("Failed to write TinyUSB bindings");

    // bindgen 0.68 emits plain extern blocks, which are rejected by this
    // crate's Rust 2024 edition. Mark generated foreign blocks unsafe as
    // required by Rust 2024. This is valid in Rust 2021 as well.
    let generated = fs::read_to_string(&bindings_path)
        .expect("Failed to read generated TinyUSB bindings");
    let generated = generated
        .replace("extern \"C\" {", "unsafe extern \"C\" {")
        .replace("extern \"system\" {", "unsafe extern \"system\" {");
    fs::write(&bindings_path, generated)
        .expect("Failed to update generated TinyUSB bindings");

    // Track all config and source inputs so incremental builds re-run correctly.
    println!("cargo:rerun-if-changed={tinyusb_root}/src");
    println!("cargo:rerun-if-changed=usb/tusb_config.h");
}
