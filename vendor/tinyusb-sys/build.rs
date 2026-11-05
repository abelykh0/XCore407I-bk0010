#![allow(warnings)]

use cc::Build;
use std::{env, path::{Path, PathBuf}};
use std::fs::File;
use std::io::prelude::*;

mod tusb_config;

fn add_all_c_files_in_dir(build: &mut Build, path: impl AsRef<Path>) {
    for entry in glob::glob(path.as_ref().join("**/*.c").to_str().unwrap()).unwrap() {
        let path = entry.unwrap();
        if path.extension().and_then(|s| s.to_str()) == Some("c") {
            build.file(&path);
        }
    }
}

fn compiler_include_paths() -> Vec<String> {
    // Feed an empty translation unit on stdin. Using "-" instead of
    // "/dev/null" keeps this working with the MinGW-hosted ARM GCC on Windows.
    let mut command = Build::new().get_compiler().to_command();
    command
        .args(["-E", "-Wp,-v", "-xc", "-"])
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::piped());

    let mut child = command.spawn().expect("Failed to run the compiler to get include paths");
    drop(child.stdin.take());

    let output = child
        .wait_with_output()
        .expect("Failed to query compiler include paths");

    if !output.status.success() {
        panic!(
            "Compiler failed while querying include paths: {}",
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
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=tusb_config.rs");
    println!("cargo:rerun-if-changed=tinyusb/src");

    let out_dir = PathBuf::from(env::var("OUT_DIR").expect("Missing OUT_DIR"));

    {
        let mut f = File::create(out_dir.join("tusb_config.h"))
            .expect("Failed to create tusb_config.h");
        f.write_all(tusb_config::generate_cfg().as_bytes())
            .expect("Failed to write to tusb_config.h");
    }

    let include_paths = compiler_include_paths();
    eprintln!("include_paths={include_paths:?}");

    let mut build = Build::new();
    add_all_c_files_in_dir(&mut build, "tinyusb/src");
    build
        .include("tinyusb/src")
        .include("tinyusb/hw/mcu/st/cmsis_device_f4/Include")
        .include("../../../Drivers/CMSIS/Include")
        .include("../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include")
        .define("STM32F407xx", None)
        .include(&out_dir)
        .compile("tinyusb");

    let target = env::var("TARGET").expect("Missing TARGET env var");
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    let bindings = bindgen::Builder::default()
        .header("tinyusb/src/tusb.h")
        .rustified_enum(".*")
        .clang_arg(&format!("-I{}", out_dir.display()))
        .derive_default(true)
        .layout_tests(false)
        .use_core()
        .rustfmt_bindings(true)
        .ctypes_prefix("cty")
        .clang_args(&[
            "-target",
            &target,
            "-fvisibility=default",
            "-fshort-enums",
        ])
        .clang_arg("-Itinyusb/src")
        .clang_arg("-Itinyusb/hw/mcu/st/cmsis_device_f4/Include")
        .clang_arg("-I../../../Drivers/CMSIS/Include")
        .clang_arg("-DSTM32F407xx")
        .clang_args(&include_paths)
        .generate()
        .expect("Unable to generate bindings");

    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Can't write bindings!");
}
