use std::env;

fn main() {
    println!("cargo:rerun-if-changed=csrc/bk");

    let target = env::var("TARGET").expect("TARGET not set");
    let mut build = cc::Build::new();
    build
        .target(&target)
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
}
