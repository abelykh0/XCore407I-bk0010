use std::env;
use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let littlefs_path = "littlefs";
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    let lfs_h = std::fs::read_to_string(format!("{littlefs_path}/lfs.h"))?;
    println!("cargo:rerun-if-changed={littlefs_path}/lfs.h");
    let out_lfs_h = out_path.join("lfs.h");
    std::fs::write(
        &out_lfs_h,
        lfs_h.replace(
            r##"#include "lfs_util.h""##,
            "#include <stdint.h>\n#include <stdbool.h>",
        ),
    )?;

    let mut builder = cc::Build::new();
    builder
        .flag("-std=c99")
        .flag("-DLFS_NO_DEBUG")
        .flag("-DLFS_NO_WARN")
        .flag("-DLFS_NO_ERROR")
        .include(&out_path)
        .include(littlefs_path)
        .file(format!("{littlefs_path}/lfs.c"))
        .file(format!("{littlefs_path}/lfs_util.c"));
    #[cfg(feature = "software-intrinsics")]
    let builder = builder.flag("-DLFS_NO_INTRINSICS");
    #[cfg(not(feature = "assertions"))]
    let builder = builder.flag("-DLFS_NO_ASSERT");
    #[cfg(feature = "trace")]
    let builder = builder.flag("-DLFS_YES_TRACE");
    #[cfg(not(feature = "malloc"))]
    builder.flag("-DLFS_NO_MALLOC");
    #[cfg(feature = "multiversion")]
    let builder = builder.flag("-DLFS_MULTIVERSION");
    builder.compile("lfs-sys");

    let bindgen = bindgen::Builder::default()
        .header(out_lfs_h.to_string_lossy())
        .clang_arg("-ffreestanding")
        .clang_arg("-std=c99")
        .clang_arg("-DLFS_NO_DEBUG")
        .clang_arg("-DLFS_NO_WARN")
        .clang_arg("-DLFS_NO_ERROR");
    #[cfg(feature = "multiversion")]
    let bindgen = bindgen.clang_arg("-DLFS_MULTIVERSION");
    bindgen
        .derive_default(true)
        .use_core()
        .allowlist_item("lfs_.*")
        .allowlist_item("LFS_.*")
        .generate()?
        .write_to_file(out_path.join("bindings.rs"))?;
    Ok(())
}
