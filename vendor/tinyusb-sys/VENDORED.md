# Vendored tinyusb-sys

This directory vendors the latest upstream `tinyusb-sys` repository revision
available at the time of import:

- Repository: https://github.com/nviennot/tinyusb-sys-rs
- Upstream commit: `7f4569eb3284887f090c4f6448d699f59f255c04`
- Upstream change: update bindgen from 0.59 to 0.68
- TinyUSB source: Git submodule at `tinyusb/`, pinned to
  `e188117823f0406cfd8fa65e1e0e3a35019c96f2`, matching the upstream wrapper.

Local change: `build.rs` queries compiler include paths using stdin (`-`)
instead of the Unix-only `/dev/null`, so the build script can run with the
Windows MinGW-hosted ARM GNU Toolchain.

Initialize the TinyUSB submodule after cloning:

```sh
git submodule update --init --recursive
```
