use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    let (src, label) = match (
        env::var("CARGO_FEATURE_RP2040").is_ok(),
        env::var("CARGO_FEATURE_RP2350").is_ok(),
    ) {
        (true, false) => ("memory_rp2040.x", "rp2040"),
        (false, true) => ("memory_rp2350.x", "rp2350"),
        (true, true) => panic!("features `rp2040` and `rp2350` are mutually exclusive"),
        (false, false) => panic!("enable exactly one of the `rp2040` or `rp2350` features"),
    };

    let bytes = fs::read(src).unwrap_or_else(|e| panic!("reading {src}: {e}"));
    fs::write(out.join("memory.x"), bytes).unwrap();

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed={src}");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rustc-cfg=chip_{label}");
    println!("cargo:rustc-check-cfg=cfg(chip_rp2040,chip_rp2350)");
}
