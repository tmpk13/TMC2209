use std::env;
use std::fs;
use std::path::PathBuf;
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

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

    // Version stamp: Unix epoch + git short rev. Surfaced over USB via the
    // GetVersion command so the host UI can confirm the chip is running
    // exactly the binary that was last built. Re-runs every time so a
    // simple `cargo run` always gets a fresh stamp.
    println!("cargo:rerun-if-changed=.git/HEAD");
    println!("cargo:rerun-if-changed=.git/refs");
    let epoch = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    println!("cargo:rustc-env=BUILD_EPOCH={epoch}");
    let git = Command::new("git")
        .args(["rev-parse", "--short=8", "HEAD"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .unwrap_or_else(|| "nogit".to_string());
    println!("cargo:rustc-env=GIT_SHORT={git}");
}
