use std::{env, path::PathBuf};

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    let status = std::process::Command::new("rustc")
        .args([
            "--crate-type=cdylib",
            "stub/lib.rs",
            "-o",
            &format!("{}/libagnocast.so", out_dir.display()),
        ])
        .status()
        .expect("failed to compile stub library");

    assert!(status.success(), "rustc failed to compile stub library");

    println!("cargo:rustc-link-search=native={}", out_dir.display());
    println!("cargo:rustc-link-lib=dylib=agnocast");

    // Detect glibc version. On glibc < 2.35, dlsym(RTLD_NEXT, ...) deadlocks
    // when called from malloc hooks during LD_PRELOAD initialization.
    // Allow explicit override for cross-compilation where the build host's
    // glibc differs from the target's.
    println!("cargo:rustc-check-cfg=cfg(glibc_pre_2_35)");
    let glibc_pre_2_35 = match env::var("AGNOCAST_GLIBC_PRE_2_35") {
        Ok(val) => match val.as_str() {
            "1" | "true" => true,
            "0" | "false" => false,
            _ => is_glibc_pre_2_35(),
        },
        Err(_) => is_glibc_pre_2_35(),
    };
    if glibc_pre_2_35 {
        println!("cargo:rustc-cfg=glibc_pre_2_35");
    }
}

fn is_glibc_pre_2_35() -> bool {
    let output = match std::process::Command::new("ldd").arg("--version").output() {
        Ok(o) => o,
        Err(_) => return false,
    };
    let stdout = String::from_utf8_lossy(&output.stdout);
    // ldd --version prints e.g. "ldd (Ubuntu GLIBC 2.31-0ubuntu9.9) 2.31"
    // Verify this is actually glibc (not musl or another libc).
    let version_line = match stdout.lines().next() {
        Some(line) if line.contains("GLIBC") || line.contains("GNU libc") => line,
        _ => return false,
    };
    let version_str = match version_line.rsplit(' ').next() {
        Some(s) => s,
        None => return false,
    };
    let mut parts = version_str.split('.');
    let major: u32 = match parts.next().and_then(|s| s.parse().ok()) {
        Some(v) => v,
        None => return false,
    };
    let minor: u32 = match parts.next().and_then(|s| s.parse().ok()) {
        Some(v) => v,
        None => return false,
    };
    major < 2 || (major == 2 && minor < 35)
}
