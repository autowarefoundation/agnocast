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
    println!("cargo:rustc-check-cfg=cfg(glibc_pre_2_35)");
    if let Some(true) = is_glibc_pre_2_35() {
        println!("cargo:rustc-cfg=glibc_pre_2_35");
    }
}

fn is_glibc_pre_2_35() -> Option<bool> {
    let output = std::process::Command::new("ldd").arg("--version").output().ok()?;
    let stdout = String::from_utf8_lossy(&output.stdout);
    // ldd --version prints e.g. "ldd (Ubuntu GLIBC 2.31-0ubuntu9.9) 2.31"
    let version_line = stdout.lines().next()?;
    let version_str = version_line.rsplit(' ').next()?;
    let mut parts = version_str.split('.');
    let major: u32 = parts.next()?.parse().ok()?;
    let minor: u32 = parts.next()?.parse().ok()?;
    Some(major < 2 || (major == 2 && minor < 35))
}
