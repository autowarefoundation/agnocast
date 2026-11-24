//! Exposes shared heap usage to external tools.
//!
//! When `AGNOCAST_HEAPHOOK_MEMORY_STATS` is set, each process maps `/dev/shm/agnocast_heaphook_<pid>`,
//! a 24-byte file holding `current: u64`, `max: u64` and `high_water_mark: u64` (native endian, in bytes).
//! `high_water_mark` is the peak span from the lowest to the highest allocated address, i.e. allocated
//! memory plus external fragmentation. 
//! The allocator updates it on every size change, so tools can poll it without touching the process,
//! e.g. `od -An -t u8 -N24 /dev/shm/agnocast_heaphook_<pid>`.

#[cfg(not(test))]
use std::sync::atomic::AtomicI32;
use std::{
    ptr,
    sync::atomic::{AtomicPtr, AtomicU64, Ordering},
};

#[repr(C)]
struct MemoryStats {
    current: AtomicU64,
    max: AtomicU64,
    high_water_mark: AtomicU64,
}

/// Null when stats are disabled.
static MEMORY_STATS: AtomicPtr<MemoryStats> = AtomicPtr::new(ptr::null_mut());

/// The pid that created the current stats file. Forked children inherit `atexit` handlers,
/// so this prevents them from unlinking their parent's file.
#[cfg(not(test))]
static OWNER_PID: AtomicI32 = AtomicI32::new(0);

#[inline]
pub(crate) fn publish(current: usize, max: usize, high_water_mark: usize) {
    let stats = MEMORY_STATS.load(Ordering::Acquire);
    if stats.is_null() {
        return;
    }
    // SAFETY: a non-null pointer always refers to a live `MemoryStats` mapping created in `init`.
    let stats = unsafe { &*stats };
    stats.current.store(current as u64, Ordering::Relaxed);
    stats.max.store(max as u64, Ordering::Relaxed);
    stats.high_water_mark.store(high_water_mark as u64, Ordering::Relaxed);
}

/// Builds the NUL-terminated name `/agnocast_heaphook_<pid>` without allocating,
/// so it is also safe to call from the exit handler.
#[cfg(not(test))]
fn shm_name(pid: libc::pid_t) -> [u8; 32] {
    const PREFIX: &[u8] = b"/agnocast_heaphook_";
    let mut name = [0u8; 32];
    name[..PREFIX.len()].copy_from_slice(PREFIX);

    let mut digits = [0u8; 10];
    let mut len = 0;
    let mut n = pid as u32;
    loop {
        digits[len] = b'0' + (n % 10) as u8;
        len += 1;
        n /= 10;
        if n == 0 {
            break;
        }
    }
    for i in 0..len {
        name[PREFIX.len() + i] = digits[len - 1 - i];
    }
    name
}

/// Creates the stats file for the current process if `AGNOCAST_HEAPHOOK_MEMORY_STATS` is set.
///
/// Must be called before `main` or while the shared memory allocator is not in use, since it allocates.
/// Failures only print a warning: monitoring must never break the node.
#[cfg(not(test))]
pub(crate) fn init() {
    use std::sync::Once;

    static REGISTER_CLEANUP: Once = Once::new();

    if std::env::var_os("AGNOCAST_HEAPHOOK_MEMORY_STATS").is_none() {
        return;
    }

    // A bridge child inherits the parent's mapping; stop writing into it before replacing it.
    MEMORY_STATS.store(ptr::null_mut(), Ordering::Release);

    let pid = unsafe { libc::getpid() };
    let name = shm_name(pid);
    let size = std::mem::size_of::<MemoryStats>();

    let warn = |what: &str| {
        eprintln!(
            "[WARN] [Agnocast] agnocast_heaphook: memory stats disabled, {} failed: {}",
            what,
            std::io::Error::last_os_error()
        );
    };

    let fd = unsafe {
        libc::shm_open(
            name.as_ptr().cast(),
            libc::O_CREAT | libc::O_RDWR | libc::O_TRUNC,
            0o644,
        )
    };
    if fd == -1 {
        warn("shm_open");
        return;
    }

    if unsafe { libc::ftruncate(fd, size as libc::off_t) } == -1 {
        warn("ftruncate");
        unsafe {
            libc::close(fd);
            libc::shm_unlink(name.as_ptr().cast());
        }
        return;
    }

    let addr = unsafe {
        libc::mmap(
            ptr::null_mut(),
            size,
            libc::PROT_READ | libc::PROT_WRITE,
            libc::MAP_SHARED,
            fd,
            0,
        )
    };
    unsafe { libc::close(fd) };
    if addr == libc::MAP_FAILED {
        warn("mmap");
        unsafe { libc::shm_unlink(name.as_ptr().cast()) };
        return;
    }

    OWNER_PID.store(pid, Ordering::Relaxed);
    MEMORY_STATS.store(addr as *mut MemoryStats, Ordering::Release);

    REGISTER_CLEANUP.call_once(|| {
        if unsafe { libc::atexit(cleanup) } != 0 {
            eprintln!("[WARN] [Agnocast] agnocast_heaphook: atexit failed, memory stats file will not be removed on exit");
        }
    });
}

#[cfg(not(test))]
extern "C" fn cleanup() {
    let pid = unsafe { libc::getpid() };
    if MEMORY_STATS.load(Ordering::Acquire).is_null() || OWNER_PID.load(Ordering::Relaxed) != pid {
        return;
    }
    let name = shm_name(pid);
    unsafe { libc::shm_unlink(name.as_ptr().cast()) };
}
