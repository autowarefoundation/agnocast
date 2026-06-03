//! Pure-Rust client for the cie_thread_configurator non-ROS thread protocol.
//!
//! Announces a thread's TID and logical name to the configurator daemon over an
//! abstract Unix datagram socket so the daemon can manage its scheduling. Rust
//! counterpart of C++ `spawn_non_ros2_thread` / `send_non_ros_thread_info`.
//!
//! **Linux only.** Uses abstract Unix domain sockets and `SYS_gettid`, both
//! Linux-specific; building on other targets fails at compile time.

#[cfg(not(target_os = "linux"))]
compile_error!(
    "agnocast_cie_thread_configurator_client is Linux-only (uses abstract Unix sockets and SYS_gettid)"
);

mod wire;

use std::os::linux::net::SocketAddrExt;
use std::os::unix::net::{SocketAddr, UnixDatagram};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const TOTAL_TIMEOUT: Duration = Duration::from_secs(1);
const RETRY_INTERVAL: Duration = Duration::from_millis(50);

/// Spawn a thread whose scheduling policy can be managed through
/// cie_thread_configurator. `thread_name` must be unique among managed threads.
///
/// The announcement is best-effort: IPC failures are logged via `log::warn!`
/// and never propagated, so `f` always runs even if the daemon is unreachable.
pub fn spawn_non_ros2_thread<F, T>(thread_name: &str, f: F) -> JoinHandle<T>
where
    F: FnOnce() -> T + Send + 'static,
    T: Send + 'static,
{
    let name = thread_name.to_owned();
    std::thread::spawn(move || {
        report_current_thread(&name);
        f()
    })
}

/// Announce the calling thread to cie_thread_configurator. Use this for threads
/// not created via `spawn_non_ros2_thread` (e.g. the main thread).
///
/// `thread_name` must be unique among all threads managed by
/// cie_thread_configurator (same constraint as `spawn_non_ros2_thread`).
///
/// Best-effort: IPC failures are logged via `log::warn!` and never propagated,
/// so the caller always proceeds.
pub fn report_current_thread(thread_name: &str) {
    let tid = current_tid();
    let buf = match wire::encode(tid, thread_name.as_bytes()) {
        Some(buf) => buf,
        None => {
            log::warn!(
                "thread_name too long ({} bytes); skipping send",
                thread_name.len()
            );
            return;
        }
    };
    send_with_retry(&buf, thread_name);
}

fn current_tid() -> i64 {
    // std exposes no TID; gettid via raw syscall matches the C++ side.
    // SAFETY: SYS_gettid takes no arguments and cannot fail.
    unsafe { libc::syscall(libc::SYS_gettid) as i64 }
}

fn send_with_retry(buf: &[u8], thread_name: &str) {
    let addr = match SocketAddr::from_abstract_name(wire::SOCKET_NAME) {
        Ok(addr) => addr,
        Err(e) => {
            log::warn!("failed to build abstract socket address: {e}");
            return;
        }
    };
    let sock = match UnixDatagram::unbound() {
        Ok(sock) => sock,
        Err(e) => {
            log::warn!("socket() failed: {e}");
            return;
        }
    };

    let deadline = Instant::now() + TOTAL_TIMEOUT;
    loop {
        match sock.send_to_addr(buf, &addr) {
            Ok(_) => return,
            Err(e) if is_retryable(&e) => {
                if Instant::now() >= deadline {
                    log::warn!(
                        "No listener for non_ros_thread_info; thread '{thread_name}' will not be configured."
                    );
                    return;
                }
                std::thread::sleep(RETRY_INTERVAL);
            }
            Err(e) => {
                log::warn!("sendto() failed: {e}");
                return;
            }
        }
    }
}

fn is_retryable(e: &std::io::Error) -> bool {
    use std::io::ErrorKind;
    // ConnectionRefused (ECONNREFUSED): daemon not yet bound. WouldBlock
    // (EAGAIN/EWOULDBLOCK) and ENOBUFS: transient AUDS queue pressure.
    // Interrupted (EINTR): signal during send. All retryable until the deadline.
    matches!(
        e.kind(),
        ErrorKind::ConnectionRefused | ErrorKind::WouldBlock | ErrorKind::Interrupted
    ) || e.raw_os_error() == Some(libc::ENOBUFS)
}
