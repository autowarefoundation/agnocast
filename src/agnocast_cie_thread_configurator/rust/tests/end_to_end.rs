use std::os::linux::net::SocketAddrExt;
use std::os::unix::net::{SocketAddr, UnixDatagram};
use std::time::Duration;

use agnocast_cie_thread_configurator_client::{report_current_thread, spawn_non_ros2_thread};

const SOCKET_NAME: &str = "agnocast_cie_thread_configurator/non_ros_thread_info";

// Bind the well-known abstract name to receive announcements. Returns None only
// when the name is already bound (e.g. a real daemon is running) so the test can
// skip; any other bind/address error fails the test rather than masking it.
fn bind_listener_or_skip() -> Option<UnixDatagram> {
    let addr = SocketAddr::from_abstract_name(SOCKET_NAME).expect("build abstract socket address");
    match UnixDatagram::bind_addr(&addr) {
        Ok(sock) => Some(sock),
        Err(e) if e.kind() == std::io::ErrorKind::AddrInUse => None,
        Err(e) => panic!("failed to bind abstract socket: {e}"),
    }
}

fn decode(buf: &[u8]) -> (i64, Vec<u8>) {
    assert!(
        buf.len() >= 10,
        "datagram too short for header: {} bytes",
        buf.len()
    );
    let tid = i64::from_ne_bytes(buf[0..8].try_into().unwrap());
    let name_len = u16::from_ne_bytes(buf[8..10].try_into().unwrap()) as usize;
    assert_eq!(
        buf.len(),
        10 + name_len,
        "datagram length disagrees with declared name_len {name_len}"
    );
    (tid, buf[10..10 + name_len].to_vec())
}

// One combined test: there is a single well-known abstract name, so binding it
// in two parallel tests would let one steal the other's datagrams. Exercising
// both entry points through one bound listener avoids that race.
#[test]
fn delivers_announcements_and_runs_closure() {
    let Some(listener) = bind_listener_or_skip() else {
        eprintln!("skipping: abstract name already bound (daemon running?)");
        return;
    };
    listener
        .set_read_timeout(Some(Duration::from_secs(2)))
        .unwrap();

    report_current_thread("rs_test_main");

    let handle = spawn_non_ros2_thread("rs_test_worker", || 21 * 2);
    assert_eq!(handle.join().unwrap(), 42);

    let mut names = Vec::new();
    for _ in 0..2 {
        let mut buf = [0u8; 10 + 65535];
        let n = listener.recv(&mut buf).expect("recv announcement");
        let (tid, name) = decode(&buf[..n]);
        assert!(tid > 0);
        names.push(String::from_utf8(name).unwrap());
    }
    names.sort();
    assert_eq!(
        names,
        vec!["rs_test_main".to_string(), "rs_test_worker".to_string()]
    );
}
