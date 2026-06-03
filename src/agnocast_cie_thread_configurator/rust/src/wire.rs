//! Wire format for non-ROS thread announcements.
//!
//! Layout (native endian, single-host AUDS): [i64 tid][u16 name_len][name bytes].
//! Must stay byte-identical to the C++ encoder in non_ros_thread_ipc.hpp; the
//! golden-vector test below is asserted on both sides as a drift guard.

/// Abstract socket name (without the leading NUL; std adds the abstract prefix).
pub const SOCKET_NAME: &str = "agnocast_cie_thread_configurator/non_ros_thread_info";

// Mirrors the C++ `static_assert` in non_ros_thread_ipc.hpp: the abstract path
// (leading NUL byte + name bytes) must fit in `sockaddr_un.sun_path`
// (108 on Linux), so the name length budget is 107.
const _: () = assert!(
    SOCKET_NAME.len() < 108,
    "abstract socket path too long for sockaddr_un"
);

const TID_SIZE: usize = 8;
const NAMELEN_SIZE: usize = 2;
pub const HEADER_SIZE: usize = TID_SIZE + NAMELEN_SIZE;
pub const MAX_NAME_LEN: usize = u16::MAX as usize;

/// Encode a thread announcement. Returns `None` if the name exceeds
/// `MAX_NAME_LEN` (the C++ encoder refuses oversize names rather than truncate).
pub fn encode(tid: i64, name: &[u8]) -> Option<Vec<u8>> {
    if name.len() > MAX_NAME_LEN {
        return None;
    }
    let name_len = name.len() as u16;
    let mut buf = Vec::with_capacity(HEADER_SIZE + name.len());
    buf.extend_from_slice(&tid.to_ne_bytes());
    buf.extend_from_slice(&name_len.to_ne_bytes());
    buf.extend_from_slice(name);
    Some(buf)
}

/// Decode a thread announcement. Returns `None` if the buffer is shorter than
/// the header or its length disagrees with the declared name length. Test-only:
/// production code only encodes.
#[cfg(test)]
pub fn decode(buf: &[u8]) -> Option<(i64, Vec<u8>)> {
    if buf.len() < HEADER_SIZE {
        return None;
    }
    let tid = i64::from_ne_bytes(buf[0..TID_SIZE].try_into().unwrap());
    let name_len = u16::from_ne_bytes(buf[TID_SIZE..HEADER_SIZE].try_into().unwrap()) as usize;
    if buf.len() != HEADER_SIZE + name_len {
        return None;
    }
    Some((tid, buf[HEADER_SIZE..].to_vec()))
}

#[cfg(test)]
mod tests {
    use super::*;

    // Golden vector shared with the C++ test (test_non_ros_thread_ipc.cpp,
    // GoldenVectorMatchesRust). Pins the exact bytes so either side drifting
    // fails CI. Native-endian wire format, so little-endian hosts only.
    #[test]
    #[cfg(target_endian = "little")]
    fn golden_vector_le() {
        let buf = encode(123456789, b"worker_thread").unwrap();
        let expected: [u8; 23] = [
            0x15, 0xCD, 0x5B, 0x07, 0x00, 0x00, 0x00, 0x00, // tid = 123456789 (i64 LE)
            0x0D, 0x00, // name_len = 13 (u16 LE)
            b'w', b'o', b'r', b'k', b'e', b'r', b'_', b't', b'h', b'r', b'e', b'a', b'd',
        ];
        assert_eq!(buf, expected);
    }

    #[test]
    fn rejects_oversize_name() {
        let name = vec![b'x'; MAX_NAME_LEN + 1];
        assert!(encode(0, &name).is_none());
    }

    #[test]
    fn round_trip_ascii() {
        let buf = encode(123456789, b"worker_thread").unwrap();
        let (tid, name) = decode(&buf).unwrap();
        assert_eq!(tid, 123456789);
        assert_eq!(name, b"worker_thread");
    }

    #[test]
    fn round_trip_max_len() {
        let name = vec![b'x'; MAX_NAME_LEN];
        let buf = encode(-1, &name).unwrap();
        assert_eq!(buf.len(), HEADER_SIZE + MAX_NAME_LEN);
        let (tid, decoded) = decode(&buf).unwrap();
        assert_eq!(tid, -1);
        assert_eq!(decoded, name);
    }

    #[test]
    fn decode_rejects_short_buffer() {
        assert!(decode(&[0u8; 5]).is_none());
    }

    #[test]
    fn decode_rejects_length_mismatch() {
        let buf = encode(7, b"abc").unwrap();
        assert!(decode(&buf[..buf.len() - 1]).is_none());
    }
}
