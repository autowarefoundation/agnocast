// Blocking framed message I/O over a connected stream socket, plus a poll-based
// readability wait. Kept separate from the server so it can be unit tested over a
// socketpair without any accept loop.
#pragma once

#include "agnocast_gpu_shared_memory_daemon/protocol.hpp"

#include <cstdint>
#include <vector>

namespace agnocast::gpu_shared_memory_daemon
{

// Upper bound on an accepted payload, so a corrupt or hostile peer cannot make us
// allocate an enormous buffer. Comfortably larger than any real ListResponse.
constexpr std::uint32_t kMaxPayloadSize = 64u * 1024u * 1024u;  // 64 MiB

// Reads exactly one framed message (header + payload) from `fd`. Returns false on
// EOF, socket error, an invalid header (bad magic/version), or a payload larger
// than kMaxPayloadSize. Partial reads are handled internally.
bool read_message(int fd, MessageHeader & header, std::vector<std::uint8_t> & payload);

// Frames and writes one message. Returns false on socket error (including a peer
// that closed the connection). Never raises SIGPIPE.
bool write_message(int fd, MessageType type, const std::vector<std::uint8_t> & payload);

// Waits up to timeout_ms for `fd` to become readable.
//   >0 : readable
//    0 : timed out (or interrupted by a signal)
//   -1 : error
int wait_readable(int fd, int timeout_ms);

}  // namespace agnocast::gpu_shared_memory_daemon
