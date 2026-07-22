#include "socket_io.hpp"

#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>

namespace agnocast::gpu_shared_memory_daemon
{

namespace
{

// Reads exactly n bytes, retrying on partial reads and EINTR. Returns false on
// EOF or error.
bool read_exact(int fd, void * buf, std::size_t n)
{
  auto * p = static_cast<std::uint8_t *>(buf);
  std::size_t got = 0;
  while (got < n) {
    const ssize_t r = ::read(fd, p + got, n - got);
    if (r == 0) {
      return false;  // peer closed
    }
    if (r < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    got += static_cast<std::size_t>(r);
  }
  return true;
}

// Writes exactly n bytes. MSG_NOSIGNAL suppresses SIGPIPE when the peer has gone.
bool write_all(int fd, const void * buf, std::size_t n)
{
  const auto * p = static_cast<const std::uint8_t *>(buf);
  std::size_t sent = 0;
  while (sent < n) {
    const ssize_t r = ::send(fd, p + sent, n - sent, MSG_NOSIGNAL);
    if (r < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    sent += static_cast<std::size_t>(r);
  }
  return true;
}

}  // namespace

bool read_message(int fd, MessageHeader & header, std::vector<std::uint8_t> & payload)
{
  std::array<std::uint8_t, kHeaderWireSize> header_bytes{};
  if (!read_exact(fd, header_bytes.data(), header_bytes.size())) {
    return false;
  }
  if (!deserialize_header(header_bytes.data(), header_bytes.size(), &header)) {
    return false;
  }
  if (!header_is_valid(header)) {
    return false;
  }
  if (header.payload_size > kMaxPayloadSize) {
    return false;
  }

  payload.resize(header.payload_size);
  if (header.payload_size > 0 && !read_exact(fd, payload.data(), payload.size())) {
    return false;
  }
  return true;
}

bool write_message(int fd, MessageType type, const std::vector<std::uint8_t> & payload)
{
  const auto frame = frame_message(type, payload);
  return write_all(fd, frame.data(), frame.size());
}

int wait_readable(int fd, int timeout_ms)
{
  struct pollfd pfd;
  pfd.fd = fd;
  pfd.events = POLLIN;
  pfd.revents = 0;
  const int r = ::poll(&pfd, 1, timeout_ms);
  if (r < 0) {
    return (errno == EINTR) ? 0 : -1;
  }
  return r;
}

}  // namespace agnocast::gpu_shared_memory_daemon
