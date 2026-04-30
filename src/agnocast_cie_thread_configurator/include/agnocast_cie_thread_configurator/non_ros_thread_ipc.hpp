#pragma once

#include "rclcpp/logger.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <thread>

namespace agnocast_cie_thread_configurator
{

constexpr const char k_non_ros_thread_info_socket_path[] =
  "agnocast_cie_thread_configurator/non_ros_thread_info";

static_assert(
  offsetof(sockaddr_un, sun_path) + 1u + (sizeof(k_non_ros_thread_info_socket_path) - 1u) <=
    sizeof(sockaddr_un),
  "abstract socket path too long for sockaddr_un");

// Wire format: [int64 tid][uint16 name_len][name_len bytes of name].
// Host byte order — single-host AUDS, no endianness concern.
struct NonRosThreadInfo
{
  int64_t tid = 0;
  std::string name;
};

constexpr size_t k_non_ros_thread_info_max_name_len = 65535u;
constexpr size_t k_non_ros_thread_info_max_wire_size = 10u + k_non_ros_thread_info_max_name_len;

inline socklen_t setup_non_ros_thread_info_sockaddr(sockaddr_un & addr)
{
  std::memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  addr.sun_path[0] = '\0';
  constexpr size_t path_len = sizeof(k_non_ros_thread_info_socket_path) - 1u;
  std::memcpy(addr.sun_path + 1, k_non_ros_thread_info_socket_path, path_len);
  return static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1u + path_len);
}

inline bool encode_non_ros_thread_info(
  const NonRosThreadInfo & info, uint8_t * buf, size_t buf_cap, size_t & out_len)
{
  static_assert(sizeof(NonRosThreadInfo::tid) == 8, "wire format assumes 8-byte tid");
  if (info.name.size() > k_non_ros_thread_info_max_name_len) {
    return false;
  }
  const uint16_t name_len = static_cast<uint16_t>(info.name.size());
  out_len = 10u + name_len;
  if (out_len > buf_cap) {
    return false;
  }
  std::memcpy(buf, &info.tid, sizeof(info.tid));
  std::memcpy(buf + 8, &name_len, sizeof(name_len));
  if (name_len > 0) {
    std::memcpy(buf + 10, info.name.data(), name_len);
  }
  return true;
}

inline bool decode_non_ros_thread_info(const uint8_t * buf, size_t len, NonRosThreadInfo & info)
{
  if (len < 10u) {
    return false;
  }
  uint16_t name_len = 0;
  std::memcpy(&info.tid, buf, sizeof(info.tid));
  std::memcpy(&name_len, buf + 8, sizeof(name_len));
  if (len != 10u + static_cast<size_t>(name_len)) {
    return false;
  }
  info.name.assign(reinterpret_cast<const char *>(buf + 10), name_len);
  return true;
}

inline void send_non_ros_thread_info(
  const NonRosThreadInfo & info, std::chrono::milliseconds total_timeout = std::chrono::seconds(1),
  std::chrono::milliseconds retry_interval = std::chrono::milliseconds(50))
{
  std::array<uint8_t, k_non_ros_thread_info_max_wire_size> buf;
  size_t buf_len = 0;
  if (!encode_non_ros_thread_info(info, buf.data(), k_non_ros_thread_info_max_wire_size, buf_len)) {
    std::fprintf(
      stderr,
      "[agnocast_cie_thread_configurator][WARN] thread_name too long (%zu bytes); "
      "skipping send\n",
      info.name.size());
    return;
  }

  int fd = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    std::fprintf(
      stderr, "[agnocast_cie_thread_configurator][WARN] socket() failed: errno=%d\n", errno);
    return;
  }

  sockaddr_un addr{};
  socklen_t addr_len = setup_non_ros_thread_info_sockaddr(addr);

  const auto deadline = std::chrono::steady_clock::now() + total_timeout;
  bool sent = false;
  while (true) {
    ssize_t n = ::sendto(fd, buf.data(), buf_len, 0, reinterpret_cast<sockaddr *>(&addr), addr_len);
    if (n >= 0) {
      sent = true;
      break;
    }
    // ECONNREFUSED: no listener bound on the abstract path yet (configurator
    // not yet up). EAGAIN/EWOULDBLOCK/ENOBUFS: transient kernel resource
    // pressure on the AUDS receive queue. EINTR: signal during sendto.
    // All are retryable until the deadline.
    if (
      errno == ECONNREFUSED || errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS ||
      errno == EINTR) {
      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(retry_interval);
      continue;
    }
    std::fprintf(
      stderr, "[agnocast_cie_thread_configurator][WARN] sendto() failed: errno=%d\n", errno);
    break;
  }

  if (!sent) {
    std::fprintf(
      stderr,
      "[agnocast_cie_thread_configurator][WARN] No listener for non_ros_thread_info; "
      "thread '%s' will not be configured.\n",
      info.name.c_str());
  }
  ::close(fd);
}

/// Receives non-ROS thread announcements over an abstract Unix datagram
/// socket and dispatches each to the user callback on a private reader
/// thread.
///
/// At most one NonRosThreadInfoListener may exist per host: the bound
/// abstract socket name is a fixed compile-time constant
/// (`k_non_ros_thread_info_socket_path`), so a second concurrent
/// constructor on the same host throws `std::system_error` (EADDRINUSE).
///
/// Lifetime: the reader thread is started in the constructor and joined in
/// `stop()` / destructor. `stop()` is idempotent when called from a single
/// thread.
class NonRosThreadInfoListener
{
public:
  using Callback = std::function<void(NonRosThreadInfo /*info*/)>;

  NonRosThreadInfoListener(Callback callback, rclcpp::Logger logger);
  ~NonRosThreadInfoListener() noexcept;

  void stop() noexcept;

  NonRosThreadInfoListener(const NonRosThreadInfoListener &) = delete;
  NonRosThreadInfoListener & operator=(const NonRosThreadInfoListener &) = delete;
  NonRosThreadInfoListener(NonRosThreadInfoListener &&) = delete;
  NonRosThreadInfoListener & operator=(NonRosThreadInfoListener &&) = delete;

private:
  void run();

  Callback callback_;
  rclcpp::Logger logger_;
  int listener_fd_ = -1;
  int stop_eventfd_ = -1;
  int epfd_ = -1;
  std::thread thread_;
  std::atomic<bool> stopped_{false};
};

}  // namespace agnocast_cie_thread_configurator
