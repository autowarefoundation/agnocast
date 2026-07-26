#pragma once

#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_uds.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/signalfd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace agnocast
{

class IpcEventLoopBase
{
public:
  using MessageCallback = std::function<void(const void * data, std::size_t size)>;
  using SignalCallback = std::function<void()>;
  using SocketCallback = std::function<std::string()>;

  IpcEventLoopBase(
    const rclcpp::Logger & logger, const std::string & uds_addr, std::size_t max_msg_size,
    const std::vector<int> & signals_to_block, const std::vector<int> & signals_to_ignore);

  virtual ~IpcEventLoopBase();

  IpcEventLoopBase(const IpcEventLoopBase &) = delete;
  IpcEventLoopBase & operator=(const IpcEventLoopBase &) = delete;

  bool spin_once(int timeout_ms);

  void set_message_handler(MessageCallback cb);
  void set_signal_handler(SignalCallback cb);
  void set_socket_handler(SocketCallback cb);

  const std::string & get_uds_addr() const { return uds_addr_; }

protected:
  rclcpp::Logger logger_;
  virtual void handle_signal();
  virtual void handle_socket(int client_fd);

private:
  int epoll_fd_ = -1;
  int signal_fd_ = -1;
  int socket_fd_ = -1;

  int listener_fd_ = -1;
  std::string uds_addr_;
  std::vector<uint8_t> recv_buf_;

  MessageCallback message_cb_;
  SignalCallback signal_cb_;
  SocketCallback socket_cb_;

  void setup_listener();
  void setup_signals(
    const std::vector<int> & signals_to_block, const std::vector<int> & signals_to_ignore);
  void setup_socket();
  void setup_epoll();
  void cleanup_resources();

  void add_fd_to_epoll(int fd, const std::string & label) const;
  void drain_listener();

  static void ignore_signals_impl(const std::vector<int> & signals);
  static sigset_t block_signals_impl(const std::vector<int> & signals);
};

inline IpcEventLoopBase::IpcEventLoopBase(
  const rclcpp::Logger & logger, const std::string & uds_addr, std::size_t max_msg_size,
  const std::vector<int> & signals_to_block, const std::vector<int> & signals_to_ignore)
: logger_(logger), uds_addr_(uds_addr), recv_buf_(max_msg_size)
{
  try {
    setup_listener();
    setup_signals(signals_to_block, signals_to_ignore);
    setup_socket();
    setup_epoll();
  } catch (...) {
    cleanup_resources();
    throw;
  }
}

inline IpcEventLoopBase::~IpcEventLoopBase()
{
  cleanup_resources();
}

inline bool IpcEventLoopBase::spin_once(int timeout_ms)
{
  constexpr int MAX_EVENTS = 10;
  std::array<struct epoll_event, MAX_EVENTS> events{};

  int event_count = -1;
  do {
    event_count = epoll_wait(epoll_fd_, events.data(), MAX_EVENTS, timeout_ms);
  } while (event_count < 0 && errno == EINTR);
  if (event_count < 0) {
    RCLCPP_ERROR(logger_, "epoll_wait failed: %s", strerror(errno));
    return false;
  }
  if (event_count == 0) {
    return true;
  }
  for (int event_index = 0; event_index < event_count; ++event_index) {
    int fd = events[event_index].data.fd;
    if (fd == listener_fd_) {
      drain_listener();
    } else if (fd == signal_fd_) {
      struct signalfd_siginfo fdsi
      {
      };
      ssize_t s = read(signal_fd_, &fdsi, sizeof(struct signalfd_siginfo));
      if (s == sizeof(struct signalfd_siginfo)) {
        handle_signal();
      }
    } else if (fd == socket_fd_) {
      int client_fd = accept4(socket_fd_, nullptr, nullptr, SOCK_CLOEXEC | SOCK_NONBLOCK);
      if (client_fd == -1) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN(logger_, "accept4 on debug socket failed: %s", strerror(errno));
        }
      } else {
        handle_socket(client_fd);
        close(client_fd);
      }
    }
  }
  return true;
}

inline void IpcEventLoopBase::handle_signal()
{
  if (signal_cb_) {
    signal_cb_();
  }
}

inline void IpcEventLoopBase::handle_socket(int client_fd)
{
  if (!socket_cb_) {
    return;
  }

  const std::string response = socket_cb_();
  if (response.empty()) {
    return;
  }

  // This socket is used for debug purposes only and must not block the main IpcEventLoop
  // processing. The client fd is non-blocking (SOCK_NONBLOCK), so send() may return EAGAIN if the
  // send buffer is temporarily full. In that case, we retry up to MAX_RETRIES consecutive times. If
  // partial progress is made between retries, the retry counter is reset so that only consecutive
  // EAGAIN failures count toward the limit. EINTR is retried transparently and does not increment
  // the retry counter. For any other error (e.g. EPIPE), or when the retry limit is reached, a
  // warning is logged and the send is abandoned without affecting the main processing.
  constexpr int MAX_RETRIES = 3;
  size_t sent = 0;
  int retries = 0;

  while (sent < response.size() && retries < MAX_RETRIES) {
    ssize_t n = send(client_fd, response.data() + sent, response.size() - sent, MSG_NOSIGNAL);
    if (n > 0) {
      sent += static_cast<size_t>(n);
      retries = 0;  // reset on progress; retries counts only consecutive EAGAIN failures
    } else if (n == -1) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        ++retries;
      } else if (errno == EINTR) {
        // EINTR is harmless; retry without counting it as a failure
      } else {
        RCLCPP_WARN(logger_, "send on debug socket failed: %s", strerror(errno));
        return;
      }
    }
  }

  if (sent < response.size()) {
    RCLCPP_WARN(
      logger_, "send on debug socket incomplete after retries: sent %zu of %zu bytes", sent,
      response.size());
  }
}

inline void IpcEventLoopBase::set_message_handler(MessageCallback cb)
{
  message_cb_ = std::move(cb);
}

inline void IpcEventLoopBase::set_signal_handler(SignalCallback cb)
{
  signal_cb_ = std::move(cb);
}

inline void IpcEventLoopBase::set_socket_handler(SocketCallback cb)
{
  socket_cb_ = std::move(cb);
}

inline void IpcEventLoopBase::drain_listener()
{
  while (true) {
    ssize_t n = recv(listener_fd_, recv_buf_.data(), recv_buf_.size(), 0);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) break;
      if (errno == EINTR) continue;
      RCLCPP_WARN(logger_, "bridge UDS recv() failed: %s", strerror(errno));
      break;
    }
    if (message_cb_) {
      message_cb_(recv_buf_.data(), static_cast<size_t>(n));
    }
  }
}

inline void IpcEventLoopBase::setup_listener()
{
  listener_fd_ = create_bridge_uds_listener(uds_addr_);
}

inline void IpcEventLoopBase::setup_signals(
  const std::vector<int> & signals_to_block, const std::vector<int> & signals_to_ignore)
{
  ignore_signals_impl(signals_to_ignore);
  sigset_t mask = block_signals_impl(signals_to_block);

  signal_fd_ = signalfd(-1, &mask, SFD_NONBLOCK | SFD_CLOEXEC);
  if (signal_fd_ == -1) {
    throw std::system_error(errno, std::generic_category(), "signalfd failed");
  }
}

// Sets up a Unix domain socket in the abstract namespace for debug use.
//
// Communication is one-way (Bridge process -> CLI): when a client connects, the
// string returned by socket_cb_ is immediately sent to the client and the
// connection is closed. The server never reads from the client; the act of
// connecting is itself the trigger.
//
// Currently this socket is used for liveness checks: the CLI connects to
// confirm the Bridge process is running and receives a status string in
// response. Because no request payload is expected, recv()/read() is
// intentionally absent from handle_socket().
//
// The abstract namespace (sun_path[0] == '\0') is used instead of filesystem
// paths so that the socket is automatically released by the OS when the process
// terminates, eliminating stale socket files left by abnormal process exits.
// Abstract socket names are scoped to the network namespace.
//
// Name format: "\0agnocast_bridge/{ipc_ns_inode}/{pid}"
inline void IpcEventLoopBase::setup_socket()
{
  const std::string name =
    "agnocast_bridge/" + std::to_string(get_self_ipc_ns_inode()) + "/" + std::to_string(getpid());

  int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd == -1) {
    RCLCPP_WARN(logger_, "setup_socket: socket() failed: %s (socket disabled)", strerror(errno));
    return;
  }

  struct sockaddr_un addr
  {
  };
  addr.sun_family = AF_UNIX;
  // Abstract namespace: sun_path[0] == '\0', name follows without null terminator.
  // addrlen must cover exactly the '\0' prefix + name bytes (no null terminator).
  if (name.size() + 1 > sizeof(addr.sun_path)) {
    RCLCPP_WARN(
      logger_, "setup_socket: abstract name too long: '%s' (socket disabled)", name.c_str());
    close(fd);
    return;
  }
  addr.sun_path[0] = '\0';
  memcpy(addr.sun_path + 1, name.c_str(), name.size());
  const socklen_t addrlen =
    static_cast<socklen_t>(offsetof(struct sockaddr_un, sun_path) + 1 + name.size());

  if (bind(fd, reinterpret_cast<struct sockaddr *>(&addr), addrlen) == -1) {
    RCLCPP_WARN(logger_, "setup_socket: bind() failed: %s (socket disabled)", strerror(errno));
    close(fd);
    return;
  }

  if (listen(fd, 4) == -1) {
    RCLCPP_WARN(logger_, "setup_socket: listen() failed: %s (socket disabled)", strerror(errno));
    close(fd);
    return;
  }

  socket_fd_ = fd;
}

inline void IpcEventLoopBase::setup_epoll()
{
  epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
  if (epoll_fd_ == -1) {
    throw std::runtime_error("epoll_create1 failed: " + std::string(strerror(errno)));
  }

  add_fd_to_epoll(listener_fd_, "BridgeUDS");
  add_fd_to_epoll(signal_fd_, "Signal");
  if (socket_fd_ != -1) {
    add_fd_to_epoll(socket_fd_, "DebugSocket");
  }
}

inline void IpcEventLoopBase::add_fd_to_epoll(int fd, const std::string & label) const
{
  struct epoll_event ev = {};
  ev.events = EPOLLIN;
  ev.data.fd = fd;

  if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &ev) == -1) {
    throw std::runtime_error("epoll_ctl (" + label + ") failed: " + std::string(strerror(errno)));
  }
}

inline void IpcEventLoopBase::ignore_signals_impl(const std::vector<int> & signals)
{
  struct sigaction sa
  {
  };
  sa.sa_handler = SIG_IGN;
  sa.sa_flags = 0;
  sigemptyset(&sa.sa_mask);

  for (int sig : signals) {
    if (sigaction(sig, &sa, nullptr) == -1) {
      throw std::system_error(errno, std::generic_category(), "sigaction(SIG_IGN) failed");
    }
  }
}

inline sigset_t IpcEventLoopBase::block_signals_impl(const std::vector<int> & signals)
{
  sigset_t mask;
  sigemptyset(&mask);
  for (int sig : signals) {
    sigaddset(&mask, sig);
  }

  if (int err = pthread_sigmask(SIG_BLOCK, &mask, nullptr); err != 0) {
    throw std::system_error(err, std::generic_category(), "pthread_sigmask failed");
  }

  return mask;
}

inline void IpcEventLoopBase::cleanup_resources()
{
  if (epoll_fd_ != -1) {
    if (close(epoll_fd_) == -1) {
      RCLCPP_WARN(logger_, "Failed to close epoll_fd: %s", strerror(errno));
    }
    epoll_fd_ = -1;
  }

  if (socket_fd_ != -1) {
    if (close(socket_fd_) == -1) {
      RCLCPP_WARN(logger_, "Failed to close debug socket_fd: %s", strerror(errno));
    }
    socket_fd_ = -1;
  }

  if (signal_fd_ != -1) {
    if (close(signal_fd_) == -1) {
      RCLCPP_WARN(logger_, "Failed to close signal_fd: %s", strerror(errno));
    }
    signal_fd_ = -1;
  }

  if (listener_fd_ != -1) {
    if (close(listener_fd_) == -1) {
      RCLCPP_WARN(logger_, "Failed to close bridge UDS listener_fd: %s", strerror(errno));
    }
    listener_fd_ = -1;
  }
}

}  // namespace agnocast
