#pragma once

#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_utils.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <fcntl.h>
#include <mqueue.h>
#include <sys/epoll.h>
#include <sys/signalfd.h>
#include <sys/stat.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <functional>
#include <initializer_list>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace agnocast
{

class IpcEventLoopBase
{
public:
  using EventCallback = std::function<void(int)>;
  using SignalCallback = std::function<void()>;

  IpcEventLoopBase(
    const rclcpp::Logger & logger, const std::string & mq_name, long mq_msg_size,
    const std::vector<int> & signals_to_block, const std::vector<int> & signals_to_ignore);

  virtual ~IpcEventLoopBase();

  IpcEventLoopBase(const IpcEventLoopBase &) = delete;
  IpcEventLoopBase & operator=(const IpcEventLoopBase &) = delete;

  bool spin_once(int timeout_ms);

  void set_mq_handler(EventCallback cb);
  void set_signal_handler(SignalCallback cb);

protected:
  rclcpp::Logger logger_;
  virtual void handle_signal();

private:
  int epoll_fd_ = -1;
  int signal_fd_ = -1;

  mqd_t mq_fd_ = (mqd_t)-1;
  std::string mq_name_;

  long mq_msg_size_;

  EventCallback mq_cb_;
  SignalCallback signal_cb_;

  void setup_mq();
  void setup_signals(
    const std::vector<int> & signals_to_block, const std::vector<int> & signals_to_ignore);
  void setup_epoll();
  void cleanup_resources();

  mqd_t create_and_open_mq(const std::string & name) const;
  void add_fd_to_epoll(int fd, const std::string & label) const;

  static void ignore_signals_impl(const std::vector<int> & signals);
  static sigset_t block_signals_impl(const std::vector<int> & signals);
};

inline IpcEventLoopBase::IpcEventLoopBase(
  const rclcpp::Logger & logger, const std::string & mq_name, long mq_msg_size,
  const std::vector<int> & signals_to_block, const std::vector<int> & signals_to_ignore)
: logger_(logger), mq_name_(mq_name), mq_msg_size_(mq_msg_size)
{
  try {
    setup_mq();
    setup_signals(signals_to_block, signals_to_ignore);
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
    if (fd == mq_fd_) {
      if (mq_cb_) {
        mq_cb_(fd);
      }
    } else if (fd == signal_fd_) {
      struct signalfd_siginfo fdsi
      {
      };
      ssize_t s = read(signal_fd_, &fdsi, sizeof(struct signalfd_siginfo));
      if (s == sizeof(struct signalfd_siginfo)) {
        handle_signal();
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

inline void IpcEventLoopBase::set_mq_handler(EventCallback cb)
{
  mq_cb_ = std::move(cb);
}

inline void IpcEventLoopBase::set_signal_handler(SignalCallback cb)
{
  signal_cb_ = std::move(cb);
}

inline void IpcEventLoopBase::setup_mq()
{
  mq_fd_ = create_and_open_mq(mq_name_);
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

inline void IpcEventLoopBase::setup_epoll()
{
  epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
  if (epoll_fd_ == -1) {
    throw std::runtime_error("epoll_create1 failed: " + std::string(strerror(errno)));
  }

  add_fd_to_epoll(mq_fd_, "MQ");
  add_fd_to_epoll(signal_fd_, "Signal");
}

inline mqd_t IpcEventLoopBase::create_and_open_mq(const std::string & name) const
{
  struct mq_attr attr = {};
  int64_t max_messages = BRIDGE_MQ_MAX_MESSAGES;
  if (get_bridge_mode() == BridgeMode::Performance) {
    max_messages = PERFORMANCE_BRIDGE_MQ_MAX_MESSAGES;
  }
  attr.mq_maxmsg = max_messages;
  attr.mq_msgsize = mq_msg_size_;

  mqd_t fd =
    mq_open(name.c_str(), O_CREAT | O_RDONLY | O_NONBLOCK | O_CLOEXEC, BRIDGE_MQ_PERMS, &attr);

  if (fd == -1) {
    throw std::system_error(errno, std::generic_category(), "MQ open failed: " + name);
  }

  return fd;
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

  if (signal_fd_ != -1) {
    if (close(signal_fd_) == -1) {
      RCLCPP_WARN(logger_, "Failed to close signal_fd: %s", strerror(errno));
    }
    signal_fd_ = -1;
  }

  if (mq_fd_ != -1) {
    if (mq_close(mq_fd_) == -1) {
      RCLCPP_WARN(logger_, "Failed to close mq_fd: %s", strerror(errno));
    }
    mq_fd_ = -1;
  }

  mq_name_.clear();
}

}  // namespace agnocast
