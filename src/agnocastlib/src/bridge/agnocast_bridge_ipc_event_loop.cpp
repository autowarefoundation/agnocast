#include "agnocast/bridge/agnocast_bridge_ipc_event_loop.hpp"

#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_utils.hpp"

#include <rclcpp/logging.hpp>

#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/signalfd.h>
#include <sys/stat.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

namespace agnocast
{

BridgeIpcEventLoop::BridgeIpcEventLoop(const rclcpp::Logger & logger) : logger_(logger)
{
  try {
    setup_mq();
    setup_signals();
    setup_epoll();
  } catch (...) {
    cleanup_resources();
    throw;
  }
}

BridgeIpcEventLoop::~BridgeIpcEventLoop()
{
  cleanup_resources();
}

bool BridgeIpcEventLoop::spin_once(int timeout_ms)
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
        if ((fdsi.ssi_signo == SIGTERM || fdsi.ssi_signo == SIGINT) && signal_cb_) {
          signal_cb_();
        }
      }
    }
  }
  return true;
}

void BridgeIpcEventLoop::set_mq_handler(EventCallback cb)
{
  mq_cb_ = std::move(cb);
}

void BridgeIpcEventLoop::set_signal_handler(SignalCallback cb)
{
  signal_cb_ = std::move(cb);
}

void BridgeIpcEventLoop::ignore_signals(std::initializer_list<int> signals)
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

sigset_t BridgeIpcEventLoop::block_signals(std::initializer_list<int> signals)
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

void BridgeIpcEventLoop::setup_mq()
{
  mq_name_ = create_mq_name_for_bridge(getpid());
  mq_fd_ = create_and_open_mq(mq_name_);
}

void BridgeIpcEventLoop::setup_signals()
{
  ignore_signals({SIGPIPE, SIGHUP});
  sigset_t mask = block_signals({SIGTERM, SIGINT});

  signal_fd_ = signalfd(-1, &mask, SFD_NONBLOCK | SFD_CLOEXEC);
  if (signal_fd_ == -1) {
    throw std::system_error(errno, std::generic_category(), "signalfd failed");
  }
}

void BridgeIpcEventLoop::setup_epoll()
{
  epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
  if (epoll_fd_ == -1) {
    throw std::runtime_error("epoll_create1 failed: " + std::string(strerror(errno)));
  }

  add_fd_to_epoll(mq_fd_, "MQ");
  add_fd_to_epoll(signal_fd_, "Signal");
}

mqd_t BridgeIpcEventLoop::create_and_open_mq(const std::string & name)
{
  struct mq_attr attr = {};
  attr.mq_maxmsg = BRIDGE_MQ_MAX_MESSAGES;
  attr.mq_msgsize = BRIDGE_MQ_MESSAGE_SIZE;

  mqd_t fd =
    mq_open(name.c_str(), O_CREAT | O_RDONLY | O_NONBLOCK | O_CLOEXEC, BRIDGE_MQ_PERMS, &attr);

  if (fd == -1) {
    throw std::system_error(errno, std::generic_category(), "MQ open failed");
  }

  return fd;
}

void BridgeIpcEventLoop::add_fd_to_epoll(int fd, const std::string & label) const
{
  struct epoll_event ev = {};
  ev.events = EPOLLIN;
  ev.data.fd = fd;

  if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &ev) == -1) {
    throw std::runtime_error("epoll_ctl (" + label + ") failed: " + std::string(strerror(errno)));
  }
}

void BridgeIpcEventLoop::cleanup_resources()
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
      RCLCPP_WARN(logger_, "Failed to close mq_peer_fd: %s", strerror(errno));
    }
    mq_fd_ = -1;
  }

  mq_name_.clear();
}

}  // namespace agnocast
