#include "agnocast_cie_thread_configurator/non_ros_thread_ipc.hpp"

#include "rclcpp/logging.hpp"

#include <sys/epoll.h>
#include <sys/eventfd.h>

#include <system_error>
#include <utility>

namespace agnocast_cie_thread_configurator
{

NonRosThreadInfoListener::NonRosThreadInfoListener(Callback callback, rclcpp::Logger logger)
: callback_(std::move(callback)), logger_(std::move(logger))
{
  listener_fd_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (listener_fd_ < 0) {
    throw std::system_error(errno, std::generic_category(), "socket");
  }

  sockaddr_un addr{};
  socklen_t addr_len = setup_non_ros_thread_info_sockaddr(addr);
  if (::bind(listener_fd_, reinterpret_cast<sockaddr *>(&addr), addr_len) < 0) {
    const int saved = errno;
    ::close(listener_fd_);
    listener_fd_ = -1;
    throw std::system_error(saved, std::generic_category(), "bind");
  }

  stop_eventfd_ = ::eventfd(0, EFD_CLOEXEC);
  if (stop_eventfd_ < 0) {
    const int saved = errno;
    ::close(listener_fd_);
    listener_fd_ = -1;
    throw std::system_error(saved, std::generic_category(), "eventfd");
  }

  epfd_ = ::epoll_create1(EPOLL_CLOEXEC);
  if (epfd_ < 0) {
    const int saved = errno;
    ::close(listener_fd_);
    ::close(stop_eventfd_);
    listener_fd_ = stop_eventfd_ = -1;
    throw std::system_error(saved, std::generic_category(), "epoll_create1");
  }

  epoll_event ev{};
  ev.events = EPOLLIN;
  ev.data.fd = listener_fd_;
  if (::epoll_ctl(epfd_, EPOLL_CTL_ADD, listener_fd_, &ev) < 0) {
    const int saved = errno;
    ::close(epfd_);
    ::close(listener_fd_);
    ::close(stop_eventfd_);
    epfd_ = listener_fd_ = stop_eventfd_ = -1;
    throw std::system_error(saved, std::generic_category(), "epoll_ctl listener");
  }

  ev.events = EPOLLIN;
  ev.data.fd = stop_eventfd_;
  if (::epoll_ctl(epfd_, EPOLL_CTL_ADD, stop_eventfd_, &ev) < 0) {
    const int saved = errno;
    ::close(epfd_);
    ::close(listener_fd_);
    ::close(stop_eventfd_);
    epfd_ = listener_fd_ = stop_eventfd_ = -1;
    throw std::system_error(saved, std::generic_category(), "epoll_ctl eventfd");
  }

  try {
    thread_ = std::thread(&NonRosThreadInfoListener::run, this);
  } catch (...) {
    ::close(epfd_);
    ::close(listener_fd_);
    ::close(stop_eventfd_);
    epfd_ = listener_fd_ = stop_eventfd_ = -1;
    throw;
  }
}

NonRosThreadInfoListener::~NonRosThreadInfoListener() noexcept
{
  stop();
}

void NonRosThreadInfoListener::stop() noexcept
{
  bool expected = false;
  if (!stopped_.compare_exchange_strong(expected, true)) {
    return;
  }
  if (stop_eventfd_ >= 0) {
    uint64_t one = 1;
    [[maybe_unused]] ssize_t w = ::write(stop_eventfd_, &one, sizeof(one));
  }
  if (thread_.joinable()) {
    try {
      thread_.join();
    } catch (const std::system_error &) {
      // join() can throw on EDEADLK/EINVAL/etc. The destructor must not
      // propagate exceptions, so we treat join failure as best-effort.
    }
  }
  if (epfd_ >= 0) {
    ::close(epfd_);
    epfd_ = -1;
  }
  if (listener_fd_ >= 0) {
    ::close(listener_fd_);
    listener_fd_ = -1;
  }
  if (stop_eventfd_ >= 0) {
    ::close(stop_eventfd_);
    stop_eventfd_ = -1;
  }
}

void NonRosThreadInfoListener::run()
{
  std::array<uint8_t, k_non_ros_thread_info_max_wire_size> buf;
  constexpr int k_max_events = 2;
  epoll_event events[k_max_events];

  while (true) {
    int n_ev = ::epoll_wait(epfd_, events, k_max_events, -1);
    if (n_ev < 0) {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(
        logger_,
        "epoll_wait() failed: errno=%d; reader thread exiting. "
        "Subsequent non-ROS thread announcements will not be received.",
        errno);
      return;
    }

    bool exit_loop = false;
    for (int i = 0; i < n_ev; ++i) {
      if (events[i].data.fd == stop_eventfd_) {
        exit_loop = true;
        break;
      }
      if (events[i].data.fd != listener_fd_) {
        continue;
      }
      // EPOLLERR/EPOLLHUP on the listener fd is unrecoverable. Without this
      // check, level-triggered epoll would re-fire the same error event in a
      // tight loop, busy-spinning the reader thread.
      if (events[i].events & (EPOLLERR | EPOLLHUP)) {
        RCLCPP_ERROR(
          logger_,
          "epoll signaled error/hangup on listener fd (events=0x%x); reader thread exiting. "
          "Subsequent non-ROS thread announcements will not be received.",
          events[i].events);
        exit_loop = true;
        break;
      }
      // MSG_TRUNC makes recv() return the real datagram length even when it
      // exceeds the buffer (Linux UNIX-DGRAM since 3.4), so we can detect
      // and drop oversized datagrams instead of silently feeding truncated
      // bytes to decode_non_ros_thread_info().
      ssize_t n = ::recv(listener_fd_, buf.data(), buf.size(), MSG_TRUNC);
      if (n < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_WARN(logger_, "recv() failed: errno=%d", errno);
        continue;
      }
      if (static_cast<size_t>(n) > buf.size()) {
        RCLCPP_WARN(
          logger_, "Discarded oversized NonRosThreadInfo datagram (real_size=%zd > buf=%zu)", n,
          buf.size());
        continue;
      }
      NonRosThreadInfo info;
      if (!decode_non_ros_thread_info(buf.data(), static_cast<size_t>(n), info)) {
        RCLCPP_WARN(logger_, "Discarded malformed NonRosThreadInfo datagram (size=%zd)", n);
        continue;
      }
      // The user-supplied callback is invoked on this private reader thread,
      // which is the std::thread entry point. Letting an exception escape it
      // would call std::terminate() and take down the whole process, so we
      // contain failures here, log them, and drop the message.
      try {
        callback_(std::move(info));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          logger_, "Exception thrown from NonRosThreadInfo callback: %s; dropping message",
          e.what());
      } catch (...) {
        RCLCPP_ERROR(
          logger_, "Unknown exception thrown from NonRosThreadInfo callback; dropping message");
      }
    }
    if (exit_loop) {
      break;
    }
  }
}

}  // namespace agnocast_cie_thread_configurator
