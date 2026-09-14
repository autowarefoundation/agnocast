#include "agnocast_cie_thread_configurator/non_ros_thread_ipc.hpp"

#include "rclcpp/logging.hpp"

#include <sys/epoll.h>
#include <sys/eventfd.h>

#include <system_error>
#include <utility>

namespace agnocast_cie_thread_configurator
{

namespace
{

UniqueFd fd_or_throw(int fd, const char * what)
{
  if (fd < 0) {
    throw std::system_error(errno, std::generic_category(), what);
  }
  return UniqueFd(fd);
}

void add_to_epoll(int epfd, int fd, const char * what)
{
  epoll_event ev{};
  ev.events = EPOLLIN;
  ev.data.fd = fd;
  if (::epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev) < 0) {
    throw std::system_error(errno, std::generic_category(), what);
  }
}

}  // namespace

// The fd members own what has been acquired so far, so a throw from any step
// closes it on unwind.
NonRosThreadInfoListener::NonRosThreadInfoListener(Callback callback, rclcpp::Logger logger)
: callback_(std::move(callback)), logger_(std::move(logger))
{
  listener_fd_ = fd_or_throw(::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0), "socket");

  sockaddr_un addr{};
  const socklen_t addr_len = setup_non_ros_thread_info_sockaddr(addr);
  if (::bind(listener_fd_.get(), reinterpret_cast<sockaddr *>(&addr), addr_len) < 0) {
    throw std::system_error(errno, std::generic_category(), "bind");
  }

  stop_eventfd_ = fd_or_throw(::eventfd(0, EFD_CLOEXEC), "eventfd");
  epfd_ = fd_or_throw(::epoll_create1(EPOLL_CLOEXEC), "epoll_create1");
  add_to_epoll(epfd_.get(), listener_fd_.get(), "epoll_ctl listener");
  add_to_epoll(epfd_.get(), stop_eventfd_.get(), "epoll_ctl eventfd");

  thread_ = std::thread(&NonRosThreadInfoListener::run, this);
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
  if (stop_eventfd_) {
    uint64_t one = 1;
    [[maybe_unused]] ssize_t w = ::write(stop_eventfd_.get(), &one, sizeof(one));
  }
  if (thread_.joinable()) {
    try {
      thread_.join();
    } catch (const std::system_error &) {
      // join() can throw on EDEADLK/EINVAL/etc. The destructor must not
      // propagate exceptions, so we treat join failure as best-effort.
    }
  }
  epfd_.reset();
  listener_fd_.reset();
  stop_eventfd_.reset();
}

void NonRosThreadInfoListener::run()
{
  std::array<uint8_t, k_non_ros_thread_info_max_wire_size> buf;
  constexpr int k_max_events = 2;
  epoll_event events[k_max_events];

  while (true) {
    int n_ev = ::epoll_wait(epfd_.get(), events, k_max_events, -1);
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
      if (events[i].data.fd == stop_eventfd_.get()) {
        exit_loop = true;
        break;
      }
      if (events[i].data.fd != listener_fd_.get()) {
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
      ssize_t n = ::recv(listener_fd_.get(), buf.data(), buf.size(), MSG_TRUNC);
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
