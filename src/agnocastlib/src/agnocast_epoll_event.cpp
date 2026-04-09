#include "agnocast/agnocast_epoll_event.hpp"

#include "agnocast/agnocast_utils.hpp"

#include <sys/epoll.h>

namespace agnocast
{

Epoll::Epoll() : epoll_fd_(epoll_create1(0))
{
  if (epoll_fd_ == -1) {
    RCLCPP_ERROR(logger, "epoll_create1 failed: %s", strerror(errno));
    exit(EXIT_FAILURE);
  }
}

Epoll::~Epoll()
{
  if (epoll_fd_ >= 0) {
    close(epoll_fd_);
  }
}

Epoll & Epoll::operator=(Epoll && other) noexcept
{
  if (this != &other) {
    if (epoll_fd_ >= 0) {
      close(epoll_fd_);
    }

    epoll_fd_ = other.epoll_fd_;
    other.epoll_fd_ = -1;
  }
  return *this;
}

[[nodiscard]] int Epoll::add_source(int fd, EpollEventType type, EpollEventLocalID local_id) const
{
  struct epoll_event ev = {};
  ev.events = EPOLLIN;
  ev.data.u64 = pack_epoll_data(type, local_id);
  return epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &ev);
}

void Epoll::remove(int fd) const
{
  epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr);
}

int Epoll::wait(std::vector<EpollResult> & results, int timeout_ms) const
{
  std::array<epoll_event, EPOLL_WAIT_MAX_EVENTS> events{};
  const int nfds = epoll_wait(epoll_fd_, events.data(), events.size(), timeout_ms);

  if (nfds < 0 || EPOLL_WAIT_MAX_EVENTS < nfds) {
    return -1;
  }

  if (!results.empty()) {
    return -1;
  }

  results.clear();
  for (int i = 0; i < nfds; ++i) {
    auto [type, id] = unpack_epoll_data(events[i].data.u64);
    results.emplace_back(type, id);
  }
  return nfds;
}

}  // namespace agnocast
