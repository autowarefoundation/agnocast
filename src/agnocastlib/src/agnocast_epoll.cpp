#include "agnocast/agnocast_epoll.hpp"

#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_epoll_event.hpp"

#include <sys/epoll.h>
#include <unistd.h>

#include <cstdint>

namespace agnocast
{

void DummyEventHandler::handle(EpollEventLocalID event_local_id)
{
  RCLCPP_WARN(
    logger, "DummyEventHandler received an unexpected event (local_id=%u). This event is ignored.",
    event_local_id);
}

EpollManager::EpollManager(EventHandlerArray sources)
: epoll_fd_(epoll_create1(0)), sources_(std::move(sources))
{
  if (epoll_fd_ == -1) {
    RCLCPP_ERROR(logger, "epoll_create1 failed: %s", strerror(errno));
    exit(EXIT_FAILURE);
  }

  for (uint32_t type = 0; type < static_cast<uint32_t>(EpollEventType::NrEventType); type++) {
    if (!sources_[type]) {
      RCLCPP_ERROR(logger, "invalid epoll event source array: sources_[%u] is nullptr", type);
      exit(EXIT_FAILURE);
    }
    auto source_type = static_cast<uint32_t>(sources_[type]->get_type());
    if (source_type != type && source_type != static_cast<uint32_t>(EpollEventType::Dummy)) {
      RCLCPP_ERROR(logger, "invalid epoll event type: expected %u, got %u", type, source_type);
      exit(EXIT_FAILURE);
    }
  }
}

EpollManager::~EpollManager()
{
  if (epoll_fd_ >= 0) {
    close(epoll_fd_);
  }
}

bool EpollManager::add_event(int fd, EpollEventType type, EpollEventLocalID local_id) const
{
  struct epoll_event ev = {};
  ev.events = EPOLLIN;
  ev.data.u64 = pack_epoll_data(type, local_id);
  if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &ev) == -1) {
    RCLCPP_WARN(logger, "epoll_ctl failed: %s", strerror(errno));
    return false;
  }
  return true;
}

void EpollManager::prepare_epoll(const CallbackGroupValidator & validate_callback_group)
{
  for (uint32_t type = 0; type < static_cast<uint32_t>(EpollEventType::NrEventType); type++) {
    sources_[type]->prepare_epoll(epoll_fd_, validate_callback_group);
  }
}

void EpollManager::wait_and_handle_epoll_event(const int timeout_ms)
{
  struct epoll_event event = {};

  // blocking with timeout
  const int nfds = epoll_wait(epoll_fd_, &event, 1 /*maxevents*/, timeout_ms);

  if (nfds == -1) {
    if (errno != EINTR) {  // signal handler interruption is not error
      RCLCPP_ERROR(logger, "epoll_wait failed: %s", strerror(errno));
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }

    return;
  }

  // timeout
  if (nfds == 0) {
    return;
  }

  const auto [event_type, event_local_id] = unpack_epoll_data(event.data.u64);

  if (event_type >= EpollEventType::NrEventType) {
    RCLCPP_ERROR(
      logger, "Agnocast internal implementation error: invalid epoll event type %u",
      static_cast<uint32_t>(event_type));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  sources_[static_cast<uint32_t>(event_type)]->handle(event_local_id);
}

}  // namespace agnocast
