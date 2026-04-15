#include "agnocast/agnocast_epoll.hpp"

#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_epoll_event.hpp"
#include "agnocast/agnocast_mq.hpp"

#include <unistd.h>

#include <vector>

namespace agnocast
{

struct AgnocastExecutable;

EpollManager::EpollManager(EventSourceArray sources) : sources_(std::move(sources))
{
  for (uint32_t type = 0; type < static_cast<size_t>(EpollEventType::NrEventType); type++) {
    if (!sources_[type]) {
      RCLCPP_ERROR(logger, "invalid epoll event source array: sources_[%u] is nullptr", type);
      exit(EXIT_FAILURE);
    }
  }
}

void EpollManager::prepare_epoll(const CallbackGroupValidator & validate_callback_group)
{
  for (uint32_t type = 0; type < static_cast<uint32_t>(EpollEventType::NrEventType); type++) {
    sources_[type]->prepare_epoll(epoll_, validate_callback_group);
  }
}

bool EpollManager::wait_and_handle_epoll_event(const int timeout_ms)
{
  std::vector<EpollResult> results;
  const int nr_results = epoll_.wait(results, timeout_ms);

  if (nr_results == -1) {
    if (errno != EINTR) {  // signal handler interruption is not error
      RCLCPP_ERROR(logger, "Epoll::wait failed: %s", strerror(errno));
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }

    return false;
  }

  // timeout
  if (nr_results == 0) {
    return false;
  }

  bool should_shutdown = false;
  for (auto [event_type, event_local_id] : results) {
    if (EpollEventType::NrEventType <= event_type) {
      RCLCPP_ERROR(
        logger, "Agnocast internal implementation error: invalid epoll event type %u",
        static_cast<uint32_t>(event_type));
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }

    bool ret = sources_[static_cast<uint32_t>(event_type)]->handle(event_local_id);
    should_shutdown = should_shutdown || ret;
  }
  return should_shutdown;
}

}  // namespace agnocast
