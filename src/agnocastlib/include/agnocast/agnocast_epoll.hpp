#pragma once

#include "agnocast/agnocast_callback_info.hpp"
#include "sys/epoll.h"

#include <atomic>
#include <mutex>
#include <vector>

namespace agnocast
{

struct AgnocastExecutable;

extern std::atomic<bool> need_epoll_updates;

void wait_and_handle_epoll_event(
  int epoll_fd, pid_t my_pid, int timeout_ms, std::mutex & ready_agnocast_executables_mutex,
  std::vector<AgnocastExecutable> & ready_agnocast_executables);

template <class ValidateFn>
void prepare_epoll_impl(
  const int epoll_fd, const pid_t my_pid, std::mutex & ready_agnocast_executables_mutex,
  std::vector<AgnocastExecutable> & ready_agnocast_executables,
  ValidateFn && validate_callback_group)
{
  std::lock_guard<std::mutex> lock(id2_callback_info_mtx);

  for (auto & it : id2_callback_info) {
    const uint32_t callback_info_id = it.first;
    CallbackInfo & callback_info = it.second;

    if (!callback_info.need_epoll_update) {
      continue;
    }

    if (!validate_callback_group(callback_info.callback_group)) {
      continue;
    }

    struct epoll_event ev = {};
    ev.events = EPOLLIN;
    ev.data.u32 = callback_info_id;

    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, callback_info.mqdes, &ev) == -1) {
      RCLCPP_ERROR(logger, "epoll_ctl failed: %s", strerror(errno));
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }

    if (callback_info.is_transient_local) {
      agnocast::receive_message(
        callback_info_id, my_pid, callback_info, ready_agnocast_executables_mutex,
        ready_agnocast_executables);
    }

    callback_info.need_epoll_update = false;
  }

  const bool all_updated = std::none_of(
    id2_callback_info.begin(), id2_callback_info.end(),
    [](const auto & it) { return it.second.need_epoll_update; });

  if (all_updated) {
    need_epoll_updates.store(false);
  }
}

}  // namespace agnocast
