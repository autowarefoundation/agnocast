#pragma once

#include "agnocast/agnocast_callback_info.hpp"
#include "agnocast/agnocast_timer_info.hpp"
#include "sys/epoll.h"

#include <atomic>
#include <mutex>
#include <shared_mutex>
#include <vector>

namespace agnocast
{

struct AgnocastExecutable;

extern std::atomic<bool> need_epoll_updates;

constexpr uint32_t TIMER_EVENT_FLAG = 0x80000000;
constexpr uint32_t CLOCK_EVENT_FLAG = 0x40000000;     // For clock_eventfd events (ROS_TIME timers)
constexpr uint32_t SHUTDOWN_EVENT_FLAG = 0x20000000;  // For shutdown events (AgnocastOnlyExecutor)
constexpr uint32_t EPOLL_EVENT_ID_RESERVED_MASK =
  TIMER_EVENT_FLAG | CLOCK_EVENT_FLAG | SHUTDOWN_EVENT_FLAG;

/// @return true if shutdown event detected, false otherwise
bool wait_and_handle_epoll_event(
  int epoll_fd, pid_t my_pid, int timeout_ms, std::mutex & ready_agnocast_executables_mutex,
  std::vector<AgnocastExecutable> & ready_agnocast_executables);

template <class ValidateFn>
void prepare_epoll_impl(
  const int epoll_fd, const pid_t my_pid, std::mutex & ready_agnocast_executables_mutex,
  std::vector<AgnocastExecutable> & ready_agnocast_executables,
  ValidateFn && validate_callback_group)
{
  // Register subscription callbacks to epoll
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
        agnocast::enqueue_receive_and_execute(
          callback_info_id, my_pid, callback_info, ready_agnocast_executables_mutex,
          ready_agnocast_executables);
      }

      callback_info.need_epoll_update = false;
    }
  }

  // Register timers to epoll
  {
    std::lock_guard<std::mutex> lock(id2_timer_info_mtx);

    for (auto & it : id2_timer_info) {
      const uint32_t timer_id = it.first;
      TimerInfo & timer_info = *it.second;

      if (!timer_info.need_epoll_update) {
        continue;
      }

      if (!timer_info.timer.lock() || !validate_callback_group(timer_info.callback_group)) {
        continue;
      }

      std::shared_lock fd_lock(timer_info.fd_mutex);

      // Register clock_eventfd for ROS_TIME timers (simulation time support)
      if (timer_info.clock_eventfd >= 0) {
        struct epoll_event clock_ev = {};
        clock_ev.events = EPOLLIN;
        clock_ev.data.u32 = timer_id | CLOCK_EVENT_FLAG;

        if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, timer_info.clock_eventfd, &clock_ev) == -1) {
          RCLCPP_ERROR(logger, "epoll_ctl failed for clock_eventfd: %s", strerror(errno));
          close(agnocast_fd);
          exit(EXIT_FAILURE);
        }
      }

      // Register timerfd (wall clock based firing)
      if (timer_info.timer_fd >= 0) {
        struct epoll_event ev = {};
        ev.events = EPOLLIN;
        ev.data.u32 = timer_id | TIMER_EVENT_FLAG;

        if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, timer_info.timer_fd, &ev) == -1) {
          RCLCPP_ERROR(logger, "epoll_ctl failed for timer: %s", strerror(errno));
          close(agnocast_fd);
          exit(EXIT_FAILURE);
        }
      }

      timer_info.need_epoll_update = false;
    }
  }

  // Check if all updates are done
  const bool all_callbacks_updated = [&]() {
    std::lock_guard<std::mutex> lock(id2_callback_info_mtx);
    return std::none_of(id2_callback_info.begin(), id2_callback_info.end(), [](const auto & it) {
      return it.second.need_epoll_update;
    });
  }();

  const bool all_timers_updated = [&]() {
    std::lock_guard<std::mutex> lock(id2_timer_info_mtx);
    return std::none_of(id2_timer_info.begin(), id2_timer_info.end(), [](const auto & it) {
      return it.second->need_epoll_update;
    });
  }();

  if (all_callbacks_updated && all_timers_updated) {
    need_epoll_updates.store(false);
  }
}

}  // namespace agnocast
