#include "agnocast/agnocast_epoll.hpp"

#include "agnocast/agnocast.hpp"

#include <unistd.h>

namespace agnocast
{

std::atomic<bool> need_epoll_updates{false};

bool wait_and_handle_epoll_event(
  const int epoll_fd, const pid_t my_pid, const int timeout_ms,
  std::mutex & ready_agnocast_executables_mutex,
  std::vector<AgnocastExecutable> & ready_agnocast_executables)
{
  struct epoll_event event = {};

  // blocking with timeout
  const int nfds = epoll_wait(epoll_fd, &event, 1 /*maxevents*/, timeout_ms);

  if (nfds == -1) {
    if (errno != EINTR) {  // signal handler interruption is not error
      throw std::runtime_error(std::string("epoll_wait failed: ") + strerror(errno));
    }

    return false;
  }

  // timeout
  if (nfds == 0) {
    return false;
  }

  const uint32_t event_id = event.data.u32;

  // Shutdown event - only used by AgnocastOnlyExecutor
  if ((event_id & SHUTDOWN_EVENT_FLAG) != 0U) {
    return true;
  }

  if ((event_id & TIMER_EVENT_FLAG) != 0U) {
    // Timer event (timerfd fired)
    const uint32_t timer_id = event_id & ~TIMER_EVENT_FLAG;
    rclcpp::CallbackGroup::SharedPtr callback_group;
    std::shared_ptr<agnocast::TimerBase> timer_ptr;

    std::shared_ptr<TimerInfo> timer_info;
    {
      std::lock_guard<std::mutex> lock(id2_timer_info_mtx);
      const auto it = id2_timer_info.find(timer_id);
      if (it == id2_timer_info.end()) {
        throw std::logic_error("Agnocast internal implementation error: timer info cannot be found");
      }
      timer_info = it->second;
      timer_ptr = timer_info->timer.lock();
      if (!timer_ptr) {
        return false;  // Timer object has been destroyed
      }
      callback_group = timer_info->callback_group;
    }

    // Read the number of expirations to clear the event
    uint64_t expirations = 0;
    {
      std::shared_lock fd_lock(timer_info->fd_mutex);
      if (timer_info->timer_fd < 0) {
        return false;  // Timer fd was closed (ROS time activated)
      }
      const ssize_t ret = read(timer_info->timer_fd, &expirations, sizeof(expirations));
      if (ret == -1 || expirations == 0) {
        return false;
      }
    }

    auto callable = std::make_shared<std::function<void()>>();
    // For tracepoints.
    const void * callable_ptr = callable.get();
    // Create a callable that handles the timer event
    *callable = [timer_info, callable_ptr]() {
      TRACEPOINT(agnocast_callable_start, callable_ptr);
      handle_timer_event(*timer_info);
      TRACEPOINT(agnocast_callable_end, callable_ptr);
    };

    TRACEPOINT(
      agnocast_create_timer_callable, static_cast<const void *>(callable_ptr),
      static_cast<const void *>(timer_ptr.get()));

    {
      std::lock_guard<std::mutex> ready_lock{ready_agnocast_executables_mutex};
      ready_agnocast_executables.emplace_back(AgnocastExecutable{callable, callback_group});
    }
  } else if ((event_id & CLOCK_EVENT_FLAG) != 0U) {
    // Clock event (ROS_TIME clock updated via time jump callback)
    const uint32_t timer_id = event_id & ~CLOCK_EVENT_FLAG;
    rclcpp::CallbackGroup::SharedPtr callback_group;
    std::shared_ptr<agnocast::TimerBase> timer_ptr;

    std::shared_ptr<TimerInfo> timer_info;
    {
      std::lock_guard<std::mutex> lock(id2_timer_info_mtx);
      const auto it = id2_timer_info.find(timer_id);
      if (it == id2_timer_info.end()) {
        throw std::logic_error("Agnocast internal implementation error: timer info cannot be found");
      }
      timer_info = it->second;
      timer_ptr = timer_info->timer.lock();
      if (!timer_ptr) {
        return false;  // Timer object has been destroyed
      }
      callback_group = timer_info->callback_group;
    }

    uint64_t val = 0;
    const ssize_t ret = read(timer_info->clock_eventfd, &val, sizeof(val));
    if (ret == -1 || val == 0) {
      return false;
    }

    // Check if timer is ready (corresponds to rcl_timer_is_ready)
    const int64_t now_ns = timer_info->clock->now().nanoseconds();
    const int64_t next_call_ns = timer_info->next_call_time_ns.load(std::memory_order_relaxed);
    if (now_ns < next_call_ns) {
      return false;
    }

    // Create a callable that handles the clock event
    auto callable = std::make_shared<std::function<void()>>();
    const void * callable_ptr = callable.get();

    *callable = [timer_info, callable_ptr]() {
      TRACEPOINT(agnocast_callable_start, callable_ptr);
      handle_timer_event(*timer_info);
      TRACEPOINT(agnocast_callable_end, callable_ptr);
    };

    TRACEPOINT(
      agnocast_create_timer_callable, static_cast<const void *>(callable_ptr),
      static_cast<const void *>(timer_ptr.get()));

    {
      std::lock_guard<std::mutex> ready_lock{ready_agnocast_executables_mutex};
      ready_agnocast_executables.emplace_back(AgnocastExecutable{callable, callback_group});
    }
  } else {
    // Subscription callback event
    const uint32_t callback_info_id = event_id;
    CallbackInfo callback_info;

    {
      std::lock_guard<std::mutex> lock(id2_callback_info_mtx);

      const auto it = id2_callback_info.find(callback_info_id);
      if (it == id2_callback_info.end()) {
        // Callback was unregistered (subscription destroyed) - this is normal
        return false;
      }

      callback_info = it->second;
    }

    MqMsgAgnocast mq_msg = {};

    // non-blocking
    auto ret =
      mq_receive(callback_info.mqdes, reinterpret_cast<char *>(&mq_msg), sizeof(mq_msg), nullptr);
    if (ret < 0) {
      if (errno != EAGAIN) {
        throw std::runtime_error(
          "mq_receive failed for topic '" + callback_info.topic_name + "' (subscriber_id=" +
          std::to_string(callback_info.subscriber_id) + "): " + strerror(errno));
      }

      return false;
    }

    agnocast::enqueue_receive_and_execute(
      callback_info_id, my_pid, callback_info, ready_agnocast_executables_mutex,
      ready_agnocast_executables);
  }

  return false;
}

}  // namespace agnocast
