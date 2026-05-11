#include "agnocast/agnocast_timer_info.hpp"

#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_epoll.hpp"
#include "agnocast/agnocast_epoll_event.hpp"
#include "agnocast/agnocast_epoll_update_dispatcher.hpp"
#include "agnocast/agnocast_executor.hpp"
#include "agnocast/agnocast_utils.hpp"

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace agnocast
{

std::mutex id2_timer_info_mtx;
std::unordered_map<uint32_t, std::shared_ptr<TimerInfo>> id2_timer_info;
std::atomic<uint32_t> next_timer_id{0};

// Corresponds to _rcl_timer_time_jump (before_jump=true) in rcl/src/rcl/timer.c.
// Unlike RCL, we save time_credit unconditionally because rclcpp's pre_callback
// doesn't receive jump info. This is safe as time_credit is only consumed in
// post_jump for clock_change cases.
void handle_pre_time_jump(TimerInfo & timer_info)
{
  int64_t now_ns = 0;
  try {
    now_ns = timer_info.clock->now().nanoseconds();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Agnocast"), "Failed to get current time in pre jump callback: %s",
      e.what());
    return;
  }

  if (now_ns == 0) {
    // No time credit if clock is uninitialized
    return;
  }
  // Source of time is changing, but the timer has elapsed some portion of its period.
  // Save elapsed duration pre jump so the timer only waits the remainder in the new epoch.
  const int64_t next_call_ns = timer_info.next_call_time_ns.load(std::memory_order_relaxed);
  timer_info.time_credit.store(next_call_ns - now_ns, std::memory_order_relaxed);
}

// Corresponds to _rcl_timer_time_jump (before_jump=false) in rcl/src/rcl/timer.c
void handle_post_time_jump(TimerInfo & timer_info, const rcl_time_jump_t & jump)
{
  int64_t now_ns = 0;
  try {
    now_ns = timer_info.clock->now().nanoseconds();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Agnocast"), "Failed to get current time in post jump callback: %s",
      e.what());
    return;
  }

  const int64_t last_call_ns = timer_info.last_call_time_ns.load(std::memory_order_relaxed);
  const int64_t next_call_ns = timer_info.next_call_time_ns.load(std::memory_order_relaxed);
  const int64_t period_ns = timer_info.period.count();

  if (jump.clock_change == RCL_ROS_TIME_ACTIVATED) {
    // ROS time activated: close timerfd (simulation time will use clock_eventfd)
    {
      std::unique_lock lock(timer_info.fd_mutex);
      if (timer_info.timer_fd >= 0) {
        close(timer_info.timer_fd);
        timer_info.timer_fd = -1;
      }
    }

    if (now_ns == 0) {
      // Can't apply time credit if clock is uninitialized
      return;
    }
    const int64_t time_credit = timer_info.time_credit.exchange(0, std::memory_order_relaxed);
    if (time_credit != 0) {
      // Set times in new epoch so timer only waits the remainder of the period
      timer_info.next_call_time_ns.store(
        now_ns - time_credit + period_ns, std::memory_order_relaxed);
      timer_info.last_call_time_ns.store(now_ns - time_credit, std::memory_order_relaxed);
    }
  } else if (jump.clock_change == RCL_ROS_TIME_DEACTIVATED) {
    // TODO(Koichi98): Support dynamic ROS time deactivation (use_sim_time changed from true to
    // false at runtime). This requires recreating timerfd and re-registering it with epoll, which
    // involves request epoll update under unique_lock and needs careful synchronization with
    // the shared_lock reader in prepare_epoll.
    RCLCPP_WARN(
      rclcpp::get_logger("Agnocast"),
      "ROS time deactivation is not yet supported. Timer behavior may be incorrect.");
  } else if (next_call_ns <= now_ns) {
    // Post forward jump and timer is ready
    if (timer_info.clock_eventfd >= 0) {
      const uint64_t val = 1;
      if (write(timer_info.clock_eventfd, &val, sizeof(val)) == -1) {
        RCLCPP_WARN(
          rclcpp::get_logger("Agnocast"), "Failed to write to clock_eventfd: %s",
          std::strerror(errno));
      }
    }
  } else if (now_ns < last_call_ns) {
    // Post backwards time jump that went further back than 1 period
    // Next callback should happen after 1 period
    timer_info.next_call_time_ns.store(now_ns + period_ns, std::memory_order_relaxed);
    timer_info.last_call_time_ns.store(now_ns, std::memory_order_relaxed);
  }
}

void setup_time_jump_callback(
  const std::shared_ptr<TimerInfo> & timer_info, const rclcpp::Clock::SharedPtr & clock)
{
  if (clock->get_clock_type() != RCL_ROS_TIME) {
    return;
  }

  rcl_jump_threshold_t threshold;
  threshold.on_clock_change = true;
  threshold.min_forward.nanoseconds = 1;
  threshold.min_backward.nanoseconds = -1;

  std::weak_ptr<TimerInfo> weak_timer_info = timer_info;

  timer_info->jump_handler = clock->create_jump_callback(
    [weak_timer_info]() {
      auto ti = weak_timer_info.lock();
      if (!ti) {
        return;
      }
      handle_pre_time_jump(*ti);
    },
    [weak_timer_info](const rcl_time_jump_t & jump) {
      auto ti = weak_timer_info.lock();
      if (!ti) {
        return;
      }
      handle_post_time_jump(*ti, jump);
    },
    threshold);
}

TimerInfo::~TimerInfo()
{
  if (timer_fd >= 0) {
    close(timer_fd);
  }
  if (clock_eventfd >= 0) {
    close(clock_eventfd);
  }
}

int create_timer_fd(uint32_t timer_id, std::chrono::nanoseconds period, rcl_clock_type_t clock_type)
{
  // Use CLOCK_MONOTONIC for STEADY_TIME, CLOCK_REALTIME for others (SYSTEM_TIME, ROS_TIME)
  // This matches rclcpp's behavior where:
  // - RCL_STEADY_TIME uses monotonic clock
  // - RCL_SYSTEM_TIME and RCL_ROS_TIME use system clock
  const int clockid = (clock_type == RCL_STEADY_TIME) ? CLOCK_MONOTONIC : CLOCK_REALTIME;
  int timer_fd = timerfd_create(clockid, TFD_NONBLOCK | TFD_CLOEXEC);
  if (timer_fd == -1) {
    throw std::runtime_error(
      "timerfd_create failed for timer_id=" + std::to_string(timer_id) + ": " +
      std::strerror(errno));
  }

  struct itimerspec spec = {};
  const auto period_count = period.count();
  if (period_count == 0) {
    // Workaround: timerfd_settime() disarms the timer when both it_value and it_interval
    // are zero. Use 1ns to keep the timer armed and achieve "always ready" semantics.
    spec.it_interval.tv_sec = 0;
    spec.it_interval.tv_nsec = 1;
  } else {
    spec.it_interval.tv_sec = period_count / NANOSECONDS_PER_SECOND;
    spec.it_interval.tv_nsec = period_count % NANOSECONDS_PER_SECOND;
  }
  spec.it_value = spec.it_interval;

  if (timerfd_settime(timer_fd, 0, &spec, nullptr) == -1) {
    const int saved_errno = errno;
    close(timer_fd);
    throw std::runtime_error(
      "timerfd_settime failed for timer_id=" + std::to_string(timer_id) +
      ", period=" + std::to_string(period_count) + "ns: " + std::strerror(saved_errno));
  }

  return timer_fd;
}

uint32_t allocate_timer_id()
{
  const uint32_t timer_id = next_timer_id.fetch_add(1);
  if (timer_id >= MAX_TIMER_ID) {
    throw std::runtime_error("Timer ID overflow: too many timers created");
  }
  return timer_id;
}

void register_timer_info(
  uint32_t timer_id, const std::shared_ptr<TimerBase> & timer, std::chrono::nanoseconds period,
  const rclcpp::CallbackGroup::SharedPtr & callback_group, const rclcpp::Clock::SharedPtr & clock)
{
  const bool is_ros_time = (clock->get_clock_type() == RCL_ROS_TIME);
  const int64_t now_ns = clock->now().nanoseconds();

  auto timer_info = std::make_shared<TimerInfo>();
  timer_info->timer_id = timer_id;
  timer_info->timer = timer;
  timer_info->last_call_time_ns.store(now_ns, std::memory_order_relaxed);
  timer_info->next_call_time_ns.store(now_ns + period.count(), std::memory_order_relaxed);
  timer_info->period = period;
  timer_info->callback_group = callback_group;
  timer_info->need_epoll_update = true;
  timer_info->clock = clock;

  if (is_ros_time) {
    // ROS_TIME timers use clock_eventfd for simulation time support
    timer_info->clock_eventfd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (timer_info->clock_eventfd == -1) {
      throw std::runtime_error(
        "eventfd creation failed for timer_id=" + std::to_string(timer_id) + ": " +
        std::strerror(errno));
    }

    // Only create timerfd if ros_time is not active (system time mode)
    // If ros_time is already active, timer will be driven by clock_eventfd
    if (!clock->ros_time_is_active()) {
      timer_info->timer_fd = create_timer_fd(timer_id, period, clock->get_clock_type());
    }
  } else {
    // Non-ROS_TIME timers always use timerfd
    timer_info->timer_fd = create_timer_fd(timer_id, period, clock->get_clock_type());
  }

  setup_time_jump_callback(timer_info, clock);

  if (timer_info->timer_fd >= 0) {
    timer_info->timer_fd_need_update = true;
  }

  if (timer_info->clock_eventfd >= 0) {
    timer_info->clock_eventfd_need_update = true;
  }

  {
    std::lock_guard<std::mutex> lock(id2_timer_info_mtx);
    id2_timer_info[timer_id] = std::move(timer_info);
  }

  EpollUpdateDispatcher::get_instance().request_update_all();
}

void handle_timer_event(TimerInfo & timer_info)
{
  // TODO(Koichi98): Add canceled check here

  auto timer = timer_info.timer.lock();
  if (!timer) {
    return;  // Timer object has been destroyed
  }

  const int64_t now_ns = timer_info.clock->now().nanoseconds();

  timer_info.last_call_time_ns.store(now_ns, std::memory_order_relaxed);

  const int64_t period_ns = timer_info.period.count();
  int64_t next_call_time_ns =
    timer_info.next_call_time_ns.load(std::memory_order_relaxed) + period_ns;

  // in case the timer has missed at least one cycle
  if (next_call_time_ns < now_ns) {
    if (period_ns == 0) {
      // a timer with a period of zero is considered always ready
      next_call_time_ns = now_ns;
    } else {
      // move the next call time forward by as many periods as necessary
      const int64_t now_ahead = now_ns - next_call_time_ns;
      // rounding up without overflow
      const int64_t periods_ahead = 1 + (now_ahead - 1) / period_ns;
      next_call_time_ns += periods_ahead * period_ns;
    }
  }
  timer_info.next_call_time_ns.store(next_call_time_ns, std::memory_order_relaxed);

  timer->execute_callback();
}

void unregister_timer_info(uint32_t timer_id)
{
  std::lock_guard<std::mutex> lock(id2_timer_info_mtx);
  id2_timer_info.erase(timer_id);
}

void TimerEventHandler::prepare_epoll(
  int epoll_fd, const CallbackGroupValidator & validate_callback_group)
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

    // Register timerfd (wall clock based firing)
    if (timer_info.timer_fd >= 0 && timer_info.timer_fd_need_update) {
      struct epoll_event clock_ev = {};
      clock_ev.events = EPOLLIN;
      clock_ev.data.u64 = pack_epoll_data(EpollEventType::Timer, timer_id);
      if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, timer_info.timer_fd, &clock_ev) == -1) {
        RCLCPP_ERROR(logger, "epoll_ctl failed for timer: %s", strerror(errno));
        close(agnocast_fd);
        exit(EXIT_FAILURE);
      }
      timer_info.timer_fd_need_update = false;
    }

    timer_info.need_epoll_update =
      timer_info.timer_fd_need_update || timer_info.clock_eventfd_need_update;
  }
}

void TimerEventHandler::handle(EpollEventLocalID event_local_id)
{
  // Timer event (timerfd fired)
  const uint32_t timer_id = event_local_id;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  std::shared_ptr<agnocast::TimerBase> timer_ptr;

  std::shared_ptr<TimerInfo> timer_info;
  {
    std::lock_guard<std::mutex> lock(id2_timer_info_mtx);
    const auto it = id2_timer_info.find(timer_id);
    if (it == id2_timer_info.end()) {
      RCLCPP_ERROR(logger, "Agnocast internal implementation error: timer info cannot be found");
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }
    timer_info = it->second;
    timer_ptr = timer_info->timer.lock();
    if (!timer_ptr) {
      return;  // Timer object has been destroyed
    }
    callback_group = timer_info->callback_group;
  }

  // Read the number of expirations to clear the event
  uint64_t expirations = 0;
  {
    std::shared_lock fd_lock(timer_info->fd_mutex);
    if (timer_info->timer_fd < 0) {
      return;  // Timer fd was closed (ROS time activated)
    }
    const ssize_t ret = read(timer_info->timer_fd, &expirations, sizeof(expirations));
    if (ret == -1 || expirations == 0) {
      return;
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
    std::lock_guard<std::mutex> ready_lock{*ready_agnocast_executables_mutex_};
    ready_agnocast_executables_->emplace_back(AgnocastExecutable{callable, callback_group});
  }
}

void ClockEventHandler::prepare_epoll(
  int epoll_fd, const CallbackGroupValidator & validate_callback_group)
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
    if (timer_info.clock_eventfd >= 0 && timer_info.clock_eventfd_need_update) {
      struct epoll_event ev = {};
      ev.events = EPOLLIN;
      ev.data.u64 = pack_epoll_data(EpollEventType::Clock, timer_id);
      if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, timer_info.clock_eventfd, &ev) == -1) {
        RCLCPP_ERROR(logger, "epoll_ctl failed for clock_eventfd: %s", strerror(errno));
        close(agnocast_fd);
        exit(EXIT_FAILURE);
      }
      timer_info.clock_eventfd_need_update = false;
    }

    timer_info.need_epoll_update =
      timer_info.timer_fd_need_update || timer_info.clock_eventfd_need_update;
  }
}

void ClockEventHandler::handle(EpollEventLocalID event_local_id)
{
  // Clock event (ROS_TIME clock updated via time jump callback)
  const uint32_t timer_id = event_local_id;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  std::shared_ptr<agnocast::TimerBase> timer_ptr;

  std::shared_ptr<TimerInfo> timer_info;
  {
    std::lock_guard<std::mutex> lock(id2_timer_info_mtx);
    const auto it = id2_timer_info.find(timer_id);
    if (it == id2_timer_info.end()) {
      RCLCPP_ERROR(logger, "Agnocast internal implementation error: timer info cannot be found");
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }
    timer_info = it->second;
    timer_ptr = timer_info->timer.lock();
    if (!timer_ptr) {
      return;  // Timer object has been destroyed
    }
    callback_group = timer_info->callback_group;
  }

  uint64_t val = 0;
  const ssize_t ret = read(timer_info->clock_eventfd, &val, sizeof(val));
  if (ret == -1 || val == 0) {
    return;
  }

  // Check if timer is ready (corresponds to rcl_timer_is_ready)
  const int64_t now_ns = timer_info->clock->now().nanoseconds();
  const int64_t next_call_ns = timer_info->next_call_time_ns.load(std::memory_order_relaxed);
  if (now_ns < next_call_ns) {
    return;
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
    std::lock_guard<std::mutex> ready_lock{*ready_agnocast_executables_mutex_};
    ready_agnocast_executables_->emplace_back(AgnocastExecutable{callable, callback_group});
  }
}

}  // namespace agnocast
