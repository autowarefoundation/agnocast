#include "agnocast/agnocast_timer_info.hpp"

#include "agnocast/agnocast_epoll.hpp"
#include "agnocast/agnocast_utils.hpp"

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

TimerInfo::~TimerInfo()
{
  if (timer_fd >= 0) {
    close(timer_fd);
  }
}

int create_timer_fd(uint32_t timer_id, std::chrono::nanoseconds period)
{
  int timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
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
  if ((timer_id & TIMER_EVENT_FLAG) != 0U) {
    throw std::runtime_error("Timer ID overflow: too many timers created");
  }
  return timer_id;
}

void register_timer_info(
  uint32_t timer_id, const std::shared_ptr<TimerBase> & timer, std::chrono::nanoseconds period,
  const rclcpp::CallbackGroup::SharedPtr & callback_group)
{
  const int timer_fd = create_timer_fd(timer_id, period);
  const auto now = std::chrono::steady_clock::now();

  {
    std::lock_guard<std::mutex> lock(id2_timer_info_mtx);
    const int64_t now_ns = to_nanoseconds(now);
    auto timer_info = std::make_shared<TimerInfo>();
    timer_info->timer_fd = timer_fd;
    timer_info->timer = timer;
    timer_info->last_call_time_ns.store(now_ns, std::memory_order_relaxed);
    timer_info->next_call_time_ns.store(now_ns + period.count(), std::memory_order_relaxed);
    timer_info->period = period;
    timer_info->callback_group = callback_group;
    timer_info->need_epoll_update = true;
    id2_timer_info[timer_id] = std::move(timer_info);
  }

  need_epoll_updates.store(true);
}

void handle_timer_event(TimerInfo & timer_info)
{
  // TODO(Koichi98): Add canceled check here

  // Read the number of expirations to clear the event
  uint64_t expirations = 0;
  const ssize_t ret = read(timer_info.timer_fd, &expirations, sizeof(expirations));

  if (ret == -1) {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN(logger, "Failed to read timer fd: %s", strerror(errno));
      return;
    }
  }

  if (expirations > 0) {
    auto timer = timer_info.timer.lock();
    if (!timer) {
      return;  // Timer object has been destroyed
    }

    const auto now = std::chrono::steady_clock::now();
    const int64_t now_ns = to_nanoseconds(now);

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
}

void unregister_timer_info(uint32_t timer_id)
{
  std::lock_guard<std::mutex> lock(id2_timer_info_mtx);
  id2_timer_info.erase(timer_id);
}

}  // namespace agnocast
