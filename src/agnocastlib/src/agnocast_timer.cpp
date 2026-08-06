#include "agnocast/agnocast_timer.hpp"

#include "agnocast/agnocast_timer_info.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "rclcpp/logging.hpp"

namespace agnocast
{

TimerBase::~TimerBase()
{
  unregister_timer_info(timer_id_);
}

void TimerBase::reset()
{
  auto timer_info = timer_info_.lock();
  if (!timer_info) {
    return;
  }
  timer_info->reset();
  canceled_.store(false);

  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  if (on_reset_callback_) {
    trigger_on_reset_callback(1);
  } else {
    reset_counter_++;
  }
}

std::chrono::nanoseconds TimerBase::time_until_trigger()
{
  if (canceled_.load()) {
    return std::chrono::nanoseconds::max();
  }

  auto timer_info = timer_info_.lock();
  if (!timer_info) {
    return std::chrono::nanoseconds::max();
  }
  const int64_t now_ns = timer_info->clock->now().nanoseconds();
  const int64_t next_ns = timer_info->next_call_time_ns.load();
  return std::chrono::nanoseconds(next_ns - now_ns);
}

void TimerBase::set_period(std::chrono::nanoseconds period)
{
  auto timer_info = timer_info_.lock();
  if (!timer_info) {
    throw std::runtime_error("set_period called on an invalidated timer (timer_info expired)");
  }
  timer_info->set_period(period);
}

void TimerBase::set_on_reset_callback(std::function<void(size_t)> callback)
{
  if (!callback) {
    throw std::invalid_argument("The callback passed to set_on_reset_callback is not callable.");
  }

  auto new_callback = [callback = std::move(callback), this](size_t reset_calls) {
    try {
      callback(reset_calls);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        logger, "agnocast::TimerBase (id=%u) caught exception in on_reset_callback: %s", timer_id_,
        exception.what());
    } catch (...) {
      RCLCPP_ERROR(
        logger, "agnocast::TimerBase (id=%u) caught unhandled exception in on_reset_callback",
        timer_id_);
    }
  };

  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  on_reset_callback_ = std::move(new_callback);
  if (reset_counter_ > 0) {
    trigger_on_reset_callback(reset_counter_);
    reset_counter_ = 0;
  }
}

void TimerBase::clear_on_reset_callback()
{
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  on_reset_callback_ = nullptr;
}

void TimerBase::trigger_on_reset_callback(size_t reset_count)
{
  // Copy before calling: if the callback itself calls set_on_reset_callback or
  // clear_on_reset_callback, on_reset_callback_ will be overwritten (destroying
  // the original std::function), but this local copy keeps the callable alive.
  const auto on_reset_callback = on_reset_callback_;
  on_reset_callback(reset_count);
}

}  // namespace agnocast
