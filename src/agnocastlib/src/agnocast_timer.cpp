#include "agnocast/agnocast_timer.hpp"

#include "agnocast/agnocast_timer_info.hpp"

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

  // TODO(tomiy-0x62): call on_reset_callback
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

}  // namespace agnocast
