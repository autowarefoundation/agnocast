#pragma once

#include "rclcpp/macros.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <type_traits>

namespace agnocast
{

class TimerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(TimerBase)

  virtual ~TimerBase();

  // TODO: The following methods are planned to be added for rclcpp API compatibility:
  // void cancel(), bool is_canceled(), void reset(), std::chrono::nanoseconds time_until_trigger(),
  // etc.

  bool is_steady() const { return true; }

  virtual void execute_callback() = 0;

protected:
  TimerBase(uint32_t timer_id, std::chrono::nanoseconds period)
  : timer_id_(timer_id), period_(period)
  {
  }

  uint32_t timer_id_;
  std::chrono::nanoseconds period_;
};

template <typename FunctorT>
class WallTimer : public TimerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(WallTimer)

  WallTimer(uint32_t timer_id, std::chrono::nanoseconds period, FunctorT && callback)
  : TimerBase(timer_id, period), callback_(std::forward<FunctorT>(callback))
  {
  }

  void execute_callback() override
  {
    if constexpr (std::is_invocable_v<FunctorT, TimerBase &>) {
      callback_(*this);
    } else {
      callback_();
    }
  }

protected:
  RCLCPP_DISABLE_COPY(WallTimer)

  FunctorT callback_;
};

}  // namespace agnocast
