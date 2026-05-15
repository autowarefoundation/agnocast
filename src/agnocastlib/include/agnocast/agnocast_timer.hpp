#pragma once

#include "agnocast/agnocast_public_api.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <type_traits>

namespace agnocast
{

struct TimerInfo;

/**
 * @brief Base class for Agnocast timers providing periodic callback execution.
 *
 * Defines the common interface for all timer types, including callback execution,
 * clock access, and period storage.
 */
AGNOCAST_PUBLIC
class TimerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(TimerBase)

  virtual ~TimerBase();

  AGNOCAST_PUBLIC
  void cancel() { canceled_.store(true); }

  // Non-const to align with rclcpp::TimerBase::is_canceled().
  AGNOCAST_PUBLIC
  bool is_canceled() { return canceled_.load(); }

  AGNOCAST_PUBLIC
  void reset();

  AGNOCAST_PUBLIC
  std::chrono::nanoseconds time_until_trigger();

  void set_timer_info(std::weak_ptr<TimerInfo> timer_info) { timer_info_ = timer_info; }

  /** @brief Update the timer's period.
   *
   * Aligned with `rcl_timer_exchange_period`: the already-scheduled next firing keeps its
   * time; only subsequent firings adopt the new period. Throws on an invalidated timer
   * (rcl's equivalent is `RCL_RET_TIMER_INVALID`).
   *
   * @param period New firing period.
   * @throw std::runtime_error If the underlying TimerInfo has been unregistered. */
  void set_period(std::chrono::nanoseconds period);

  /** @brief Return whether this timer uses a steady clock.
   *  @return True if the clock is steady. */
  AGNOCAST_PUBLIC
  virtual bool is_steady() const = 0;

  /** @brief Get the clock associated with this timer.
   *  @return Shared pointer to the clock. */
  AGNOCAST_PUBLIC
  virtual rclcpp::Clock::SharedPtr get_clock() const = 0;

  virtual void execute_callback() = 0;

protected:
  TimerBase(uint32_t timer_id, [[maybe_unused]] std::chrono::nanoseconds period)
  : timer_id_(timer_id), timer_info_(), canceled_(false)
  {
  }

  uint32_t timer_id_;
  std::weak_ptr<TimerInfo> timer_info_;
  std::atomic<bool> canceled_;
};

/**
 * @brief Timer that fires periodically using a user-provided clock.
 *
 * @tparam FunctorT Callback type; must be invocable as `void()` or `void(TimerBase&)`.
 *
 * The callback signature is detected at compile time: if the functor accepts a
 * `TimerBase&` argument it receives a reference to this timer, otherwise it is
 * called with no arguments.
 */
AGNOCAST_PUBLIC
template <typename FunctorT>
class GenericTimer : public TimerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GenericTimer)

  GenericTimer(
    uint32_t timer_id, std::chrono::nanoseconds period, rclcpp::Clock::SharedPtr clock,
    FunctorT && callback)
  : TimerBase(timer_id, period),
    clock_(std::move(clock)),
    callback_(std::forward<FunctorT>(callback))
  {
    if (!clock_) {
      throw std::invalid_argument("clock cannot be null");
    }
  }

  void execute_callback() override
  {
    if constexpr (std::is_invocable_v<FunctorT, TimerBase &>) {
      callback_(*this);
    } else {
      callback_();
    }
  }

  /** @brief Return whether this timer uses a steady clock.
   *  @return True if the clock is steady. */
  AGNOCAST_PUBLIC
  bool is_steady() const override { return clock_->get_clock_type() == RCL_STEADY_TIME; }

  /** @brief Get the clock associated with this timer.
   *  @return Shared pointer to the clock. */
  AGNOCAST_PUBLIC
  rclcpp::Clock::SharedPtr get_clock() const override { return clock_; }

protected:
  RCLCPP_DISABLE_COPY(GenericTimer)

  rclcpp::Clock::SharedPtr clock_;
  FunctorT callback_;
};

/**
 * @brief Timer that uses a steady (wall) clock.
 *
 * @tparam FunctorT Callback type; must be invocable as `void()` or `void(TimerBase&)`.
 *
 * Convenience specialization of GenericTimer that automatically creates an
 * `RCL_STEADY_TIME` clock, suitable for wall-time periodic execution.
 */
AGNOCAST_PUBLIC
template <typename FunctorT>
class WallTimer : public GenericTimer<FunctorT>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(WallTimer)

  WallTimer(uint32_t timer_id, std::chrono::nanoseconds period, FunctorT && callback)
  : GenericTimer<FunctorT>(
      timer_id, period, std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME),
      std::forward<FunctorT>(callback))
  {
  }

protected:
  RCLCPP_DISABLE_COPY(WallTimer)
};

}  // namespace agnocast
