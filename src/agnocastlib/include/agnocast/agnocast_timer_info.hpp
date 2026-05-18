#pragma once

#include "agnocast/agnocast_epoll.hpp"
#include "agnocast/agnocast_timer.hpp"
#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>

namespace agnocast
{

constexpr int64_t NANOSECONDS_PER_SECOND = 1000000000;
// Capped slightly below UINT32_MAX to provide a safe margin against
// atomic wrap-around (overflow back to 0) during concurrent fetch_add calls.
constexpr uint32_t MAX_TIMER_ID_SAFETY_MARGIN = 1000;
constexpr uint32_t MAX_TIMER_ID = UINT32_MAX - MAX_TIMER_ID_SAFETY_MARGIN;

struct AgnocastExecutable;

struct TimerInfo
{
  ~TimerInfo();

  void reset();

  // Implementation of TimerBase::set_period; see that for semantics.
  void set_period(std::chrono::nanoseconds new_period);

  // Mutex to protect timer_fd access.
  // - shared_lock: for reading timer_fd (read(), epoll_ctl()).
  // - unique_lock: for writing timer_fd (close()).
  std::shared_mutex fd_mutex;

  uint32_t timer_id = 0;
  int timer_fd = -1;
  bool timer_fd_need_update = false;
  int clock_eventfd = -1;  // eventfd to wake epoll on clock updates (ROS_TIME only)
  bool clock_eventfd_need_update = false;
  std::weak_ptr<TimerBase> timer;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  std::atomic<int64_t> last_call_time_ns;
  std::atomic<int64_t> next_call_time_ns;
  std::atomic<int64_t> time_credit{0};  // Credit for time elapsed before ROS time is activated
  std::atomic<int64_t> period_ns;
  bool need_epoll_update = true;

  rclcpp::Clock::SharedPtr clock;
  rclcpp::JumpHandler::SharedPtr jump_handler;
};

// Lock ordering: when acquiring both id2_callback_info_mtx and id2_timer_info_mtx,
// always lock id2_callback_info_mtx first to avoid deadlocks.
extern std::mutex id2_timer_info_mtx;
extern std::unordered_map<uint32_t, std::shared_ptr<TimerInfo>> id2_timer_info;
extern std::atomic<uint32_t> next_timer_id;

int create_timer_fd(
  uint32_t timer_id, std::chrono::nanoseconds period, rcl_clock_type_t clock_type);

void handle_timer_event(TimerInfo & timer_info);

uint32_t allocate_timer_id();

void register_timer_info(
  uint32_t timer_id, const std::shared_ptr<TimerBase> & timer, std::chrono::nanoseconds period,
  const rclcpp::CallbackGroup::SharedPtr & callback_group, const rclcpp::Clock::SharedPtr & clock);

void unregister_timer_info(uint32_t timer_id);

class TimerEventHandler : public EpollEventHandler
{
  pid_t my_pid_;
  std::mutex * ready_agnocast_executables_mutex_;
  std::vector<AgnocastExecutable> * ready_agnocast_executables_;

public:
  TimerEventHandler(
    const pid_t my_pid, std::mutex * ready_agnocast_executables_mutex,
    std::vector<AgnocastExecutable> * ready_agnocast_executables)
  : my_pid_(my_pid),
    ready_agnocast_executables_mutex_(ready_agnocast_executables_mutex),
    ready_agnocast_executables_(ready_agnocast_executables)
  {
  }

  [[nodiscard]] EpollEventType get_type() const override { return EpollEventType::Timer; }

  void prepare_epoll(int epoll_fd, const CallbackGroupValidator & validate_callback_group) override;

  void handle(EpollEventLocalID event_local_id) override;
};

class ClockEventHandler : public EpollEventHandler
{
  pid_t my_pid_;
  std::mutex * ready_agnocast_executables_mutex_;
  std::vector<AgnocastExecutable> * ready_agnocast_executables_;

public:
  ClockEventHandler(
    const pid_t my_pid, std::mutex * ready_agnocast_executables_mutex,
    std::vector<AgnocastExecutable> * ready_agnocast_executables)
  : my_pid_(my_pid),
    ready_agnocast_executables_mutex_(ready_agnocast_executables_mutex),
    ready_agnocast_executables_(ready_agnocast_executables)
  {
  }

  [[nodiscard]] EpollEventType get_type() const override { return EpollEventType::Clock; }

  void prepare_epoll(int epoll_fd, const CallbackGroupValidator & validate_callback_group) override;

  void handle(EpollEventLocalID event_local_id) override;
};

}  // namespace agnocast
