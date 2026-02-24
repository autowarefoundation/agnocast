#pragma once

#include "agnocast/agnocast_timer.hpp"
#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>

namespace agnocast
{

constexpr int64_t NANOSECONDS_PER_SECOND = 1000000000;

struct TimerInfo
{
  ~TimerInfo();

  // Mutex to protect timer_fd access.
  // Use shared_lock for read operations (read(), epoll_ctl()).
  // Use unique_lock for write operations (close(), create_timer_fd()).
  std::shared_mutex fd_mutex;

  uint32_t timer_id = 0;
  int timer_fd = -1;
  int clock_eventfd = -1;  // eventfd to wake epoll on clock updates (ROS_TIME only)
  std::weak_ptr<TimerBase> timer;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  std::atomic<int64_t> last_call_time_ns;
  std::atomic<int64_t> next_call_time_ns;
  std::atomic<int64_t> time_credit{
    0};  // Credit for time elapsed before ROS time is activated/deactivated
  std::chrono::nanoseconds period;
  bool need_epoll_update = true;

  rclcpp::Clock::SharedPtr clock;
  rclcpp::JumpHandler::SharedPtr jump_handler;
};

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

}  // namespace agnocast
