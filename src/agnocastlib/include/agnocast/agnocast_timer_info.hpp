#pragma once

#include "agnocast/agnocast_timer.hpp"
#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace agnocast
{

constexpr int64_t NANOSECONDS_PER_SECOND = 1000000000;

inline int64_t to_nanoseconds(const std::chrono::steady_clock::time_point & tp)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
}

struct TimerInfo
{
  ~TimerInfo();

  int timer_fd = -1;
  std::weak_ptr<TimerBase> timer;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  std::atomic<int64_t> last_call_time_ns;
  std::atomic<int64_t> next_call_time_ns;
  std::chrono::nanoseconds period;
  bool need_epoll_update = true;
};

extern std::mutex id2_timer_info_mtx;
extern std::unordered_map<uint32_t, std::shared_ptr<TimerInfo>> id2_timer_info;
extern std::atomic<uint32_t> next_timer_id;

int create_timer_fd(uint32_t timer_id, std::chrono::nanoseconds period);

void handle_timer_event(TimerInfo & timer_info);

uint32_t allocate_timer_id();

void register_timer_info(
  uint32_t timer_id, const std::shared_ptr<TimerBase> & timer, std::chrono::nanoseconds period,
  const rclcpp::CallbackGroup::SharedPtr & callback_group);

void unregister_timer_info(uint32_t timer_id);

}  // namespace agnocast
