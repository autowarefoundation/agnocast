#pragma once

#include "agnocast/agnocast_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <chrono>

namespace agnocast
{

class MultiThreadedAgnocastExecutor : public agnocast::AgnocastExecutor
{
  RCLCPP_DISABLE_COPY(MultiThreadedAgnocastExecutor)

  size_t number_of_ros2_threads_;
  size_t number_of_agnocast_threads_;

  bool yield_before_execute_;
  std::chrono::nanoseconds ros2_next_exec_timeout_;
  const int agnocast_next_exec_timeout_ms_;

  std::atomic<bool> worker_thread_failed_{false};

  void ros2_spin();
  void agnocast_spin();
  bool validate_callback_group(const rclcpp::CallbackGroup::SharedPtr & group) const override;

public:
  RCLCPP_PUBLIC
  explicit MultiThreadedAgnocastExecutor(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions(),
    size_t number_of_ros2_threads = 0, size_t number_of_agnocast_threads = 0,
    bool yield_before_execute = false,
    std::chrono::nanoseconds ros2_next_exec_timeout = std::chrono::nanoseconds(-1),
    int agnocast_next_exec_timeout_ms = 50);

  RCLCPP_PUBLIC
  void spin() override;
};

}  // namespace agnocast
