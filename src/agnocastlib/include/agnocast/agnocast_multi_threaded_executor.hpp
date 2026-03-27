#pragma once

#include "agnocast/agnocast_executor.hpp"
#include "agnocast/agnocast_public_api.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>

namespace agnocast
{

/** @brief Multi-threaded executor for Stage 1 with configurable thread counts for ROS 2 and
 * Agnocast callbacks. */
AGNOCAST_PUBLIC
class MultiThreadedAgnocastExecutor : public agnocast::AgnocastExecutor
{
  RCLCPP_DISABLE_COPY(MultiThreadedAgnocastExecutor)

  size_t number_of_ros2_threads_;
  size_t number_of_agnocast_threads_;

  bool yield_before_execute_;
  std::chrono::nanoseconds ros2_next_exec_timeout_;
  const int agnocast_next_exec_timeout_ms_;

  void ros2_spin();
  void agnocast_spin();
  bool validate_callback_group(const rclcpp::CallbackGroup::SharedPtr & group) const override;

public:
  /// Construct the executor.
  /// @param options Executor options.
  /// @param number_of_ros2_threads Number of threads for ROS 2 callbacks (0 = auto).
  /// @param number_of_agnocast_threads Number of threads for Agnocast callbacks (0 = auto).
  /// @param yield_before_execute If true, call std::this_thread::yield() before each callback
  /// execution to reduce CPU usage at the cost of latency.
  /// @param ros2_next_exec_timeout Timeout for ROS 2 executables.
  /// @param agnocast_next_exec_timeout_ms Timeout in ms for Agnocast executables.
  AGNOCAST_PUBLIC
  explicit MultiThreadedAgnocastExecutor(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions(),
    size_t number_of_ros2_threads = 0, size_t number_of_agnocast_threads = 0,
    bool yield_before_execute = false,
    std::chrono::nanoseconds ros2_next_exec_timeout = std::chrono::nanoseconds(-1),
    int agnocast_next_exec_timeout_ms = 50);

  /// Block the calling thread and process callbacks in a loop until rclcpp::shutdown() is called or
  /// the executor is cancelled.
  AGNOCAST_PUBLIC
  void spin() override;
};

}  // namespace agnocast
