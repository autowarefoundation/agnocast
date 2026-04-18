#pragma once

#include "agnocast/agnocast_callback_info.hpp"
#include "agnocast/agnocast_executor.hpp"
#include "agnocast/agnocast_public_api.hpp"
#include "rclcpp/rclcpp.hpp"

#include <unordered_set>

namespace agnocast
{

/** @brief Single-threaded executor for Stage 1 that processes both ROS 2 and Agnocast callbacks on
 * one thread. */
AGNOCAST_PUBLIC
class SingleThreadedAgnocastExecutor : public agnocast::AgnocastExecutor
{
  RCLCPP_DISABLE_COPY(SingleThreadedAgnocastExecutor)

  const int next_exec_timeout_ms_;
  bool validate_callback_group(const rclcpp::CallbackGroup::SharedPtr & group) const override;

  // Activate when this Executor is used for the implementation of a
  // CallbackIsolatedAgnocastExecutor.
  bool is_dedicated_to_one_callback_group_ = false;
  rclcpp::CallbackGroup::SharedPtr dedicated_callback_group_ = nullptr;

  std::unordered_set<const rclcpp::CallbackGroup *> warned_mixed_groups_;
  void warn_if_mixed_callback_groups();

public:
  /// Construct the executor.
  /// @param options Executor options.
  /// @param next_exec_timeout_ms Timeout in ms for waiting on the next executable.
  AGNOCAST_PUBLIC
  explicit SingleThreadedAgnocastExecutor(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions(),
    int next_exec_timeout_ms = 50);

  /// Block the calling thread and process callbacks in a loop until rclcpp::shutdown() is called or
  /// the executor is cancelled.
  AGNOCAST_PUBLIC
  void spin() override;

  // Not used for public API, but required to be exposed for the internal implementation
  void dedicate_to_callback_group(
    const rclcpp::CallbackGroup::SharedPtr & group,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node);
};

}  // namespace agnocast
