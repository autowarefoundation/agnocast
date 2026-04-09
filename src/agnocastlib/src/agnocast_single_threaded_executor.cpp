#include "agnocast/agnocast_single_threaded_executor.hpp"

#include "agnocast/agnocast.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sys/epoll.h"

namespace agnocast
{

SingleThreadedAgnocastExecutor::SingleThreadedAgnocastExecutor(
  const rclcpp::ExecutorOptions & options, int next_exec_timeout_ms)
: agnocast::AgnocastExecutor(options), next_exec_timeout_ms_(next_exec_timeout_ms)
{
  TRACEPOINT(
    agnocast_construct_executor, static_cast<const void *>(this),
    "agnocast_single_threaded_executor");

  const int next_exec_timeout_ms_threshold = 500;  // Rough value
  if (next_exec_timeout_ms_ > next_exec_timeout_ms_threshold) {
    RCLCPP_WARN(
      logger,
      "Due to the large next_exec_timeout_ms value, the callbacks registered after spin and ROS 2 "
      "callbacks may be extremely slow to execute.");
  }
}

bool SingleThreadedAgnocastExecutor::validate_callback_group(
  const rclcpp::CallbackGroup::SharedPtr & group) const
{
  if (!group) {
    RCLCPP_ERROR(logger, "Callback group is nullptr. The node may have been destructed.");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  if (is_dedicated_to_one_callback_group_) {
    return group == dedicated_callback_group_;
  }

  return true;
}

void SingleThreadedAgnocastExecutor::spin()
{
  if (spinning.exchange(true)) {
    RCLCPP_ERROR(logger, "spin() called while already spinning");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false););

  while (rclcpp::ok(this->context_) && spinning.load()) {
    if (epoll_update_tracker_.take_update_request()) {
      prepare_epoll();
    }

    agnocast::AgnocastExecutable agnocast_executable;
    if (get_next_agnocast_executable(
          agnocast_executable, next_exec_timeout_ms_ /*timed-blocking*/)) {
      execute_agnocast_executable(agnocast_executable);
    }

    rclcpp::AnyExecutable any_executable;
    if (get_next_executable(any_executable, std::chrono::nanoseconds(0) /*non-blocking*/)) {
      execute_any_executable(any_executable);
    }
  }
}

void SingleThreadedAgnocastExecutor::dedicate_to_callback_group(
  const rclcpp::CallbackGroup::SharedPtr & group,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node)
{
  if (!group) {
    RCLCPP_ERROR(logger, "The passed callback group is nullptr.");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  is_dedicated_to_one_callback_group_ = true;
  dedicated_callback_group_ = group;

  add_callback_group(group, node);
}

}  // namespace agnocast
