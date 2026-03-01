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
    throw std::invalid_argument("Callback group is nullptr. The node may have been destructed.");
  }

  if (is_dedicated_to_one_callback_group_) {
    return group == dedicated_callback_group_;
  }

  return true;
}

void SingleThreadedAgnocastExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::logic_error("spin() called while already spinning");
  }

  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false););

  while (rclcpp::ok(this->context_) && spinning.load()) {
    if (need_epoll_updates.load()) {
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
    throw std::invalid_argument("The passed callback group is nullptr.");
  }

  is_dedicated_to_one_callback_group_ = true;
  dedicated_callback_group_ = group;

  add_callback_group(group, node);
}

}  // namespace agnocast
