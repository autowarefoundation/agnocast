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
    if (need_epoll_updates.load()) {
      prepare_epoll();
      warn_if_mixed_callback_groups();
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

void SingleThreadedAgnocastExecutor::warn_if_mixed_callback_groups()
{
  if (is_dedicated_to_one_callback_group_) {
    return;
  }

  // Collect callback groups used by agnocast callbacks
  std::set<rclcpp::CallbackGroup::SharedPtr> agnocast_groups;

  {
    std::lock_guard<std::mutex> lock(id2_callback_info_mtx);
    for (const auto & [id, info] : id2_callback_info) {
      if (info.callback_group) {
        agnocast_groups.insert(info.callback_group);
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(id2_timer_info_mtx);
    for (const auto & [id, info] : id2_timer_info) {
      if (info->callback_group) {
        agnocast_groups.insert(info->callback_group);
      }
    }
  }

  // Filter out already-warned groups after releasing locks
  for (auto it = agnocast_groups.begin(); it != agnocast_groups.end();) {
    if (warned_mixed_groups_.count(it->get())) {
      it = agnocast_groups.erase(it);
    } else {
      ++it;
    }
  }

  for (const auto & group : agnocast_groups) {
    bool has_ros_callback = false;
    group->collect_all_ptrs(
      [&has_ros_callback](const rclcpp::SubscriptionBase::SharedPtr &) { has_ros_callback = true; },
      [&has_ros_callback](const rclcpp::ServiceBase::SharedPtr &) { has_ros_callback = true; },
      [&has_ros_callback](const rclcpp::ClientBase::SharedPtr &) { has_ros_callback = true; },
      [&has_ros_callback](const rclcpp::TimerBase::SharedPtr &) { has_ros_callback = true; },
      [&has_ros_callback](const rclcpp::Waitable::SharedPtr &) { has_ros_callback = true; });

    if (has_ros_callback) {
      warned_mixed_groups_.insert(group.get());
      auto agnocast_topics = get_agnocast_topics_by_group(group);
      std::string agnocast_entities_str = "Agnocast callbacks";
      if (!agnocast_topics.empty()) {
        std::string topics_str;
        for (const auto & t : agnocast_topics) {
          if (!topics_str.empty()) topics_str += ", ";
          topics_str += t;
        }
        agnocast_entities_str += " (topics: [" + topics_str + "])";
      }
      RCLCPP_WARN(
        logger,
        "A callback group (%s) contains both ROS 2 and Agnocast callbacks. "
        "In SingleThreadedAgnocastExecutor, the Agnocast timed wait may block "
        "ROS 2 callbacks in the same callback group every spin iteration. "
        "Consider separating them into different callback groups.",
        agnocast_entities_str.c_str());
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
