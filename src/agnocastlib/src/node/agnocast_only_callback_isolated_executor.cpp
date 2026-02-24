#include "agnocast/node/agnocast_only_callback_isolated_executor.hpp"

#include "agnocast/agnocast.hpp"
#include "agnocast/cie_client_utils.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/agnocast_only_single_threaded_executor.hpp"
#include "agnocast/node/node_interfaces/node_base.hpp"

#include <sys/syscall.h>
#include <unistd.h>

namespace agnocast
{

AgnocastOnlyCallbackIsolatedExecutor::AgnocastOnlyCallbackIsolatedExecutor(
  int next_exec_timeout_ms, int monitor_polling_interval_ms)
: next_exec_timeout_ms_(next_exec_timeout_ms),
  monitor_polling_interval_ms_(monitor_polling_interval_ms > 0 ? monitor_polling_interval_ms : 1)
{
}

AgnocastOnlyCallbackIsolatedExecutor::~AgnocastOnlyCallbackIsolatedExecutor()
{
  std::lock_guard<std::mutex> guard{mutex_};
  for (auto & weak_node : registered_agnocast_nodes_) {
    if (auto node = weak_node.lock()) {
      node->set_on_callback_group_created({});
    }
  }
}

void AgnocastOnlyCallbackIsolatedExecutor::spin()
{
  if (spinning_.exchange(true)) {
    RCLCPP_ERROR(logger, "spin() called while already spinning");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  RCPPUTILS_SCOPE_EXIT(this->spinning_.store(false););

  std::vector<std::pair<
    rclcpp::CallbackGroup::SharedPtr, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>>
    groups_and_nodes;

  {
    std::lock_guard<std::mutex> guard{mutex_};

    // Traverse manually-added callback groups
    for (const auto & weak_group_to_node : weak_groups_associated_with_executor_to_nodes_) {
      auto group = weak_group_to_node.first.lock();
      if (!group) {
        continue;
      }

      auto node = weak_group_to_node.second.lock();
      if (!node) {
        continue;
      }

      groups_and_nodes.emplace_back(group, node);
    }

    // Traverse auto-added callback groups from nodes
    for (const auto & weak_node : weak_nodes_) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }

      node->for_each_callback_group(
        [&groups_and_nodes, node](const rclcpp::CallbackGroup::SharedPtr & group) {
          if (group && group->automatically_add_to_executor_with_node()) {
            groups_and_nodes.emplace_back(group, node);
          }
        });
    }
  }  // guard mutex_

  std::mutex client_publisher_mutex;
  auto client_publisher = agnocast::create_agnocast_client_publisher();

  // Note: spawn_child_executor must be called while holding child_resources_mutex_.
  auto spawn_child_executor =
    [this, &client_publisher, &client_publisher_mutex](
      const rclcpp::CallbackGroup::SharedPtr & group,
      const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node) {
      auto agnocast_topics = agnocast::get_agnocast_topics_by_group(group);
      auto callback_group_id = agnocast::create_callback_group_id(group, node, agnocast_topics);

      auto agnocast_executor =
        std::make_shared<AgnocastOnlySingleThreadedExecutor>(next_exec_timeout_ms_);
      agnocast_executor->add_callback_group(group, node);

      weak_child_executors_.push_back(agnocast_executor);

      child_threads_.emplace_back([executor = std::move(agnocast_executor),
                                   callback_group_id = std::move(callback_group_id),
                                   &client_publisher, &client_publisher_mutex]() {
        auto tid = static_cast<pid_t>(syscall(SYS_gettid));

        {
          std::lock_guard<std::mutex> lock{client_publisher_mutex};
          agnocast::publish_callback_group_info(client_publisher, tid, callback_group_id);
        }

        executor->spin();
      });
    };

  {
    std::lock_guard<std::mutex> guard{child_resources_mutex_};
    if (!spinning_.load()) {
      return;
    }
    for (auto & [group, node] : groups_and_nodes) {
      spawn_child_executor(group, node);
    }
  }  // guard child_resources_mutex_

  // Monitoring loop: wait for notification when new callback groups are created
  while (spinning_.load() && rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lock(callback_group_created_cv_mutex_);
      callback_group_created_cv_.wait_for(
        lock, std::chrono::milliseconds(monitor_polling_interval_ms_));
    }

    if (!spinning_.load() || !rclcpp::ok()) {
      break;
    }

    std::vector<std::pair<
      rclcpp::CallbackGroup::SharedPtr, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>>
      new_groups;

    {
      std::lock_guard<std::mutex> guard{mutex_};
      for (const auto & weak_node : weak_nodes_) {
        auto node = weak_node.lock();
        if (!node) {
          continue;
        }

        node->for_each_callback_group(
          [&new_groups, &node](const rclcpp::CallbackGroup::SharedPtr & group) {
            if (
              group && group->automatically_add_to_executor_with_node() &&
              !group->get_associated_with_executor_atomic().load()) {
              new_groups.emplace_back(group, node);
            }
          });
      }
    }

    if (new_groups.empty()) {
      continue;
    }

    std::lock_guard<std::mutex> guard{child_resources_mutex_};
    if (!spinning_.load() || !rclcpp::ok()) {
      break;
    }
    for (auto & [group, node] : new_groups) {
      if (group->get_associated_with_executor_atomic().load()) {
        continue;
      }
      spawn_child_executor(group, node);
    }
  }

  // Join all child threads after monitoring loop exits
  std::lock_guard<std::mutex> guard{child_resources_mutex_};
  for (auto & weak_child_executor : weak_child_executors_) {
    if (auto child_executor = weak_child_executor.lock()) {
      child_executor->cancel();
    }
  }
  for (auto & thread : child_threads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
  child_threads_.clear();
  weak_child_executors_.clear();
}

void AgnocastOnlyCallbackIsolatedExecutor::cancel()
{
  spinning_.store(false);
  callback_group_created_cv_.notify_one();
  std::lock_guard<std::mutex> guard{child_resources_mutex_};
  for (auto & weak_child_executor : weak_child_executors_) {
    if (auto child_executor = weak_child_executor.lock()) {
      child_executor->cancel();
    }
  }
}

void AgnocastOnlyCallbackIsolatedExecutor::add_node(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_ptr, bool notify)
{
  (void)notify;

  std::lock_guard<std::mutex> guard{mutex_};

  // Confirm that any callback group in weak_groups_associated_with_executor_to_nodes_ does not
  // refer to any of the callback groups held by node_ptr.
  for (const auto & weak_group_to_node : weak_groups_associated_with_executor_to_nodes_) {
    auto group = weak_group_to_node.first.lock();

    if (!group) {
      continue;
    }

    if (node_ptr->callback_group_in_node(group)) {
      RCLCPP_ERROR(
        logger, "One of the callback groups in node %s already exists in the executor.",
        node_ptr->get_fully_qualified_name());
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }
  }

  for (const auto & weak_node : weak_nodes_) {
    if (weak_node.lock() == node_ptr) {
      RCLCPP_ERROR(
        logger, "Node already exists in the executor: %s", node_ptr->get_fully_qualified_name());
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }
  }

  weak_nodes_.push_back(node_ptr);

  auto agnocast_node_base =
    std::dynamic_pointer_cast<agnocast::node_interfaces::NodeBase>(node_ptr);
  if (agnocast_node_base) {
    agnocast_node_base->set_on_callback_group_created(
      [this]() { callback_group_created_cv_.notify_one(); });
    registered_agnocast_nodes_.push_back(agnocast_node_base);
  }
}

void AgnocastOnlyCallbackIsolatedExecutor::add_node(
  const agnocast::Node::SharedPtr & node_ptr, bool notify)
{
  add_node(node_ptr->get_node_base_interface(), notify);
}

}  // namespace agnocast
