#include <agnocast/agnocast.hpp>
#include <agnocast/agnocast_callback_isolated_executor.hpp>
#include <agnocast/agnocast_single_threaded_executor.hpp>
#include <cie_thread_configurator/cie_thread_configurator.hpp>
#include <rclcpp/rclcpp.hpp>

namespace agnocast
{

CallbackIsolatedAgnocastExecutor::CallbackIsolatedAgnocastExecutor(
  const rclcpp::ExecutorOptions & options, int next_exec_timeout_ms)
: rclcpp::Executor(options), next_exec_timeout_ms_(next_exec_timeout_ms)
{
}

void CallbackIsolatedAgnocastExecutor::spin()
{
  if (spinning.exchange(true)) {
    RCLCPP_ERROR(logger, "spin() called while already spinning");
    if (agnocast_fd != -1) {
      close(agnocast_fd);
    }
    exit(EXIT_FAILURE);
  }

  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false););

  std::vector<std::thread> threads;
  std::vector<std::pair<
    rclcpp::CallbackGroup::SharedPtr, rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>>
    groups_and_nodes;

  {
    std::lock_guard<std::mutex> guard{mutex_};

    for (const auto & weak_group_to_node : weak_groups_to_nodes_) {
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
  auto client_publisher = cie_thread_configurator::create_client_publisher();
  threads.reserve(groups_and_nodes.size());

  {
    std::lock_guard<std::mutex> guard{weak_child_executors_mutex_};
    for (auto & [group, node] : groups_and_nodes) {
      std::shared_ptr<rclcpp::Executor> executor;
      auto agnocast_topics = agnocast::get_agnocast_topics_by_group(group);
      auto callback_group_id =
        cie_thread_configurator::create_callback_group_id(group, node, agnocast_topics);

      if (agnocast_topics.empty()) {
        executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        std::static_pointer_cast<rclcpp::executors::SingleThreadedExecutor>(executor)
          ->add_callback_group(group, node);
      } else {
        executor = std::make_shared<SingleThreadedAgnocastExecutor>(
          rclcpp::ExecutorOptions{}, next_exec_timeout_ms_);
        std::static_pointer_cast<SingleThreadedAgnocastExecutor>(executor)
          ->dedicate_to_callback_group(group, node);
      }

      weak_child_executors_.push_back(executor);

      threads.emplace_back([executor, callback_group_id = std::move(callback_group_id),
                            &client_publisher, &client_publisher_mutex]() {
        auto tid = static_cast<pid_t>(syscall(SYS_gettid));

        {
          std::lock_guard<std::mutex> lock{client_publisher_mutex};
          cie_thread_configurator::publish_callback_group_info(
            client_publisher, tid, callback_group_id);
        }

        executor->spin();
      });
    }
  }  // guard weak_child_executors_mutex_

  for (auto & thread : threads) {
    if (thread.joinable()) {
      thread.join();
    }
  }
}

void CallbackIsolatedAgnocastExecutor::add_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  (void)notify;

  std::weak_ptr<rclcpp::CallbackGroup> weak_group_ptr = group_ptr;
  std::weak_ptr<rclcpp::node_interfaces::NodeBaseInterface> weak_node_ptr = node_ptr;

  std::lock_guard<std::mutex> guard{mutex_};

  // Confirm that group_ptr does not refer to any of the callback groups held by nodes in
  // weak_nodes_.
  for (const auto & weak_node : weak_nodes_) {
    auto n = weak_node.lock();

    if (!n) {
      continue;
    }

    if (n->callback_group_in_node(group_ptr)) {
      RCLCPP_ERROR(
        logger, "Callback group already exists in node: %s", n->get_fully_qualified_name());
      if (agnocast_fd != -1) {
        close(agnocast_fd);
      }
      exit(EXIT_FAILURE);
    }
  }

  auto insert_info = weak_groups_to_nodes_.insert(std::make_pair(weak_group_ptr, weak_node_ptr));

  if (!insert_info.second) {
    RCLCPP_ERROR(logger, "Callback group already exists in the executor");
    if (agnocast_fd != -1) {
      close(agnocast_fd);
    }
    exit(EXIT_FAILURE);
  }
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
CallbackIsolatedAgnocastExecutor::get_all_callback_groups()
{
  std::lock_guard<std::mutex> guard{mutex_};
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;

  auto manually_added_groups = get_manually_added_callback_groups_internal();
  auto automatically_added_groups = get_automatically_added_callback_groups_from_nodes_internal();

  groups.reserve(manually_added_groups.size() + automatically_added_groups.size());
  groups.insert(groups.end(), manually_added_groups.begin(), manually_added_groups.end());
  groups.insert(groups.end(), automatically_added_groups.begin(), automatically_added_groups.end());

  return groups;
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
CallbackIsolatedAgnocastExecutor::get_manually_added_callback_groups()
{
  std::lock_guard<std::mutex> guard{mutex_};
  return get_manually_added_callback_groups_internal();
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
CallbackIsolatedAgnocastExecutor::get_manually_added_callback_groups_internal() const
{
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;

  for (const auto & weak_group_to_node : weak_groups_to_nodes_) {
    auto group = weak_group_to_node.first.lock();
    if (group) {
      groups.push_back(group);
    }
  }

  return groups;
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
CallbackIsolatedAgnocastExecutor::get_automatically_added_callback_groups_from_nodes()
{
  std::lock_guard<std::mutex> guard{mutex_};
  return get_automatically_added_callback_groups_from_nodes_internal();
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
CallbackIsolatedAgnocastExecutor::get_automatically_added_callback_groups_from_nodes_internal()
  const
{
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;

  for (const auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();

    if (!node) {
      continue;
    }

    node->for_each_callback_group([&groups](const rclcpp::CallbackGroup::SharedPtr & group) {
      if (group && group->automatically_add_to_executor_with_node()) {
        groups.push_back(group);
      }
    });
  }

  return groups;
}

void CallbackIsolatedAgnocastExecutor::remove_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr, bool notify)
{
  (void)notify;

  std::lock_guard<std::mutex> guard{mutex_};

  auto it = weak_groups_to_nodes_.find(group_ptr);

  if (it != weak_groups_to_nodes_.end()) {
    weak_groups_to_nodes_.erase(it);
  } else {
    RCLCPP_ERROR(logger, "Callback group not found in the executor");
    if (agnocast_fd != -1) {
      close(agnocast_fd);
    }
    exit(EXIT_FAILURE);
  }
}

void CallbackIsolatedAgnocastExecutor::add_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  (void)notify;

  std::lock_guard<std::mutex> guard{mutex_};

  // Confirm that any callback group in weak_groups_to_nodes_ does not refer to any of the callback
  // groups held by node_ptr.
  for (const auto & weak_group_to_node : weak_groups_to_nodes_) {
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

  std::weak_ptr<rclcpp::node_interfaces::NodeBaseInterface> weak_node_ptr = node_ptr;

  auto insert_info = weak_nodes_.insert(weak_node_ptr);
  if (!insert_info.second) {
    RCLCPP_ERROR(
      logger, "Node already exists in the executor: %s", node_ptr->get_fully_qualified_name());
    if (agnocast_fd != -1) {
      close(agnocast_fd);
    }
    exit(EXIT_FAILURE);
  }
}

void CallbackIsolatedAgnocastExecutor::add_node(rclcpp::Node::SharedPtr node_ptr, bool notify)
{
  (void)notify;
  add_node(node_ptr->get_node_base_interface(), notify);
}

void CallbackIsolatedAgnocastExecutor::remove_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  (void)notify;

  std::weak_ptr<rclcpp::node_interfaces::NodeBaseInterface> weak_node_ptr = node_ptr;
  std::lock_guard<std::mutex> guard{mutex_};

  auto it = weak_nodes_.find(weak_node_ptr);

  if (it != weak_nodes_.end()) {
    weak_nodes_.erase(it);
  } else {
    RCLCPP_ERROR(
      logger, "Node not found in the executor: %s", node_ptr->get_fully_qualified_name());
    if (agnocast_fd != -1) {
      close(agnocast_fd);
    }
    exit(EXIT_FAILURE);
  }
}

void CallbackIsolatedAgnocastExecutor::remove_node(rclcpp::Node::SharedPtr node_ptr, bool notify)
{
  remove_node(node_ptr->get_node_base_interface(), notify);
}

void CallbackIsolatedAgnocastExecutor::cancel()
{
  spinning.store(false);
  std::lock_guard<std::mutex> guard{weak_child_executors_mutex_};
  for (auto & weak_child_executor : weak_child_executors_) {
    if (auto child_executor = weak_child_executor.lock()) {
      child_executor->cancel();
    }
  }
  weak_child_executors_.clear();
}

}  // namespace agnocast
