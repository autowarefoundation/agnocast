#pragma once

#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/agnocast_only_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace agnocast::node_interfaces
{
class NodeBase;
}  // namespace agnocast::node_interfaces

namespace agnocast
{

class Node;
class AgnocastOnlySingleThreadedExecutor;

class AgnocastOnlyCallbackIsolatedExecutor : public AgnocastOnlyExecutor
{
  RCLCPP_DISABLE_COPY(AgnocastOnlyCallbackIsolatedExecutor)

  const int next_exec_timeout_ms_;

  // Condition variable notified when a new callback group is created on any registered node.
  std::condition_variable callback_group_created_cv_;
  std::mutex callback_group_created_cv_mutex_;
  bool callback_group_created_{false};

  // Mutex to protect weak_child_executors_ and child_threads_
  mutable std::mutex child_resources_mutex_;

  // Child executors created during spin()
  std::vector<std::weak_ptr<AgnocastOnlyExecutor>> weak_child_executors_
    RCPPUTILS_TSA_GUARDED_BY(child_resources_mutex_);

  // Child threads created during spin()
  std::vector<std::thread> child_threads_ RCPPUTILS_TSA_GUARDED_BY(child_resources_mutex_);

  // Nodes that have registered callback-group-created callbacks, used for cleanup.
  std::vector<std::weak_ptr<agnocast::node_interfaces::NodeBase>> registered_agnocast_nodes_
    RCPPUTILS_TSA_GUARDED_BY(mutex_);

public:
  RCLCPP_PUBLIC
  explicit AgnocastOnlyCallbackIsolatedExecutor(int next_exec_timeout_ms = 50);

  RCLCPP_PUBLIC
  ~AgnocastOnlyCallbackIsolatedExecutor() override;

  RCLCPP_PUBLIC
  void spin() override;

  RCLCPP_PUBLIC
  void cancel() override;

  /// Add a node to this executor. Unlike the base class add_node(), this does NOT set
  /// the has_executor atomic flag on the node or its callback groups, because the CIE
  /// distributes callback groups to child executors which claim ownership individually.
  RCLCPP_PUBLIC
  void add_node(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_ptr, bool notify = false);

  RCLCPP_PUBLIC
  void add_node(const agnocast::Node::SharedPtr & node_ptr, bool notify = false);
};

}  // namespace agnocast
