#pragma once

#include "agnocast/node/agnocast_node.hpp"
#include "agnocast/node/agnocast_only_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <mutex>
#include <vector>

namespace agnocast
{

class Node;
class AgnocastOnlySingleThreadedExecutor;

class AgnocastOnlyCallbackIsolatedExecutor : public AgnocastOnlyExecutor
{
  RCLCPP_DISABLE_COPY(AgnocastOnlyCallbackIsolatedExecutor)

  const int next_exec_timeout_ms_;

  // Mutex to protect weak_child_executors_
  mutable std::mutex weak_child_executors_mutex_;

  // Child executors created during spin()
  std::vector<std::weak_ptr<AgnocastOnlyExecutor>> weak_child_executors_
    RCPPUTILS_TSA_GUARDED_BY(weak_child_executors_mutex_);

public:
  RCLCPP_PUBLIC
  explicit AgnocastOnlyCallbackIsolatedExecutor(int next_exec_timeout_ms = 50);

  RCLCPP_PUBLIC
  void spin() override;

  RCLCPP_PUBLIC
  void cancel();

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
