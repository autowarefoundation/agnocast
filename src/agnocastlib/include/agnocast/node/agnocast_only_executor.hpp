#pragma once

#include "rclcpp/callback_group.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rcpputils/thread_safety_annotations.hpp"

#include <atomic>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace agnocast
{

using WeakCallbackGroupsToNodesMap = std::map<
  rclcpp::CallbackGroup::WeakPtr, rclcpp::node_interfaces::NodeBaseInterface::WeakPtr,
  std::owner_less<rclcpp::CallbackGroup::WeakPtr>>;

struct AgnocastExecutable;
class Node;

class AgnocastOnlyExecutor
{
protected:
  std::atomic_bool spinning_;
  int epoll_fd_;
  int shutdown_event_fd_;
  pid_t my_pid_;

  // Lock ordering: When both mutexes are needed, always acquire
  // ready_agnocast_executables_mutex_ before mutex_ to prevent deadlocks.
  std::mutex ready_agnocast_executables_mutex_;
  std::vector<AgnocastExecutable> ready_agnocast_executables_;

  mutable std::mutex mutex_;
  WeakCallbackGroupsToNodesMap weak_groups_associated_with_executor_to_nodes_
    RCPPUTILS_TSA_GUARDED_BY(mutex_);
  WeakCallbackGroupsToNodesMap weak_groups_to_nodes_associated_with_executor_
    RCPPUTILS_TSA_GUARDED_BY(mutex_);
  std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> weak_nodes_
    RCPPUTILS_TSA_GUARDED_BY(mutex_);

  bool get_next_agnocast_executable(
    AgnocastExecutable & agnocast_executable, const int timeout_ms, bool & shutdown_detected);
  bool get_next_ready_agnocast_executable(AgnocastExecutable & agnocast_executable);
  void execute_agnocast_executable(AgnocastExecutable & agnocast_executable);

  /// Returns true if the callback group is associated with this executor.
  /// Returns false if no groups or nodes have been explicitly associated.
  bool is_callback_group_associated(const rclcpp::CallbackGroup::SharedPtr & group);

  /// Discover callback groups created on associated nodes after add_node() was called.
  void add_callback_groups_from_nodes_associated_to_executor();

public:
  explicit AgnocastOnlyExecutor();

  virtual ~AgnocastOnlyExecutor();

  virtual void spin() = 0;
  void cancel();

  void add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  void remove_callback_group(rclcpp::CallbackGroup::SharedPtr group_ptr, bool notify = true);

  std::vector<rclcpp::CallbackGroup::WeakPtr> get_all_callback_groups();

  std::vector<rclcpp::CallbackGroup::WeakPtr> get_manually_added_callback_groups();

  std::vector<rclcpp::CallbackGroup::WeakPtr> get_automatically_added_callback_groups_from_nodes();

  void add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  void add_node(const std::shared_ptr<agnocast::Node> & node, bool notify = true);

  void remove_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  void remove_node(const std::shared_ptr<agnocast::Node> & node, bool notify = true);
};

}  // namespace agnocast
