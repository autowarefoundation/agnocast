#pragma once
#include "agnocast/agnocast_executor.hpp"
#include "agnocast/agnocast_public_api.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agnocast
{

/** @brief Callback-isolated executor for Stage 1. Assigns a dedicated thread to each callback
 * group, ensuring that callbacks in different groups never run concurrently on the same thread.
 * Handles both ROS 2 and Agnocast callbacks. */
AGNOCAST_PUBLIC
class CallbackIsolatedAgnocastExecutor : public rclcpp::Executor
{
  RCLCPP_DISABLE_COPY(CallbackIsolatedAgnocastExecutor)

  const int next_exec_timeout_ms_;
  const int monitor_polling_interval_ms_;

  // Nodes associated with this AgnocastCallbackIsolatedExecutor, appended by add_node() and removed
  // by remove_node()
  std::set<
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr,
    std::owner_less<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>>
    weak_nodes_ RCPPUTILS_TSA_GUARDED_BY(mutex_);

  // CallbackGroups (and their belonging nodes) associated with this
  // AgnocastCallbackIsolatedExecutor, appended by add_callback_group() and removed by
  // remove_callback_group()
  std::map<
    rclcpp::CallbackGroup::WeakPtr, rclcpp::node_interfaces::NodeBaseInterface::WeakPtr,
    std::owner_less<rclcpp::CallbackGroup::WeakPtr>>
    weak_groups_to_nodes_ RCPPUTILS_TSA_GUARDED_BY(mutex_);

  // Mutex to protect child_callback_groups_, weak_child_executors_, and child_threads_
  mutable std::mutex child_resources_mutex_;

  // Callback groups corresponding to each child executor, used by stop_callback_group()
  std::vector<rclcpp::CallbackGroup::WeakPtr> child_callback_groups_
    RCPPUTILS_TSA_GUARDED_BY(child_resources_mutex_);

  // Child executors created during spin()
  std::vector<rclcpp::Executor::WeakPtr> weak_child_executors_
    RCPPUTILS_TSA_GUARDED_BY(child_resources_mutex_);

  // Child threads created during spin()
  std::vector<std::thread> child_threads_ RCPPUTILS_TSA_GUARDED_BY(child_resources_mutex_);

  std::vector<rclcpp::CallbackGroup::WeakPtr> get_manually_added_callback_groups_internal() const
    RCPPUTILS_TSA_REQUIRES(mutex_);

  std::vector<rclcpp::CallbackGroup::WeakPtr>
  get_automatically_added_callback_groups_from_nodes_internal() const
    RCPPUTILS_TSA_REQUIRES(mutex_);

public:
  /// Construct the executor.
  /// @param options Executor options.
  /// @param next_exec_timeout_ms Timeout in ms for waiting on the next executable.
  /// @param monitor_polling_interval_ms Polling interval in ms for monitoring new callback groups.
  AGNOCAST_PUBLIC
  explicit CallbackIsolatedAgnocastExecutor(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions(),
    int next_exec_timeout_ms = 50, int monitor_polling_interval_ms = 100);

  /// Block the calling thread and process callbacks in a loop until rclcpp::shutdown() is called or
  /// the executor is cancelled.
  AGNOCAST_PUBLIC
  void spin() override;

  /// Request the executor to stop spinning. Causes the current or next spin() call to return.
  AGNOCAST_PUBLIC
  void cancel();

  /// Stop the child executor running the given callback group, join its thread, and remove it.
  /// If group_ptr is nullptr or not found, this is a no-op.
  void stop_callback_group(const rclcpp::CallbackGroup::SharedPtr & group_ptr);

  /// Add a callback group to this executor.
  /// @param group_ptr Callback group to add.
  /// @param node_ptr Node the group belongs to.
  /// @param notify If true, wake the executor so it picks up the change immediately.
  AGNOCAST_PUBLIC
  void add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true) override;

  /// Return all callback groups known to this executor.
  /// @return Vector of weak pointers to callback groups.
  AGNOCAST_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr> get_all_callback_groups() override;

  /// Return callback groups that were manually added.
  /// @return Vector of weak pointers to callback groups.
  AGNOCAST_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr> get_manually_added_callback_groups() override;

  /// Return callback groups automatically discovered from added nodes.
  /// @return Vector of weak pointers to callback groups.
  AGNOCAST_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr> get_automatically_added_callback_groups_from_nodes()
    override;

  /// Remove a callback group from this executor.
  /// @param group_ptr Callback group to remove.
  /// @param notify If true, wake the executor so it picks up the change immediately.
  AGNOCAST_PUBLIC
  void remove_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr, bool notify = true) override;

  /// Add a node to this executor.
  /// @param node_ptr Node to add.
  /// @param notify If true, wake the executor so it picks up the change immediately.
  AGNOCAST_PUBLIC
  void add_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true) override;

  /// Add a node to this executor.
  /// @param node_ptr Node to add.
  /// @param notify If true, wake the executor so it picks up the change immediately.
  AGNOCAST_PUBLIC
  void add_node(rclcpp::Node::SharedPtr node_ptr, bool notify = true) override;

  /// Remove a node from this executor.
  /// @param node_ptr Node to remove.
  /// @param notify If true, wake the executor so it picks up the change immediately.
  AGNOCAST_PUBLIC
  void remove_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true) override;

  /// Remove a node from this executor.
  /// @param node_ptr Node to remove.
  /// @param notify If true, wake the executor so it picks up the change immediately.
  AGNOCAST_PUBLIC
  void remove_node(rclcpp::Node::SharedPtr node_ptr, bool notify = true) override;
};

}  // namespace agnocast
