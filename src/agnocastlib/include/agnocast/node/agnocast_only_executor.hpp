#pragma once

#include "agnocast/agnocast_epoll.hpp"
#include "agnocast/agnocast_epoll_update_dispatcher.hpp"
#include "agnocast/agnocast_public_api.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rcpputils/thread_safety_annotations.hpp"

#include <atomic>
#include <chrono>
#include <future>
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

/** @brief Base class for Stage 2 executors that handle only Agnocast callbacks (no RMW). Used with
 * agnocast::Node. */
AGNOCAST_PUBLIC
class AgnocastOnlyExecutor
{
protected:
  std::atomic_bool spinning_;
  std::unique_ptr<EpollManager> epoll_manager_;
  int shutdown_event_fd_;
  pid_t my_pid_;

  EpollUpdateTracker epoll_update_tracker_;

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
    AgnocastExecutable & agnocast_executable,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  bool get_next_agnocast_executable(AgnocastExecutable & agnocast_executable, const int timeout_ms);
  bool get_next_ready_agnocast_executable(AgnocastExecutable & agnocast_executable);
  void execute_agnocast_executable(AgnocastExecutable & agnocast_executable);

  bool is_callback_group_associated(const rclcpp::CallbackGroup::SharedPtr & group);

  void add_callback_groups_from_nodes_associated_to_executor();

  virtual rclcpp::FutureReturnCode spin_until_future_complete_impl(
    std::chrono::nanoseconds timeout,
    const std::function<std::future_status(std::chrono::nanoseconds wait_time)> & wait_for_future);

  virtual void spin_once_impl(std::chrono::nanoseconds timeout);

  void wait_for_work(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

public:
  /// Construct the executor.
  AGNOCAST_PUBLIC
  explicit AgnocastOnlyExecutor();

  virtual ~AgnocastOnlyExecutor();

  /// Block the calling thread and process Agnocast callbacks in a loop until cancel() is called.
  AGNOCAST_PUBLIC
  virtual void spin() = 0;

  /// Request the executor to stop spinning. Causes the current or next spin() call to return.
  AGNOCAST_PUBLIC
  virtual void cancel();

  /// Add a callback group to this executor.
  /// @param group_ptr Callback group to add.
  /// @param node_ptr Node the group belongs to.
  /// @param notify If true, wake the executor so it picks up the change immediately.
  AGNOCAST_PUBLIC
  void add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  /// Remove a callback group from this executor.
  /// @param group_ptr Callback group to remove.
  /// @param notify If true, wake the executor so it picks up the change immediately.
  AGNOCAST_PUBLIC
  void remove_callback_group(rclcpp::CallbackGroup::SharedPtr group_ptr, bool notify = true);

  /// Return all callback groups known to this executor.
  /// @return Vector of weak pointers to callback groups.
  AGNOCAST_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr> get_all_callback_groups();

  /// Return callback groups that were manually added.
  /// @return Vector of weak pointers to callback groups.
  AGNOCAST_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr> get_manually_added_callback_groups();

  /// Return callback groups automatically discovered from added nodes.
  /// @return Vector of weak pointers to callback groups.
  AGNOCAST_PUBLIC
  std::vector<rclcpp::CallbackGroup::WeakPtr> get_automatically_added_callback_groups_from_nodes();

  /// Add a node to this executor.
  /// @param node_ptr Node to add.
  /// @param notify If true, wake the executor so it picks up the change immediately.
  AGNOCAST_PUBLIC
  void add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  /// Add a node to this executor.
  /// @param node Node to add.
  /// @param notify If true, wake the executor so it picks up the change immediately.
  AGNOCAST_PUBLIC
  void add_node(const std::shared_ptr<agnocast::Node> & node, bool notify = true);

  /// Remove a node from this executor.
  /// @param node_ptr Node to remove.
  /// @param notify If true, wake the executor so it picks up the change immediately.
  AGNOCAST_PUBLIC
  void remove_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  /// Remove a node from this executor.
  /// @param node Node to remove.
  /// @param notify If true, wake the executor so it picks up the change immediately.
  AGNOCAST_PUBLIC
  void remove_node(const std::shared_ptr<agnocast::Node> & node, bool notify = true);

  /// Collect work once and execute the next available work, optionally within a duration.
  /// @param timeout timeout The maximum amount of time to spend waiting for work. `-1` is
  /// potentially block forever waiting for work.
  AGNOCAST_PUBLIC
  virtual void spin_once(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// Spin (blocking) until the future is complete, it times out waiting, or rclcpp is interrupted.
  /// @param future The future to wait on. If this function returns SUCCESS, the future can be
  /// accessed without blocking (though it may still throw an exception).
  /// @param timeout Optional timeout parameter, which gets passed to Executor::spin_node_once. `-1`
  /// is block forever, `0` is non-blocking.
  /// @return The return code, one of `SUCCESS`, `INTERRUPTED`, or `TIMEOUT`.
  template <typename FutureT, typename TimeRepT = int64_t, typename TimeT = std::milli>
  rclcpp::FutureReturnCode spin_until_future_complete(
    const FutureT & future,
    std::chrono::duration<TimeRepT, TimeT> timeout = std::chrono::duration<TimeRepT, TimeT>(-1))
  {
    return spin_until_future_complete_impl(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout),
      [&future](std::chrono::nanoseconds wait_time) { return future.wait_for(wait_time); });
  }

  /// Returns true if the executor is currently spinning.
  /// @return True if the executor is currently spinning.
  AGNOCAST_PUBLIC
  bool is_spinning();
};

}  // namespace agnocast
