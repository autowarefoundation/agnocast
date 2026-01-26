#include "agnocast/agnocast_executor.hpp"

#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_tracepoint_wrapper.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"
#include "sys/epoll.h"

#include <algorithm>

namespace agnocast
{

AgnocastExecutor::AgnocastExecutor(const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options), epoll_fd_(epoll_create1(0)), my_pid_(getpid())
{
  if (epoll_fd_ == -1) {
    RCLCPP_ERROR(logger, "epoll_create1 failed: %s", strerror(errno));
    exit(EXIT_FAILURE);
  }
}

AgnocastExecutor::~AgnocastExecutor()
{
  close(epoll_fd_);
}

void AgnocastExecutor::prepare_epoll()
{
  agnocast::prepare_epoll_impl(
    epoll_fd_, my_pid_, ready_agnocast_executables_mutex_, ready_agnocast_executables_,
    [this](const rclcpp::CallbackGroup::SharedPtr & group) {
      return validate_callback_group(group);
    });
}

bool AgnocastExecutor::get_next_agnocast_executable(
  AgnocastExecutable & agnocast_executable, const int timeout_ms)
{
  if (get_next_ready_agnocast_executable(agnocast_executable)) {
    return true;
  }

  agnocast::wait_and_handle_epoll_event(
    epoll_fd_, my_pid_, timeout_ms, ready_agnocast_executables_mutex_, ready_agnocast_executables_);

  // Try again
  return get_next_ready_agnocast_executable(agnocast_executable);
}

bool AgnocastExecutor::get_next_ready_agnocast_executable(AgnocastExecutable & agnocast_executable)
{
  std::lock_guard<std::mutex> ready_wait_lock{ready_agnocast_executables_mutex_};

  for (auto it = ready_agnocast_executables_.begin(); it != ready_agnocast_executables_.end();
       ++it) {
    // Prevent a race where an Agnocast::Subscription callback fires before the
    // rclcpp::Node is fully constructed. In Agnocast, a subscription callback
    // becomes runnable as soon as register_callback() is invoked, but this is
    // fundamentally independent of rclcpp::Node: an Agnocast Executable (e.g.,
    // Subscription) has no lifecycle coupling with rclcpp::Node.
    //
    // To guard against callbacks executing on a not-yet-instantiated node, we
    // verify that rclcpp::Executor::add_node() has already been called for this
    // node. If the executor has added the node, its construction is complete.
    //
    // If the executor->add_node() is not called for the node that has this callback_group,
    // the callback group won't be in the executor's list.
    // rclcpp 28+ (Jazzy) removed get_node_by_group() and weak_groups_to_nodes_ from public API.
    // Use get_all_callback_groups() instead to check if the callback group is registered.
#if RCLCPP_VERSION_MAJOR >= 28
    auto all_groups = get_all_callback_groups();
    bool group_found = std::any_of(
      all_groups.begin(), all_groups.end(),
      [&it](const rclcpp::CallbackGroup::WeakPtr & weak_group) {
        return weak_group.lock() == it->callback_group;
      });
    if (!group_found) {
      continue;
    }
#else
    if (
      rclcpp::Executor::get_node_by_group(
        rclcpp::Executor::weak_groups_to_nodes_, it->callback_group) == nullptr) {
      continue;
    }
#endif

    if (
      it->callback_group->type() == rclcpp::CallbackGroupType::MutuallyExclusive &&
      !it->callback_group->can_be_taken_from().exchange(false)) {
      continue;
    }

    agnocast_executable = *it;
    ready_agnocast_executables_.erase(it);

    return true;
  }

  return false;
}

void AgnocastExecutor::execute_agnocast_executable(AgnocastExecutable & agnocast_executable)
{
  TRACEPOINT(
    agnocast_callable_start, static_cast<const void *>(agnocast_executable.callable.get()));
  (*agnocast_executable.callable)();
  TRACEPOINT(agnocast_callable_end, static_cast<const void *>(agnocast_executable.callable.get()));

  if (agnocast_executable.callback_group->type() == rclcpp::CallbackGroupType::MutuallyExclusive) {
    agnocast_executable.callback_group->can_be_taken_from().store(true);
  }
}

}  // namespace agnocast
