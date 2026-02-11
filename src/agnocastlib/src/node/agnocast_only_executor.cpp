#include "agnocast/node/agnocast_only_executor.hpp"

#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_epoll.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "agnocast_signal_handler.hpp"
#include "rclcpp/rclcpp.hpp"

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <memory>

namespace agnocast
{

AgnocastOnlyExecutor::AgnocastOnlyExecutor()
: spinning_(false),
  epoll_fd_(epoll_create1(0)),
  shutdown_event_fd_(eventfd(0, EFD_NONBLOCK)),
  my_pid_(getpid())
{
  if (epoll_fd_ == -1) {
    RCLCPP_ERROR(logger, "epoll_create1 failed: %s", strerror(errno));
    exit(EXIT_FAILURE);
  }

  if (shutdown_event_fd_ == -1) {
    RCLCPP_ERROR(logger, "eventfd failed: %s", strerror(errno));
    close(epoll_fd_);
    exit(EXIT_FAILURE);
  }

  struct epoll_event ev
  {
  };
  ev.events = EPOLLIN;
  ev.data.u32 = SHUTDOWN_EVENT_FLAG;
  if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, shutdown_event_fd_, &ev) == -1) {
    RCLCPP_ERROR(logger, "epoll_ctl for shutdown_event_fd failed: %s", strerror(errno));
    close(shutdown_event_fd_);
    close(epoll_fd_);
    exit(EXIT_FAILURE);
  }

  SignalHandler::install();
  SignalHandler::register_shutdown_event(shutdown_event_fd_);
}

AgnocastOnlyExecutor::~AgnocastOnlyExecutor()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto & pair : weak_groups_associated_with_executor_to_nodes_) {
      auto group = pair.first.lock();
      if (group) {
        group->get_associated_with_executor_atomic().store(false);
      }
    }
    weak_groups_associated_with_executor_to_nodes_.clear();

    for (auto & pair : weak_groups_to_nodes_associated_with_executor_) {
      auto group = pair.first.lock();
      if (group) {
        group->get_associated_with_executor_atomic().store(false);
      }
    }
    weak_groups_to_nodes_associated_with_executor_.clear();

    for (auto & weak_node : weak_nodes_) {
      auto node = weak_node.lock();
      if (node) {
        node->get_associated_with_executor_atomic().store(false);
      }
    }
    weak_nodes_.clear();
  }

  SignalHandler::unregister_shutdown_event(shutdown_event_fd_);
  close(shutdown_event_fd_);
  close(epoll_fd_);
}

bool AgnocastOnlyExecutor::get_next_agnocast_executable(
  AgnocastExecutable & agnocast_executable, const int timeout_ms, bool & shutdown_detected)
{
  shutdown_detected = false;

  if (get_next_ready_agnocast_executable(agnocast_executable)) {
    return true;
  }

  shutdown_detected = agnocast::wait_and_handle_epoll_event(
    epoll_fd_, my_pid_, timeout_ms, ready_agnocast_executables_mutex_, ready_agnocast_executables_);

  if (shutdown_detected) {
    return false;
  }

  // Try again
  return get_next_ready_agnocast_executable(agnocast_executable);
}

bool AgnocastOnlyExecutor::get_next_ready_agnocast_executable(
  AgnocastExecutable & agnocast_executable)
{
  std::lock_guard<std::mutex> ready_wait_lock{ready_agnocast_executables_mutex_};

  for (auto it = ready_agnocast_executables_.begin(); it != ready_agnocast_executables_.end();
       ++it) {
    if (!is_callback_group_associated(it->callback_group)) {
      continue;
    }

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

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
void AgnocastOnlyExecutor::execute_agnocast_executable(AgnocastExecutable & agnocast_executable)
{
  TRACEPOINT(
    agnocast_callable_start, static_cast<const void *>(agnocast_executable.callable.get()));
  (*agnocast_executable.callable)();
  TRACEPOINT(agnocast_callable_end, static_cast<const void *>(agnocast_executable.callable.get()));

  if (agnocast_executable.callback_group->type() == rclcpp::CallbackGroupType::MutuallyExclusive) {
    agnocast_executable.callback_group->can_be_taken_from().store(true);
  }
}

void AgnocastOnlyExecutor::cancel()
{
  spinning_.store(false);
  uint64_t val = 1;
  if (write(shutdown_event_fd_, &val, sizeof(val)) == -1) {
    RCLCPP_WARN(logger, "Failed to write to shutdown eventfd: %s", strerror(errno));
  }
}

void AgnocastOnlyExecutor::add_callback_group(
  rclcpp::CallbackGroup::SharedPtr
    group_ptr,  // NOLINT(performance-unnecessary-value-param): align with rclcpp API
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
    node_ptr,  // NOLINT(performance-unnecessary-value-param): align with rclcpp API
  bool notify)
{
  (void)notify;
  std::atomic_bool & has_executor = group_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    RCLCPP_ERROR(logger, "Callback group has already been added to an executor.");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  bool was_inserted =
    weak_groups_associated_with_executor_to_nodes_.insert({group_ptr, node_ptr}).second;
  if (!was_inserted) {
    RCLCPP_ERROR(logger, "Callback group was already added to executor.");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }
}

void AgnocastOnlyExecutor::remove_callback_group(
  // NOLINTNEXTLINE(performance-unnecessary-value-param): align with rclcpp API
  rclcpp::CallbackGroup::SharedPtr group_ptr, bool notify)
{
  (void)notify;
  std::lock_guard<std::mutex> lock(mutex_);
  const rclcpp::CallbackGroup::WeakPtr weak(group_ptr);
  const auto it = weak_groups_associated_with_executor_to_nodes_.find(weak);
  if (it == weak_groups_associated_with_executor_to_nodes_.end()) {
    RCLCPP_ERROR(logger, "Callback group needs to be associated with this executor.");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }
  if (it->second.expired()) {
    RCLCPP_ERROR(logger, "Node must not be deleted before its callback group(s).");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }
  weak_groups_associated_with_executor_to_nodes_.erase(it);
  group_ptr->get_associated_with_executor_atomic().store(false);
}

std::vector<rclcpp::CallbackGroup::WeakPtr> AgnocastOnlyExecutor::get_all_callback_groups()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  groups.reserve(
    weak_groups_associated_with_executor_to_nodes_.size() +
    weak_groups_to_nodes_associated_with_executor_.size());
  for (const auto & pair : weak_groups_associated_with_executor_to_nodes_) {
    groups.push_back(pair.first);
  }
  for (const auto & pair : weak_groups_to_nodes_associated_with_executor_) {
    groups.push_back(pair.first);
  }
  return groups;
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
AgnocastOnlyExecutor::get_manually_added_callback_groups()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  groups.reserve(weak_groups_associated_with_executor_to_nodes_.size());
  for (const auto & pair : weak_groups_associated_with_executor_to_nodes_) {
    groups.push_back(pair.first);
  }
  return groups;
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
AgnocastOnlyExecutor::get_automatically_added_callback_groups_from_nodes()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups;
  groups.reserve(weak_groups_to_nodes_associated_with_executor_.size());
  for (const auto & pair : weak_groups_to_nodes_associated_with_executor_) {
    groups.push_back(pair.first);
  }
  return groups;
}

bool AgnocastOnlyExecutor::is_callback_group_associated(
  const rclcpp::CallbackGroup::SharedPtr & group)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (
    weak_groups_associated_with_executor_to_nodes_.empty() &&
    weak_groups_to_nodes_associated_with_executor_.empty() && weak_nodes_.empty()) {
    return false;
  }
  const rclcpp::CallbackGroup::WeakPtr weak(group);
  return weak_groups_associated_with_executor_to_nodes_.count(weak) > 0 ||
         weak_groups_to_nodes_associated_with_executor_.count(weak) > 0;
}

void AgnocastOnlyExecutor::add_callback_groups_from_nodes_associated_to_executor()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (node) {
      node->for_each_callback_group(
        [this, node](const rclcpp::CallbackGroup::SharedPtr & group_ptr) {
          if (
            group_ptr->automatically_add_to_executor_with_node() &&
            !group_ptr->get_associated_with_executor_atomic().exchange(true)) {
            weak_groups_to_nodes_associated_with_executor_.insert({group_ptr, node});
          }
        });
    }
  }
}

void AgnocastOnlyExecutor::add_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
    node_ptr,  // NOLINT(performance-unnecessary-value-param): align with rclcpp API
  bool notify)
{
  (void)notify;
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    RCLCPP_ERROR(
      logger, "Node '%s' has already been added to an executor.",
      node_ptr->get_fully_qualified_name());
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  node_ptr->for_each_callback_group(
    [this, node_ptr](const rclcpp::CallbackGroup::SharedPtr & group_ptr) {
      if (
        group_ptr->automatically_add_to_executor_with_node() &&
        !group_ptr->get_associated_with_executor_atomic().exchange(true)) {
        weak_groups_to_nodes_associated_with_executor_.insert({group_ptr, node_ptr});
      }
    });
  weak_nodes_.push_back(node_ptr);
}

void AgnocastOnlyExecutor::add_node(const std::shared_ptr<agnocast::Node> & node, bool notify)
{
  add_node(node->get_node_base_interface(), notify);
}

void AgnocastOnlyExecutor::remove_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
    node_ptr,  // NOLINT(performance-unnecessary-value-param): align with rclcpp API
  bool notify)
{
  (void)notify;
  if (!node_ptr->get_associated_with_executor_atomic().load()) {
    RCLCPP_ERROR(logger, "Node needs to be associated with an executor.");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  std::lock_guard<std::mutex> lock(mutex_);

  bool found_node = false;
  for (auto node_it = weak_nodes_.begin(); node_it != weak_nodes_.end();) {
    if (node_it->lock() == node_ptr) {
      found_node = true;
      node_it = weak_nodes_.erase(node_it);
    } else {
      ++node_it;
    }
  }
  if (!found_node) {
    RCLCPP_ERROR(logger, "Node needs to be associated with this executor.");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  for (auto it = weak_groups_to_nodes_associated_with_executor_.begin();
       it != weak_groups_to_nodes_associated_with_executor_.end();) {
    if (it->second.lock() == node_ptr) {
      auto group = it->first.lock();
      if (group) {
        group->get_associated_with_executor_atomic().store(false);
      }
      it = weak_groups_to_nodes_associated_with_executor_.erase(it);
    } else {
      ++it;
    }
  }

  node_ptr->get_associated_with_executor_atomic().store(false);
}

void AgnocastOnlyExecutor::remove_node(const std::shared_ptr<agnocast::Node> & node, bool notify)
{
  remove_node(node->get_node_base_interface(), notify);
}

}  // namespace agnocast
