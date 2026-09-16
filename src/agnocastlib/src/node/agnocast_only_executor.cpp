#include "agnocast/node/agnocast_only_executor.hpp"

#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_epoll.hpp"
#include "agnocast/agnocast_epoll_event.hpp"
#include "agnocast/agnocast_epoll_update_dispatcher.hpp"
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
: shutdown_event_fd_(eventfd(0, EFD_NONBLOCK)),
  my_pid_(getpid()),
  epoll_update_tracker_(EpollUpdateDispatcher::get_instance().register_tracker())
{
  EventHandlerArray sources;
  sources[static_cast<uint32_t>(EpollEventType::Subscription)] =
    std::make_unique<SubscriptionEventHandler>(
      my_pid_, &ready_agnocast_executables_mutex_, &ready_agnocast_executables_);
  sources[static_cast<uint32_t>(EpollEventType::Timer)] = std::make_unique<TimerEventHandler>(
    my_pid_, &ready_agnocast_executables_mutex_, &ready_agnocast_executables_);
  sources[static_cast<uint32_t>(EpollEventType::Clock)] = std::make_unique<ClockEventHandler>(
    my_pid_, &ready_agnocast_executables_mutex_, &ready_agnocast_executables_);
  sources[static_cast<uint32_t>(EpollEventType::Shutdown)] =
    std::make_unique<ShutdownEventHandler>();

  epoll_manager_ = std::make_unique<EpollManager>(std::move(sources));

  if (shutdown_event_fd_ == -1) {
    RCLCPP_ERROR(logger, "eventfd failed: %s", strerror(errno));
    exit(EXIT_FAILURE);
  }

  if (!epoll_manager_->add_event(shutdown_event_fd_, EpollEventType::Shutdown, 0)) {
    RCLCPP_ERROR(logger, "epoll_ctl for shutdown_event_fd failed: %s", strerror(errno));
    close(shutdown_event_fd_);
    exit(EXIT_FAILURE);
  }

  if (!SignalHandler::register_shutdown_event(shutdown_event_fd_)) {
    RCLCPP_ERROR(logger, "Failed to register shutdown eventfd with signal handler");
    close(shutdown_event_fd_);
    exit(EXIT_FAILURE);
  }
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
}

bool AgnocastOnlyExecutor::get_next_agnocast_executable(
  AgnocastExecutable & agnocast_executable, std::chrono::nanoseconds timeout)
{
  if (get_next_ready_agnocast_executable(agnocast_executable)) {
    return true;
  }

  wait_for_work(timeout);

  if (!agnocast::ok()) {
    return false;
  }

  // Try again
  return get_next_ready_agnocast_executable(agnocast_executable);
}

bool AgnocastOnlyExecutor::get_next_agnocast_executable(
  AgnocastExecutable & agnocast_executable, const int timeout_ms)
{
  return get_next_agnocast_executable(agnocast_executable, std::chrono::milliseconds{timeout_ms});
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
  (*agnocast_executable.callable)();

  if (agnocast_executable.callback_group->type() == rclcpp::CallbackGroupType::MutuallyExclusive) {
    agnocast_executable.callback_group->can_be_taken_from().store(true);
  }
}

void AgnocastOnlyExecutor::cancel()
{
  cancel_requested_.store(true);
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

#ifndef TRACETOOLS_DISABLED
  const auto group_type_enum = group_ptr->type();
  const char * group_type_str = (group_type_enum == rclcpp::CallbackGroupType::MutuallyExclusive)
                                  ? "mutually_exclusive"
                                  : "reentrant";

  TRACEPOINT(
    agnocast_add_callback_group, static_cast<const void *>(this),
    static_cast<const void *>(node_ptr.get()), static_cast<const void *>(group_ptr.get()),
    group_type_str);
#endif

  EpollUpdateDispatcher::get_instance().request_update(epoll_update_tracker_.id());
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

  EpollUpdateDispatcher::get_instance().request_update(epoll_update_tracker_.id());
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
      node->for_each_callback_group([this,
                                     node](const rclcpp::CallbackGroup::SharedPtr & group_ptr) {
        if (
          group_ptr->automatically_add_to_executor_with_node() &&
          !group_ptr->get_associated_with_executor_atomic().exchange(true)) {
          weak_groups_to_nodes_associated_with_executor_.insert({group_ptr, node});

#ifndef TRACETOOLS_DISABLED
          const auto group_type_enum = group_ptr->type();
          const char * group_type_str =
            (group_type_enum == rclcpp::CallbackGroupType::MutuallyExclusive) ? "mutually_exclusive"
                                                                              : "reentrant";

          TRACEPOINT(
            agnocast_add_callback_group, static_cast<const void *>(this),
            static_cast<const void *>(node.get()), static_cast<const void *>(group_ptr.get()),
            group_type_str);
#endif
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

#ifndef TRACETOOLS_DISABLED
        const auto group_type_enum = group_ptr->type();
        const char * group_type_str =
          (group_type_enum == rclcpp::CallbackGroupType::MutuallyExclusive) ? "mutually_exclusive"
                                                                            : "reentrant";

        TRACEPOINT(
          agnocast_add_callback_group, static_cast<const void *>(this),
          static_cast<const void *>(node_ptr.get()), static_cast<const void *>(group_ptr.get()),
          group_type_str);
#endif
      }
    });
  weak_nodes_.push_back(node_ptr);

  EpollUpdateDispatcher::get_instance().request_update(epoll_update_tracker_.id());
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

  EpollUpdateDispatcher::get_instance().request_update(epoll_update_tracker_.id());
}

void AgnocastOnlyExecutor::remove_node(const std::shared_ptr<agnocast::Node> & node, bool notify)
{
  remove_node(node->get_node_base_interface(), notify);
}

rclcpp::FutureReturnCode AgnocastOnlyExecutor::spin_until_future_complete_impl(
  std::chrono::nanoseconds timeout,
  const std::function<std::future_status(std::chrono::nanoseconds wait_time)> & wait_for_future)
{
  std::future_status status = wait_for_future(std::chrono::seconds(0));
  if (status == std::future_status::ready) {
    return rclcpp::FutureReturnCode::SUCCESS;
  }

  auto end_time = std::chrono::steady_clock::now();
  std::chrono::nanoseconds timeout_ns = timeout;
  if (timeout_ns > std::chrono::nanoseconds::zero()) {
    end_time += timeout_ns;
  }
  std::chrono::nanoseconds timeout_left = timeout_ns;

  if (spinning_.exchange(true)) {
    throw std::runtime_error("spin_until_future_complete() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning_.store(false););

  while (spinning_.load() && !cancel_requested_.load() && agnocast::ok()) {
    // Do one item of work.
    spin_once_impl(timeout_left);

    // Check if the future is set, return SUCCESS if it is.
    status = wait_for_future(std::chrono::seconds(0));
    if (status == std::future_status::ready) {
      return rclcpp::FutureReturnCode::SUCCESS;
    }
    // If the original timeout is < 0, then this is blocking, never TIMEOUT.
    if (timeout_ns < std::chrono::nanoseconds::zero()) {
      continue;
    }
    // Otherwise check if we still have time to wait, return TIMEOUT if not.
    auto now = std::chrono::steady_clock::now();
    if (now >= end_time) {
      return rclcpp::FutureReturnCode::TIMEOUT;
    }
    // Subtract the elapsed time from the original timeout.
    timeout_left = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - now);
  }

  // The future did not complete before ok() returned false, return INTERRUPTED.
  return rclcpp::FutureReturnCode::INTERRUPTED;
}

void AgnocastOnlyExecutor::spin_some(std::chrono::nanoseconds max_duration)
{
  return this->spin_some_impl(max_duration, false);
}

void AgnocastOnlyExecutor::spin_all(std::chrono::nanoseconds max_duration)
{
  if (max_duration < std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument("max_duration must be greater than or equal to 0");
  }
  return this->spin_some_impl(max_duration, true);
}

void AgnocastOnlyExecutor::spin_some_impl(std::chrono::nanoseconds max_duration, bool exhaustive)
{
  auto start = std::chrono::steady_clock::now();
  auto max_duration_not_elapsed = [max_duration, start]() {
    if (std::chrono::nanoseconds(0) == max_duration) {
      return true;
    } else if (std::chrono::steady_clock::now() - start < max_duration) {
      return true;
    }
    return false;
  };

  if (spinning_.exchange(true)) {
    throw std::runtime_error("spin_some() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning_.store(false););

  wait_for_work(std::chrono::milliseconds(0));
  bool entity_states_fully_polled = true;

  while (spinning_.load() && !cancel_requested_.load() && agnocast::ok() &&
         max_duration_not_elapsed()) {
    AgnocastExecutable agnocast_exec;
    if (get_next_ready_agnocast_executable(agnocast_exec)) {
      execute_agnocast_executable(agnocast_exec);
      entity_states_fully_polled = false;
    } else {
      if (entity_states_fully_polled) {
        break;
      }
      if (exhaustive) {
        wait_for_work(std::chrono::milliseconds(0));
        entity_states_fully_polled = true;
      } else {
        break;
      }
    }
  }
}

void AgnocastOnlyExecutor::spin_once_impl(std::chrono::nanoseconds timeout)
{
  AgnocastExecutable agnocast_exec;
  if (get_next_agnocast_executable(agnocast_exec, timeout)) {
    execute_agnocast_executable(agnocast_exec);
  }
}

void AgnocastOnlyExecutor::spin_once(std::chrono::nanoseconds timeout)
{
  if (cancel_requested_.load()) {
    return;
  }
  if (spinning_.exchange(true)) {
    throw std::runtime_error("spin_once() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning_.store(false););
  spin_once_impl(timeout);
}

void AgnocastOnlyExecutor::wait_for_work(std::chrono::nanoseconds timeout)
{
  if (epoll_update_tracker_.take_update_request()) {
    add_callback_groups_from_nodes_associated_to_executor();
    epoll_manager_->prepare_epoll([this](const rclcpp::CallbackGroup::SharedPtr & group) {
      return is_callback_group_associated(group);
    });
  }

  using IntMs = std::chrono::duration<int, std::milli>;
  int timeout_ms;
  if (timeout < std::chrono::nanoseconds::zero()) {
    timeout_ms = -1;
  } else if (timeout > IntMs::max()) {
    timeout_ms = IntMs::max().count();
  } else {
    timeout_ms = std::chrono::ceil<IntMs>(timeout).count();
  }
  epoll_manager_->wait_and_handle_epoll_event(timeout_ms);
}

bool AgnocastOnlyExecutor::is_spinning()
{
  return spinning_;
}

}  // namespace agnocast
