#include "agnocast/node/agnocast_only_executor.hpp"

#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_epoll.hpp"
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

// Implemented align to unify the API with rclcpp::Executor
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
void AgnocastOnlyExecutor::add_node(const std::shared_ptr<agnocast::Node> & node)
{
  (void)node;
}

}  // namespace agnocast
