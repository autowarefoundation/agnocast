#include "agnocast/node/agnocast_only_single_threaded_executor.hpp"

#include "agnocast/agnocast.hpp"

namespace agnocast
{

AgnocastOnlySingleThreadedExecutor::AgnocastOnlySingleThreadedExecutor(int next_exec_timeout_ms)
: next_exec_timeout_ms_(next_exec_timeout_ms)
{
  const int next_exec_timeout_ms_threshold = 500;  // Rough value
  if (next_exec_timeout_ms_ > next_exec_timeout_ms_threshold) {
    RCLCPP_WARN(
      logger,
      "Due to the large next_exec_timeout_ms value, the callbacks registered after spin and ROS 2 "
      "callbacks may be extremely slow to execute.");
  }

  // TODO(atsushi421): CARET tracepoint for executor creation
}

void AgnocastOnlySingleThreadedExecutor::spin()
{
  if (spinning_.exchange(true)) {
    RCLCPP_ERROR(logger, "spin() called while already spinning");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  RCPPUTILS_SCOPE_EXIT(this->spinning_.store(false););

  while (spinning_.load()) {
    if (need_epoll_updates.load()) {
      agnocast::prepare_epoll_impl(
        epoll_fd_, my_pid_, ready_agnocast_executables_mutex_, ready_agnocast_executables_,
        [](const rclcpp::CallbackGroup::SharedPtr & group) {
          (void)group;
          return true;
        });
    }

    agnocast::AgnocastExecutable agnocast_executable;
    bool shutdown_detected = false;
    if (get_next_agnocast_executable(
          agnocast_executable, next_exec_timeout_ms_, shutdown_detected)) {
      execute_agnocast_executable(agnocast_executable);
    }

    if (shutdown_detected) {
      break;
    }
  }
}

}  // namespace agnocast
