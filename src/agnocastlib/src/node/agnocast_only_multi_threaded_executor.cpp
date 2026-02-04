#include "agnocast/node/agnocast_only_multi_threaded_executor.hpp"

#include "agnocast/agnocast.hpp"

namespace agnocast
{

AgnocastOnlyMultiThreadedExecutor::AgnocastOnlyMultiThreadedExecutor(
  size_t number_of_threads, bool yield_before_execute, int next_exec_timeout_ms)
: number_of_threads_(
    number_of_threads != 0 ? number_of_threads : std::thread::hardware_concurrency()),
  yield_before_execute_(yield_before_execute),
  next_exec_timeout_ms_(next_exec_timeout_ms)
{
  // TODO(atsushi421): CARET tracepoint for executor creation
}

void AgnocastOnlyMultiThreadedExecutor::spin()
{
  if (spinning_.exchange(true)) {
    RCLCPP_ERROR(logger, "spin() called while already spinning");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  RCPPUTILS_SCOPE_EXIT(this->spinning_.store(false););

  std::vector<std::thread> threads;

  for (size_t i = 0; i < number_of_threads_ - 1; i++) {
    auto func = [this] { agnocast_spin(); };
    threads.emplace_back(func);
  }

  agnocast_spin();

  for (auto & thread : threads) {
    thread.join();
  }
}

void AgnocastOnlyMultiThreadedExecutor::agnocast_spin()
{
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

    if (!spinning_.load()) {
      return;
    }

    // As each thread is dedicated to handling Agnocast callbacks, get_next_agnocast_executable()
    // can block indefinitely without a timeout. However, since we need to periodically check for
    // epoll updates, we should implement a long timeout period instead of an infinite block.
    bool shutdown_detected = false;
    if (get_next_agnocast_executable(
          agnocast_executable, next_exec_timeout_ms_ /* timed-blocking*/, shutdown_detected)) {
      if (yield_before_execute_) {
        std::this_thread::yield();
      }

      execute_agnocast_executable(agnocast_executable);
    }

    if (shutdown_detected) {
      spinning_.store(false);
      return;
    }
  }
}

}  // namespace agnocast
