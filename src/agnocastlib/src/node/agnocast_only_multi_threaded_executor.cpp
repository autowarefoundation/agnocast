#include "agnocast/node/agnocast_only_multi_threaded_executor.hpp"

#include "agnocast/agnocast.hpp"

#include <algorithm>

namespace agnocast
{

AgnocastOnlyMultiThreadedExecutor::AgnocastOnlyMultiThreadedExecutor(
  size_t number_of_threads, bool yield_before_execute, int next_exec_timeout_ms)
: number_of_threads_(
    number_of_threads != 0 ? number_of_threads
                           : std::max<size_t>(1, std::thread::hardware_concurrency())),
  yield_before_execute_(yield_before_execute),
  next_exec_timeout_ms_(next_exec_timeout_ms)
{
  TRACEPOINT(
    agnocast_construct_executor, static_cast<const void *>(this),
    "agnocast_only_multi_threaded_executor");
}

void AgnocastOnlyMultiThreadedExecutor::spin()
{
  if (spinning_.exchange(true)) {
    RCLCPP_ERROR(logger, "spin() called while already spinning");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  RCPPUTILS_SCOPE_EXIT(this->spinning_.store(false););

  if (cancel_requested_.load()) {
    return;
  }

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
  while (spinning_.load() && !cancel_requested_.load() && agnocast::ok()) {
    if (epoll_update_tracker_.take_update_request()) {
      add_callback_groups_from_nodes_associated_to_executor();
      epoll_manager_->prepare_epoll([this](const rclcpp::CallbackGroup::SharedPtr & group) {
        return is_callback_group_associated(group);
      });
    }

    agnocast::AgnocastExecutable agnocast_executable;

    if (!spinning_.load() || cancel_requested_.load() || !agnocast::ok()) {
      return;
    }

    // As each thread is dedicated to handling Agnocast callbacks, get_next_agnocast_executable()
    // can block indefinitely without a timeout. However, since we need to periodically check for
    // epoll updates, we should implement a long timeout period instead of an infinite block.
    if (get_next_agnocast_executable(
          agnocast_executable, next_exec_timeout_ms_ /* timed-blocking*/)) {
      if (yield_before_execute_) {
        std::this_thread::yield();
      }

      execute_agnocast_executable(agnocast_executable);
    }
  }
}

}  // namespace agnocast
