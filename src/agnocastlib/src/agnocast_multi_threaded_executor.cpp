#include "agnocast/agnocast_multi_threaded_executor.hpp"

#include "agnocast/agnocast.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"

#include <algorithm>

namespace agnocast
{

MultiThreadedAgnocastExecutor::MultiThreadedAgnocastExecutor(
  const rclcpp::ExecutorOptions & options, size_t number_of_ros2_threads,
  size_t number_of_agnocast_threads, bool yield_before_execute,
  std::chrono::nanoseconds ros2_next_exec_timeout, int agnocast_next_exec_timeout_ms)
: agnocast::AgnocastExecutor(options),
  number_of_ros2_threads_(
    number_of_ros2_threads != 0 ? number_of_ros2_threads
                                : std::max<size_t>(1, std::thread::hardware_concurrency() / 2)),
  number_of_agnocast_threads_(
    number_of_agnocast_threads != 0 ? number_of_agnocast_threads
                                    : std::max<size_t>(1, std::thread::hardware_concurrency() / 2)),
  yield_before_execute_(yield_before_execute),
  ros2_next_exec_timeout_(ros2_next_exec_timeout),
  agnocast_next_exec_timeout_ms_(agnocast_next_exec_timeout_ms)
{
  TRACEPOINT(
    agnocast_construct_executor, static_cast<const void *>(this),
    "agnocast_multi_threaded_executor");
}

bool MultiThreadedAgnocastExecutor::validate_callback_group(
  const rclcpp::CallbackGroup::SharedPtr & group) const
{
  if (!group) {
    throw std::invalid_argument("Callback group is nullptr. The node may have been destructed.");
  }

  if (group->type() == rclcpp::CallbackGroupType::Reentrant) {
    return true;
  }

  bool is_shared_with_ros2 = false;
  group->collect_all_ptrs(
    [&](const rclcpp::SubscriptionBase::SharedPtr &) {
      is_shared_with_ros2 = true;
      return;
    },
    [&](const rclcpp::ServiceBase::SharedPtr &) {
      is_shared_with_ros2 = true;
      return;
    },
    [&](const rclcpp::ClientBase::SharedPtr &) {
      is_shared_with_ros2 = true;
      return;
    },
    [&](const rclcpp::TimerBase::SharedPtr &) {
      is_shared_with_ros2 = true;
      return;
    },
    [&](const rclcpp::Waitable::SharedPtr &) {
      is_shared_with_ros2 = true;
      return;
    });

  if (is_shared_with_ros2) {
    throw std::invalid_argument(
      "To prevent performance degradation, MultiThreadedAgnocastExecutor prohibits the agnocast "
      "callback and the ros2 callback from belonging to the same MutuallyExclusive callback group. "
      "If mutual exclusion between callbacks is not required, consider using Reentrant callback "
      "group. If mutual exclusion is required, separate them into different callback groups and "
      "use a mutex or other synchronization mechanism.");
  }

  return true;
}

void MultiThreadedAgnocastExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::logic_error("spin() called while already spinning");
  }

  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false););

  std::vector<std::thread> threads;

  {
    std::lock_guard<std::mutex> wait_lock{wait_mutex_};

    for (size_t i = 0; i < number_of_ros2_threads_ - 1; i++) {
      auto func = [this] { ros2_spin(); };
      threads.emplace_back(func);
    }

    for (size_t i = 0; i < number_of_agnocast_threads_; i++) {
      auto func = [this] { agnocast_spin(); };
      threads.emplace_back(func);
    }
  }

  ros2_spin();

  for (auto & thread : threads) {
    thread.join();
  }

  if (worker_thread_failed_.load()) {
    throw std::runtime_error("Exception occurred in agnocast worker thread");
  }
}

void MultiThreadedAgnocastExecutor::ros2_spin()
{
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_executable;

    {
      std::lock_guard<std::mutex> wait_lock{wait_mutex_};

      if (!rclcpp::ok(this->context_) || !spinning.load()) {
        return;
      }
      if (!get_next_executable(any_executable, ros2_next_exec_timeout_)) {
        continue;
      }
    }

    if (yield_before_execute_) {
      std::this_thread::yield();
    }

    execute_any_executable(any_executable);

    // On rclcpp 28+ (Jazzy), interrupt_guard_condition_ is a shared_ptr.
    // Wake up threads that may be blocked in wait_for_work() so they can re-check
    // MutuallyExclusive callback groups whose can_be_taken_from was just restored.
#if RCLCPP_VERSION_MAJOR >= 28
    if (
      any_executable.callback_group &&
      any_executable.callback_group->type() == rclcpp::CallbackGroupType::MutuallyExclusive) {
      interrupt_guard_condition_->trigger();
    }
#endif

    // Clear the callback_group to prevent the AnyExecutable destructor from
    // resetting the callback group `can_be_taken_from`.
    // See issue #702 in ros2/rclcpp.
    any_executable.callback_group.reset();
  }
}

void MultiThreadedAgnocastExecutor::agnocast_spin()
{
  try {
    while (rclcpp::ok(this->context_) && spinning.load()) {
      if (need_epoll_updates.load()) {
        prepare_epoll();
      }

      agnocast::AgnocastExecutable agnocast_executable;

      if (!rclcpp::ok(this->context_) || !spinning.load()) {
        return;
      }

      // Unlike a single-threaded executor, in a multi-threaded executor, each thread is dedicated
      // to handling either ROS 2 callbacks or Agnocast callbacks exclusively. Given this
      // separation, get_next_agnocast_executable() can block indefinitely without a timeout.
      // However, since we need to periodically check for epoll updates, we should implement a long
      // timeout period instead of an infinite block.
      if (get_next_agnocast_executable(
            agnocast_executable, agnocast_next_exec_timeout_ms_ /* timed-blocking*/)) {
        if (yield_before_execute_) {
          std::this_thread::yield();
        }

        execute_agnocast_executable(agnocast_executable);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Exception in agnocast worker thread: %s", e.what());
    worker_thread_failed_.store(true);
    spinning.store(false);
  }
}

}  // namespace agnocast
