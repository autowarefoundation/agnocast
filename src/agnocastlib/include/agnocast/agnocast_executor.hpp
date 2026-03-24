#pragma once

#include "agnocast/agnocast_epoll.hpp"
#include "agnocast/agnocast_public_api.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agnocast
{

extern std::mutex mmap_mtx;

void map_read_only_area(const pid_t pid, const uint64_t shm_addr, const uint64_t shm_size);

struct AgnocastExecutable
{
  std::shared_ptr<std::function<void()>> callable;
  rclcpp::CallbackGroup::SharedPtr callback_group{nullptr};
};

/** @brief Base class for Stage 1 executors that handle both ROS 2 (RMW) and Agnocast callbacks.
 * Inherits from rclcpp::Executor. */
AGNOCAST_PUBLIC
class AgnocastExecutor : public rclcpp::Executor
{
  std::mutex ready_agnocast_executables_mutex_;
  std::vector<AgnocastExecutable> ready_agnocast_executables_;

  bool get_next_ready_agnocast_executable(AgnocastExecutable & agnocast_executable);

  // Return false iff this Executor is SingleThreadedAgnocastExecutor
  // and used for internal implementation of CallbackIsolatedAgnocastExecutor
  // and `group` is "not" the callback group dedicated by this Executor.
  virtual bool validate_callback_group(const rclcpp::CallbackGroup::SharedPtr & group) const = 0;

protected:
  int epoll_fd_;
  pid_t my_pid_;
  std::mutex wait_mutex_;

  void prepare_epoll();
  bool get_next_agnocast_executable(AgnocastExecutable & agnocast_executable, const int timeout_ms);
  static void execute_agnocast_executable(AgnocastExecutable & agnocast_executable);

public:
  /// Construct the executor.
  /// @param options Executor options.
  AGNOCAST_PUBLIC
  explicit AgnocastExecutor(const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());

  virtual ~AgnocastExecutor();

  /// Block the calling thread and process callbacks in a loop until rclcpp::shutdown() is called or
  /// the executor is cancelled.
  AGNOCAST_PUBLIC
  virtual void spin() override = 0;
};

}  // namespace agnocast
