#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

namespace agnocast
{

class AgnocastExecutable;
class Node;

class AgnocastOnlyExecutor
{
protected:
  std::atomic_bool spinning_;
  int epoll_fd_;
  pid_t my_pid_;

  std::mutex ready_agnocast_executables_mutex_;
  std::vector<AgnocastExecutable> ready_agnocast_executables_;

  bool get_next_agnocast_executable(AgnocastExecutable & agnocast_executable, const int timeout_ms);
  bool get_next_ready_agnocast_executable(AgnocastExecutable & agnocast_executable);
  void execute_agnocast_executable(AgnocastExecutable & agnocast_executable);

public:
  explicit AgnocastOnlyExecutor();

  virtual ~AgnocastOnlyExecutor();

  virtual void spin() = 0;

  // Implemented align to unify the API with rclcpp::Executor
  void add_node(const std::shared_ptr<agnocast::Node> & node);
};

}  // namespace agnocast
