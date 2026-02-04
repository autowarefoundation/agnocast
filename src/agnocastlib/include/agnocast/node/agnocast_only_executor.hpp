#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

namespace agnocast
{

struct AgnocastExecutable;
class Node;

class AgnocastOnlyExecutor
{
protected:
  std::atomic_bool spinning_;
  int epoll_fd_;
  int shutdown_event_fd_;
  pid_t my_pid_;

  std::mutex ready_agnocast_executables_mutex_;
  std::vector<AgnocastExecutable> ready_agnocast_executables_;

  bool get_next_agnocast_executable(
    AgnocastExecutable & agnocast_executable, const int timeout_ms, bool & shutdown_detected);
  bool get_next_ready_agnocast_executable(AgnocastExecutable & agnocast_executable);
  void execute_agnocast_executable(AgnocastExecutable & agnocast_executable);

public:
  explicit AgnocastOnlyExecutor();

  virtual ~AgnocastOnlyExecutor();

  virtual void spin() = 0;
  void cancel();

  // Implemented align to unify the API with rclcpp::Executor
  void add_node(const std::shared_ptr<agnocast::Node> & node);
};

}  // namespace agnocast
