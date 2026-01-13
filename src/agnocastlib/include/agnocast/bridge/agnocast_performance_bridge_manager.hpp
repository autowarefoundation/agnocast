#pragma once

#include "agnocast/agnocast_multi_threaded_executor.hpp"
#include "agnocast/bridge/agnocast_performance_bridge_config.hpp"
#include "agnocast/bridge/agnocast_performance_bridge_ipc_event_loop.hpp"
#include "agnocast/bridge/agnocast_performance_bridge_loader.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <thread>

namespace agnocast
{

class PerformanceBridgeManager
{
public:
  PerformanceBridgeManager();
  ~PerformanceBridgeManager();

  void run();

private:
  rclcpp::Logger logger_;
  PerformanceBridgeIpcEventLoop event_loop_;
  PerformanceBridgeLoader loader_;
  PerformanceBridgeConfig config_;

  std::shared_ptr<rclcpp::Node> container_node_;
  std::shared_ptr<agnocast::MultiThreadedAgnocastExecutor> executor_;
  std::thread executor_thread_;

  bool shutdown_requested_ = false;

  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> active_r2a_bridges_;
  std::unordered_map<std::string, std::shared_ptr<agnocast::SubscriptionBase>> active_a2r_bridges_;

  void start_ros_execution();

  void on_mq_request(int fd);
  void on_signal();

  void check_and_remove_bridges();
  void check_and_request_shutdown();
};

}  // namespace agnocast
