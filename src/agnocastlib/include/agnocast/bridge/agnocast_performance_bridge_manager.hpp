#pragma once

#include "agnocast/bridge/agnocast_performance_bridge_config.hpp"
#include "agnocast/bridge/agnocast_performance_bridge_ipc_event_loop.hpp"
#include "agnocast/bridge/agnocast_performance_bridge_loader.hpp"

#include <rclcpp/rclcpp.hpp>

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

  bool shutdown_requested_ = false;

  void check_and_request_shutdown();
};

}  // namespace agnocast
