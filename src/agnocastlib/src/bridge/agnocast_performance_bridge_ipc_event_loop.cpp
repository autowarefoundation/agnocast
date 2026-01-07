
#include "agnocast/bridge/agnocast_performance_bridge_ipc_event_loop.hpp"

#include <rclcpp/logging.hpp>

namespace agnocast
{

PerformanceBridgeIpcEventLoop::PerformanceBridgeIpcEventLoop(const rclcpp::Logger & logger)
: logger_(logger)
{
  RCLCPP_INFO(logger_, "PerformanceBridgeIpcEventLoop initialized (Skeleton).");
}

PerformanceBridgeIpcEventLoop::~PerformanceBridgeIpcEventLoop() = default;

void PerformanceBridgeIpcEventLoop::spin_once()
{
}

}  // namespace agnocast
