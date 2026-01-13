#pragma once

#include "agnocast/bridge/agnocast_bridge_ipc_event_loop_base.hpp"

#include <rclcpp/logger.hpp>

namespace agnocast
{

class PerformanceBridgeIpcEventLoop : public IpcEventLoopBase
{
public:
  explicit PerformanceBridgeIpcEventLoop(const rclcpp::Logger & logger);
  ~PerformanceBridgeIpcEventLoop() override = default;
};

}  // namespace agnocast
