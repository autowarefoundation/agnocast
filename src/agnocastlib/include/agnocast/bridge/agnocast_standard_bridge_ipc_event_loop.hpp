#pragma once

#include "agnocast/bridge/agnocast_bridge_ipc_event_loop_base.hpp"

#include <rclcpp/logger.hpp>

namespace agnocast
{

class StandardBridgeIpcEventLoop : public IpcEventLoopBase
{
public:
  explicit StandardBridgeIpcEventLoop(const rclcpp::Logger & logger);
  ~StandardBridgeIpcEventLoop() override = default;
};

}  // namespace agnocast
