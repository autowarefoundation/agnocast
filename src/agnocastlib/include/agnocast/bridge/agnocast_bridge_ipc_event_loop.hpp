#pragma once

#include "agnocast/bridge/agnocast_bridge_ipc_event_loop_base.hpp"

#include <rclcpp/logger.hpp>

namespace agnocast
{

class BridgeIpcEventLoop : public IpcEventLoopBase
{
public:
  explicit BridgeIpcEventLoop(const rclcpp::Logger & logger);
  ~BridgeIpcEventLoop() override = default;
};

}  // namespace agnocast
