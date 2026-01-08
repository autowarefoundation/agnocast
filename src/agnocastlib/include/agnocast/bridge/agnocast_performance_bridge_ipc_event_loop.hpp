#pragma once

#include "agnocast/bridge/agnocast_bridge_ipc_event_loop_base.hpp"

#include <rclcpp/logger.hpp>

#include <functional>

namespace agnocast
{

class PerformanceBridgeIpcEventLoop : public IpcEventLoopBase
{
public:
  using ReloadCallback = std::function<void()>;

  explicit PerformanceBridgeIpcEventLoop(const rclcpp::Logger & logger);
  ~PerformanceBridgeIpcEventLoop() override = default;

  void set_reload_handler(ReloadCallback cb);

protected:
  void handle_signal(int signo) override;

private:
  ReloadCallback reload_cb_;
};

}  // namespace agnocast
