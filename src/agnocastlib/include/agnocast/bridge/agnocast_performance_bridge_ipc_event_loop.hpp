#pragma once

#include <rclcpp/logger.hpp>

namespace agnocast
{

class PerformanceBridgeIpcEventLoop
{
public:
  explicit PerformanceBridgeIpcEventLoop(const rclcpp::Logger & logger);
  ~PerformanceBridgeIpcEventLoop();

  void spin_once();

private:
  rclcpp::Logger logger_;
};

}  // namespace agnocast
