#pragma once

#include <rclcpp/logger.hpp>

namespace agnocast
{

class PerformanceBridgeConfig
{
public:
  explicit PerformanceBridgeConfig(const rclcpp::Logger & logger);
  ~PerformanceBridgeConfig();

private:
  rclcpp::Logger logger_;
};

}  // namespace agnocast
