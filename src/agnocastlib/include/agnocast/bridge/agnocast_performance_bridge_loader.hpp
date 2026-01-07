#pragma once

#include <rclcpp/logger.hpp>

namespace agnocast
{

class PerformanceBridgeLoader
{
public:
  explicit PerformanceBridgeLoader(const rclcpp::Logger & logger);
  ~PerformanceBridgeLoader();

private:
  rclcpp::Logger logger_;
};

}  // namespace agnocast
