
#include "agnocast/bridge/agnocast_performance_bridge_config.hpp"

#include <rclcpp/logging.hpp>

namespace agnocast
{

PerformanceBridgeConfig::PerformanceBridgeConfig(const rclcpp::Logger & logger) : logger_(logger)
{
  RCLCPP_INFO(logger_, "PerformanceBridgeConfig initialized (Skeleton).");
}

PerformanceBridgeConfig::~PerformanceBridgeConfig() = default;

}  // namespace agnocast
