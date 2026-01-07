
#include "agnocast/bridge/agnocast_performance_bridge_loader.hpp"

#include "agnocast/agnocast_utils.hpp"

namespace agnocast
{

PerformanceBridgeLoader::PerformanceBridgeLoader(const rclcpp::Logger & logger) : logger_(logger)
{
  RCLCPP_INFO(logger_, "PerformanceBridgeLoader initialized (Skeleton).");
}

PerformanceBridgeLoader::~PerformanceBridgeLoader() = default;

}  // namespace agnocast
