
#include "agnocast/bridge/agnocast_performance_bridge_manager.hpp"

#include "agnocast/agnocast_utils.hpp"

#include <unistd.h>

namespace agnocast
{

PerformanceBridgeManager::PerformanceBridgeManager()
: logger_(rclcpp::get_logger("agnocast_performance_bridge_manager")),
  event_loop_(logger_),
  loader_(logger_),
  config_(logger_)
{
  if (!rclcpp::ok()) {
    rclcpp::InitOptions init_options{};
    init_options.shutdown_on_signal = false;
    rclcpp::init(0, nullptr, init_options);
  }

  RCLCPP_INFO(logger_, "Performance Bridge Manager initialized (PID: %d).", getpid());
}

PerformanceBridgeManager::~PerformanceBridgeManager()
{
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void PerformanceBridgeManager::run()
{
  RCLCPP_INFO(logger_, "Performance Bridge Manager started.");
}

}  // namespace agnocast
