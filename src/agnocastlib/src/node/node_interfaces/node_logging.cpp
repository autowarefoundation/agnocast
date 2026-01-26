#include "agnocast/node/node_interfaces/node_logging.hpp"

#include <stdexcept>

namespace agnocast::node_interfaces
{

NodeLogging::NodeLogging(const rclcpp::Logger & logger) : logger_(logger)
{
}

rclcpp::Logger NodeLogging::get_logger() const
{
  return logger_;
}

const char * NodeLogging::get_logger_name() const
{
  return logger_.get_name();
}

// rclcpp 28+ (Jazzy) added this method to NodeLoggingInterface.
// This is a stub implementation - logger services are not yet supported.
#if RCLCPP_VERSION_MAJOR >= 28
void NodeLogging::create_logger_services(
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr /* node_services */)
{
  throw std::runtime_error("create_logger_services is not yet implemented");
}
#endif

}  // namespace agnocast::node_interfaces
