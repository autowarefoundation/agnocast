#include "agnocast/node/node_interfaces/node_logging.hpp"

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

}  // namespace agnocast::node_interfaces
