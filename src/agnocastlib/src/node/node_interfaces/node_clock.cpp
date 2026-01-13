#include "agnocast/node/node_interfaces/node_clock.hpp"

namespace agnocast::node_interfaces
{

NodeClock::NodeClock(rcl_clock_type_t clock_type)
: clock_(std::make_shared<rclcpp::Clock>(clock_type))
{
}

rclcpp::Clock::SharedPtr NodeClock::get_clock()
{
  return clock_;
}

rclcpp::Clock::ConstSharedPtr NodeClock::get_clock() const
{
  return clock_;
}

}  // namespace agnocast::node_interfaces
