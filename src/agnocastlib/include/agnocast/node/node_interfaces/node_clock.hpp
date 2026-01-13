#pragma once

#include "rclcpp/clock.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"

#include <rcl/time.h>

namespace agnocast::node_interfaces
{

class NodeClock : public rclcpp::node_interfaces::NodeClockInterface
{
public:
  using SharedPtr = std::shared_ptr<NodeClock>;
  using WeakPtr = std::weak_ptr<NodeClock>;

  explicit NodeClock(rcl_clock_type_t clock_type);

  virtual ~NodeClock() = default;

  NodeClock(const NodeClock &) = delete;
  NodeClock & operator=(const NodeClock &) = delete;

  rclcpp::Clock::SharedPtr get_clock() override;

  rclcpp::Clock::ConstSharedPtr get_clock() const override;

private:
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace agnocast::node_interfaces
