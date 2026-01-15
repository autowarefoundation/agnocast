#pragma once

#include "agnocast/agnocast_subscription.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_time_source_interface.hpp"
#include "rclcpp/qos.hpp"

#include "rosgraph_msgs/msg/clock.hpp"

#include <mutex>

namespace agnocast
{
class Node;
}

namespace agnocast::node_interfaces
{

class NodeTimeSource : public rclcpp::node_interfaces::NodeTimeSourceInterface
{
public:
  using SharedPtr = std::shared_ptr<NodeTimeSource>;
  using WeakPtr = std::weak_ptr<NodeTimeSource>;

  NodeTimeSource(
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & node_clock,
    agnocast::Node * node, const rclcpp::QoS & qos = rclcpp::ClockQoS());

  ~NodeTimeSource() override;

private:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  agnocast::Node * agnocast_node_{nullptr};

  rclcpp::QoS qos_;
  agnocast::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
  rclcpp::Clock::SharedPtr clock_;
  bool ros_time_active_{false};
  std::mutex clock_sub_lock_;
  rclcpp::Logger logger_{rclcpp::get_logger("agnocast")};

  void enable_ros_time();
  void disable_ros_time();
  void set_clock(const builtin_interfaces::msg::Time & msg, bool set_ros_time_enabled);
  void create_clock_sub();
  void destroy_clock_sub();
  void clock_cb(const agnocast::ipc_shared_ptr<rosgraph_msgs::msg::Clock> & msg);

  void attachNode(agnocast::Node * node);
  void detachNode();
  void attachClock(rclcpp::Clock::SharedPtr clock);
};

}  // namespace agnocast::node_interfaces
