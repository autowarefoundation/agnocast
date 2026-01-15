#include "agnocast/node/node_interfaces/node_time_source.hpp"

#include "agnocast/bridge/agnocast_bridge_node.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "rcl/time.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"

#include <memory>
#include <string>

namespace agnocast::node_interfaces
{

const std::string use_sim_time_name = "use_sim_time";

NodeTimeSource::NodeTimeSource(
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & node_clock, agnocast::Node * node,
  const rclcpp::QoS & qos)
: qos_(qos)
{
  attachNode(node);
  attachClock(node_clock->get_clock());
}

NodeTimeSource::~NodeTimeSource()
{
  if (node_base_ || node_parameters_) {
    detachNode();
  }
}

// Attach a node to this time source
void NodeTimeSource::attachNode(agnocast::Node * node)
{
  node_base_ = node->get_node_base_interface();
  node_parameters_ = node->get_node_parameters_interface();
  agnocast_node_ = node;

  // Though this defaults to false, it can be overridden by initial parameter values for the
  // node, which may be given by the user at the node's construction or even by command-line
  // arguments.
  rclcpp::ParameterValue use_sim_time_param;
  if (!node_parameters_->has_parameter(use_sim_time_name)) {
    use_sim_time_param =
      node_parameters_->declare_parameter(use_sim_time_name, rclcpp::ParameterValue(false));
  } else {
    use_sim_time_param = node_parameters_->get_parameter(use_sim_time_name).get_parameter_value();
  }
  if (use_sim_time_param.get_type() == rclcpp::PARAMETER_BOOL) {
    if (use_sim_time_param.get<bool>()) {
      enable_ros_time();
      create_clock_sub();
    }
  } else {
    RCLCPP_ERROR(
      logger_, "Invalid type '%s' for parameter 'use_sim_time', should be 'bool'",
      rclcpp::to_string(use_sim_time_param.get_type()).c_str());
    throw std::invalid_argument("Invalid type for parameter 'use_sim_time', should be 'bool'");
  }

  // TODO(Koichi98): Add parameter event handling
}

// Detach the attached node
void NodeTimeSource::detachNode()
{
  // destroy_clock_sub() *must* be first here, to ensure that the executor
  // can't possibly call any of the callbacks as we are cleaning up.
  destroy_clock_sub();
  disable_ros_time();
  node_base_.reset();
  node_parameters_.reset();
  agnocast_node_ = nullptr;
}

void NodeTimeSource::attachClock(rclcpp::Clock::SharedPtr clock)
{
  if (clock->get_clock_type() != RCL_ROS_TIME) {
    throw std::invalid_argument("Cannot attach a clock that is not ROS_TIME");
  }

  clock_ = std::move(clock);

  // TODO(Koichi98): Set the clock to zero unless there's a recently received message

  builtin_interfaces::msg::Time time_msg;
  set_clock(time_msg, ros_time_active_);
}

void NodeTimeSource::enable_ros_time()
{
  if (ros_time_active_) {
    // already enabled no-op
    return;
  }

  // Local storage
  ros_time_active_ = true;

  // Update the attached clock to zero
  builtin_interfaces::msg::Time time_msg;
  set_clock(time_msg, true);
}

void NodeTimeSource::disable_ros_time()
{
  if (!ros_time_active_) {
    // already disabled no-op
    return;
  }

  // Local storage
  ros_time_active_ = false;

  // Update the attached clock
  builtin_interfaces::msg::Time time_msg;
  set_clock(time_msg, false);
}

void NodeTimeSource::set_clock(const builtin_interfaces::msg::Time & msg, bool set_ros_time_enabled)
{
  if (!clock_) {
    return;
  }

  std::lock_guard<std::mutex> clock_guard(clock_->get_clock_mutex());

  // Do change
  if (!set_ros_time_enabled && clock_->ros_time_is_active()) {
    auto ret = rcl_disable_ros_time_override(clock_->get_clock_handle());
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to disable ros_time_override");
    }
  } else if (set_ros_time_enabled && !clock_->ros_time_is_active()) {
    auto ret = rcl_enable_ros_time_override(clock_->get_clock_handle());
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to enable ros_time_override");
    }
  }

  auto ret = rcl_set_ros_time_override(clock_->get_clock_handle(), rclcpp::Time(msg).nanoseconds());
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to set ros_time_override");
  }
}

void NodeTimeSource::create_clock_sub()
{
  std::lock_guard<std::mutex> guard(clock_sub_lock_);
  if (clock_subscription_) {
    // Subscription already created.
    return;
  }

  clock_subscription_ = agnocast_node_->create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock", qos_,
    [this](const agnocast::ipc_shared_ptr<rosgraph_msgs::msg::Clock> & msg) { clock_cb(msg); });
}

void NodeTimeSource::destroy_clock_sub()
{
  std::lock_guard<std::mutex> guard(clock_sub_lock_);
  clock_subscription_.reset();
}

void NodeTimeSource::clock_cb(const agnocast::ipc_shared_ptr<rosgraph_msgs::msg::Clock> & msg)
{
  if (!ros_time_active_) {
    enable_ros_time();
  }

  set_clock(msg->clock, true);
}

}  // namespace agnocast::node_interfaces
