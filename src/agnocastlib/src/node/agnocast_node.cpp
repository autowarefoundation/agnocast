#include "agnocast/node/agnocast_node.hpp"

#include "agnocast/node/agnocast_arguments.hpp"
#include "agnocast/node/agnocast_context.hpp"

#include <rcl/time.h>

namespace agnocast
{

Node::Node(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, "", options)
{
}

Node::Node(
  const std::string & node_name, const std::string & namespace_,
  const rclcpp::NodeOptions & options)
: local_args_(parse_arguments(options.arguments()))
{
  node_base_ = std::make_shared<node_interfaces::NodeBase>(
    node_name, namespace_, options.context(), local_args_.get(), options.use_intra_process_comms(),
    options.enable_topic_statistics());
  logger_ = rclcpp::get_logger(node_base_->get_name());

  node_topics_ = std::make_shared<node_interfaces::NodeTopics>(node_base_);

  node_parameters_ = std::make_shared<node_interfaces::NodeParameters>(
    node_base_, options.parameter_overrides(), local_args_.get());

  node_clock_ = std::make_shared<node_interfaces::NodeClock>(RCL_ROS_TIME);

  node_time_source_ = std::make_shared<node_interfaces::NodeTimeSource>(node_clock_, this);

  node_services_ = std::make_shared<node_interfaces::NodeServices>(node_base_);
}

}  // namespace agnocast
