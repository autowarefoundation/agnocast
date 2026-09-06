#include "agnocast/node/agnocast_node.hpp"

#include "agnocast/agnocast_tracepoint_wrapper.h"
#include "agnocast/node/agnocast_arguments.hpp"
#include "agnocast/node/agnocast_context.hpp"
#include "agnocast_context_internal.hpp"

#include <rcl/time.h>

namespace agnocast
{

namespace
{

// local_args_ is the first member, so this runs before anything can observe agnocast::ok().
// That ordering is what lets everything hung off a node -- its clients, its updater, the tf2
// buffer -- rely on "an agnocast::Node exists" implying "agnocast::ok()".
ParsedArguments init_and_parse_arguments(const rclcpp::NodeOptions & options)
{
  ensure_initialized(options.context());
  return parse_arguments(options.arguments());
}

}  // namespace

Node::Node(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, "", options)
{
}

Node::Node(
  const std::string & node_name, const std::string & namespace_,
  const rclcpp::NodeOptions & options)
: local_args_(init_and_parse_arguments(options)),
  node_base_(std::make_shared<node_interfaces::NodeBase>(
    node_name, namespace_, options.context(), local_args_.get(), options.use_global_arguments(),
    options.use_intra_process_comms(), options.enable_topic_statistics())),
  logger_(rclcpp::get_logger(node_base_->get_name())),
  node_graph_(std::make_shared<node_interfaces::NodeGraph>(node_base_)),
  node_topics_(std::make_shared<node_interfaces::NodeTopics>(node_base_)),
  node_services_(std::make_shared<node_interfaces::NodeServices>(node_base_)),
  node_parameters_(std::make_shared<node_interfaces::NodeParameters>(
    node_base_, options.parameter_overrides(), local_args_.get(), options.use_global_arguments(),
    options.allow_undeclared_parameters())),
  node_clock_(std::make_shared<node_interfaces::NodeClock>(RCL_ROS_TIME)),
  node_time_source_(std::make_shared<node_interfaces::NodeTimeSource>(
    node_clock_, this, options.clock_qos(), options.use_clock_thread())),
  node_logging_(std::make_shared<node_interfaces::NodeLogging>(logger_))
{
  if (options.start_parameter_services()) {
    node_parameters_->start_parameter_services(this);
  }

  TRACEPOINT(
    agnocast_node_init, static_cast<const void *>(node_base_.get()), this->get_name().c_str(),
    this->get_namespace().c_str());
}

}  // namespace agnocast
