#include "agnocast/node_interfaces/node_topics.hpp"

#include <rcl/expand_topic_name.h>
#include <rcl/remap.h>
#include <rcutils/types/string_map.h>

#include <stdexcept>
#include <utility>

namespace agnocast::node_interfaces
{

NodeTopics::NodeTopics(NodeBase::SharedPtr node_base) : node_base_(std::move(node_base))
{
}

std::string NodeTopics::resolve_topic_name(const std::string & input_name, bool only_expand) const
{
  if (input_name.empty()) {
    throw std::invalid_argument("topic name must not be empty");
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcutils_allocator_t rcutils_allocator = rcutils_get_default_allocator();

  rcutils_string_map_t substitutions = rcutils_get_zero_initialized_string_map();
  rcutils_ret_t rcutils_ret = rcutils_string_map_init(&substitutions, 0, rcutils_allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    throw std::runtime_error("Failed to initialize substitutions map");
  }
  // TODO(Koichi98): Call rcl_get_default_topic_name_substitutions when custom substitutions are
  // supported.

  // Expand the topic name using rcl_expand_topic_name
  char * expanded_name = nullptr;
  rcl_ret_t ret = rcl_expand_topic_name(
    input_name.c_str(), node_base_->get_name(), node_base_->get_namespace(), &substitutions,
    allocator, &expanded_name);

  rcutils_ret = rcutils_string_map_fini(&substitutions);

  if (ret != RCL_RET_OK) {
    auto error_state = rcl_get_error_string();
    std::string error_msg = &error_state.str[0];
    rcl_reset_error();
    throw std::runtime_error("Failed to expand topic name: " + error_msg);
  }

  if (rcutils_ret != RCUTILS_RET_OK) {
    if (expanded_name != nullptr) {
      allocator.deallocate(expanded_name, allocator.state);
    }
    auto error_state = rcutils_get_error_string();
    throw std::runtime_error(
      "Failed to fini substitutions map: " + std::string(&error_state.str[0]));
  }

  std::string result(expanded_name);
  allocator.deallocate(expanded_name, allocator.state);

  if (only_expand) {
    return result;
  }

  // Remap the topic name using rcl_remap_topic_name
  char * remapped_name = nullptr;
  ret = rcl_remap_topic_name(
    node_base_->get_local_args(), node_base_->get_global_args(), result.c_str(),
    node_base_->get_name(), node_base_->get_namespace(), allocator, &remapped_name);

  if (ret != RCL_RET_OK) {
    rcl_reset_error();
    return result;
  }

  if (remapped_name != nullptr) {
    result = remapped_name;
    allocator.deallocate(remapped_name, allocator.state);
  }

  return result;
}

rclcpp::node_interfaces::NodeBaseInterface * NodeTopics::get_node_base_interface() const
{
  return node_base_.get();
}

rclcpp::PublisherBase::SharedPtr NodeTopics::create_publisher(
  const std::string & topic_name, const rclcpp::PublisherFactory & publisher_factory,
  const rclcpp::QoS & qos)
{
  (void)topic_name;
  (void)publisher_factory;
  (void)qos;
  throw std::runtime_error(
    "NodeTopics::create_publisher is not supported in agnocast. "
    "Use agnocast::create_publisher instead.");
}

void NodeTopics::add_publisher(
  rclcpp::PublisherBase::SharedPtr publisher, rclcpp::CallbackGroup::SharedPtr callback_group)
{
  (void)publisher;
  (void)callback_group;
  throw std::runtime_error(
    "NodeTopics::add_publisher is not supported in agnocast. "
    "Use agnocast::create_publisher instead.");
}

rclcpp::SubscriptionBase::SharedPtr NodeTopics::create_subscription(
  const std::string & topic_name, const rclcpp::SubscriptionFactory & subscription_factory,
  const rclcpp::QoS & qos)
{
  (void)topic_name;
  (void)subscription_factory;
  (void)qos;
  throw std::runtime_error(
    "NodeTopics::create_subscription is not supported in agnocast. "
    "Use agnocast::create_subscription instead.");
}

void NodeTopics::add_subscription(
  rclcpp::SubscriptionBase::SharedPtr subscription, rclcpp::CallbackGroup::SharedPtr callback_group)
{
  (void)subscription;
  (void)callback_group;
  throw std::runtime_error(
    "NodeTopics::add_subscription is not supported in agnocast. "
    "Use agnocast::create_subscription instead.");
}

rclcpp::node_interfaces::NodeTimersInterface * NodeTopics::get_node_timers_interface() const
{
  throw std::runtime_error(
    "NodeTopics::get_node_timers_interface is not supported in agnocast. "
    "Timers interface is not available.");
}

}  // namespace agnocast::node_interfaces
