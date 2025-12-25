#pragma once

#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/node/agnocast_arguments.hpp"
#include "agnocast/node/agnocast_context.hpp"
#include "agnocast/node/node_interfaces/node_base.hpp"
#include "agnocast/node/node_interfaces/node_parameters.hpp"
#include "agnocast/node/node_interfaces/node_topics.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace agnocast
{

using ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor;

class Node
{
public:
  using SharedPtr = std::shared_ptr<Node>;
  using ParameterValue = rclcpp::ParameterValue;

  explicit Node(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit Node(
    const std::string & node_name, const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  std::string get_name() const { return node_base_->get_name(); }
  rclcpp::Logger get_logger() const { return logger_; }
  std::string get_namespace() const { return node_base_->get_namespace(); }
  std::string get_fully_qualified_name() const { return node_base_->get_fully_qualified_name(); }

  rclcpp::CallbackGroup::SharedPtr get_default_callback_group()
  {
    return node_base_->get_default_callback_group();
  }

  rclcpp::CallbackGroup::SharedPtr create_callback_group(
    rclcpp::CallbackGroupType group_type, bool automatically_add_to_executor_with_node = true)
  {
    return node_base_->create_callback_group(group_type, automatically_add_to_executor_with_node);
  }

  bool callback_group_in_node(const rclcpp::CallbackGroup::SharedPtr & callback_group)
  {
    return node_base_->callback_group_in_node(callback_group);
  }

  // Non-const to align with rclcpp::Node API
  // cppcheck-suppress functionConst
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_base_;
  }

  // Non-const to align with rclcpp::Node API
  // cppcheck-suppress functionConst
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface()
  {
    return node_topics_;
  }

  // Non-const to align with rclcpp::Node API
  // cppcheck-suppress functionConst
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr get_node_parameters_interface()
  {
    return node_parameters_;
  }

  const ParameterValue & declare_parameter(
    const std::string & name, const ParameterValue & default_value,
    const ParameterDescriptor & descriptor = ParameterDescriptor{}, bool ignore_override = false)
  {
    return node_parameters_->declare_parameter(name, default_value, descriptor, ignore_override);
  }

  template <typename ParameterT>
  ParameterT declare_parameter(
    const std::string & name, const ParameterT & default_value,
    const ParameterDescriptor & descriptor = ParameterDescriptor{}, bool ignore_override = false)
  {
    return declare_parameter(
             name, rclcpp::ParameterValue(default_value), descriptor, ignore_override)
      .get<ParameterT>();
  }

  bool has_parameter(const std::string & name) const
  {
    return node_parameters_->has_parameter(name);
  }

  void undeclare_parameter(const std::string & name)
  {
    node_parameters_->undeclare_parameter(name);
  }

  rclcpp::Parameter get_parameter(const std::string & name) const
  {
    return node_parameters_->get_parameter(name);
  }

  bool get_parameter(const std::string & name, rclcpp::Parameter & parameter) const
  {
    return node_parameters_->get_parameter(name, parameter);
  }

  template <typename ParameterT>
  bool get_parameter(const std::string & name, ParameterT & parameter) const
  {
    rclcpp::Parameter param;
    bool result = node_parameters_->get_parameter(name, param);
    if (result) {
      parameter = param.get_value<ParameterT>();
    }
    return result;
  }

  std::vector<rclcpp::Parameter> get_parameters(const std::vector<std::string> & names) const
  {
    return node_parameters_->get_parameters(names);
  }

  template <typename MessageT, typename Func>
  typename agnocast::Subscription<MessageT>::SharedPtr create_subscription(
    const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions{})
  {
    return std::make_shared<Subscription<MessageT>>(
      this, topic_name, qos, std::forward<Func>(callback), options);
  }

  template <typename MessageT, typename Func>
  typename agnocast::Subscription<MessageT>::SharedPtr create_subscription(
    const std::string & topic_name, size_t queue_size, Func && callback,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions{})
  {
    return std::make_shared<Subscription<MessageT>>(
      this, topic_name, rclcpp::QoS(rclcpp::KeepLast(queue_size)), std::forward<Func>(callback),
      options);
  }

private:
  // ParsedArguments must be stored to keep rcl_arguments_t alive
  ParsedArguments local_args_;

  rclcpp::Logger logger_{rclcpp::get_logger("agnocast_node")};
  node_interfaces::NodeBase::SharedPtr node_base_;
  node_interfaces::NodeParameters::SharedPtr node_parameters_;
  node_interfaces::NodeTopics::SharedPtr node_topics_;
};

}  // namespace agnocast
