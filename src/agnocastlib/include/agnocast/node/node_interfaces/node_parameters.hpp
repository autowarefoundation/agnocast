#pragma once

#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/parameter.hpp"

#include <rcl/arguments.h>

#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace agnocast::node_interfaces
{

using rclcpp::node_interfaces::ParameterInfo;

class NodeParameters : public rclcpp::node_interfaces::NodeParametersInterface
{
public:
  using SharedPtr = std::shared_ptr<NodeParameters>;
  using WeakPtr = std::weak_ptr<NodeParameters>;
  using CallbacksContainerType =
    std::list<rclcpp::node_interfaces::OnSetParametersCallbackHandle::WeakPtr>;

  explicit NodeParameters(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const std::vector<rclcpp::Parameter> & parameter_overrides, const rcl_arguments_t * local_args,
    bool allow_undeclared_parameters = false);

  virtual ~NodeParameters() = default;

  const rclcpp::ParameterValue & declare_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
      rcl_interfaces::msg::ParameterDescriptor(),
    bool ignore_override = false) override;

  const rclcpp::ParameterValue & declare_parameter(
    const std::string & name, rclcpp::ParameterType type,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
      rcl_interfaces::msg::ParameterDescriptor(),
    bool ignore_override = false) override;

  void undeclare_parameter(const std::string & name) override;

  bool has_parameter(const std::string & name) const override;

  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters(
    const std::vector<rclcpp::Parameter> & parameters) override;

  rcl_interfaces::msg::SetParametersResult set_parameters_atomically(
    const std::vector<rclcpp::Parameter> & parameters) override;

  std::vector<rclcpp::Parameter> get_parameters(
    const std::vector<std::string> & names) const override;

  rclcpp::Parameter get_parameter(const std::string & name) const override;

  bool get_parameter(const std::string & name, rclcpp::Parameter & parameter) const override;

  bool get_parameters_by_prefix(
    const std::string & prefix,
    std::map<std::string, rclcpp::Parameter> & parameters) const override;

  std::vector<rcl_interfaces::msg::ParameterDescriptor> describe_parameters(
    const std::vector<std::string> & names) const override;

  std::vector<uint8_t> get_parameter_types(const std::vector<std::string> & names) const override;

  rcl_interfaces::msg::ListParametersResult list_parameters(
    const std::vector<std::string> & prefixes, uint64_t depth) const override;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr add_on_set_parameters_callback(
    OnParametersSetCallbackType callback) override;

  void remove_on_set_parameters_callback(
    const rclcpp::node_interfaces::OnSetParametersCallbackHandle * const handler) override;

  const std::map<std::string, rclcpp::ParameterValue> & get_parameter_overrides() const override;

private:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;

  mutable std::mutex parameters_mutex_;
  std::map<std::string, rclcpp::ParameterValue> parameter_overrides_;
  std::map<std::string, ParameterInfo> parameters_;

  CallbacksContainerType on_parameters_set_callback_container_;

  bool allow_undeclared_ = false;
};

}  // namespace agnocast::node_interfaces
