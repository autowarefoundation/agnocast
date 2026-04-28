#include "agnocast/node/agnocast_parameter_service.hpp"

#include "agnocast/node/agnocast_node.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/parameter.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace agnocast
{

namespace parameter_service_names
{

static constexpr const char * get_parameters = "get_parameters";
static constexpr const char * get_parameter_types = "get_parameter_types";
static constexpr const char * set_parameters = "set_parameters";
static constexpr const char * set_parameters_atomically = "set_parameters_atomically";
static constexpr const char * describe_parameters = "describe_parameters";
static constexpr const char * list_parameters = "list_parameters";

}  // namespace parameter_service_names

ParameterService::ParameterService(
  agnocast::Node * node, agnocast::node_interfaces::NodeParameters * node_params,
  const rclcpp::QoS & qos_profile)
{
  using GetParameters = rcl_interfaces::srv::GetParameters;
  using GetParameterTypes = rcl_interfaces::srv::GetParameterTypes;
  using SetParameters = rcl_interfaces::srv::SetParameters;
  using SetParametersAtomically = rcl_interfaces::srv::SetParametersAtomically;
  using DescribeParameters = rcl_interfaces::srv::DescribeParameters;
  using ListParameters = rcl_interfaces::srv::ListParameters;

  const std::string node_name = node->get_name();

  get_parameters_service_ = std::make_shared<Service<GetParameters>>(
    node, node_name + "/" + parameter_service_names::get_parameters,
    [node, node_params](
      const agnocast::ipc_shared_ptr<GetParameters::Request> & request,
      const agnocast::ipc_shared_ptr<GetParameters::Response> & response) {
      try {
        auto parameters = node_params->get_parameters(request->names);
        for (const auto & param : parameters) {
          response->values.push_back(param.get_value_message());
        }
      } catch (const rclcpp::exceptions::ParameterNotDeclaredException & ex) {
        RCLCPP_WARN(node->get_logger(), "Failed to get parameters: %s", ex.what());
      } catch (const rclcpp::exceptions::ParameterUninitializedException & ex) {
        RCLCPP_WARN(node->get_logger(), "Failed to get parameters: %s", ex.what());
      }
    },
    qos_profile, nullptr);

  get_parameter_types_service_ = std::make_shared<Service<GetParameterTypes>>(
    node, node_name + "/" + parameter_service_names::get_parameter_types,
    [node, node_params](
      const agnocast::ipc_shared_ptr<GetParameterTypes::Request> & request,
      const agnocast::ipc_shared_ptr<GetParameterTypes::Response> & response) {
      try {
        auto types = node_params->get_parameter_types(request->names);
        std::transform(
          types.cbegin(), types.cend(), std::back_inserter(response->types),
          [](const uint8_t & type) { return static_cast<rclcpp::ParameterType>(type); });
      } catch (const rclcpp::exceptions::ParameterNotDeclaredException & ex) {
        RCLCPP_WARN(node->get_logger(), "Failed to get parameter types: %s", ex.what());
      }
    },
    qos_profile, nullptr);

  set_parameters_service_ = std::make_shared<Service<SetParameters>>(
    node, node_name + "/" + parameter_service_names::set_parameters,
    [node, node_params](
      const agnocast::ipc_shared_ptr<SetParameters::Request> & request,
      const agnocast::ipc_shared_ptr<SetParameters::Response> & response) {
      auto result = rcl_interfaces::msg::SetParametersResult();
      for (auto & p : request->parameters) {
        try {
          result =
            node_params->set_parameters_atomically({rclcpp::Parameter::from_parameter_msg(p)});
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException & ex) {
          RCLCPP_WARN(node->get_logger(), "Failed to set parameter: %s", ex.what());
          result.successful = false;
          result.reason = ex.what();
        }
        response->results.push_back(result);
      }
    },
    qos_profile, nullptr);

  set_parameters_atomically_service_ = std::make_shared<Service<SetParametersAtomically>>(
    node, node_name + "/" + parameter_service_names::set_parameters_atomically,
    [node, node_params](
      const agnocast::ipc_shared_ptr<SetParametersAtomically::Request> & request,
      const agnocast::ipc_shared_ptr<SetParametersAtomically::Response> & response) {
      std::vector<rclcpp::Parameter> pvariants;
      std::transform(
        request->parameters.cbegin(), request->parameters.cend(), std::back_inserter(pvariants),
        [](const rcl_interfaces::msg::Parameter & p) {
          return rclcpp::Parameter::from_parameter_msg(p);
        });
      try {
        auto result = node_params->set_parameters_atomically(pvariants);
        response->result = result;
      } catch (const rclcpp::exceptions::ParameterNotDeclaredException & ex) {
        RCLCPP_WARN(node->get_logger(), "Failed to set parameters atomically: %s", ex.what());
        response->result.successful = false;
        response->result.reason = "One or more parameters were not declared before setting";
      }
    },
    qos_profile, nullptr);

  describe_parameters_service_ = std::make_shared<Service<DescribeParameters>>(
    node, node_name + "/" + parameter_service_names::describe_parameters,
    [node, node_params](
      const agnocast::ipc_shared_ptr<DescribeParameters::Request> & request,
      const agnocast::ipc_shared_ptr<DescribeParameters::Response> & response) {
      try {
        auto descriptors = node_params->describe_parameters(request->names);
        response->descriptors = descriptors;
      } catch (const rclcpp::exceptions::ParameterNotDeclaredException & ex) {
        RCLCPP_WARN(node->get_logger(), "Failed to describe parameters: %s", ex.what());
      }
    },
    qos_profile, nullptr);

  list_parameters_service_ = std::make_shared<Service<ListParameters>>(
    node, node_name + "/" + parameter_service_names::list_parameters,
    [node_params](
      const agnocast::ipc_shared_ptr<ListParameters::Request> & request,
      const agnocast::ipc_shared_ptr<ListParameters::Response> & response) {
      auto result = node_params->list_parameters(request->prefixes, request->depth);
      response->result = result;
    },
    qos_profile, nullptr);
}

}  // namespace agnocast
