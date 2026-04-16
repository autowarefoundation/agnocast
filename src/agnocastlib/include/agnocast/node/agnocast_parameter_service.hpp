#pragma once

#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/agnocast_service.hpp"
#include "agnocast/node/node_interfaces/node_base.hpp"
#include "agnocast/node/node_interfaces/node_parameters.hpp"
#include "agnocast/node/node_interfaces/node_services.hpp"
#include "rcl_interfaces/srv/describe_parameters.hpp"
#include "rcl_interfaces/srv/get_parameter_types.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rclcpp/qos.hpp"

namespace agnocast
{

/// @brief Manage ROS 2 parameter service endpoints for agnocast::Node.
AGNOCAST_PUBLIC
class ParameterService
{
public:
  using SharedPtr = std::shared_ptr<ParameterService>;
  using WeakPtr = std::weak_ptr<ParameterService>;
  using UniquePtr = std::unique_ptr<ParameterService>;

  /// @brief Construct a ParameterService, registering all parameter management services.
  /// @param node The node's pointer
  /// @param node_params The node's parameters interface
  /// @param qos_profile The QoS profile for all parameter services. Defaults to `rclcpp::QoS()`.
  AGNOCAST_PUBLIC
  ParameterService(
    agnocast::Node * node, agnocast::node_interfaces::NodeParameters * node_params,
    const rclcpp::QoS & qos_profile = rclcpp::ParametersQoS());

private:
  agnocast::Service<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_service_;
  agnocast::Service<rcl_interfaces::srv::GetParameterTypes>::SharedPtr get_parameter_types_service_;
  agnocast::Service<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_service_;
  agnocast::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr
    set_parameters_atomically_service_;
  agnocast::Service<rcl_interfaces::srv::DescribeParameters>::SharedPtr
    describe_parameters_service_;
  agnocast::Service<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_service_;
};

}  // namespace agnocast
