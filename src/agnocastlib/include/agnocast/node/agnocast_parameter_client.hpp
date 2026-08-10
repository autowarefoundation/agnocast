#pragma once

#include "agnocast/agnocast_client.hpp"
#include "agnocast/agnocast_public_api.hpp"
#include "rcl_interfaces/srv/describe_parameters.hpp"
#include "rcl_interfaces/srv/get_parameter_types.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/qos.hpp"

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>

namespace agnocast
{

/// @brief Async parameters client for agnocast::Node.
AGNOCAST_PUBLIC
class AsyncParametersClient
{
public:
  AGNOCAST_PUBLIC
  using SharedPtr = std::shared_ptr<AsyncParametersClient>;

  /// @brief Construct an async parameter client.
  /// @param node The async parameters client will be added to this node.
  /// @param remote_node_name Name of the remote node. Defaults to the empty string, meaning self
  ///        node.
  /// @param qos QoS profile for the underlying clients.
  /// @param group Callback group for the underlying clients.
  AGNOCAST_PUBLIC
  explicit AsyncParametersClient(
    agnocast::Node * node, const std::string & remote_node_name = "",
    const rclcpp::QoS & qos = rclcpp::ParametersQoS(),
    const rclcpp::CallbackGroup::SharedPtr & group = nullptr);

  /// @brief Read parameters from the remote node.
  /// @param names Parameter names to read.
  /// @param callback Invoked with the resolved future when the response arrives.
  /// @return Shared future resolving to the parameters.
  AGNOCAST_PUBLIC
  std::shared_future<std::vector<rclcpp::Parameter>> get_parameters(
    const std::vector<std::string> & names,
    std::function<void(std::shared_future<std::vector<rclcpp::Parameter>>)> callback = nullptr);

  /// @brief Read the types of parameters from the remote node.
  /// @param names Parameter names to query.
  /// @param callback Invoked with the resolved future when the response arrives.
  /// @return Shared future resolving to the parameter types.
  AGNOCAST_PUBLIC
  std::shared_future<std::vector<rclcpp::ParameterType>> get_parameter_types(
    const std::vector<std::string> & names,
    std::function<void(std::shared_future<std::vector<rclcpp::ParameterType>>)> callback = nullptr);

  /// @brief Set parameters on the remote node, one by one.
  /// @param parameters Parameters to set.
  /// @param callback Invoked with the resolved future when the response arrives.
  /// @return Shared future resolving to one result per parameter.
  AGNOCAST_PUBLIC
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> set_parameters(
    const std::vector<rclcpp::Parameter> & parameters,
    std::function<void(std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>)>
      callback = nullptr);

  /// @brief Set parameters on the remote node as a single all-or-nothing operation.
  /// @param parameters Parameters to set.
  /// @param callback Invoked with the resolved future when the response arrives.
  /// @return Shared future resolving to the combined result.
  AGNOCAST_PUBLIC
  std::shared_future<rcl_interfaces::msg::SetParametersResult> set_parameters_atomically(
    const std::vector<rclcpp::Parameter> & parameters,
    std::function<void(std::shared_future<rcl_interfaces::msg::SetParametersResult>)> callback =
      nullptr);

  /// @brief Read the descriptors of parameters from the remote node.
  /// @param names Parameter names to describe.
  /// @param callback Invoked with the resolved future when the response arrives.
  /// @return Shared future resolving to the descriptors.
  AGNOCAST_PUBLIC
  std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> describe_parameters(
    const std::vector<std::string> & names,
    std::function<void(std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>>)>
      callback = nullptr);

  /// @brief List the parameters of the remote node.
  /// @param prefixes Only list parameters under these prefixes. Empty lists everything.
  /// @param depth Maximum number of name separators to descend. 0 means unlimited.
  /// @param callback Invoked with the resolved future when the response arrives.
  /// @return Shared future resolving to the listing.
  AGNOCAST_PUBLIC
  std::shared_future<rcl_interfaces::msg::ListParametersResult> list_parameters(
    const std::vector<std::string> & prefixes, uint64_t depth,
    std::function<void(std::shared_future<rcl_interfaces::msg::ListParametersResult>)> callback =
      nullptr);

  /// @brief Check whether every parameter service endpoint of the remote node is available.
  /// @return true if all the endpoints are available.
  AGNOCAST_PUBLIC
  bool service_is_ready() const;

  /// @brief Block until every parameter service endpoint is available or the timeout expires.
  /// @param timeout Maximum total duration to wait across all endpoints (-1 = wait forever).
  /// @return true if all endpoints became available, false on timeout.
  AGNOCAST_PUBLIC
  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    return wait_for_service_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

private:
  bool wait_for_service_nanoseconds(std::chrono::nanoseconds timeout);

  Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;
  Client<rcl_interfaces::srv::GetParameterTypes>::SharedPtr get_parameter_types_client_;
  Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_client_;
  Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_parameters_atomically_client_;
  Client<rcl_interfaces::srv::DescribeParameters>::SharedPtr describe_parameters_client_;
  Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_client_;
  std::string remote_node_name_;
};

}  // namespace agnocast
