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

/// @brief Read and write another node's parameters over Agnocast.
///
/// The counterpart of agnocast::ParameterService: it holds one agnocast::Client per parameter
/// service endpoint of the remote node (`<remote_node>/get_parameters` and friends) and exposes
/// them as a single object, mirroring rclcpp::AsyncParametersClient.
///
/// This class exists because rclcpp::AsyncParametersClient cannot be used from an agnocast::Node:
/// its node-taking constructors require get_node_graph_interface(), which agnocast::Node does not
/// provide, and in an AgnocastOnly process rclcpp::init() is never called so its clients could not
/// discover anything anyway.
///
/// Responses are delivered on the executor thread serving @p group, exactly like the underlying
/// agnocast::Client, so the returned futures only resolve while the node is being spun. Requests
/// to a remote node that still speaks ROS 2 are carried by the A2R service bridge.
///
/// Differences from rclcpp::AsyncParametersClient:
///   - on_parameter_event() is not provided; agnocast::Node does not publish /parameter_events.
///   - delete_parameters() / load_parameters() are not provided; call set_parameters() directly.
///   - There is no synchronous counterpart (rclcpp::SyncParametersClient), because it is built on
///     spin_until_future_complete(), which the agnocast executors do not offer.
///   - service_is_ready() checks all six endpoints (rclcpp skips set_parameters_atomically).
class AsyncParametersClient
{
public:
  AGNOCAST_PUBLIC
  using SharedPtr = std::shared_ptr<AsyncParametersClient>;

  /// @brief Construct a parameter client targeting @p remote_node_name.
  /// @tparam NodeT agnocast::Node or rclcpp::Node.
  /// @param node The node that owns the underlying clients.
  /// @param remote_node_name Name of the node whose parameters are accessed. Resolved like a
  ///        service name, so a relative name is expanded against @p node's namespace. Defaults to
  ///        the empty string, meaning @p node itself.
  /// @param qos Quality of service profile for the underlying clients.
  /// @param group Callback group the responses are delivered on. Defaults to `nullptr` (the
  ///        node's default callback group).
  template <typename NodeT>
  explicit AsyncParametersClient(
    NodeT * node, const std::string & remote_node_name = "",
    const rclcpp::QoS & qos = rclcpp::ParametersQoS(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    remote_node_name_ =
      remote_node_name.empty() ? node->get_fully_qualified_name() : remote_node_name;

    get_parameters_client_ = std::make_shared<Client<rcl_interfaces::srv::GetParameters>>(
      node, remote_node_name_ + "/get_parameters", qos, group);
    get_parameter_types_client_ = std::make_shared<Client<rcl_interfaces::srv::GetParameterTypes>>(
      node, remote_node_name_ + "/get_parameter_types", qos, group);
    set_parameters_client_ = std::make_shared<Client<rcl_interfaces::srv::SetParameters>>(
      node, remote_node_name_ + "/set_parameters", qos, group);
    set_parameters_atomically_client_ =
      std::make_shared<Client<rcl_interfaces::srv::SetParametersAtomically>>(
        node, remote_node_name_ + "/set_parameters_atomically", qos, group);
    describe_parameters_client_ = std::make_shared<Client<rcl_interfaces::srv::DescribeParameters>>(
      node, remote_node_name_ + "/describe_parameters", qos, group);
    list_parameters_client_ = std::make_shared<Client<rcl_interfaces::srv::ListParameters>>(
      node, remote_node_name_ + "/list_parameters", qos, group);
  }

  virtual ~AsyncParametersClient() = default;

  AsyncParametersClient(const AsyncParametersClient &) = delete;
  AsyncParametersClient & operator=(const AsyncParametersClient &) = delete;
  AsyncParametersClient(AsyncParametersClient &&) = delete;
  AsyncParametersClient & operator=(AsyncParametersClient &&) = delete;

  /// @brief Read parameters from the remote node.
  /// @param names Parameter names to read.
  /// @param callback Optional; invoked with the resolved future when the response arrives.
  /// @return Future resolving to the values, in the order of @p names. Note that a single
  ///         undeclared name makes the server return nothing at all rather than skipping just that
  ///         name, so a short result does not identify which names exist.
  AGNOCAST_PUBLIC
  std::shared_future<std::vector<rclcpp::Parameter>> get_parameters(
    const std::vector<std::string> & names,
    std::function<void(std::shared_future<std::vector<rclcpp::Parameter>>)> callback = nullptr);

  /// @brief Read the types of parameters on the remote node.
  /// @param names Parameter names to query.
  /// @param callback Optional; invoked with the resolved future when the response arrives.
  /// @return Future resolving to the parameter types.
  AGNOCAST_PUBLIC
  std::shared_future<std::vector<rclcpp::ParameterType>> get_parameter_types(
    const std::vector<std::string> & names,
    std::function<void(std::shared_future<std::vector<rclcpp::ParameterType>>)> callback = nullptr);

  /// @brief Set parameters on the remote node, one by one.
  /// @param parameters Parameters to set.
  /// @param callback Optional; invoked with the resolved future when the response arrives.
  /// @return Future resolving to one result per parameter.
  AGNOCAST_PUBLIC
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> set_parameters(
    const std::vector<rclcpp::Parameter> & parameters,
    std::function<void(std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>)>
      callback = nullptr);

  /// @brief Set parameters on the remote node as a single all-or-nothing operation.
  /// @param parameters Parameters to set.
  /// @param callback Optional; invoked with the resolved future when the response arrives.
  /// @return Future resolving to the combined result.
  AGNOCAST_PUBLIC
  std::shared_future<rcl_interfaces::msg::SetParametersResult> set_parameters_atomically(
    const std::vector<rclcpp::Parameter> & parameters,
    std::function<void(std::shared_future<rcl_interfaces::msg::SetParametersResult>)> callback =
      nullptr);

  /// @brief Read the descriptors of parameters on the remote node.
  /// @param names Parameter names to describe.
  /// @param callback Optional; invoked with the resolved future when the response arrives.
  /// @return Future resolving to the descriptors.
  AGNOCAST_PUBLIC
  std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> describe_parameters(
    const std::vector<std::string> & names,
    std::function<void(std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>>)>
      callback = nullptr);

  /// @brief List the parameters of the remote node.
  /// @param prefixes Only list parameters under these prefixes. Empty lists everything.
  /// @param depth Maximum number of name separators to descend. 0 means unlimited.
  /// @param callback Optional; invoked with the resolved future when the response arrives.
  /// @return Future resolving to the listing.
  AGNOCAST_PUBLIC
  std::shared_future<rcl_interfaces::msg::ListParametersResult> list_parameters(
    const std::vector<std::string> & prefixes, uint64_t depth,
    std::function<void(std::shared_future<rcl_interfaces::msg::ListParametersResult>)> callback =
      nullptr);

  /// @brief Check whether every parameter service endpoint of the remote node is available.
  /// @return True if all six endpoints are available.
  AGNOCAST_PUBLIC
  bool service_is_ready() const;

  /// @brief Block until every parameter service endpoint is available or the timeout expires.
  /// @param timeout Maximum total duration to wait across all endpoints (-1 = wait forever).
  /// @return True if all endpoints became available, false on timeout.
  AGNOCAST_PUBLIC
  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    return wait_for_service_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

protected:
  AGNOCAST_PUBLIC
  bool wait_for_service_nanoseconds(std::chrono::nanoseconds timeout);

private:
  Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;
  Client<rcl_interfaces::srv::GetParameterTypes>::SharedPtr get_parameter_types_client_;
  Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_client_;
  Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_parameters_atomically_client_;
  Client<rcl_interfaces::srv::DescribeParameters>::SharedPtr describe_parameters_client_;
  Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_client_;
  std::string remote_node_name_;
};

}  // namespace agnocast
