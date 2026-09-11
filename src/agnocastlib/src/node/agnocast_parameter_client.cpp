#include "agnocast/node/agnocast_parameter_client.hpp"

#include "agnocast/node/agnocast_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace agnocast
{

AsyncParametersClient::AsyncParametersClient(
  agnocast::Node * node, const std::string & remote_node_name, const rclcpp::QoS & qos,
  const rclcpp::CallbackGroup::SharedPtr & group)
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

std::shared_future<std::vector<rclcpp::Parameter>> AsyncParametersClient::get_parameters(
  const std::vector<std::string> & names,
  std::function<void(std::shared_future<std::vector<rclcpp::Parameter>>)> callback)
{
  auto promise = std::make_shared<std::promise<std::vector<rclcpp::Parameter>>>();
  auto future = promise->get_future().share();

  auto request = get_parameters_client_->borrow_loaned_request();
  request->names = names;

  get_parameters_client_->async_send_request(
    std::move(request),
    [names, promise = std::move(promise), future, callback = std::move(callback)](
      const Client<rcl_interfaces::srv::GetParameters>::SharedFuture & response_future) {
      const auto & response = response_future.get();

      const size_t count = std::min(names.size(), response->values.size());
      std::vector<rclcpp::Parameter> parameters;
      parameters.reserve(count);
      for (size_t i = 0; i < count; ++i) {
        rcl_interfaces::msg::Parameter parameter;
        parameter.name = names[i];
        parameter.value = response->values[i];
        parameters.push_back(rclcpp::Parameter::from_parameter_msg(parameter));
      }

      promise->set_value(std::move(parameters));
      if (callback != nullptr) {
        callback(future);
      }
    });

  return future;
}

std::shared_future<std::vector<rclcpp::ParameterType>> AsyncParametersClient::get_parameter_types(
  const std::vector<std::string> & names,
  std::function<void(std::shared_future<std::vector<rclcpp::ParameterType>>)> callback)
{
  auto promise = std::make_shared<std::promise<std::vector<rclcpp::ParameterType>>>();
  auto future = promise->get_future().share();

  auto request = get_parameter_types_client_->borrow_loaned_request();
  request->names = names;

  get_parameter_types_client_->async_send_request(
    std::move(request),
    [promise = std::move(promise), future, callback = std::move(callback)](
      const Client<rcl_interfaces::srv::GetParameterTypes>::SharedFuture & response_future) {
      const auto & response = response_future.get();

      std::vector<rclcpp::ParameterType> types;
      types.reserve(response->types.size());
      for (const auto & type : response->types) {
        types.push_back(static_cast<rclcpp::ParameterType>(type));
      }

      promise->set_value(std::move(types));
      if (callback != nullptr) {
        callback(future);
      }
    });

  return future;
}

std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
AsyncParametersClient::set_parameters(
  const std::vector<rclcpp::Parameter> & parameters,
  std::function<void(std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>)>
    callback)
{
  auto promise =
    std::make_shared<std::promise<std::vector<rcl_interfaces::msg::SetParametersResult>>>();
  auto future = promise->get_future().share();

  auto request = set_parameters_client_->borrow_loaned_request();
  request->parameters.reserve(parameters.size());
  for (const auto & parameter : parameters) {
    request->parameters.push_back(parameter.to_parameter_msg());
  }

  set_parameters_client_->async_send_request(
    std::move(request),
    [promise = std::move(promise), future, callback = std::move(callback)](
      const Client<rcl_interfaces::srv::SetParameters>::SharedFuture & response_future) {
      const auto & response = response_future.get();
      promise->set_value(response->results);
      if (callback != nullptr) {
        callback(future);
      }
    });

  return future;
}

std::shared_future<rcl_interfaces::msg::SetParametersResult>
AsyncParametersClient::set_parameters_atomically(
  const std::vector<rclcpp::Parameter> & parameters,
  std::function<void(std::shared_future<rcl_interfaces::msg::SetParametersResult>)> callback)
{
  auto promise = std::make_shared<std::promise<rcl_interfaces::msg::SetParametersResult>>();
  auto future = promise->get_future().share();

  auto request = set_parameters_atomically_client_->borrow_loaned_request();
  request->parameters.reserve(parameters.size());
  for (const auto & parameter : parameters) {
    request->parameters.push_back(parameter.to_parameter_msg());
  }

  set_parameters_atomically_client_->async_send_request(
    std::move(request),
    [promise = std::move(promise), future, callback = std::move(callback)](
      const Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture & response_future) {
      const auto & response = response_future.get();
      promise->set_value(response->result);
      if (callback != nullptr) {
        callback(future);
      }
    });

  return future;
}

std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>>
AsyncParametersClient::describe_parameters(
  const std::vector<std::string> & names,
  std::function<void(std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>>)>
    callback)
{
  auto promise =
    std::make_shared<std::promise<std::vector<rcl_interfaces::msg::ParameterDescriptor>>>();
  auto future = promise->get_future().share();

  auto request = describe_parameters_client_->borrow_loaned_request();
  request->names = names;

  describe_parameters_client_->async_send_request(
    std::move(request),
    [promise = std::move(promise), future, callback = std::move(callback)](
      const Client<rcl_interfaces::srv::DescribeParameters>::SharedFuture & response_future) {
      const auto & response = response_future.get();
      promise->set_value(response->descriptors);
      if (callback != nullptr) {
        callback(future);
      }
    });

  return future;
}

std::shared_future<rcl_interfaces::msg::ListParametersResult>
AsyncParametersClient::list_parameters(
  const std::vector<std::string> & prefixes, uint64_t depth,
  std::function<void(std::shared_future<rcl_interfaces::msg::ListParametersResult>)> callback)
{
  auto promise = std::make_shared<std::promise<rcl_interfaces::msg::ListParametersResult>>();
  auto future = promise->get_future().share();

  auto request = list_parameters_client_->borrow_loaned_request();
  request->prefixes = prefixes;
  request->depth = depth;

  list_parameters_client_->async_send_request(
    std::move(request),
    [promise = std::move(promise), future, callback = std::move(callback)](
      const Client<rcl_interfaces::srv::ListParameters>::SharedFuture & response_future) {
      const auto & response = response_future.get();
      promise->set_value(response->result);
      if (callback != nullptr) {
        callback(future);
      }
    });

  return future;
}

bool AsyncParametersClient::service_is_ready() const
{
  return get_parameters_client_->service_is_ready() &&
         get_parameter_types_client_->service_is_ready() &&
         set_parameters_client_->service_is_ready() &&
         set_parameters_atomically_client_->service_is_ready() &&
         describe_parameters_client_->service_is_ready() &&
         list_parameters_client_->service_is_ready();
}

bool AsyncParametersClient::wait_for_service_nanoseconds(std::chrono::nanoseconds timeout)
{
  // The timeout is a budget for the whole set of endpoints, so charge each wait against it.
  // A negative timeout means "wait forever" and is passed through untouched.
  auto wait_one = [&timeout](const auto & client) {
    const auto start = std::chrono::steady_clock::now();
    if (!client->wait_for_service(timeout)) {
      return false;
    }
    if (timeout > std::chrono::nanoseconds::zero()) {
      timeout -= std::chrono::steady_clock::now() - start;
      timeout = std::max(timeout, std::chrono::nanoseconds::zero());
    }
    return true;
  };

  return wait_one(get_parameters_client_) && wait_one(get_parameter_types_client_) &&
         wait_one(set_parameters_client_) && wait_one(set_parameters_atomically_client_) &&
         wait_one(describe_parameters_client_) && wait_one(list_parameters_client_);
}

}  // namespace agnocast
