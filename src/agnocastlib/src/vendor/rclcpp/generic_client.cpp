// Copyright 2023 Sony Group Corporation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// This file has been modified from the original.

#include <rclcpp/version.h>

#if RCLCPP_VERSION_MAJOR < 28

#include "agnocast/vendor/rclcpp/generic_client.hpp"
#include "agnocast/vendor/rclcpp/generic_service.hpp"  // get_service_typesupport_handle

namespace agnocast::vendor_rclcpp
{

GenericClient::GenericClient(
  rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
  const rclcpp::QoS & qos)
: ClientBase(node->get_node_base_interface().get(), node->get_node_graph_interface())
{
  static const std::string ts_identifier = "rosidl_typesupport_cpp";
  static const std::string ts_introspection_identifier = "rosidl_typesupport_introspection_cpp";

  const std::string response_type = service_type + "_Response";

  const rosidl_service_type_support_t * service_ts = nullptr;
  try {
    ts_lib_ = rclcpp::get_typesupport_library(service_type, ts_identifier);
    ts_lib_introspection_ =
      rclcpp::get_typesupport_library(service_type, ts_introspection_identifier);

    service_ts = get_service_typesupport_handle(service_type, ts_identifier, *ts_lib_);

    const rosidl_message_type_support_t * response_ts = rclcpp::get_typesupport_handle(
      response_type, ts_introspection_identifier, *ts_lib_introspection_);
    response_members_ =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(response_ts->data);
  } catch (std::runtime_error & err) {
    RCLCPP_ERROR(node_logger_.get_child("agnocast.rclcpp"), "Invalid service type: %s", err.what());
    throw;
  }

  rcl_client_options_t client_options = rcl_client_get_default_options();
  client_options.qos = qos.get_rmw_qos_profile();

  rcl_ret_t ret = rcl_client_init(
    this->get_client_handle().get(), this->get_rcl_node_handle(), service_ts, service_name.c_str(),
    &client_options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_SERVICE_NAME_INVALID) {
      auto * rcl_node_handle = this->get_rcl_node_handle();
      // this will throw on any validation problem
      rcl_reset_error();
      rclcpp::expand_topic_or_service_name(
        service_name, rcl_node_get_name(rcl_node_handle), rcl_node_get_namespace(rcl_node_handle),
        true);
    }
    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create generic client");
  }
}

GenericClient::SharedPtr GenericClient::create_generic_client(
  rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
  const rclcpp::QoS & qos, const rclcpp::CallbackGroup::SharedPtr & group)
{
  auto client = std::make_shared<GenericClient>(
    node, rclcpp::extend_name_with_sub_namespace(service_name, node->get_sub_namespace()),
    service_type, qos);

  std::shared_ptr<rclcpp::ClientBase> base_sp = client;
  node->get_node_services_interface()->add_client(std::move(base_sp), group);

  return client;
}

std::shared_ptr<void> GenericClient::create_response()
{
  void * response = new uint8_t[response_members_->size_of_];
  response_members_->init_function(response, rosidl_runtime_cpp::MessageInitialization::ZERO);
  return {response, [this](void * p) {
            response_members_->fini_function(p);
            delete[] reinterpret_cast<uint8_t *>(p);  // NOLINT(cppcoreguidelines-owning-memory)
          }};
}

std::shared_ptr<rmw_request_id_t> GenericClient::create_request_header()
{
  return std::make_shared<rmw_request_id_t>();
}

void GenericClient::handle_response(
  std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response)
{
  auto optional_pending_request =
    this->get_and_erase_pending_request(request_header->sequence_number);
  if (!optional_pending_request) {
    return;
  }
  auto & value = *optional_pending_request;
  const auto & callback = std::get<CallbackType>(value);
  auto & promise = std::get<Promise>(value);
  auto & future = std::get<SharedFuture>(value);
  promise.set_value(std::move(response));
  callback(std::move(future));
}

GenericClient::SharedFutureAndRequestId GenericClient::async_send_request(
  const void *& request, GenericClient::CallbackType && cb)
{
  GenericClient::Promise promise;
  auto shared_future = promise.get_future().share();
  int64_t sequence_number = 0;

  {
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    rcl_ret_t ret = rcl_send_request(get_client_handle().get(), request, &sequence_number);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send request");
    }
    pending_requests_.try_emplace(
      sequence_number, std::make_pair(
                         std::chrono::system_clock::now(),
                         std::make_tuple(std::move(cb), shared_future, std::move(promise))));
  }

  return {std::move(shared_future), sequence_number};
}

std::optional<GenericClient::PendingRequest> GenericClient::get_and_erase_pending_request(
  int64_t sequence_number)
{
  std::lock_guard<std::mutex> lock(pending_requests_mutex_);
  auto it = pending_requests_.find(sequence_number);
  if (it == pending_requests_.end()) {
    RCUTILS_LOG_DEBUG_NAMED("agnocast.rclcpp", "Received invalid sequence number. Ignoring...");
    return std::nullopt;
  }
  auto value = std::move(it->second.second);
  pending_requests_.erase(sequence_number);
  return value;
}

}  // namespace agnocast::vendor_rclcpp

#endif
