// Copyright 2024 Sony Group Corporation.
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

#include "agnocast/vendor/rclcpp/generic_service.hpp"

#include <rclcpp/exceptions.hpp>

namespace agnocast::vendor_rclcpp
{

std::shared_ptr<void> GenericService::create_request()
{
  const auto * request_members = service_ts_bundle_.request_members;
  void * request = new uint8_t[request_members->size_of_];
  request_members->init_function(request, rosidl_runtime_cpp::MessageInitialization::ZERO);
  // The deleter can outlive this service, so it owns the bundle that keeps fini_function mapped.
  return {request, [bundle = service_ts_bundle_](void * p) {
            bundle.request_members->fini_function(p);
            delete[] static_cast<uint8_t *>(p);  // NOLINT(cppcoreguidelines-owning-memory)
          }};
}

std::shared_ptr<rmw_request_id_t> GenericService::create_request_header()
{
  return std::make_shared<rmw_request_id_t>();
}

void GenericService::handle_request(
  std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> request)
{
  std::shared_ptr<void> response = any_callback_.dispatch(
    this->shared_from_this(), request_header, std::move(request),
    [this]() { return create_response(); });
  if (response) {
    send_response(*request_header, response);
  }
}

std::shared_ptr<void> GenericService::create_response()
{
  const auto * response_members = service_ts_bundle_.response_members;
  void * response = new uint8_t[response_members->size_of_];
  response_members->init_function(response, rosidl_runtime_cpp::MessageInitialization::ZERO);
  return {response, [bundle = service_ts_bundle_](void * p) {
            bundle.response_members->fini_function(p);
            delete[] static_cast<uint8_t *>(p);  // NOLINT(cppcoreguidelines-owning-memory)
          }};
}

void GenericService::send_response(rmw_request_id_t & req_id, std::shared_ptr<void> & response)
{
  rcl_ret_t ret = rcl_send_response(get_service_handle().get(), &req_id, response.get());

  if (ret == RCL_RET_TIMEOUT) {
    RCLCPP_WARN(
      node_logger_.get_child("agnocast.rclcpp"),
      "failed to send response in service '%s' (timeout): %s", this->get_service_name(),
      rcl_get_error_string().str);
    rcl_reset_error();
    return;
  }
  if (ret != RCL_RET_OK) {
    ::rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send response");
  }
}

}  // namespace agnocast::vendor_rclcpp
