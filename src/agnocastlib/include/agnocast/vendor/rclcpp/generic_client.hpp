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

// GenericClient is not available in ROS 2 Humble, so it is vendored here to implement Agnocast's
// service bridging feature.

#pragma once

#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rcpputils/shared_library.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace agnocast::vendor_rclcpp
{

class GenericClient : public rclcpp::ClientBase
{
public:
  using SharedPtr = std::shared_ptr<GenericClient>;

  using Promise = std::promise<std::shared_ptr<void>>;

  using Future = std::future<std::shared_ptr<void>>;
  using SharedFuture = std::shared_future<std::shared_ptr<void>>;

  using CallbackType = std::function<void(SharedFuture)>;

  struct FutureAndRequestId : rclcpp::detail::FutureAndRequestId<Future>
  {
    using rclcpp::detail::FutureAndRequestId<Future>::FutureAndRequestId;
    SharedFuture share() noexcept { return this->future.share(); }
  };
  struct SharedFutureAndRequestId : rclcpp::detail::FutureAndRequestId<SharedFuture>
  {
    using rclcpp::detail::FutureAndRequestId<SharedFuture>::FutureAndRequestId;
  };

  GenericClient(
    rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
    const rclcpp::QoS & qos);

  static SharedPtr create_generic_client(
    rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
    const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
    const rclcpp::CallbackGroup::SharedPtr & group = nullptr);

  // --- rclcpp::ClientBase overrides (driven by the executor) ---
  std::shared_ptr<void> create_response() override;
  std::shared_ptr<rmw_request_id_t> create_request_header() override;
  void handle_response(
    std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response) override;

  SharedFutureAndRequestId async_send_request(const void *& request, CallbackType && cb);

protected:
  using PendingRequest = std::tuple<CallbackType, SharedFuture, Promise>;

  std::optional<PendingRequest> get_and_erase_pending_request(int64_t sequence_number);

  RCLCPP_DISABLE_COPY(GenericClient)

  std::map<int64_t, std::pair<std::chrono::time_point<std::chrono::system_clock>, PendingRequest>>
    pending_requests_;
  std::mutex pending_requests_mutex_;

private:
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_introspection_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * response_members_;
};

}  // namespace agnocast::vendor_rclcpp
