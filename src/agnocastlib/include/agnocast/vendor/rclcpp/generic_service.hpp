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

// GenericService is not available in ROS 2 Humble/Jazzy, so it is vendored here to implement
// Agnocast's service bridging feature.

#pragma once

#include <rclcpp/expand_topic_or_service_name.hpp>
#include <rclcpp/function_traits.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rcpputils/shared_library.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <rcl/node.h>
#include <rcl/service.h>
#include <rclcpp/version.h>
#include <rmw/types.h>

#include <memory>
#include <type_traits>
#include <variant>

namespace agnocast::vendor_rclcpp
{

#if RCLCPP_VERSION_MAJOR < 28

const rosidl_service_type_support_t * get_service_typesupport_handle(
  const std::string & type, const std::string & typesupport_identifier,
  rcpputils::SharedLibrary & library);

#endif

class GenericService;

class GenericServiceCallback
{
  using BasicCallback = std::function<void(std::shared_ptr<void>, std::shared_ptr<void>)>;
  using DeferredCallback = std::function<void(
    std::shared_ptr<GenericService>, std::shared_ptr<rmw_request_id_t>, std::shared_ptr<void>)>;

  std::variant<BasicCallback, DeferredCallback> callback_;

public:
  template <typename Func>
  GenericServiceCallback(Func && callback)
  {
    if constexpr (::rclcpp::detail::can_be_nullptr<std::decay_t<Func>>::value) {
      if (!callback) {
        throw std::invalid_argument("GenericServiceCallback cannot be initialized with nullptr");
      }
    }

    if constexpr (::rclcpp::function_traits::same_arguments<Func, BasicCallback>::value) {
      callback_.template emplace<BasicCallback>(std::forward<Func>(callback));
    } else if constexpr (::rclcpp::function_traits::same_arguments<Func, DeferredCallback>::value) {
      callback_.template emplace<DeferredCallback>(std::forward<Func>(callback));
    }
  }

  std::shared_ptr<void> dispatch(
    const std::shared_ptr<GenericService> & service_handle,
    const std::shared_ptr<rmw_request_id_t> & request_header, std::shared_ptr<void> request,
    const std::function<std::shared_ptr<void>()> & create_response)
  {
    if (std::holds_alternative<BasicCallback>(callback_)) {
      const auto & cb = std::get<BasicCallback>(callback_);
      std::shared_ptr<void> response = create_response();
      cb(std::move(request), response);
      return response;
    }

    // deferred callback
    const auto & cb = std::get<DeferredCallback>(callback_);
    cb(service_handle, request_header, std::move(request));
    return nullptr;
  }
};

class GenericService : public ::rclcpp::ServiceBase,
                       public std::enable_shared_from_this<GenericService>
{
public:
  template <typename Func>
  GenericService(
    ::rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
    Func && callback, const ::rclcpp::QoS & qos = ::rclcpp::ServicesQoS())
  : ServiceBase(node->get_node_base_interface()->get_shared_rcl_node_handle()),
    any_callback_(std::forward<Func>(callback)),
    service_name_(node->get_node_services_interface()->resolve_service_name(service_name))
  {
    static const std::string ts_identifier = "rosidl_typesupport_cpp";
    static const std::string ts_introspection_identifier = "rosidl_typesupport_introspection_cpp";

    const std::string request_type = service_type + "_Request";
    const std::string response_type = service_type + "_Response";

    const rosidl_service_type_support_t * service_ts = nullptr;
    try {
      ts_lib_ = ::rclcpp::get_typesupport_library(service_type, ts_identifier);
      ts_lib_introspection_ =
        ::rclcpp::get_typesupport_library(service_type, ts_introspection_identifier);

#if RCLCPP_VERSION_MAJOR >= 28
      service_ts = ::rclcpp::get_service_typesupport_handle(service_type, ts_identifier, *ts_lib_);

      const rosidl_message_type_support_t * request_ts = ::rclcpp::get_message_typesupport_handle(
        request_type, ts_introspection_identifier, *ts_lib_introspection_);
      const rosidl_message_type_support_t * response_ts = ::rclcpp::get_message_typesupport_handle(
        response_type, ts_introspection_identifier, *ts_lib_introspection_);
#else
      service_ts = get_service_typesupport_handle(service_type, ts_identifier, *ts_lib_);

      const rosidl_message_type_support_t * request_ts = ::rclcpp::get_typesupport_handle(
        request_type, ts_introspection_identifier, *ts_lib_introspection_);
      const rosidl_message_type_support_t * response_ts = ::rclcpp::get_typesupport_handle(
        response_type, ts_introspection_identifier, *ts_lib_introspection_);
#endif

      request_members_ =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(request_ts->data);
      response_members_ = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
        response_ts->data);
    } catch (std::runtime_error & err) {
      RCLCPP_ERROR(
        node_logger_.get_child("agnocast.rclcpp"), "Invalid service type: %s", err.what());
      throw;
    }

    service_handle_ =
      std::shared_ptr<rcl_service_t>(new rcl_service_t, [this](rcl_service_t * service) {
        if (rcl_service_fini(service, node_handle_.get()) != RCL_RET_OK) {
          RCLCPP_ERROR(
            node_logger_.get_child("agnocast.rclcpp"),
            "Error in destruction of rcl service handle '%s': %s", service_name_.c_str(),
            rcl_get_error_string().str);
          rcl_reset_error();
        }
        delete service;
      });
    *service_handle_.get() = rcl_get_zero_initialized_service();

    rcl_service_options_t service_options = rcl_service_get_default_options();
    service_options.qos = qos.get_rmw_qos_profile();

    rcl_ret_t ret = rcl_service_init(
      service_handle_.get(), node_handle_.get(), service_ts, service_name.c_str(),
      &service_options);
    if (ret != RCL_RET_OK) {
      if (ret == RCL_RET_SERVICE_NAME_INVALID) {
        auto rcl_node_handle = get_rcl_node_handle();
        // this will throw on any validation problem
        rcl_reset_error();
        ::rclcpp::expand_topic_or_service_name(
          service_name, rcl_node_get_name(rcl_node_handle), rcl_node_get_namespace(rcl_node_handle),
          true);
      }

      ::rclcpp::exceptions::throw_from_rcl_error(ret, "could not create service");
    }
  }

  template <typename Func>
  static std::shared_ptr<GenericService> create_generic_service(
    ::rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
    Func && callback, const ::rclcpp::QoS & qos = ::rclcpp::ServicesQoS(),
    const ::rclcpp::CallbackGroup::SharedPtr & group = nullptr)
  {
    auto service = std::make_shared<GenericService>(
      node, service_name, service_type, std::forward<Func>(callback), qos);

    std::shared_ptr<::rclcpp::ServiceBase> base_sp = service;
    node->get_node_services_interface()->add_service(std::move(base_sp), group);

    return service;
  }

  GenericService() = delete;
  virtual ~GenericService() = default;

  // --- rclcpp::ServiceBase overrides (driven by the executor) ---
  std::shared_ptr<void> create_request() override;
  std::shared_ptr<rmw_request_id_t> create_request_header() override;
  void handle_request(
    std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> request) override;

  std::shared_ptr<void> create_response();

  void send_response(rmw_request_id_t & req_id, std::shared_ptr<void> & response);

private:
  RCLCPP_DISABLE_COPY(GenericService)

  GenericServiceCallback any_callback_;

  std::string service_name_;

  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_introspection_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * request_members_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * response_members_;
};

}  // namespace agnocast::vendor_rclcpp
