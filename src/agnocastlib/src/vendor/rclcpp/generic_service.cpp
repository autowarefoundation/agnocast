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

#if RCLCPP_VERSION_MAJOR < 28

namespace
{

std::string string_trim(std::string_view str_v)
{
  const auto * begin =
    std::find_if_not(str_v.begin(), str_v.end(), [](unsigned char ch) { return std::isspace(ch); });
  const auto * end = std::find_if_not(str_v.rbegin(), str_v.rend(), [](unsigned char ch) {
                       return std::isspace(ch);
                     }).base();
  if (begin >= end) {
    return {};
  }
  return {begin, end};
}

std::tuple<std::string, std::string, std::string> extract_type_identifier(
  const std::string & full_type)
{
  char type_separator = '/';
  auto sep_position_back = full_type.find_last_of(type_separator);
  auto sep_position_front = full_type.find_first_of(type_separator);
  if (
    sep_position_back == std::string::npos || sep_position_front == 0 || sep_position_back == 0 ||
    sep_position_back == full_type.length() - 1) {
    throw std::runtime_error(
      "Message type is not of the form package/type and cannot be processed");
  }

  std::string package_name = full_type.substr(0, sep_position_front);
  std::string middle_module;
  if (sep_position_back - sep_position_front > 0) {
    middle_module =
      full_type.substr(sep_position_front + 1, sep_position_back - sep_position_front - 1);
  }
  std::string type_name = full_type.substr(sep_position_back + 1);

  return std::make_tuple(
    string_trim(package_name), string_trim(middle_module), string_trim(type_name));
}

const void * get_typesupport_handle_impl(
  const std::string & type, const std::string & typesupport_identifier,
  const std::string & typesupport_name, const std::string & symbol_part_name,
  const std::string & middle_module_additional, rcpputils::SharedLibrary & library)
{
  std::string package_name;
  std::string middle_module;
  std::string type_name;
  std::tie(package_name, middle_module, type_name) = extract_type_identifier(type);

  if (middle_module.empty()) {
    middle_module = middle_module_additional;
  }

  auto mk_error = [&package_name, &type_name, &typesupport_name](auto reason) {
    std::stringstream rcutils_dynamic_loading_error;
    rcutils_dynamic_loading_error << "Something went wrong loading the typesupport library for "
                                  << typesupport_name << " type " << package_name << "/"
                                  << type_name << ". " << reason;
    return rcutils_dynamic_loading_error.str();
  };

  try {
    std::string symbol_name = typesupport_identifier + symbol_part_name + package_name + "__" +
                              middle_module + "__" + type_name;
    const void * (*get_ts)() = nullptr;
    // This will throw runtime_error if the symbol was not found.
    get_ts = reinterpret_cast<decltype(get_ts)>(library.get_symbol(symbol_name));
    return get_ts();
  } catch (std::runtime_error &) {
    throw std::runtime_error{mk_error("Library could not be found.")};
  }
}

}  // namespace

const rosidl_service_type_support_t * get_service_typesupport_handle(
  const std::string & type, const std::string & typesupport_identifier,
  rcpputils::SharedLibrary & library)
{
  static const std::string typesupport_name = "service";
  static const std::string symbol_part_name = "__get_service_type_support_handle__";
  static const std::string middle_module_additional = "srv";

  return static_cast<const rosidl_service_type_support_t *>(get_typesupport_handle_impl(
    type, typesupport_identifier, typesupport_name, symbol_part_name, middle_module_additional,
    library));
}

#endif

std::shared_ptr<void> GenericService::create_request()
{
  void * request = new uint8_t[request_members_->size_of_];
  request_members_->init_function(request, rosidl_runtime_cpp::MessageInitialization::ZERO);
  return {request, [this](void * p) {
            request_members_->fini_function(p);
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
  void * response = new uint8_t[response_members_->size_of_];
  response_members_->init_function(response, rosidl_runtime_cpp::MessageInitialization::ZERO);
  return {response, [this](void * p) {
            response_members_->fini_function(p);
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
