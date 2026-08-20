#include "agnocast/agnocast_utils.hpp"

#include "agnocast/node/agnocast_node.hpp"

#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include <rclcpp/version.h>
#include <sys/stat.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <sstream>
#include <string_view>
#include <system_error>
#include <tuple>

namespace agnocast
{

rclcpp::Logger logger = rclcpp::get_logger("Agnocast");
bool is_bridge_process = false;

void validate_ld_preload()
{
  if (is_bridge_process) {
    // The bridge process is spawned with an empty LD_PRELOAD to avoid loading the heaphook library
    // in its descendant processes.
    return;
  }

  const char * ld_preload_cstr = getenv("LD_PRELOAD");
  if (
    ld_preload_cstr == nullptr ||
    std::strstr(ld_preload_cstr, "libagnocast_heaphook.so") == nullptr) {
    RCLCPP_ERROR(logger, "libagnocast_heaphook.so not found in LD_PRELOAD.");
    exit(EXIT_FAILURE);
  }

  std::string ld_preload(ld_preload_cstr);
  std::vector<std::string> paths;
  std::string::size_type start = 0;
  std::string::size_type end = 0;

  while ((end = ld_preload.find(':', start)) != std::string::npos) {
    paths.push_back(ld_preload.substr(start, end - start));
    start = end + 1;
  }
  paths.push_back(ld_preload.substr(start));

  if (paths.size() == 1) {
    RCLCPP_WARN(
      logger,
      "Pre-existing shared libraries in LD_PRELOAD may have been overwritten by "
      "libagnocast_heaphook.so");
  }
}

uint32_t get_ros_domain_id()
{
  const char * domain_id_env = getenv("ROS_DOMAIN_ID");
  if (domain_id_env == nullptr || *domain_id_env == '\0') {
    return 0;
  }
  char * end = nullptr;
  errno = 0;
  const uint64_t value = std::strtoul(domain_id_env, &end, 10);
  // Out-of-range values would silently wrap into an unintended domain (e.g. 0),
  // breaking isolation, so reject them rather than truncate.
  if (*end != '\0' || errno != 0 || value > std::numeric_limits<uint32_t>::max()) {
    return 0;
  }
  return static_cast<uint32_t>(value);
}

// UDS-address suffix that scopes the per-IPC-namespace bridge listener by domain.
// The kmod keys the bridge manager on the *parsed* domain, so this must use
// get_ros_domain_id() and not the raw env string. Domain 0 takes no suffix,
// matching the Python discovery agent (bridge_decider._bridge_uds_addr).
static std::string bridge_domain_suffix()
{
  const uint32_t domain_id = get_ros_domain_id();
  if (domain_id == 0) {
    return "";
  }
  return "_d" + std::to_string(domain_id);
}

std::string create_uds_addr_for_bridge()
{
  // Abstract-namespace UDS address is prefixed with '\0' and its length is
  // scoped by the socklen_t passed to bind()/sendto() (no trailing NUL).
  std::string addr;
  addr.push_back('\0');
  addr += "agnocast_bridge_manager_";
  addr += std::to_string(get_self_ipc_ns_inode());
  addr += bridge_domain_suffix();
  return addr;
}

uint64_t get_self_ipc_ns_inode()
{
  struct stat st
  {
  };
  if (stat("/proc/self/ns/ipc", &st) != 0) {
    throw std::system_error(errno, std::generic_category(), "stat(/proc/self/ns/ipc)");
  }
  return static_cast<uint64_t>(st.st_ino);
}

std::string create_shm_name(const pid_t pid)
{
  return "/agnocast@" + std::to_string(pid);
}

std::string create_service_request_topic_name(const std::string & service_name)
{
  return "/AGNOCAST_SRV_REQUEST" + service_name;
}

std::string create_service_response_topic_name(
  const std::string & service_name, const std::string & client_node_name)
{
  return "/AGNOCAST_SRV_RESPONSE" + service_name + "_SEP_" + client_node_name;
}

uint64_t agnocast_get_timestamp()
{
  auto now = std::chrono::system_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

const void * get_node_base_address(agnocast::Node * node)
{
  return static_cast<const void *>(node->get_node_base_interface().get());
}

const void * get_node_base_address(rclcpp::Node * node)
{
  return static_cast<const void *>(
    node->get_node_base_interface()->get_shared_rcl_node_handle().get());
}

// === Begin code from rclcpp ===
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

}  // namespace

#endif
// === End code from rclcpp ===

ServiceTsBundle load_service_typesupport(const std::string & service_type)
{
  static const std::string ts_identifier = "rosidl_typesupport_cpp";
  static const std::string ts_identifier_introspection = "rosidl_typesupport_introspection_cpp";

  const std::string request_type = service_type + "_Request";
  const std::string response_type = service_type + "_Response";

  ServiceTsBundle bundle;

  const rosidl_service_type_support_t * service_ts_introspection = nullptr;

  bundle.ts_lib = rclcpp::get_typesupport_library(service_type, ts_identifier);
  bundle.ts_lib_introspection =
    rclcpp::get_typesupport_library(service_type, ts_identifier_introspection);

#if RCLCPP_VERSION_MAJOR >= 28
  bundle.service_ts =
    rclcpp::get_service_typesupport_handle(service_type, ts_identifier, *bundle.ts_lib);

  service_ts_introspection = rclcpp::get_service_typesupport_handle(
    service_type, ts_identifier_introspection, *bundle.ts_lib_introspection);
#else
  bundle.service_ts = get_service_typesupport_handle(service_type, ts_identifier, *bundle.ts_lib);

  service_ts_introspection = get_service_typesupport_handle(
    service_type, ts_identifier_introspection, *bundle.ts_lib_introspection);
#endif

  const auto * service_members =
    static_cast<const rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_ts_introspection->data);

  bundle.request_members = service_members->request_members_;
  bundle.response_members = service_members->response_members_;

  return bundle;
}

}  // namespace agnocast
