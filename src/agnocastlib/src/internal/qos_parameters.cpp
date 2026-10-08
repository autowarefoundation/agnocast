// Copyright 2020 Open Source Robotics Foundation, Inc.
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
// Adapted from rclcpp/include/rclcpp/detail/qos_parameters.hpp so agnocastlib does not depend on
// rclcpp::detail. Behavior matches Humble and Jazzy: rolling's
// NodeParametersInterface::enable_parameter_modification() is not called.

#include "agnocast/internal/qos_parameters.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/exceptions.hpp>

#include <rmw/qos_string_conversions.h>

#include <algorithm>
#include <array>
#include <sstream>
#include <stdexcept>
#include <string>

namespace agnocast::internal
{
namespace
{

constexpr std::array kPublisherPolicies = {
  rclcpp::QosPolicyKind::AvoidRosNamespaceConventions,
  rclcpp::QosPolicyKind::Deadline,
  rclcpp::QosPolicyKind::Durability,
  rclcpp::QosPolicyKind::History,
  rclcpp::QosPolicyKind::Depth,
  rclcpp::QosPolicyKind::Lifespan,
  rclcpp::QosPolicyKind::Liveliness,
  rclcpp::QosPolicyKind::LivelinessLeaseDuration,
  rclcpp::QosPolicyKind::Reliability,
};

constexpr std::array kSubscriptionPolicies = {
  rclcpp::QosPolicyKind::AvoidRosNamespaceConventions,
  rclcpp::QosPolicyKind::Deadline,
  rclcpp::QosPolicyKind::Durability,
  rclcpp::QosPolicyKind::History,
  rclcpp::QosPolicyKind::Depth,
  rclcpp::QosPolicyKind::Liveliness,
  rclcpp::QosPolicyKind::LivelinessLeaseDuration,
  rclcpp::QosPolicyKind::Reliability,
};

const char * entity_type_name(QosOverrideEntity entity)
{
  switch (entity) {
    case QosOverrideEntity::Publisher:
      return "publisher";
    case QosOverrideEntity::Subscription:
      return "subscription";
    default:
      throw std::invalid_argument("unknown QosOverrideEntity");
  }
}

int64_t rmw_duration_to_int64_t(const rmw_time_t rmw_duration)
{
  return rclcpp::Duration(
           static_cast<int32_t>(rmw_duration.sec), static_cast<uint32_t>(rmw_duration.nsec))
    .nanoseconds();
}

const char * check_if_stringified_policy_is_null(
  const char * policy_value_stringified, const rclcpp::QosPolicyKind kind)
{
  if (policy_value_stringified == nullptr) {
    std::ostringstream oss("unknown value for policy kind {", std::ios::ate);
    oss << kind << "}";
    throw std::invalid_argument(oss.str());
  }
  return policy_value_stringified;
}

rclcpp::ParameterValue get_default_qos_param_value(
  rclcpp::QosPolicyKind kind, const rclcpp::QoS & qos)
{
  const auto & rmw_qos = qos.get_rmw_qos_profile();
  switch (kind) {
    case rclcpp::QosPolicyKind::AvoidRosNamespaceConventions:
      return rclcpp::ParameterValue(rmw_qos.avoid_ros_namespace_conventions);
    case rclcpp::QosPolicyKind::Deadline:
      return rclcpp::ParameterValue(rmw_duration_to_int64_t(rmw_qos.deadline));
    case rclcpp::QosPolicyKind::Durability:
      return rclcpp::ParameterValue(check_if_stringified_policy_is_null(
        rmw_qos_durability_policy_to_str(rmw_qos.durability), kind));
    case rclcpp::QosPolicyKind::History:
      return rclcpp::ParameterValue(
        check_if_stringified_policy_is_null(rmw_qos_history_policy_to_str(rmw_qos.history), kind));
    case rclcpp::QosPolicyKind::Depth:
      return rclcpp::ParameterValue(static_cast<int64_t>(rmw_qos.depth));
    case rclcpp::QosPolicyKind::Lifespan:
      return rclcpp::ParameterValue(rmw_duration_to_int64_t(rmw_qos.lifespan));
    case rclcpp::QosPolicyKind::Liveliness:
      return rclcpp::ParameterValue(check_if_stringified_policy_is_null(
        rmw_qos_liveliness_policy_to_str(rmw_qos.liveliness), kind));
    case rclcpp::QosPolicyKind::LivelinessLeaseDuration:
      return rclcpp::ParameterValue(rmw_duration_to_int64_t(rmw_qos.liveliness_lease_duration));
    case rclcpp::QosPolicyKind::Reliability:
      return rclcpp::ParameterValue(check_if_stringified_policy_is_null(
        rmw_qos_reliability_policy_to_str(rmw_qos.reliability), kind));
    default:
      throw std::invalid_argument("unknown QoS policy kind");
  }
}

template <typename PolicyT, typename FromStr, typename Apply>
void apply_string_qos_override(
  const rclcpp::ParameterValue & parameter_value, const char * kind_name, FromStr from_str,
  PolicyT unknown, Apply apply)
{
  const std::string policy_string = parameter_value.get<std::string>();
  const PolicyT policy_value = from_str(policy_string.c_str());
  if (policy_value == unknown) {
    throw std::invalid_argument(
      std::string("unknown QoS policy ") + kind_name + " value: " + policy_string);
  }
  apply(policy_value);
}

// NOLINTBEGIN(google-runtime-references)
void apply_qos_override(
  rclcpp::QosPolicyKind policy, const rclcpp::ParameterValue & value, rclcpp::QoS & qos)
{
  switch (policy) {
    case rclcpp::QosPolicyKind::AvoidRosNamespaceConventions:
      qos.avoid_ros_namespace_conventions(value.get<bool>());
      break;
    case rclcpp::QosPolicyKind::Deadline:
      qos.deadline(rclcpp::Duration::from_nanoseconds(value.get<int64_t>()));
      break;
    case rclcpp::QosPolicyKind::Durability:
      apply_string_qos_override(
        value, "durability", rmw_qos_durability_policy_from_str, RMW_QOS_POLICY_DURABILITY_UNKNOWN,
        [&qos](const rmw_qos_durability_policy_t policy_value) { qos.durability(policy_value); });
      break;
    case rclcpp::QosPolicyKind::History:
      apply_string_qos_override(
        value, "history", rmw_qos_history_policy_from_str, RMW_QOS_POLICY_HISTORY_UNKNOWN,
        [&qos](const rmw_qos_history_policy_t policy_value) { qos.history(policy_value); });
      break;
    case rclcpp::QosPolicyKind::Depth:
      qos.get_rmw_qos_profile().depth = static_cast<size_t>(value.get<int64_t>());
      break;
    case rclcpp::QosPolicyKind::Lifespan:
      qos.lifespan(rclcpp::Duration::from_nanoseconds(value.get<int64_t>()));
      break;
    case rclcpp::QosPolicyKind::Liveliness:
      apply_string_qos_override(
        value, "liveliness", rmw_qos_liveliness_policy_from_str, RMW_QOS_POLICY_LIVELINESS_UNKNOWN,
        [&qos](const rmw_qos_liveliness_policy_t policy_value) { qos.liveliness(policy_value); });
      break;
    case rclcpp::QosPolicyKind::LivelinessLeaseDuration:
      qos.liveliness_lease_duration(rclcpp::Duration::from_nanoseconds(value.get<int64_t>()));
      break;
    case rclcpp::QosPolicyKind::Reliability:
      apply_string_qos_override(
        value, "reliability", rmw_qos_reliability_policy_from_str,
        RMW_QOS_POLICY_RELIABILITY_UNKNOWN,
        [&qos](const rmw_qos_reliability_policy_t policy_value) { qos.reliability(policy_value); });
      break;
    default:
      throw std::invalid_argument("unknown QosPolicyKind");
  }
}
// NOLINTEND(google-runtime-references)

// NOLINTBEGIN(google-runtime-references)
rclcpp::ParameterValue declare_parameter_or_get(
  rclcpp::node_interfaces::NodeParametersInterface & parameters_interface,
  const std::string & param_name, const rclcpp::ParameterValue & param_value,
  const rcl_interfaces::msg::ParameterDescriptor & descriptor)
{
  try {
    return parameters_interface.declare_parameter(param_name, param_value, descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    return parameters_interface.get_parameter(param_name).get_parameter_value();
  }
}
// NOLINTEND(google-runtime-references)

template <size_t N>
// NOLINTBEGIN(google-runtime-references)
void declare_selected_policies(
  const std::array<rclcpp::QosPolicyKind, N> & allowed,
  const rclcpp::QosOverridingOptions & options,
  rclcpp::node_interfaces::NodeParametersInterface & parameters_interface,
  const std::string & param_prefix, const std::string & param_description_suffix, rclcpp::QoS & qos)
{
  const auto & selected = options.get_policy_kinds();
  for (const rclcpp::QosPolicyKind policy : allowed) {
    if (std::find(selected.begin(), selected.end(), policy) == selected.end()) {
      continue;
    }
    const char * policy_name = rclcpp::qos_policy_kind_to_cstr(policy);
    const std::string param_name = param_prefix + policy_name;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = std::string("qos policy {") + policy_name + param_description_suffix;
    descriptor.read_only = true;
    const rclcpp::ParameterValue value = declare_parameter_or_get(
      parameters_interface, param_name, get_default_qos_param_value(policy, qos), descriptor);
    apply_qos_override(policy, value, qos);
  }
}
// NOLINTEND(google-runtime-references)

}  // namespace

[[nodiscard]] rclcpp::QoS declare_qos_parameters(
  const rclcpp::QosOverridingOptions & options,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters,
  const std::string & topic_name, const rclcpp::QoS & default_qos, QosOverrideEntity entity)
{
  if (node_parameters == nullptr) {
    throw std::invalid_argument("node parameters interface cannot be nullptr");
  }

  const char * entity_type = entity_type_name(entity);
  const std::string & id = options.get_id();

  std::string param_prefix;
  {
    std::ostringstream oss("qos_overrides.", std::ios::ate);
    oss << topic_name << "." << entity_type;
    if (!id.empty()) {
      oss << "_" << id;
    }
    oss << ".";
    param_prefix = oss.str();
  }
  std::string param_description_suffix;
  {
    std::ostringstream oss("} for ", std::ios::ate);
    oss << entity_type << " {" << topic_name << "}";
    if (!id.empty()) {
      oss << " with id {" << id << "}";
    }
    param_description_suffix = oss.str();
  }

  rclcpp::QoS qos = default_qos;
  if (entity == QosOverrideEntity::Publisher) {
    declare_selected_policies(
      kPublisherPolicies, options, *node_parameters, param_prefix, param_description_suffix, qos);
  } else {
    declare_selected_policies(
      kSubscriptionPolicies, options, *node_parameters, param_prefix, param_description_suffix,
      qos);
  }

  const rclcpp::QosCallback & validation_callback = options.get_validation_callback();
  if (validation_callback) {
    const rclcpp::QosCallbackResult result = validation_callback(qos);
    if (!result.successful) {
      throw rclcpp::exceptions::InvalidQosOverridesException(
        "validation callback failed: " + result.reason);
    }
  }
  return qos;
}

}  // namespace agnocast::internal
