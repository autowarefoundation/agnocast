#include "agnocast/node/node_interfaces/node_parameters.hpp"

#include "agnocast/node/agnocast_arguments.hpp"
#include "agnocast/node/agnocast_context.hpp"
#include "rclcpp/exceptions/exceptions.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace agnocast::node_interfaces
{

namespace
{

bool lockless_has_parameter(
  const std::map<std::string, ParameterInfo> & parameters, const std::string & name)
{
  return parameters.find(name) != parameters.end();
}

rclcpp::Parameter lockless_get_parameter(
  const std::map<std::string, ParameterInfo> & parameters, const std::string & name,
  bool allow_undeclared)
{
  auto param_iter = parameters.find(name);
  if (parameters.end() != param_iter) {
    if (
      param_iter->second.value.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET ||
      param_iter->second.descriptor.dynamic_typing) {
      return rclcpp::Parameter{name, param_iter->second.value};
    }
    throw rclcpp::exceptions::ParameterUninitializedException(name);
  }
  if (allow_undeclared) {
    return rclcpp::Parameter{name};
  }
  throw rclcpp::exceptions::ParameterNotDeclaredException(name);
}

const rclcpp::ParameterValue & declare_parameter_helper(
  const std::string & name, rclcpp::ParameterType type,
  const rclcpp::ParameterValue & default_value,
  rcl_interfaces::msg::ParameterDescriptor parameter_descriptor, bool ignore_override,
  std::map<std::string, ParameterInfo> & parameters,
  const std::map<std::string, rclcpp::ParameterValue> & overrides)
{
  if (name.empty()) {
    throw rclcpp::exceptions::InvalidParametersException("parameter name must not be empty");
  }

  // Error if this parameter has already been declared
  if (lockless_has_parameter(parameters, name)) {
    throw rclcpp::exceptions::ParameterAlreadyDeclaredException(
      "parameter '" + name + "' has already been declared");
  }

  if (!parameter_descriptor.dynamic_typing) {
    if (rclcpp::PARAMETER_NOT_SET == type) {
      type = default_value.get_type();
    }
    if (rclcpp::PARAMETER_NOT_SET == type) {
      throw rclcpp::exceptions::InvalidParameterTypeException{
        name, "cannot declare a statically typed parameter with an uninitialized value"};
    }
    parameter_descriptor.type = static_cast<uint8_t>(type);
  }

  ParameterInfo parameter_info;
  parameter_info.descriptor = parameter_descriptor;
  parameter_info.descriptor.name = name;

  // Use the value from the overrides if available, otherwise use the default.
  auto overrides_it = overrides.find(name);
  if (!ignore_override && overrides_it != overrides.end()) {
    parameter_info.value = overrides_it->second;
  } else {
    parameter_info.value = default_value;
  }

  parameters[name] = parameter_info;

  // Note: rclcpp has __declare_parameter_common which is not currently needed in Agnocast because:
  // - override handling: done directly in this function
  // - on_parameters_set callbacks: not implemented
  // - parameter events publishing: not implemented

  return parameters.at(name).value;
}

// see https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
bool are_doubles_equal(double x, double y, double ulp = 100.0)
{
  return std::abs(x - y) <= std::numeric_limits<double>::epsilon() * std::abs(x + y) * ulp;
}

std::string format_range_reason(const std::string & name, const char * range_type)
{
  std::ostringstream ss;
  ss << "Parameter {" << name << "} doesn't comply with " << range_type << " range.";
  return ss.str();
}

rcl_interfaces::msg::SetParametersResult check_parameter_value_in_range(
  const rcl_interfaces::msg::ParameterDescriptor & descriptor, const rclcpp::ParameterValue & value)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  if (!descriptor.integer_range.empty() && value.get_type() == rclcpp::PARAMETER_INTEGER) {
    int64_t v = value.get<int64_t>();
    auto integer_range = descriptor.integer_range.at(0);
    if (v == integer_range.from_value || v == integer_range.to_value) {
      return result;
    }
    if ((v < integer_range.from_value) || (v > integer_range.to_value)) {
      result.successful = false;
      result.reason = format_range_reason(descriptor.name, "integer");
      return result;
    }
    if (integer_range.step == 0) {
      return result;
    }
    if (((v - integer_range.from_value) % integer_range.step) == 0) {
      return result;
    }
    result.successful = false;
    result.reason = format_range_reason(descriptor.name, "integer");
    return result;
  }

  if (!descriptor.floating_point_range.empty() && value.get_type() == rclcpp::PARAMETER_DOUBLE) {
    double v = value.get<double>();
    auto fp_range = descriptor.floating_point_range.at(0);
    if (are_doubles_equal(v, fp_range.from_value) || are_doubles_equal(v, fp_range.to_value)) {
      return result;
    }
    if ((v < fp_range.from_value) || (v > fp_range.to_value)) {
      result.successful = false;
      result.reason = format_range_reason(descriptor.name, "floating point");
      return result;
    }
    if (fp_range.step == 0.0) {
      return result;
    }
    double rounded_div = std::round((v - fp_range.from_value) / fp_range.step);
    if (are_doubles_equal(v, fp_range.from_value + rounded_div * fp_range.step)) {
      return result;
    }
    result.successful = false;
    result.reason = format_range_reason(descriptor.name, "floating point");
    return result;
  }
  return result;
}

[[maybe_unused]] std::string format_type_reason(
  const std::string & name, const std::string & old_type, const std::string & new_type)
{
  std::ostringstream ss;
  // WARN: A condition later depends on this message starting with "Wrong parameter type",
  // check `declare_parameter` if you modify this!
  ss << "Wrong parameter type, parameter {" << name << "} is of type {" << old_type
     << "}, setting it to {" << new_type << "} is not allowed.";
  return ss.str();
}

// Return true if parameter values comply with the descriptors in parameter_infos.
[[maybe_unused]] rcl_interfaces::msg::SetParametersResult check_parameters(
  std::map<std::string, ParameterInfo> & parameter_infos,
  const std::vector<rclcpp::Parameter> & parameters, bool allow_undeclared)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter & parameter : parameters) {
    const std::string & name = parameter.get_name();
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    if (allow_undeclared) {
      auto it = parameter_infos.find(name);
      if (it != parameter_infos.cend()) {
        descriptor = it->second.descriptor;
      } else {
        // implicitly declared parameters are dynamically typed!
        descriptor.dynamic_typing = true;
      }
    } else {
      descriptor = parameter_infos[name].descriptor;
    }
    if (descriptor.name.empty()) {
      descriptor.name = name;
    }
    const auto new_type = parameter.get_type();
    const auto specified_type = static_cast<rclcpp::ParameterType>(descriptor.type);
    result.successful = descriptor.dynamic_typing || specified_type == new_type;
    if (!result.successful) {
      result.reason =
        format_type_reason(name, rclcpp::to_string(specified_type), rclcpp::to_string(new_type));
      return result;
    }
    result = check_parameter_value_in_range(descriptor, parameter.get_parameter_value());
    if (!result.successful) {
      return result;
    }
  }
  return result;
}

}  // namespace

NodeParameters::NodeParameters(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  const std::vector<rclcpp::Parameter> & parameter_overrides, const rcl_arguments_t * local_args,
  bool allow_undeclared_parameters)
: node_base_(std::move(node_base)), allow_undeclared_(allow_undeclared_parameters)
{
  const rcl_arguments_t * global_args = nullptr;
  {
    std::lock_guard<std::mutex> lock(g_context_mtx);
    if (g_context.is_initialized()) {
      global_args = g_context.get_parsed_arguments();
    }
  }

  std::string combined_name = node_base_->get_fully_qualified_name();
  parameter_overrides_ =
    resolve_parameter_overrides(combined_name, parameter_overrides, local_args, global_args);
}

const rclcpp::ParameterValue & NodeParameters::declare_parameter(
  const std::string & name, const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor, bool ignore_override)
{
  std::lock_guard<std::mutex> lock(parameters_mutex_);
  // Note: rclcpp uses ParameterMutationRecursionGuard here to prevent parameter modification
  // from within callbacks. Not needed in Agnocast since callbacks are not implemented.

  return declare_parameter_helper(
    name, rclcpp::PARAMETER_NOT_SET, default_value, parameter_descriptor, ignore_override,
    parameters_, parameter_overrides_);
}

const rclcpp::ParameterValue & NodeParameters::declare_parameter(
  const std::string & name, rclcpp::ParameterType type,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor, bool ignore_override)
{
  std::lock_guard<std::mutex> lock(parameters_mutex_);
  // Note: rclcpp uses ParameterMutationRecursionGuard here to prevent parameter modification
  // from within callbacks. Not needed in Agnocast since callbacks are not implemented.

  if (rclcpp::PARAMETER_NOT_SET == type) {
    throw std::invalid_argument{
      "declare_parameter(): the provided parameter type cannot be rclcpp::PARAMETER_NOT_SET"};
  }

  if (parameter_descriptor.dynamic_typing) {
    throw std::invalid_argument{
      "declare_parameter(): cannot declare parameter of specific type and pass descriptor "
      "with `dynamic_typing=true`"};
  }

  return declare_parameter_helper(
    name, type, rclcpp::ParameterValue{}, parameter_descriptor, ignore_override, parameters_,
    parameter_overrides_);
}

void NodeParameters::undeclare_parameter(const std::string & name)
{
  std::lock_guard<std::mutex> lock(parameters_mutex_);
  // Note: rclcpp uses ParameterMutationRecursionGuard here to prevent parameter modification
  // from within callbacks. Not needed in Agnocast since callbacks are not implemented.

  auto parameter_info = parameters_.find(name);
  if (parameter_info == parameters_.end()) {
    throw rclcpp::exceptions::ParameterNotDeclaredException(
      "cannot undeclare parameter '" + name + "' which has not yet been declared");
  }

  if (parameter_info->second.descriptor.read_only) {
    throw rclcpp::exceptions::ParameterImmutableException(
      "cannot undeclare parameter '" + name + "' because it is read-only");
  }
  if (!parameter_info->second.descriptor.dynamic_typing) {
    throw rclcpp::exceptions::InvalidParameterTypeException{
      name, "cannot undeclare a statically typed parameter"};
  }

  parameters_.erase(parameter_info);
}

bool NodeParameters::has_parameter(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(parameters_mutex_);

  return lockless_has_parameter(parameters_, name);
}

std::vector<rcl_interfaces::msg::SetParametersResult> NodeParameters::set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // TODO(Koichi98)
  (void)parameters;
  throw std::runtime_error("NodeParameters::set_parameters is not yet implemented in agnocast");
}

rcl_interfaces::msg::SetParametersResult NodeParameters::set_parameters_atomically(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // TODO(Koichi98)
  (void)parameters;
  throw std::runtime_error(
    "NodeParameters::set_parameters_atomically is not yet implemented in agnocast");
}

std::vector<rclcpp::Parameter> NodeParameters::get_parameters(
  const std::vector<std::string> & names) const
{
  std::vector<rclcpp::Parameter> results;
  results.reserve(names.size());

  std::lock_guard<std::mutex> lock(parameters_mutex_);
  for (const auto & name : names) {
    results.emplace_back(lockless_get_parameter(parameters_, name, allow_undeclared_));
  }
  return results;
}

rclcpp::Parameter NodeParameters::get_parameter(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(parameters_mutex_);

  return lockless_get_parameter(parameters_, name, allow_undeclared_);
}

bool NodeParameters::get_parameter(const std::string & name, rclcpp::Parameter & parameter) const
{
  std::lock_guard<std::mutex> lock(parameters_mutex_);

  auto param_iter = parameters_.find(name);
  if (
    parameters_.end() != param_iter &&
    param_iter->second.value.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    parameter = {name, param_iter->second.value};
    return true;
  }
  return false;
}

bool NodeParameters::get_parameters_by_prefix(
  const std::string & prefix, std::map<std::string, rclcpp::Parameter> & parameters) const
{
  // TODO(Koichi98)
  (void)prefix;
  (void)parameters;
  throw std::runtime_error(
    "NodeParameters::get_parameters_by_prefix is not yet implemented in agnocast");
}

std::vector<rcl_interfaces::msg::ParameterDescriptor> NodeParameters::describe_parameters(
  const std::vector<std::string> & names) const
{
  // TODO(Koichi98)
  (void)names;
  throw std::runtime_error(
    "NodeParameters::describe_parameters is not yet implemented in agnocast");
}

std::vector<uint8_t> NodeParameters::get_parameter_types(
  const std::vector<std::string> & names) const
{
  // TODO(Koichi98)
  (void)names;
  throw std::runtime_error(
    "NodeParameters::get_parameter_types is not yet implemented in agnocast");
}

rcl_interfaces::msg::ListParametersResult NodeParameters::list_parameters(
  const std::vector<std::string> & prefixes, uint64_t depth) const
{
  // TODO(Koichi98)
  (void)prefixes;
  (void)depth;
  throw std::runtime_error("NodeParameters::list_parameters is not yet implemented in agnocast");
}

rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
NodeParameters::add_on_set_parameters_callback(OnParametersSetCallbackType callback)
{
  // TODO(Koichi98)
  (void)callback;
  throw std::runtime_error(
    "NodeParameters::add_on_set_parameters_callback is not yet implemented in agnocast");
}

void NodeParameters::remove_on_set_parameters_callback(
  const rclcpp::node_interfaces::OnSetParametersCallbackHandle * const handler)
{
  // TODO(Koichi98)
  (void)handler;
  throw std::runtime_error(
    "NodeParameters::remove_on_set_parameters_callback is not yet implemented in agnocast");
}

const std::map<std::string, rclcpp::ParameterValue> & NodeParameters::get_parameter_overrides()
  const
{
  return parameter_overrides_;
}

}  // namespace agnocast::node_interfaces
