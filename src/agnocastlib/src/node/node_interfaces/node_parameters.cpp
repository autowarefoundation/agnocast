#include "agnocast/node/node_interfaces/node_parameters.hpp"

#include "agnocast/node/agnocast_arguments.hpp"
#include "agnocast/node/agnocast_context.hpp"
#include "rclcpp/exceptions/exceptions.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace agnocast::node_interfaces
{

namespace
{

using CallbacksContainerType = NodeParameters::CallbacksContainerType;

// Forward declaration
rcl_interfaces::msg::SetParametersResult declare_parameter_common(
  const std::string & name, const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor,
  std::map<std::string, ParameterInfo> & parameters_out,
  const std::map<std::string, rclcpp::ParameterValue> & overrides,
  CallbacksContainerType & callback_container, bool ignore_override);

rcl_interfaces::msg::SetParametersResult call_on_parameters_set_callbacks(
  const std::vector<rclcpp::Parameter> & parameters, CallbacksContainerType & callback_container)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  auto it = callback_container.begin();
  while (it != callback_container.end()) {
    auto shared_handle = it->lock();
    if (nullptr != shared_handle) {
      result = shared_handle->callback(parameters);
      if (!result.successful) {
        return result;
      }
      it++;
    } else {
      it = callback_container.erase(it);
    }
  }
  return result;
}

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
  const std::map<std::string, rclcpp::ParameterValue> & overrides,
  CallbacksContainerType & callback_container)
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

  auto result = declare_parameter_common(
    name, default_value, parameter_descriptor, parameters, overrides, callback_container,
    ignore_override);

  // If it failed to be set, then throw an exception.
  if (!result.successful) {
    constexpr auto type_error_msg_start = "Wrong parameter type";
    if (result.reason.rfind(type_error_msg_start, 0) == 0) {
      throw rclcpp::exceptions::InvalidParameterTypeException(name, result.reason);
    }
    throw rclcpp::exceptions::InvalidParameterValueException(
      "parameter '" + name + "' could not be set: " + result.reason);
  }

  // TODO(Koichi98): rclcpp publishes parameter event here.

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

std::string format_type_reason(
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
rcl_interfaces::msg::SetParametersResult check_parameters(
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

rcl_interfaces::msg::SetParametersResult set_parameters_atomically_common(
  const std::vector<rclcpp::Parameter> & parameters,
  std::map<std::string, ParameterInfo> & parameter_infos,
  CallbacksContainerType & callback_container, bool allow_undeclared = false)
{
  // Check if the value being set complies with the descriptor.
  rcl_interfaces::msg::SetParametersResult result =
    check_parameters(parameter_infos, parameters, allow_undeclared);
  if (!result.successful) {
    return result;
  }

  // Call the user callbacks to see if the new value(s) are allowed.
  result = call_on_parameters_set_callbacks(parameters, callback_container);
  if (!result.successful) {
    return result;
  }

  // If accepted, actually set the values.
  for (const auto & parameter : parameters) {
    const std::string & name = parameter.get_name();
    parameter_infos[name].descriptor.name = parameter.get_name();
    parameter_infos[name].descriptor.type = parameter.get_type();
    parameter_infos[name].value = parameter.get_parameter_value();
  }

  return result;
}

rcl_interfaces::msg::SetParametersResult declare_parameter_common(
  const std::string & name, const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor,
  std::map<std::string, ParameterInfo> & parameters_out,
  const std::map<std::string, rclcpp::ParameterValue> & overrides,
  CallbacksContainerType & callback_container, bool ignore_override = false)
{
  std::map<std::string, ParameterInfo> parameter_infos{{name, ParameterInfo()}};
  parameter_infos.at(name).descriptor = parameter_descriptor;

  // Use the value from the overrides if available, otherwise use the default.
  const rclcpp::ParameterValue * initial_value = &default_value;
  auto overrides_it = overrides.find(name);
  if (!ignore_override && overrides_it != overrides.end()) {
    initial_value = &overrides_it->second;
  }

  // If there is no initial value, then skip initialization
  if (initial_value->get_type() == rclcpp::PARAMETER_NOT_SET) {
    // Add declared parameters to storage (without a value)
    parameter_infos[name].descriptor.name = name;
    if (parameter_descriptor.dynamic_typing) {
      parameter_infos[name].descriptor.type = rclcpp::PARAMETER_NOT_SET;
    } else {
      parameter_infos[name].descriptor.type = parameter_descriptor.type;
    }
    parameters_out[name] = parameter_infos.at(name);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // Check with the user's callback to see if the initial value can be set.
  std::vector<rclcpp::Parameter> parameter_wrappers{rclcpp::Parameter(name, *initial_value)};
  // This function also takes care of default vs initial value.
  auto result =
    set_parameters_atomically_common(parameter_wrappers, parameter_infos, callback_container);

  if (!result.successful) {
    return result;
  }

  // Add declared parameters to storage.
  parameters_out[name] = parameter_infos.at(name);

  // TODO(Koichi98): rclcpp extends the given parameter event here.

  return result;
}

template <typename ParameterVectorType>
auto find_parameter_by_name(ParameterVectorType & parameters, const std::string & name)
{
  return std::find_if(parameters.begin(), parameters.end(), [&](const auto & parameter) {
    return parameter.get_name() == name;
  });
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
  std::lock_guard<std::recursive_mutex> lock(parameters_mutex_);
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  return declare_parameter_helper(
    name, rclcpp::PARAMETER_NOT_SET, default_value, parameter_descriptor, ignore_override,
    parameters_, parameter_overrides_, on_parameters_set_callback_container_);
}

const rclcpp::ParameterValue & NodeParameters::declare_parameter(
  const std::string & name, rclcpp::ParameterType type,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor, bool ignore_override)
{
  std::lock_guard<std::recursive_mutex> lock(parameters_mutex_);
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

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
    parameter_overrides_, on_parameters_set_callback_container_);
}

void NodeParameters::undeclare_parameter(const std::string & name)
{
  std::lock_guard<std::recursive_mutex> lock(parameters_mutex_);
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

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
  std::lock_guard<std::recursive_mutex> lock(parameters_mutex_);

  return lockless_has_parameter(parameters_, name);
}

std::vector<rcl_interfaces::msg::SetParametersResult> NodeParameters::set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::vector<rcl_interfaces::msg::SetParametersResult> results;
  results.reserve(parameters.size());

  for (const auto & p : parameters) {
    auto result = set_parameters_atomically(std::vector<rclcpp::Parameter>{p});
    results.push_back(result);
  }

  return results;
}

rcl_interfaces::msg::SetParametersResult NodeParameters::set_parameters_atomically(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::recursive_mutex> lock(parameters_mutex_);
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  rcl_interfaces::msg::SetParametersResult result;

  // Check if any of the parameters are read-only, or if any parameters are not
  // declared.
  // If not declared, keep track of them in order to declare them later, when
  // undeclared parameters are allowed, and if they're not allowed, fail.
  std::vector<const rclcpp::Parameter *> parameters_to_be_declared;
  for (const auto & parameter : parameters) {
    const std::string & name = parameter.get_name();

    // Check to make sure the parameter name is valid.
    if (name.empty()) {
      throw rclcpp::exceptions::InvalidParametersException("parameter name must not be empty");
    }

    // Check to see if it is declared.
    auto parameter_info = parameters_.find(name);
    if (parameter_info == parameters_.end()) {
      // If not check to see if undeclared parameters are allowed, ...
      if (allow_undeclared_) {
        // If so, mark the parameter to be declared for the user implicitly.
        parameters_to_be_declared.push_back(&parameter);
        // continue as it cannot be read-only, and because the declare will
        // implicitly set the parameter and parameter_infos is for setting only.
        continue;
      }
      // If not, then throw the exception as documented.
      throw rclcpp::exceptions::ParameterNotDeclaredException(
        "parameter '" + name + "' cannot be set because it was not declared");
    }

    // Check to see if it is read-only.
    if (parameter_info->second.descriptor.read_only) {
      result.successful = false;
      result.reason = "parameter '" + name + "' cannot be set because it is read-only";
      return result;
    }
  }

  // Declare parameters into a temporary "staging area", incase one of the declares fail.
  // We will use the staged changes as input to the "set atomically" action.
  // We explicitly avoid calling the user callback here, so that it may be called once, with
  // all the other parameters to be set (already declared parameters).
  std::map<std::string, ParameterInfo> staged_parameter_changes;
  CallbacksContainerType empty_callback_container;

  // Implicit declare uses dynamic type descriptor.
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  for (const auto * parameter_to_be_declared : parameters_to_be_declared) {
    // This should not throw, because we validated the name and checked that
    // the parameter was not already declared.
    result = declare_parameter_common(
      parameter_to_be_declared->get_name(), parameter_to_be_declared->get_parameter_value(),
      descriptor, staged_parameter_changes, parameter_overrides_,
      // Only call callbacks once below
      empty_callback_container, true);
    if (!result.successful) {
      // Declare failed, return knowing that nothing was changed because the
      // staged changes were not applied.
      return result;
    }
  }

  // If there were implicitly declared parameters, then we may need to copy the input parameters
  // and then assign the value that was selected after the declare (could be affected by the
  // initial parameter values).
  const std::vector<rclcpp::Parameter> * parameters_to_be_set = &parameters;
  std::vector<rclcpp::Parameter> parameters_copy;
  if (!staged_parameter_changes.empty()) {  // If there were any implicitly declared parameters.
    bool any_initial_values_used = false;
    for (const auto & staged_parameter_change : staged_parameter_changes) {
      auto it = find_parameter_by_name(parameters, staged_parameter_change.first);
      if (it->get_parameter_value() != staged_parameter_change.second.value) {
        // In this case, the value of the staged parameter differs from the
        // input from the user, and therefore we need to update things before setting.
        any_initial_values_used = true;
        // No need to search further since at least one initial value needs to be used.
        break;
      }
    }
    if (any_initial_values_used) {
      parameters_copy = parameters;
      for (const auto & staged_parameter_change : staged_parameter_changes) {
        auto it = find_parameter_by_name(parameters_copy, staged_parameter_change.first);
        *it =
          rclcpp::Parameter(staged_parameter_change.first, staged_parameter_change.second.value);
      }
      parameters_to_be_set = &parameters_copy;
    }
  }

  // Collect parameters who will have had their type changed to
  // rclcpp::PARAMETER_NOT_SET so they can later be implicitly undeclared.
  std::vector<const rclcpp::Parameter *> parameters_to_be_undeclared;
  for (const auto & parameter : *parameters_to_be_set) {
    if (rclcpp::PARAMETER_NOT_SET == parameter.get_type()) {
      auto it = parameters_.find(parameter.get_name());
      if (it != parameters_.end() && rclcpp::PARAMETER_NOT_SET != it->second.value.get_type()) {
        if (!it->second.descriptor.dynamic_typing) {
          result.reason = "cannot undeclare a statically typed parameter";
          result.successful = false;
          return result;
        }
        parameters_to_be_undeclared.push_back(&parameter);
      }
    }
  }

  // Set all of the parameters including the ones declared implicitly above.
  result = set_parameters_atomically_common(
    // either the original parameters given by the user, or ones updated with initial values
    *parameters_to_be_set,
    // they are actually set on the official parameter storage
    parameters_,
    // this will get called once, with all the parameters to be set
    on_parameters_set_callback_container_,
    allow_undeclared_);  // allow undeclared

  // If not successful, then stop here.
  if (!result.successful) {
    return result;
  }

  // If successful, then update the parameter infos from the implicitly declared parameter's.
  for (const auto & kv_pair : staged_parameter_changes) {
    // assumption: the parameter is already present in parameters_ due to the above "set"
    assert(lockless_has_parameter(parameters_, kv_pair.first));
    // assumption: the value in parameters_ is the same as the value resulting from the declare
    assert(parameters_[kv_pair.first].value == kv_pair.second.value);
    // This assignment should not change the name, type, or value, but may
    // change other things from the ParameterInfo.
    parameters_[kv_pair.first] = kv_pair.second;
  }

  // Undeclare parameters that need to be.
  for (const auto * parameter_to_undeclare : parameters_to_be_undeclared) {
    auto it = parameters_.find(parameter_to_undeclare->get_name());
    // assumption: the parameter to be undeclared should be in the parameter infos map
    assert(it != parameters_.end());
    if (it != parameters_.end()) {
      // TODO(Koichi98): rclcpp updates the parameter event message here.
      parameters_.erase(it);
    }
  }

  // TODO(Koichi98): rclcpp updates the parameter event message for parameters which were only set
  // here.

  // TODO(Koichi98): rclcpp publishes the parameter event here.

  return result;
}

std::vector<rclcpp::Parameter> NodeParameters::get_parameters(
  const std::vector<std::string> & names) const
{
  std::vector<rclcpp::Parameter> results;
  results.reserve(names.size());

  std::lock_guard<std::recursive_mutex> lock(parameters_mutex_);
  for (const auto & name : names) {
    results.emplace_back(lockless_get_parameter(parameters_, name, allow_undeclared_));
  }
  return results;
}

rclcpp::Parameter NodeParameters::get_parameter(const std::string & name) const
{
  std::lock_guard<std::recursive_mutex> lock(parameters_mutex_);

  return lockless_get_parameter(parameters_, name, allow_undeclared_);
}

bool NodeParameters::get_parameter(const std::string & name, rclcpp::Parameter & parameter) const
{
  std::lock_guard<std::recursive_mutex> lock(parameters_mutex_);

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
  std::lock_guard<std::recursive_mutex> lock(parameters_mutex_);

  std::string prefix_with_dot = prefix.empty() ? prefix : prefix + ".";
  bool ret = false;

  for (const auto & param : parameters_) {
    // TODO(bdm-k): Account for PARAMETER_NOT_SET?
    //   The current implementation mirrors that of rclcpp.
    if (param.first.find(prefix_with_dot) == 0 && param.first.length() > prefix_with_dot.length()) {
      parameters[param.first.substr(prefix_with_dot.length())] = rclcpp::Parameter(param.second);
      ret = true;
    }
  }

  return ret;
}

std::vector<rcl_interfaces::msg::ParameterDescriptor> NodeParameters::describe_parameters(
  const std::vector<std::string> & names) const
{
  std::lock_guard<std::recursive_mutex> lock(parameters_mutex_);
  std::vector<rcl_interfaces::msg::ParameterDescriptor> results;
  results.reserve(names.size());

  for (const auto & name : names) {
    auto it = parameters_.find(name);
    if (it != parameters_.cend()) {
      results.push_back(it->second.descriptor);
    } else if (allow_undeclared_) {
      rcl_interfaces::msg::ParameterDescriptor default_description;
      default_description.name = name;
      results.push_back(std::move(default_description));
    } else {
      throw rclcpp::exceptions::ParameterNotDeclaredException(name);
    }
  }

  // TODO(bdm-k): This is unreachable code and can be removed.
  //   The current implementation mirrors that of rclcpp.
  if (results.size() != names.size()) {
    throw std::runtime_error("results and names unexpectedly different sizes");
  }

  return results;
}

std::vector<uint8_t> NodeParameters::get_parameter_types(
  const std::vector<std::string> & names) const
{
  std::lock_guard<std::recursive_mutex> lock(parameters_mutex_);
  std::vector<uint8_t> results;
  results.reserve(names.size());

  for (const auto & name : names) {
    auto it = parameters_.find(name);
    if (it != parameters_.cend()) {
      results.push_back(it->second.value.get_type());
    } else if (allow_undeclared_) {
      results.push_back(rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
    } else {
      throw rclcpp::exceptions::ParameterNotDeclaredException(name);
    }
  }

  // TODO(bdm-k): This is unreachable code and can be removed.
  //    The current implementation mirrors that of rclcpp.
  if (results.size() != names.size()) {
    throw std::runtime_error("results and names unexpectedly different sizes");
  }

  return results;
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
  std::lock_guard<std::recursive_mutex> lock(parameters_mutex_);
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  auto handle = std::make_shared<rclcpp::node_interfaces::OnSetParametersCallbackHandle>();
  handle->callback = callback;
  // the last callback registered is executed first.
  on_parameters_set_callback_container_.emplace_front(handle);
  return handle;
}

void NodeParameters::remove_on_set_parameters_callback(
  const rclcpp::node_interfaces::OnSetParametersCallbackHandle * const handler)
{
  std::lock_guard<std::recursive_mutex> lock(parameters_mutex_);
  ParameterMutationRecursionGuard guard(parameter_modification_enabled_);

  auto it = std::find_if(
    on_parameters_set_callback_container_.begin(), on_parameters_set_callback_container_.end(),
    [handler](const auto & weak_handle) { return handler == weak_handle.lock().get(); });
  if (it != on_parameters_set_callback_container_.end()) {
    on_parameters_set_callback_container_.erase(it);
  } else {
    throw std::runtime_error("Callback doesn't exist");
  }
}

const std::map<std::string, rclcpp::ParameterValue> & NodeParameters::get_parameter_overrides()
  const
{
  return parameter_overrides_;
}

}  // namespace agnocast::node_interfaces
