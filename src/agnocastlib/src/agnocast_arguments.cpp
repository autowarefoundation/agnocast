#include "agnocast/agnocast_arguments.hpp"

#include <rclcpp/parameter_map.hpp>

#include <rcl_yaml_param_parser/parser.h>
#include <rcutils/allocator.h>
#include <rcutils/logging_macros.h>

#include <limits>
#include <stdexcept>

namespace agnocast
{

ParsedArguments::ParsedArguments() : args_(rcl_get_zero_initialized_arguments())
{
}

ParsedArguments::~ParsedArguments()
{
  fini();
}

void ParsedArguments::fini()
{
  if (args_.impl != nullptr) {
    rcl_ret_t ret = rcl_arguments_fini(&args_);
    if (RCL_RET_OK != ret) {
      RCUTILS_LOG_ERROR_NAMED(
        "agnocast", "Failed to finalize rcl_arguments_t: %s", rcl_get_error_string().str);
      rcl_reset_error();
    }
  }
}

ParsedArguments::ParsedArguments(ParsedArguments && other) noexcept : args_(other.args_)
{
  other.args_ = rcl_get_zero_initialized_arguments();
}

ParsedArguments & ParsedArguments::operator=(ParsedArguments && other) noexcept
{
  if (this != &other) {
    fini();
    args_ = other.args_;
    other.args_ = rcl_get_zero_initialized_arguments();
  }
  return *this;
}

ParsedArguments::ParsedArguments(const ParsedArguments & other)
: args_(rcl_get_zero_initialized_arguments())
{
  if (other.args_.impl != nullptr) {
    rcl_ret_t ret = rcl_arguments_copy(&other.args_, &args_);
    if (RCL_RET_OK != ret) {
      throw std::runtime_error(
        "Failed to copy rcl_arguments_t: " + std::string(rcl_get_error_string().str));
    }
  }
}

ParsedArguments & ParsedArguments::operator=(const ParsedArguments & other)
{
  if (this != &other) {
    fini();
    if (other.args_.impl != nullptr) {
      args_ = rcl_get_zero_initialized_arguments();
      rcl_ret_t ret = rcl_arguments_copy(&other.args_, &args_);
      if (RCL_RET_OK != ret) {
        throw std::runtime_error(
          "Failed to copy rcl_arguments_t: " + std::string(rcl_get_error_string().str));
      }
    }
  }
  return *this;
}

void ParsedArguments::parse(const std::vector<std::string> & /*arguments*/)
{
  // TODO(Koichi98)
}

ParsedArguments parse_arguments(const std::vector<std::string> & arguments)
{
  ParsedArguments result;
  result.parse(arguments);
  return result;
}

std::map<std::string, rclcpp::ParameterValue> resolve_parameter_overrides(
  const std::string & node_fqn, const std::vector<rclcpp::Parameter> & parameter_overrides,
  const rcl_arguments_t * local_args, const rcl_arguments_t * global_args)
{
  // Corresponds to rclcpp/src/rclcpp/detail/resolve_parameter_overrides.cpp
  std::map<std::string, rclcpp::ParameterValue> result;

  // global before local so that local overwrites global
  std::array<const rcl_arguments_t *, 2> argument_sources = {global_args, local_args};

  for (const rcl_arguments_t * source : argument_sources) {
    if (source == nullptr || source->impl == nullptr) {
      continue;
    }

    rcl_params_t * params = nullptr;
    rcl_ret_t ret = rcl_arguments_get_param_overrides(source, &params);
    if (RCL_RET_OK != ret) {
      RCUTILS_LOG_WARN_NAMED(
        "agnocast", "Failed to get param overrides: %s", rcl_get_error_string().str);
      rcl_reset_error();
      continue;
    }

    if (params != nullptr) {
      rclcpp::ParameterMap initial_map = rclcpp::parameter_map_from(params, node_fqn.c_str());

      if (initial_map.count(node_fqn) > 0) {
        for (const rclcpp::Parameter & param : initial_map.at(node_fqn)) {
          result[param.get_name()] = rclcpp::ParameterValue(param.get_value_message());
        }
      }

      rcl_yaml_node_struct_fini(params);
    }
  }

  // parameter overrides passed to constructor will overwrite overrides from yaml file sources
  for (const auto & param : parameter_overrides) {
    result[param.get_name()] = rclcpp::ParameterValue(param.get_value_message());
  }

  return result;
}

}  // namespace agnocast
