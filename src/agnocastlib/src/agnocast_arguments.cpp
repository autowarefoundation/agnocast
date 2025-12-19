#include "agnocast/agnocast_arguments.hpp"

#include <rclcpp/parameter_map.hpp>

#include <rcutils/allocator.h>
#include <rcutils/logging_macros.h>

#include <algorithm>

namespace agnocast
{

namespace
{

std::string parse_node_name_prefix(const std::string & arg, size_t & pos)
{
  size_t colon_pos = arg.find(':', pos);
  size_t separator_pos = arg.find(":=", pos);

  // If ':' exists and is before ':=', it's a node name prefix
  if (colon_pos != std::string::npos && colon_pos < separator_pos) {
    std::string node_name = arg.substr(pos, colon_pos - pos);
    pos = colon_pos + 1;
    return node_name;
  }

  // No node name prefix, match all nodes
  return "/**";
}

}  // namespace

ParameterOverrides::ParameterOverrides()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  params_ = rcl_yaml_node_struct_init(allocator);
  if (nullptr == params_) {
    throw std::runtime_error("Failed to initialize rcl_params_t");
  }
}

ParameterOverrides::~ParameterOverrides()
{
  if (params_ != nullptr) {
    rcl_yaml_node_struct_fini(params_);
    params_ = nullptr;
  }
}

ParameterOverrides::ParameterOverrides(const ParameterOverrides & other) : params_(nullptr)
{
  if (other.params_ != nullptr) {
    params_ = rcl_yaml_node_struct_copy(other.params_);
    if (nullptr == params_) {
      throw std::runtime_error("Failed to copy rcl_params_t");
    }
  }
}

ParameterOverrides & ParameterOverrides::operator=(const ParameterOverrides & other)
{
  if (this != &other) {
    if (params_ != nullptr) {
      rcl_yaml_node_struct_fini(params_);
      params_ = nullptr;
    }
    if (other.params_ != nullptr) {
      params_ = rcl_yaml_node_struct_copy(other.params_);
      if (nullptr == params_) {
        throw std::runtime_error("Failed to copy rcl_params_t");
      }
    }
  }
  return *this;
}

ParameterOverrides::ParameterOverrides(ParameterOverrides && other) noexcept
: params_(other.params_)
{
  other.params_ = nullptr;
}

ParameterOverrides & ParameterOverrides::operator=(ParameterOverrides && other) noexcept
{
  if (this != &other) {
    if (params_ != nullptr) {
      rcl_yaml_node_struct_fini(params_);
    }
    params_ = other.params_;
    other.params_ = nullptr;
  }
  return *this;
}

bool ParameterOverrides::parse_yaml_file(const std::string & yaml_file)
{
  return rcl_parse_yaml_file(yaml_file.c_str(), params_);
}

bool ParameterOverrides::parse_param_rule(const std::string & arg)
{
  size_t pos = 0;
  std::string node_name = parse_node_name_prefix(arg, pos);

  // Find the separator ':='
  size_t separator_pos = arg.find(":=", pos);
  if (separator_pos == std::string::npos) {
    return false;
  }

  std::string param_name = arg.substr(pos, separator_pos - pos);
  std::string yaml_value = arg.substr(separator_pos + 2);

  if (param_name.empty()) {
    return false;
  }

  return rcl_parse_yaml_value(node_name.c_str(), param_name.c_str(), yaml_value.c_str(), params_);
}

bool parse_remap_rule(const std::string & arg, RemapRule & output_rule)
{
  // Corresponds to _rcl_parse_remap_rule in rcl/src/rcl/arguments.c.
  size_t separator = arg.find(":=");
  if (separator == std::string::npos) {
    return false;
  }

  std::string from = arg.substr(0, separator);
  std::string to = arg.substr(separator + 2);

  RemapRule rule;
  rule.replacement = to;

  size_t colon_pos = from.find(':');
  if (colon_pos != std::string::npos && colon_pos < separator) {
    if (!from.empty() && from[0] != '/') {
      rule.node_name = from.substr(0, colon_pos);
      rule.match = from.substr(colon_pos + 1);
    } else {
      rule.match = from;
    }
  } else {
    rule.match = from;
  }

  if (rule.match == "__node" || rule.match == "__name") {
    rule.type = RemapType::NODE_NAME;
    rule.node_name.clear();  // __node/__name rules are always global
  } else if (rule.match == "__ns") {
    rule.type = RemapType::NAMESPACE;
    rule.node_name.clear();  // __ns rules are always global
  } else {
    rule.type = RemapType::TOPIC_OR_SERVICE;
  }

  output_rule = rule;
  return true;
}

ParsedArguments parse_arguments(const std::vector<std::string> & arguments)
{
  // Corresponds to rcl_parse_arguments in rcl/src/rcl/arguments.c
  ParsedArguments result;

  bool parsing_ros_args = false;
  for (size_t i = 0; i < arguments.size(); ++i) {
    const std::string & arg = arguments[i];

    if (parsing_ros_args) {
      // Ignore ROS specific arguments flag
      if (arg == RCL_ROS_ARGS_FLAG) {
        continue;
      }

      // Check for ROS specific arguments explicit end token
      if (arg == RCL_ROS_ARGS_EXPLICIT_END_TOKEN) {
        parsing_ros_args = false;
        continue;
      }

      // Attempt to parse argument as parameter file flag
      if (arg == RCL_PARAM_FILE_FLAG && i + 1 < arguments.size()) {
        ++i;
        if (!result.parameter_overrides.parse_yaml_file(arguments[i])) {
          RCUTILS_LOG_WARN_NAMED(
            "agnocast", "Failed to parse params file: %s", arguments[i].c_str());
        }
        continue;
      }

      // Attempt to parse argument as parameter override flag
      if ((arg == RCL_PARAM_FLAG || arg == RCL_SHORT_PARAM_FLAG) && i + 1 < arguments.size()) {
        ++i;
        if (!result.parameter_overrides.parse_param_rule(arguments[i])) {
          RCUTILS_LOG_WARN_NAMED(
            "agnocast", "Failed to parse parameter rule: %s", arguments[i].c_str());
        }
        continue;
      }

      // Attempt to parse argument as remap rule flag
      if ((arg == RCL_REMAP_FLAG || arg == RCL_SHORT_REMAP_FLAG) && i + 1 < arguments.size()) {
        ++i;
        RemapRule rule;
        if (parse_remap_rule(arguments[i], rule)) {
          result.remap_rules.push_back(rule);
        }
        continue;
      }

      // TODO(Koichi98): Parse other ROS specific arguments.

    } else {
      // Check for ROS specific arguments flag
      if (arg == RCL_ROS_ARGS_FLAG) {
        parsing_ros_args = true;
        continue;
      }

      // Argument is not a ROS specific argument
      // In RCL this would be stored in unparsed_args
    }
  }

  return result;
}

std::map<std::string, ParameterValue> resolve_parameter_overrides(
  const std::string & node_fqn, const std::vector<rclcpp::Parameter> & parameter_overrides,
  const ParsedArguments & local_args, const ParsedArguments & global_args)
{
  // Corresponds to rclcpp/src/rclcpp/detail/resolve_parameter_overrides.cpp
  std::map<std::string, ParameterValue> result;

  // global before local so that local overwrites global
  std::array<const ParameterOverrides *, 2> arguments_sources = {
    &global_args.parameter_overrides, &local_args.parameter_overrides};

  for (const ParameterOverrides * source : arguments_sources) {
    if (source == nullptr || source->get() == nullptr) {
      continue;
    }

    rclcpp::ParameterMap initial_map = rclcpp::parameter_map_from(source->get(), node_fqn.c_str());

    if (initial_map.count(node_fqn) > 0) {
      for (const rclcpp::Parameter & param : initial_map.at(node_fqn)) {
        result[param.get_name()] = rclcpp::ParameterValue(param.get_value_message());
      }
    }
  }

  // parameter overrides passed to constructor will overwrite overrides from yaml file sources
  for (const auto & param : parameter_overrides) {
    result[param.get_name()] = rclcpp::ParameterValue(param.get_value_message());
  }

  return result;
}

}  // namespace agnocast
