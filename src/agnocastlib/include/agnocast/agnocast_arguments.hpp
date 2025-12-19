#pragma once

#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>

#include <rcl/arguments.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rcl_yaml_param_parser/types.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace agnocast
{

enum class RemapType { NODE_NAME, NAMESPACE, TOPIC_OR_SERVICE };

struct RemapRule
{
  RemapType type;
  std::string node_name;  // Node name prefix (empty means global rule)
  std::string match;
  std::string replacement;
};

using ParameterValue = rclcpp::ParameterValue;

class ParameterOverrides
{
public:
  ParameterOverrides();
  ~ParameterOverrides();

  ParameterOverrides(const ParameterOverrides & other);
  ParameterOverrides & operator=(const ParameterOverrides & other);

  ParameterOverrides(ParameterOverrides && other) noexcept;
  ParameterOverrides & operator=(ParameterOverrides && other) noexcept;

  rcl_params_t * get() const { return params_; }

  bool parse_yaml_file(const std::string & yaml_file);
  bool parse_param_rule(const std::string & arg);

private:
  rcl_params_t * params_;
};

struct ParsedArguments
{
  std::vector<RemapRule> remap_rules;
  ParameterOverrides parameter_overrides;
};

bool parse_remap_rule(const std::string & arg, RemapRule & output_rule);

ParsedArguments parse_arguments(const std::vector<std::string> & arguments);

/// Resolve parameter overrides from multiple sources.
/// Corresponds to rclcpp::detail::resolve_parameter_overrides.
/// Priority (later overwrites earlier): global_args < local_args < parameter_overrides
/// @param node_fqn Fully qualified name of the node (used for node-specific filtering)
/// @param parameter_overrides Parameters from NodeOptions::parameter_overrides()
/// @param local_args Parsed arguments from NodeOptions::arguments()
/// @param global_args Parsed arguments from command line (via agnocast::init)
std::map<std::string, ParameterValue> resolve_parameter_overrides(
  const std::string & node_fqn, const std::vector<rclcpp::Parameter> & parameter_overrides,
  const ParsedArguments & local_args, const ParsedArguments & global_args);

}  // namespace agnocast
