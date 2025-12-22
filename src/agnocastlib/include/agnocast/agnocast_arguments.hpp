#pragma once

#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>

#include <rcl/arguments.h>

#include <map>
#include <string>
#include <vector>

namespace agnocast
{

class ParsedArguments
{
public:
  ParsedArguments();
  ~ParsedArguments();

  ParsedArguments(ParsedArguments && other) noexcept;
  ParsedArguments & operator=(ParsedArguments && other) noexcept;

  ParsedArguments(const ParsedArguments & other);
  ParsedArguments & operator=(const ParsedArguments & other);

  void parse(const std::vector<std::string> & arguments);

  rcl_arguments_t * get() { return &args_; }
  const rcl_arguments_t * get() const { return &args_; }

  bool is_valid() const { return args_.impl != nullptr; }

private:
  rcl_arguments_t args_;

  void fini();
};

ParsedArguments parse_arguments(const std::vector<std::string> & arguments);

/// Resolve parameter overrides from multiple sources.
/// Corresponds to rclcpp::detail::resolve_parameter_overrides.
/// Priority (later overwrites earlier): global_args < local_args < parameter_overrides
/// @param node_fqn Fully qualified name of the node (used for node-specific filtering)
/// @param parameter_overrides Parameters from NodeOptions::parameter_overrides()
/// @param local_args Parsed arguments from NodeOptions::arguments()
/// @param global_args Parsed arguments from command line (via agnocast::init)
std::map<std::string, rclcpp::ParameterValue> resolve_parameter_overrides(
  const std::string & node_fqn, const std::vector<rclcpp::Parameter> & parameter_overrides,
  const rcl_arguments_t * local_args, const rcl_arguments_t * global_args);

}  // namespace agnocast
