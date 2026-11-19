#pragma once

#include <rclcpp/context.hpp>
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

// Resolve the global arguments a node must honour, for NodeOptions::use_global_arguments().
// agnocast::init() parses them when it is called; a process whose main() belongs to someone else,
// such as a component container, parses the command line into its rclcpp context instead, and that
// is where they are then. Reading both makes the option mean the same thing for agnocast::Node as
// it does for rclcpp::Node. Returns nullptr when neither source has any. The result points into
// whichever source it came from, so the caller must keep `context` alive while it uses it.
const rcl_arguments_t * resolve_global_arguments(const rclcpp::Context::SharedPtr & context);

// Resolve parameter overrides from multiple sources.
// Corresponds to rclcpp::detail::resolve_parameter_overrides.
// Priority (later overwrites earlier): global_args < local_args < parameter_overrides
std::map<std::string, rclcpp::ParameterValue> resolve_parameter_overrides(
  const std::string & node_fqn, const std::vector<rclcpp::Parameter> & parameter_overrides,
  const rcl_arguments_t * local_args, const rcl_arguments_t * global_args);

}  // namespace agnocast
