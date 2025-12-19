#include "agnocast/node_interfaces/node_topics.hpp"

#include <stdexcept>
#include <utility>

namespace agnocast::node_interfaces
{

NodeTopics::NodeTopics(NodeBase::SharedPtr node_base) : node_base_(std::move(node_base))
{
}

std::string NodeTopics::resolve_topic_name(const std::string & name, bool only_expand) const
{
  // Corresponds to rcl_node_resolve_name in rcl/src/rcl/node_resolve_name.c:134-162
  std::string expanded_topic_name = expand_topic_name(name);
  if (only_expand) {
    return expanded_topic_name;
  }

  return remap_name(expanded_topic_name);
  // TODO(Koichi98): rmw_validate_full_topic_name (see node_resolve_name.c)
}

rclcpp::node_interfaces::NodeBaseInterface * NodeTopics::get_node_base_interface() const
{
  return node_base_.get();
}

rclcpp::PublisherBase::SharedPtr NodeTopics::create_publisher(
  const std::string & topic_name, const rclcpp::PublisherFactory & publisher_factory,
  const rclcpp::QoS & qos)
{
  (void)topic_name;
  (void)publisher_factory;
  (void)qos;
  throw std::runtime_error(
    "NodeTopics::create_publisher is not supported in agnocast. "
    "Use agnocast::create_publisher instead.");
}

void NodeTopics::add_publisher(
  rclcpp::PublisherBase::SharedPtr publisher, rclcpp::CallbackGroup::SharedPtr callback_group)
{
  (void)publisher;
  (void)callback_group;
  throw std::runtime_error(
    "NodeTopics::add_publisher is not supported in agnocast. "
    "Use agnocast::create_publisher instead.");
}

rclcpp::SubscriptionBase::SharedPtr NodeTopics::create_subscription(
  const std::string & topic_name, const rclcpp::SubscriptionFactory & subscription_factory,
  const rclcpp::QoS & qos)
{
  (void)topic_name;
  (void)subscription_factory;
  (void)qos;
  throw std::runtime_error(
    "NodeTopics::create_subscription is not supported in agnocast. "
    "Use agnocast::create_subscription instead.");
}

void NodeTopics::add_subscription(
  rclcpp::SubscriptionBase::SharedPtr subscription, rclcpp::CallbackGroup::SharedPtr callback_group)
{
  (void)subscription;
  (void)callback_group;
  throw std::runtime_error(
    "NodeTopics::add_subscription is not supported in agnocast. "
    "Use agnocast::create_subscription instead.");
}

rclcpp::node_interfaces::NodeTimersInterface * NodeTopics::get_node_timers_interface() const
{
  throw std::runtime_error(
    "NodeTopics::get_node_timers_interface is not supported in agnocast. "
    "Timers interface is not available.");
}

std::string NodeTopics::expand_topic_name(const std::string & input_topic_name) const
{
  // Corresponds to rcl_expand_topic_name
  //
  // TODO(Koichi98): Support custom substitutions via rcutils_string_map_t
  // TODO(Koichi98): Validate input_topic_name using rcl_validate_topic_name
  // TODO(Koichi98): Validate node_name using rmw_validate_node_name
  // TODO(Koichi98): Validate node_namespace using rmw_validate_namespace

  if (input_topic_name.empty()) {
    throw std::invalid_argument("topic name must not be empty");
  }

  const std::string node_name = node_base_->get_name();
  const std::string node_namespace = node_base_->get_namespace();

  // Check if the topic has substitutions to be made
  bool has_a_substitution = input_topic_name.find('{') != std::string::npos;
  bool has_a_namespace_tilde = input_topic_name[0] == '~';
  bool is_absolute = input_topic_name[0] == '/';

  // If absolute and doesn't have any substitution, nothing to do
  if (is_absolute && !has_a_substitution) {
    return input_topic_name;
  }

  std::string local_output;

  // If has_a_namespace_tilde, replace that first
  if (has_a_namespace_tilde) {
    // Special case where node_namespace is just '/'
    // then no additional separating '/' is needed
    if (node_namespace == "/") {
      local_output = node_namespace + node_name + input_topic_name.substr(1);
    } else {
      local_output = node_namespace + "/" + node_name + input_topic_name.substr(1);
    }
  }

  // If it has any substitutions, replace those
  if (has_a_substitution) {
    // Assumptions entering this scope about the topic string:
    // - All {} are matched and balanced
    // - There is no nesting, i.e. {{}}
    // - There are no empty substitution substr, i.e. '{}' versus '{something}'
    const std::string & current_output = local_output.empty() ? input_topic_name : local_output;
    std::string result = current_output;
    size_t search_pos = 0;

    while (true) {
      size_t next_opening_brace = result.find('{', search_pos);
      if (next_opening_brace == std::string::npos) {
        break;
      }
      size_t next_closing_brace = result.find('}', next_opening_brace);
      if (next_closing_brace == std::string::npos) {
        break;
      }

      // conclusion based on above assumptions: next_closing_brace - next_opening_brace > 1
      size_t substitution_substr_len = next_closing_brace - next_opening_brace + 1;
      std::string substitution = result.substr(next_opening_brace, substitution_substr_len);

      // Figure out what the replacement is for this substitution
      std::string replacement;
      if (substitution == "{node}") {
        replacement = node_name;
      } else if (substitution == "{ns}" || substitution == "{namespace}") {
        replacement = node_namespace;
      } else {
        // TODO(Koichi98): Check custom substitutions map before throwing
        throw std::invalid_argument("unknown substitution: " + substitution);
      }

      // Do the replacement
      result.replace(next_opening_brace, substitution_substr_len, replacement);
      search_pos = next_opening_brace + replacement.length();
    }
    local_output = result;
  }

  // Finally make the name absolute if it isn't already
  const std::string & name_to_check = local_output.empty() ? input_topic_name : local_output;
  if (name_to_check[0] != '/') {
    // Special case where node_namespace is just '/'
    // then no additional separating '/' is needed
    if (node_namespace == "/") {
      local_output = node_namespace + name_to_check;
    } else {
      local_output = node_namespace + "/" + name_to_check;
    }
  }

  return local_output;
}

const RemapRule * NodeTopics::remap_first_match(
  const std::vector<RemapRule> & remap_rules, const std::string & name) const
{
  // Corresponds to rcl_remap_first_match

  const std::string node_name = node_base_->get_name();

  for (const auto & rule : remap_rules) {
    if (rule.type != RemapType::TOPIC_OR_SERVICE) {
      // Not the type of remap rule we're looking for
      continue;
    }
    if (!rule.node_name.empty() && rule.node_name != node_name) {
      // Rule has a node name prefix and the supplied node name didn't match
      continue;
    }
    // topic and service rules need the match side to be expanded to a FQN
    std::string expanded_match = expand_topic_name(rule.match);
    if (expanded_match == name) {
      return &rule;
    }
  }
  return nullptr;
}

std::string NodeTopics::remap_name(const std::string & topic_name) const
{
  // Corresponds to rcl_remap_name

  std::string output_name;
  const RemapRule * rule = nullptr;

  // Look at local rules first
  rule = remap_first_match(node_base_->get_local_remap_rules(), topic_name);

  // Check global rules if no local rule matched
  if (rule == nullptr) {
    rule = remap_first_match(node_base_->get_global_remap_rules(), topic_name);
  }

  // Do the remapping
  if (rule != nullptr) {
    // topic and service rules need the replacement to be expanded to a FQN
    output_name = expand_topic_name(rule->replacement);
  } else {
    output_name = topic_name;
  }

  return output_name;
}

}  // namespace agnocast::node_interfaces
