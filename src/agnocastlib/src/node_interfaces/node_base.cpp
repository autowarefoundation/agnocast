#include "agnocast/node_interfaces/node_base.hpp"

#include "agnocast/agnocast_context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/logging.hpp"

#include <stdexcept>
#include <utility>

namespace agnocast::node_interfaces
{

NodeBase::NodeBase(
  std::string node_name, const std::string & ns, rclcpp::Context::SharedPtr context,
  std::vector<RemapRule> local_remap_rules, bool use_intra_process_default,
  bool enable_topic_statistics_default)
: node_name_(std::move(node_name)),
  context_(std::move(context)),
  use_intra_process_default_(use_intra_process_default),
  enable_topic_statistics_default_(enable_topic_statistics_default),
  local_remap_rules_(std::move(local_remap_rules))
{
  // Ensure it starts with '/' or is empty
  if (!ns.empty() && ns[0] != '/') {
    namespace_ = "/" + ns;
  } else {
    namespace_ = ns;
  }

  // Apply node name and namespace remapping from agnocast::Context.
  // Following rclcpp's "first-wins" behavior: only the first matching rule of each type is applied.
  {
    std::lock_guard<std::mutex> lock(g_context_mtx);
    if (g_context.is_initialized()) {
      global_remap_rules_ = g_context.get_remap_rules();

      auto node_name_it = std::find_if(
        global_remap_rules_.begin(), global_remap_rules_.end(),
        [](const auto & rule) { return rule.type == RemapType::NODE_NAME; });
      if (node_name_it != global_remap_rules_.end()) {
        node_name_ = node_name_it->replacement;
      }

      auto namespace_it = std::find_if(
        global_remap_rules_.begin(), global_remap_rules_.end(),
        [](const auto & rule) { return rule.type == RemapType::NAMESPACE; });
      if (namespace_it != global_remap_rules_.end()) {
        namespace_ = namespace_it->replacement;
        if (!namespace_.empty() && namespace_[0] != '/') {
          namespace_ = "/" + namespace_;
        }
      }
    }
  }

  if (namespace_.empty() || namespace_ == "/") {
    fqn_ = "/" + node_name_;
  } else {
    fqn_ = namespace_ + "/" + node_name_;
  }

  default_callback_group_ =
    std::make_shared<rclcpp::CallbackGroup>(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_groups_.push_back(default_callback_group_);
}

const char * NodeBase::get_name() const
{
  return node_name_.c_str();
}

const char * NodeBase::get_namespace() const
{
  return namespace_.c_str();
}

const char * NodeBase::get_fully_qualified_name() const
{
  return fqn_.c_str();
}

rclcpp::Context::SharedPtr NodeBase::get_context()
{
  return context_;
}

rcl_node_t * NodeBase::get_rcl_node_handle()
{
  throw std::runtime_error(
    "rcl_node_handle is not available in agnocast::Node. "
    "This node does not use DDS.");
}

const rcl_node_t * NodeBase::get_rcl_node_handle() const
{
  throw std::runtime_error(
    "rcl_node_handle is not available in agnocast::Node. "
    "This node does not use DDS.");
}

std::shared_ptr<rcl_node_t> NodeBase::get_shared_rcl_node_handle()
{
  throw std::runtime_error(
    "rcl_node_handle is not available in agnocast::Node. "
    "This node does not use DDS.");
}

std::shared_ptr<const rcl_node_t> NodeBase::get_shared_rcl_node_handle() const
{
  throw std::runtime_error(
    "rcl_node_handle is not available in agnocast::Node. "
    "This node does not use DDS.");
}

rclcpp::CallbackGroup::SharedPtr NodeBase::create_callback_group(
  rclcpp::CallbackGroupType group_type, bool automatically_add_to_executor_with_node)
{
  auto group =
    std::make_shared<rclcpp::CallbackGroup>(group_type, automatically_add_to_executor_with_node);

  std::lock_guard<std::mutex> lock(callback_groups_mutex_);
  callback_groups_.push_back(group);
  return group;
}

rclcpp::CallbackGroup::SharedPtr NodeBase::get_default_callback_group()
{
  return default_callback_group_;
}

bool NodeBase::callback_group_in_node(rclcpp::CallbackGroup::SharedPtr group)
{
  std::lock_guard<std::mutex> lock(callback_groups_mutex_);
  // NOLINTNEXTLINE(readability-use-anyofallof) - align with rclcpp::node_interfaces::NodeBase
  for (auto & weak_group : callback_groups_) {
    auto cur_group = weak_group.lock();
    if (cur_group && (cur_group == group)) {
      return true;
    }
  }
  return false;
}

void NodeBase::for_each_callback_group(const CallbackGroupFunction & func)
{
  std::lock_guard<std::mutex> lock(callback_groups_mutex_);
  for (auto & weak_group : callback_groups_) {
    auto group = weak_group.lock();
    if (group) {
      func(group);
    }
  }
}

std::atomic_bool & NodeBase::get_associated_with_executor_atomic()
{
  return associated_with_executor_;
}

rclcpp::GuardCondition & NodeBase::get_notify_guard_condition()
{
  throw std::runtime_error("notify_guard_condition is not available in agnocast::Node.");
}

bool NodeBase::get_use_intra_process_default() const
{
  // Note: rclcpp's intra-process communication is not used in Agnocast.
  // This value is propagated from NodeOptions but has no effect currently.
  RCLCPP_WARN(
    rclcpp::get_logger(node_name_),
    "use_intra_process_comms setting has no effect when using Agnocast. "
    "Agnocast uses its own zero-copy intra/inter-process communication mechanism instead of "
    "rclcpp's intra-process communication.");
  return use_intra_process_default_;
}

bool NodeBase::get_enable_topic_statistics_default() const
{
  // Note: Topic statistics collection/publication is not yet implemented in Agnocast.
  // This value is propagated from NodeOptions but has no effect currently.
  return enable_topic_statistics_default_;
}

std::string NodeBase::resolve_topic_or_service_name(
  const std::string & name, bool is_service, bool only_expand) const
{
  (void)name;
  (void)is_service;
  (void)only_expand;
  // TODO(Koichi98)
  return "";
}

const std::vector<RemapRule> & NodeBase::get_local_remap_rules() const
{
  return local_remap_rules_;
}

const std::vector<RemapRule> & NodeBase::get_global_remap_rules() const
{
  return global_remap_rules_;
}

}  // namespace agnocast::node_interfaces
