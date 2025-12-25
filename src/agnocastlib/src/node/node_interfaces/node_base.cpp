#include "agnocast/node/node_interfaces/node_base.hpp"

#include "agnocast/node/agnocast_context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/logging.hpp"

#include <rcl/error_handling.h>
#include <rcl/remap.h>

#include <stdexcept>
#include <utility>

namespace agnocast::node_interfaces
{

NodeBase::NodeBase(
  std::string node_name, const std::string & ns, rclcpp::Context::SharedPtr context,
  const rcl_arguments_t * local_args, bool use_intra_process_default,
  bool enable_topic_statistics_default)
: node_name_(std::move(node_name)),
  context_(std::move(context)),
  use_intra_process_default_(use_intra_process_default),
  enable_topic_statistics_default_(enable_topic_statistics_default),
  local_args_(local_args)
{
  if (ns.empty()) {
    namespace_ = "/";
  } else if (ns[0] != '/') {
    namespace_ = "/" + ns;
  } else {
    namespace_ = ns;
  }

  // Get global arguments from context
  {
    std::lock_guard<std::mutex> lock(g_context_mtx);
    if (g_context.is_initialized()) {
      global_args_ = g_context.get_parsed_arguments();
    }
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();

  // Apply node name remapping
  {
    char * remapped_node_name = nullptr;
    rcl_ret_t ret = rcl_remap_node_name(
      local_args_, global_args_, node_name_.c_str(), allocator, &remapped_node_name);

    if (ret != RCL_RET_OK) {
      std::string error_msg =
        std::string("Failed to remap node name: ") + rcl_get_error_string().str;
      rcl_reset_error();
      throw std::runtime_error(error_msg);
    }
    if (remapped_node_name != nullptr) {
      node_name_ = remapped_node_name;
      allocator.deallocate(remapped_node_name, allocator.state);
    }
  }

  // Apply namespace remapping
  {
    char * remapped_namespace = nullptr;
    rcl_ret_t ret = rcl_remap_node_namespace(
      local_args_, global_args_, node_name_.c_str(), allocator, &remapped_namespace);

    if (ret != RCL_RET_OK) {
      std::string error_msg =
        std::string("Failed to remap namespace: ") + rcl_get_error_string().str;
      rcl_reset_error();
      throw std::runtime_error(error_msg);
    }
    if (remapped_namespace != nullptr) {
      namespace_ = remapped_namespace;
      allocator.deallocate(remapped_namespace, allocator.state);
    }
  }

  if (namespace_ == "/") {
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

}  // namespace agnocast::node_interfaces
