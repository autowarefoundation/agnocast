#include "agnocast/node/node_interfaces/node_base.hpp"

#include "agnocast/node/agnocast_context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/logging.hpp"

#include <rcl/error_handling.h>
#include <rcl/expand_topic_name.h>
#include <rcl/remap.h>
#include <rcutils/strdup.h>
#include <rmw/error_handling.h>
#include <rmw/validate_full_topic_name.h>

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

// This implementation aligns with rcl_resolve_name() in rcl/src/rcl/node_resolve_name.c
std::string NodeBase::resolve_topic_or_service_name(
  const std::string & name, bool is_service, bool only_expand) const
{
  rcl_allocator_t allocator = rcl_get_default_allocator();

  std::string output_topic_name;

  // Create default topic name substitutions map
  rcutils_string_map_t substitutions_map = rcutils_get_zero_initialized_string_map();
  rcutils_ret_t rcutils_ret = rcutils_string_map_init(&substitutions_map, 0, allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    rcutils_error_string_t error = rcutils_get_error_string();
    rcutils_reset_error();
    if (RCUTILS_RET_BAD_ALLOC == rcutils_ret) {
      throw std::bad_alloc();
    }
    throw std::runtime_error(&error.str[0]);
  }

  char * expanded_topic_name = nullptr;
  char * remapped_topic_name = nullptr;
  int validation_result = 0;
  rmw_ret_t rmw_ret = RMW_RET_OK;
  rcl_ret_t ret = rcl_get_default_topic_name_substitutions(&substitutions_map);
  if (ret != RCL_RET_OK) {
    if (RCL_RET_BAD_ALLOC != ret) {
      ret = RCL_RET_ERROR;
    }
    goto cleanup;  // NOLINT(cppcoreguidelines-avoid-goto, hicpp-avoid-goto)
  }

  // expand topic name
  ret = rcl_expand_topic_name(
    name.c_str(), node_name_.c_str(), namespace_.c_str(), &substitutions_map, allocator,
    &expanded_topic_name);
  if (RCL_RET_OK != ret) {
    goto cleanup;  // NOLINT(cppcoreguidelines-avoid-goto, hicpp-avoid-goto)
  }

  // remap topic name
  if (!only_expand) {
    if (is_service) {
      ret = rcl_remap_service_name(
        local_args_, global_args_, expanded_topic_name, node_name_.c_str(), namespace_.c_str(),
        allocator, &remapped_topic_name);
    } else {
      ret = rcl_remap_topic_name(
        local_args_, global_args_, expanded_topic_name, node_name_.c_str(), namespace_.c_str(),
        allocator, &remapped_topic_name);
    }
    if (RCL_RET_OK != ret) {
      goto cleanup;  // NOLINT(cppcoreguidelines-avoid-goto, hicpp-avoid-goto)
    }
  }

  if (nullptr == remapped_topic_name) {
    remapped_topic_name = expanded_topic_name;
    expanded_topic_name = nullptr;
  }

  // validate the result
  rmw_ret = rmw_validate_full_topic_name(remapped_topic_name, &validation_result, nullptr);
  if (rmw_ret != RMW_RET_OK) {
    rcutils_error_string_t error = rmw_get_error_string();
    rmw_reset_error();
    RCL_SET_ERROR_MSG(&error.str[0]);
    ret = RCL_RET_ERROR;
    goto cleanup;  // NOLINT(cppcoreguidelines-avoid-goto, hicpp-avoid-goto)
  }
  if (validation_result != RMW_TOPIC_VALID) {
    RCL_SET_ERROR_MSG(rmw_full_topic_name_validation_result_string(validation_result));
    ret = RCL_RET_TOPIC_NAME_INVALID;
    goto cleanup;  // NOLINT(cppcoreguidelines-avoid-goto, hicpp-avoid-goto)
  }

  output_topic_name = remapped_topic_name;
  remapped_topic_name = nullptr;

cleanup:
  rcutils_ret = rcutils_string_map_fini(&substitutions_map);
  if (rcutils_ret != RCUTILS_RET_OK) {
    rcutils_error_string_t error = rcutils_get_error_string();
    rcutils_reset_error();
    if (RCL_RET_OK == ret) {
      RCL_SET_ERROR_MSG(&error.str[0]);
      ret = RCL_RET_ERROR;
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger(node_name_), "failed to fini string_map (%d) during error handling: %s",
        rcutils_ret, &error.str[0]);
    }
  }
  allocator.deallocate(expanded_topic_name, allocator.state);
  allocator.deallocate(remapped_topic_name, allocator.state);
  if (is_service && RCL_RET_TOPIC_NAME_INVALID == ret) {
    ret = RCL_RET_SERVICE_NAME_INVALID;
  }

  if (ret != RCL_RET_OK) {
    rcl_error_string_t error = rcl_get_error_string();
    rcl_reset_error();
    throw std::runtime_error(&error.str[0]);
  }

  return output_topic_name;
}

}  // namespace agnocast::node_interfaces
