#pragma once

#include "agnocast/node/agnocast_arguments.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/version.h"

#include <rcl/arguments.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace agnocast::node_interfaces
{

class NodeBase : public rclcpp::node_interfaces::NodeBaseInterface
{
public:
  using SharedPtr = std::shared_ptr<NodeBase>;
  using WeakPtr = std::weak_ptr<NodeBase>;

  NodeBase(
    std::string node_name, const std::string & ns, rclcpp::Context::SharedPtr context,
    const rcl_arguments_t * local_args, bool use_intra_process_default = false,
    bool enable_topic_statistics_default = false);

  virtual ~NodeBase() = default;

  const char * get_name() const override;
  const char * get_namespace() const override;
  const char * get_fully_qualified_name() const override;

  rclcpp::Context::SharedPtr get_context() override;

  rcl_node_t * get_rcl_node_handle() override;
  const rcl_node_t * get_rcl_node_handle() const override;
  std::shared_ptr<rcl_node_t> get_shared_rcl_node_handle() override;
  std::shared_ptr<const rcl_node_t> get_shared_rcl_node_handle() const override;

  rclcpp::CallbackGroup::SharedPtr create_callback_group(
    rclcpp::CallbackGroupType group_type,
    bool automatically_add_to_executor_with_node = true) override;
  rclcpp::CallbackGroup::SharedPtr get_default_callback_group() override;
  bool callback_group_in_node(rclcpp::CallbackGroup::SharedPtr group) override;
  void for_each_callback_group(const CallbackGroupFunction & func) override;

  std::atomic_bool & get_associated_with_executor_atomic() override;
  rclcpp::GuardCondition & get_notify_guard_condition() override;

  // rclcpp 28+ (Jazzy) added these methods to NodeBaseInterface.
#if RCLCPP_VERSION_MAJOR >= 28
  rclcpp::GuardCondition::SharedPtr get_shared_notify_guard_condition() override;
  void trigger_notify_guard_condition() override;
#endif

  bool get_use_intra_process_default() const override;
  bool get_enable_topic_statistics_default() const override;

  std::string resolve_topic_or_service_name(
    const std::string & name, bool is_service, bool only_expand = false) const override;

  const rcl_arguments_t * get_local_args() const { return local_args_; }
  const rcl_arguments_t * get_global_args() const { return global_args_; }

private:
  std::string node_name_;
  std::string namespace_;
  std::string fqn_;

  // When loaded as a composable node, a valid context is passed from the component manager.
  // For standalone agnocast nodes (without rclcpp::init()), this will be nullptr.
  rclcpp::Context::SharedPtr context_;
  rclcpp::CallbackGroup::SharedPtr default_callback_group_;
  std::vector<rclcpp::CallbackGroup::WeakPtr> callback_groups_;
  mutable std::mutex callback_groups_mutex_;

  std::atomic_bool associated_with_executor_{false};

  bool use_intra_process_default_;
  bool enable_topic_statistics_default_;

  const rcl_arguments_t * local_args_ = nullptr;
  const rcl_arguments_t * global_args_ = nullptr;
};

}  // namespace agnocast::node_interfaces
