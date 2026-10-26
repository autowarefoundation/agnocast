#pragma once

#include "agnocast/agnocast_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

#include "agnocast_cie_config_msgs/msg/callback_group_info.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace agnocast
{

std::string create_callback_group_id(
  const rclcpp::CallbackGroup::SharedPtr & group,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node,
  const std::vector<std::string> & agnocast_topics);

template <typename NodeT>
std::string create_callback_group_id(
  const rclcpp::CallbackGroup::SharedPtr & group, const std::shared_ptr<NodeT> & node,
  const std::vector<std::string> & agnocast_topics)
{
  return create_callback_group_id(group, node->get_node_base_interface(), agnocast_topics);
}

// Marks `group` as a bridge group, so that its callback group ID ends with "@BridgeR2A" or
// "@BridgeA2R". Without the marker, the R2A and A2R groups of one topic get the same ID.
void set_bridge_direction(const rclcpp::CallbackGroup::SharedPtr & group, bool is_r2a);

// Groups given the same key are run by one child executor (one thread) of a
// CallbackIsolatedAgnocastExecutor, which announces the key as that thread's callback group ID.
// The groups may belong to different nodes. With `agnocast` the executor is an Agnocast one, for
// groups with Agnocast subscriptions; without, a plain rclcpp one.
struct SharedExecutorKey
{
  std::string key;
  bool agnocast = false;
};
void set_shared_executor_key(
  const rclcpp::CallbackGroup::SharedPtr & group, const std::string & key, bool agnocast = false);
std::optional<SharedExecutorKey> get_shared_executor_key(
  const rclcpp::CallbackGroup::SharedPtr & group);

// Caution: Do not call in parallel
// Caution: Must be called after rclcpp::init() called
rclcpp::Publisher<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr
create_rclcpp_client_publisher();

// Caution: Do not call in parallel
// Caution: Requires agnocast kernel module
agnocast::Publisher<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr
create_agnocast_client_publisher();

// `publisher` is assumed to be the return value of create_rclcpp_client_publisher()
void publish_callback_group_info(
  const rclcpp::Publisher<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr & publisher,
  int64_t tid, const std::string & callback_group_id);

// `publisher` is assumed to be the return value of create_agnocast_client_publisher()
void publish_callback_group_info(
  const agnocast::Publisher<agnocast_cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr &
    publisher,
  int64_t tid, const std::string & callback_group_id);

}  // namespace agnocast
