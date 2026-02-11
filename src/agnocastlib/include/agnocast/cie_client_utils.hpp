#pragma once

#include "agnocast/agnocast_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

#include "cie_config_msgs/msg/callback_group_info.hpp"

#include <memory>
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

// Caution: Do not call in parallel
// Caution: Must be called after rclcpp::init() called
rclcpp::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr
create_rclcpp_client_publisher();

// Caution: Do not call in parallel
// Caution: Requires agnocast kernel module
agnocast::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr
create_agnocast_client_publisher();

// `publisher` is assumed to be the return value of create_rclcpp_client_publisher()
void publish_callback_group_info(
  const rclcpp::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr & publisher,
  int64_t tid, const std::string & callback_group_id);

// `publisher` is assumed to be the return value of create_agnocast_client_publisher()
void publish_callback_group_info(
  const agnocast::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr & publisher,
  int64_t tid, const std::string & callback_group_id);

}  // namespace agnocast
