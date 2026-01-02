#pragma once

#include "rclcpp/rclcpp.hpp"

#include "cie_config_msgs/msg/callback_group_info.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace cie_thread_configurator
{

std::string create_callback_group_id(
  rclcpp::CallbackGroup::SharedPtr group, rclcpp::Node::SharedPtr node,
  const std::vector<std::string> & agnocast_topics);

std::string create_callback_group_id(
  rclcpp::CallbackGroup::SharedPtr group,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
  const std::vector<std::string> & agnocast_topics);

// Caution: Do not call in parallel
// Caution: Must be called after rclcpp::init() called
rclcpp::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr create_client_publisher();

// `publisher` is assumed to be the return value of create_client_publisher()
void publish_callback_group_info(
  const rclcpp::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr & publisher,
  int64_t tid, const std::string & callback_group_id);

// Get hardware information from lscpu command
std::map<std::string, std::string> get_hardware_info();

}  // namespace cie_thread_configurator
