#pragma once

#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"
#include "rclcpp/rclcpp.hpp"

struct PerformanceBridgeResult
{
  std::shared_ptr<void> entity_handle;
  rclcpp::CallbackGroup::SharedPtr callback_group;
};

extern "C" PerformanceBridgeResult create_r2a_bridge(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const rclcpp::QoS & qos);

extern "C" PerformanceBridgeResult create_a2r_bridge(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const rclcpp::QoS & qos);

using BridgeEntryR2A = decltype(&create_r2a_bridge);
using BridgeEntryA2R = decltype(&create_a2r_bridge);
