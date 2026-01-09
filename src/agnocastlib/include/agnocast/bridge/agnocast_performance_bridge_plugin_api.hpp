#pragma once

#include "agnocast/agnocast_subscription.hpp"
#include "rclcpp/rclcpp.hpp"

extern "C" rclcpp::SubscriptionBase::SharedPtr create_r2a_bridge(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const rclcpp::QoS & qos);

extern "C" std::shared_ptr<agnocast::SubscriptionBase> create_a2r_bridge(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const rclcpp::QoS & qos);

using BridgeEntryR2A = decltype(&create_r2a_bridge);
using BridgeEntryA2R = decltype(&create_a2r_bridge);
