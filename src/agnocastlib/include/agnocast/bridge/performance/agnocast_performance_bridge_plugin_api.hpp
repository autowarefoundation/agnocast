#pragma once

#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"
#include "rclcpp/rclcpp.hpp"

struct PerformancePubsubBridgeResult
{
  std::shared_ptr<void> entity_handle;
  rclcpp::CallbackGroup::SharedPtr callback_group;
};

struct PerformanceServiceBridgeResult
{
  std::shared_ptr<void> entity_handle;
  rclcpp::CallbackGroup::SharedPtr ros_srv_cb_group;
  rclcpp::CallbackGroup::SharedPtr agno_client_cb_group;
};

extern "C" PerformancePubsubBridgeResult create_r2a_pubsub_bridge(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const rclcpp::QoS & qos);

extern "C" PerformancePubsubBridgeResult create_a2r_pubsub_bridge(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const rclcpp::QoS & qos);

extern "C" PerformanceServiceBridgeResult create_r2a_service_bridge(
  rclcpp::Node::SharedPtr node, const std::string & service_name, const rclcpp::QoS & qos);

using R2APubsubBridgeFactory = decltype(&create_r2a_pubsub_bridge);
using A2RPubsubBridgeFactory = decltype(&create_a2r_pubsub_bridge);

using R2AServiceBridgeFactory = decltype(&create_r2a_service_bridge);
