#pragma once

#include "agnocast/bridge/performance/agnocast_performance_bridge_plugin_api.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"

#include <functional>
#include <memory>
#include <string>

PerformancePubsubBridgeResult create_r2a_generic_bridge(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const rclcpp::QoS & sub_qos,
  const std::string & type_name,
  std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback);
