#pragma once

#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/performance/agnocast_performance_bridge_plugin_api.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace agnocast
{

class PerformanceBridgeLoader
{
public:
  explicit PerformanceBridgeLoader(const rclcpp::Logger & logger);
  ~PerformanceBridgeLoader();

  PerformancePubsubBridgeResult create_r2a_pubsub_bridge(
    rclcpp::Node::SharedPtr node, const std::string & topic_name, const std::string & message_type,
    const rclcpp::QoS & qos);

  PerformancePubsubBridgeResult create_a2r_pubsub_bridge(
    rclcpp::Node::SharedPtr node, const std::string & topic_name, const std::string & message_type,
    const rclcpp::QoS & qos);

  PerformanceServiceBridgeResult create_r2a_service_bridge(
    rclcpp::Node::SharedPtr node, const std::string & service_name,
    const std::string & service_type, const rclcpp::QoS & qos);

private:
  rclcpp::Logger logger_;

  // path -> handle
  std::unordered_map<std::string, void *> loaded_libraries_;

  static std::string convert_type_to_snake_case(const std::string & message_type);
  static std::vector<std::string> generate_library_paths(
    const std::string & snake_type, bool is_service);
  void * load_library_from_paths(const std::vector<std::string> & paths);
  void * get_bridge_factory_symbol(
    const std::string & type_name, const std::string & symbol_name, bool is_service);
};

}  // namespace agnocast
