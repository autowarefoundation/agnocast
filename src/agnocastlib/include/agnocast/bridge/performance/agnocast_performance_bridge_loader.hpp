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

  PerformanceBridgeResult create_r2a_bridge(
    rclcpp::Node::SharedPtr node, const std::string & topic_name, const std::string & message_type,
    const rclcpp::QoS & qos);

  PerformanceBridgeResult create_a2r_bridge(
    rclcpp::Node::SharedPtr node, const std::string & topic_name, const std::string & message_type,
    const rclcpp::QoS & qos);

private:
  rclcpp::Logger logger_;

  // path -> handle
  std::unordered_map<std::string, void *> loaded_libraries_;

  static std::string convert_type_to_snake_case(const std::string & message_type);
  static std::vector<std::string> generate_library_paths(
    const std::string & snake_type, const std::string & plugin_suffix);
  void * load_library_from_paths(const std::vector<std::string> & paths);
  void * get_bridge_factory_symbol(
    const std::string & message_type, const std::string & direction,
    const std::string & symbol_name);
};

}  // namespace agnocast
