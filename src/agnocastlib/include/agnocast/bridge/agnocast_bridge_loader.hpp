#pragma once

#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_plugin_api.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace agnocast
{

class BridgeLoader
{
public:
  explicit BridgeLoader(const rclcpp::Logger & logger);
  ~BridgeLoader();

  PubsubBridgeResult create_r2a_pubsub_bridge(
    rclcpp::Node::SharedPtr node, const std::string & topic_name, const std::string & message_type,
    const rclcpp::QoS & qos);

  PubsubBridgeResult create_a2r_pubsub_bridge(
    rclcpp::Node::SharedPtr node, const std::string & topic_name, const std::string & message_type,
    const rclcpp::QoS & qos);

  ServiceBridgeEntity create_r2a_service_bridge(
    rclcpp::Node::SharedPtr node, const std::string & service_name,
    const std::string & service_type, const rclcpp::QoS & qos);

  ServiceBridgeEntity create_a2r_service_bridge(
    rclcpp::Node::SharedPtr node, const std::string & service_name,
    const std::string & service_type, const rclcpp::QoS & qos);

private:
  rclcpp::Logger logger_;

  // path -> handle
  std::unordered_map<std::string, void *> loaded_libraries_;

  static PubsubBridgeResult create_r2a_pubsub_bridge_generic(
    const rclcpp::Node::SharedPtr & node, const std::string & topic_name,
    const std::string & message_type, const rclcpp::QoS & qos);

  static PubsubBridgeResult create_a2r_pubsub_bridge_generic(
    const rclcpp::Node::SharedPtr & node, const std::string & topic_name,
    const std::string & message_type, const rclcpp::QoS & qos);

  static ServiceBridgeEntity create_r2a_service_bridge_generic(
    const rclcpp::Node::SharedPtr & node, const std::string & service_name,
    const std::string & service_type, const rclcpp::QoS & qos);

  static ServiceBridgeEntity create_a2r_service_bridge_generic(
    const rclcpp::Node::SharedPtr & node, const std::string & service_name,
    const std::string & service_type, const rclcpp::QoS & qos);

  static std::string convert_type_to_snake_case(const std::string & message_type);
  static std::vector<std::string> generate_library_paths();
  void * load_library_from_paths(const std::vector<std::string> & paths, std::string & last_error);
  void * get_bridge_factory_symbol(
    const std::string & type_name, const std::string & symbol_name_prefix, bool is_service);
};

}  // namespace agnocast
