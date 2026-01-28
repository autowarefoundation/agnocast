#include "agnocast/bridge/agnocast_bridge_utils.hpp"

#include "agnocast/agnocast.hpp"

#include <rclcpp/rclcpp.hpp>

#include <stdexcept>
#include <string>

namespace agnocast
{

BridgeMode get_bridge_mode()
{
  const char * env_val = std::getenv("AGNOCAST_BRIDGE_MODE");
  if (env_val == nullptr) {
    return BridgeMode::Standard;
  }

  std::string val = env_val;
  std::transform(val.begin(), val.end(), val.begin(), ::tolower);

  if (val == "0" || val == "off") {
    return BridgeMode::Off;
  }
  if (val == "1" || val == "standard") {
    return BridgeMode::Standard;
  }
  if (val == "2" || val == "performance") {
    return BridgeMode::Performance;
  }

  RCLCPP_WARN(logger, "Unknown AGNOCAST_BRIDGE_MODE: %s. Fallback to STANDARD.", env_val);
  return BridgeMode::Standard;
}

rclcpp::QoS get_subscriber_qos(const std::string & topic_name, topic_local_id_t subscriber_id)
{
  struct ioctl_get_subscriber_qos_args get_subscriber_qos_args = {};
  get_subscriber_qos_args.topic_name = {topic_name.c_str(), topic_name.size()};
  get_subscriber_qos_args.subscriber_id = subscriber_id;

  if (ioctl(agnocast_fd, AGNOCAST_GET_SUBSCRIBER_QOS_CMD, &get_subscriber_qos_args) < 0) {
    // This exception is intended to be caught by the factory function that instantiates the bridge.
    throw std::runtime_error("Failed to fetch subscriber QoS from agnocast kernel module");
  }
  return rclcpp::QoS(get_subscriber_qos_args.ret_depth)
    .durability(
      get_subscriber_qos_args.ret_is_transient_local ? rclcpp::DurabilityPolicy::TransientLocal
                                                     : rclcpp::DurabilityPolicy::Volatile)
    .reliability(
      get_subscriber_qos_args.ret_is_reliable ? rclcpp::ReliabilityPolicy::Reliable
                                              : rclcpp::ReliabilityPolicy::BestEffort);
}

rclcpp::QoS get_publisher_qos(const std::string & topic_name, topic_local_id_t publisher_id)
{
  struct ioctl_get_publisher_qos_args get_publisher_qos_args = {};
  get_publisher_qos_args.topic_name = {topic_name.c_str(), topic_name.size()};
  get_publisher_qos_args.publisher_id = publisher_id;

  if (ioctl(agnocast_fd, AGNOCAST_GET_PUBLISHER_QOS_CMD, &get_publisher_qos_args) < 0) {
    // This exception is intended to be caught by the factory function that instantiates the bridge.
    throw std::runtime_error("Failed to fetch publisher QoS from agnocast kernel module");
  }

  return rclcpp::QoS(get_publisher_qos_args.ret_depth)
    .durability(
      get_publisher_qos_args.ret_is_transient_local ? rclcpp::DurabilityPolicy::TransientLocal
                                                    : rclcpp::DurabilityPolicy::Volatile);
}

SubscriberCountResult get_agnocast_subscriber_count(const std::string & topic_name)
{
  union ioctl_get_subscriber_num_args args = {};
  args.topic_name = {topic_name.c_str(), topic_name.size()};
  if (ioctl(agnocast_fd, AGNOCAST_GET_SUBSCRIBER_NUM_CMD, &args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_SUBSCRIBER_NUM_CMD failed: %s", strerror(errno));
    return {-1, false};
  }
  return {static_cast<int>(args.ret_other_process_subscriber_num), args.ret_a2r_bridge_exist};
}

PublisherCountResult get_agnocast_publisher_count(const std::string & topic_name)
{
  union ioctl_get_publisher_num_args args = {};
  args.topic_name = {topic_name.c_str(), topic_name.size()};
  if (ioctl(agnocast_fd, AGNOCAST_GET_PUBLISHER_NUM_CMD, &args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_PUBLISHER_NUM_CMD failed: %s", strerror(errno));
    return {-1, false};
  }
  return {static_cast<int>(args.ret_publisher_num), args.ret_bridge_exist};
}

bool has_external_ros2_publisher(const rclcpp::Node * node, const std::string & topic_name)
{
  if (node == nullptr) {
    return false;
  }

  const std::string self_name = node->get_name();
  const auto publishers = node->get_publishers_info_by_topic(topic_name);

  return std::any_of(publishers.begin(), publishers.end(), [&self_name](const auto & info) {
    return info.node_name() != self_name;
  });
}

bool has_external_ros2_subscriber(const rclcpp::Node * node, const std::string & topic_name)
{
  if (node == nullptr) {
    return false;
  }

  const std::string self_name = node->get_name();
  const auto subscribers = node->get_subscriptions_info_by_topic(topic_name);

  return std::any_of(subscribers.begin(), subscribers.end(), [&self_name](const auto & info) {
    return info.node_name() != self_name;
  });
}

}  // namespace agnocast
