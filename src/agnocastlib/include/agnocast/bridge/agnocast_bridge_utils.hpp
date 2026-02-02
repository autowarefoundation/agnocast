#pragma once

#include "agnocast/agnocast_ioctl.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace agnocast
{

inline constexpr std::string_view SUFFIX_R2A = "_R2A";
inline constexpr std::string_view SUFFIX_A2R = "_A2R";
inline constexpr size_t SUFFIX_LEN = SUFFIX_R2A.length();

enum class BridgeMode : int { Off = 0, Standard = 1, Performance = 2 };

class BridgeBase
{
public:
  virtual ~BridgeBase() = default;
  virtual rclcpp::CallbackGroup::SharedPtr get_callback_group() const = 0;
};

struct SubscriberCountResult
{
  int count;          // -1 on error
  bool bridge_exist;  // true if A2R bridge exists
};

struct PublisherCountResult
{
  int count;          // -1 on error
  bool bridge_exist;  // true if R2A bridge exists
};

BridgeMode get_bridge_mode();
rclcpp::QoS get_subscriber_qos(const std::string & topic_name, topic_local_id_t subscriber_id);
rclcpp::QoS get_publisher_qos(const std::string & topic_name, topic_local_id_t publisher_id);
PublisherCountResult get_agnocast_publisher_count(const std::string & topic_name);
SubscriberCountResult get_agnocast_subscriber_count(const std::string & topic_name);
bool has_external_ros2_publisher(const rclcpp::Node * node, const std::string & topic_name);
bool has_external_ros2_subscriber(const rclcpp::Node * node, const std::string & topic_name);
bool update_ros2_subscriber_num(const rclcpp::Node * node, const std::string & topic_name);

}  // namespace agnocast
