#pragma once

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_mq.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <utility>

namespace agnocast
{

inline constexpr std::string_view SUFFIX_PUBSUB_R2A = "_P_R2A";
inline constexpr std::string_view SUFFIX_PUBSUB_A2R = "_P_A2R";
inline constexpr std::string_view SUFFIX_SERVICE_R2A = "_S_R2A";
inline constexpr std::string_view SUFFIX_SERVICE_A2R = "_S_A2R";

inline constexpr size_t SUFFIX_LEN = 6;
static_assert(SUFFIX_PUBSUB_R2A.length() == SUFFIX_LEN);
static_assert(SUFFIX_PUBSUB_A2R.length() == SUFFIX_LEN);
static_assert(SUFFIX_SERVICE_R2A.length() == SUFFIX_LEN);
static_assert(SUFFIX_SERVICE_A2R.length() == SUFFIX_LEN);

enum class BridgeMode : int { Off = 0, Standard = 1, Performance = 2 };

class PubsubBridgeBase
{
public:
  virtual ~PubsubBridgeBase() = default;
  virtual rclcpp::CallbackGroup::SharedPtr get_callback_group() const = 0;
};

class ServiceBridgeBase
{
public:
  virtual ~ServiceBridgeBase() = default;
  virtual std::pair<rclcpp::CallbackGroup::SharedPtr, rclcpp::CallbackGroup::SharedPtr>
  get_callback_groups() const = 0;
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
bool update_ros2_subscriber_num(const rclcpp::Node * node, const std::string & topic_name);
bool update_ros2_publisher_num(const rclcpp::Node * node, const std::string & topic_name);
bool has_external_ros2_publisher(const rclcpp::Node * node, const std::string & topic_name);
bool has_external_ros2_subscriber(const rclcpp::Node * node, const std::string & topic_name);
rclcpp::QoS get_service_qos(const std::string & service_name);
bool is_agnocast_service_alive(const std::string & service_name, std::string & reason);

/// @brief Build `BridgeFactoryInfo` for standard bridge requests.
/// @return true when the bridge factory is successfully built, false otherwise.
bool build_bridge_factory_info(
  BridgeFactoryInfo & factory, uintptr_t fn_current, uintptr_t fn_reverse,
  const rclcpp::Logger & logger);

}  // namespace agnocast
