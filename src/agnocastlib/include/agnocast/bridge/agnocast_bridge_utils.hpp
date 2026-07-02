#pragma once

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_mq.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace agnocast
{

enum class BridgeMode { Off, On };

// How long a daemon cross-NS bridge request keeps a bridge forced after the last
// request. Must exceed the daemon's publish interval (1 s) so a few missed ticks
// don't tear down a healthy cross-NS bridge, yet stay short so a vanished remote
// endpoint stops being forced promptly.
inline constexpr std::chrono::seconds DAEMON_FORCE_TTL{5};

// A daemon-forced lease registered at `now` lasts until `now + DAEMON_FORCE_TTL`,
// and is active while the current time is strictly before that deadline — i.e. the
// half-open window [registered, registered + DAEMON_FORCE_TTL). Factored out of the
// bridge manager so the lease/TTL boundary is unit-testable with an injected clock.
inline std::chrono::steady_clock::time_point daemon_force_deadline(
  const std::chrono::steady_clock::time_point now)
{
  return now + DAEMON_FORCE_TTL;
}
inline bool is_daemon_force_active(
  const std::chrono::steady_clock::time_point deadline,
  const std::chrono::steady_clock::time_point now)
{
  return now < deadline;
}

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
// Rebuild a QoS profile from the fields a daemon cross-NS request carries. The
// daemon has no local endpoint to query, so reliability / durability / depth are
// reconstructed from the request rather than from the kernel.
rclcpp::QoS daemon_request_qos(const BridgeMsgDaemonPubSubPayload & req);
PublisherCountResult get_agnocast_publisher_count(const std::string & topic_name);
SubscriberCountResult get_agnocast_subscriber_count(const std::string & topic_name);
bool update_ros2_subscriber_num(const rclcpp::Node * node, const std::string & topic_name);
bool update_ros2_publisher_num(const rclcpp::Node * node, const std::string & topic_name);
bool has_external_ros2_publisher(const rclcpp::Node * node, const std::string & topic_name);
bool has_external_ros2_subscriber(const rclcpp::Node * node, const std::string & topic_name);
rclcpp::QoS get_service_qos(const std::string & service_name);
bool is_agnocast_service_alive(const std::string & service_name, std::string & reason);

/// @brief A builder class for creating bridge registration messages.
///
/// It detects errors such as string truncation when copying string fields, and
/// `build_message()` returns the error reason. However, it does not check whether
/// the computed message as a whole is valid. It's the caller's responsibility to invoke a correct
/// set of setters to build a valid message.
class BridgeRegistrationMsgBuilder
{
  BridgeMsgPubSubPayload pubsub_{};
  BridgeMsgServicePayload service_{};
  BridgeDirection direction_{BridgeDirection::ROS2_TO_AGNOCAST};
  bool is_service_{false};
  bool failed_;
  std::string reason_;

  BridgeRegistrationMsgBuilder & fail(const char * format, ...);
  int checked_snprintf(
    const std::string & member, char * buffer, size_t size, const char * format, ...);

public:
  BridgeRegistrationMsgBuilder();

  BridgeRegistrationMsgBuilder & set_direction(BridgeDirection direction);
  BridgeRegistrationMsgBuilder & set_is_service(bool is_service);
  BridgeRegistrationMsgBuilder & set_message_type(const char * message_type);
  BridgeRegistrationMsgBuilder & set_topic_name(const char * topic_name);
  BridgeRegistrationMsgBuilder & set_pubsub_target_id(topic_local_id_t target_id);
  BridgeRegistrationMsgBuilder & set_service_type(const char * service_type);
  BridgeRegistrationMsgBuilder & set_service_name(const char * service_name);
  BridgeRegistrationMsgBuilder & set_shadow_node_identity(
    const std::optional<std::pair<std::string, std::string>> & shadow_node_identity);

  std::pair<BridgeMsg, std::string> build_message();
};

template <typename MapT>
std::shared_ptr<rcl_node_t> find_or_create_shadow_node(
  const MapT & active_r2a_service_bridges, const std::string & ns, const std::string & name)
{
  for (const auto & [_, item] : active_r2a_service_bridges) {
    const std::shared_ptr<rcl_node_t> & shadow_node = item.shadow_node;
    if (
      shadow_node != nullptr && strcmp(rcl_node_get_name(shadow_node.get()), name.c_str()) == 0 &&
      strcmp(rcl_node_get_namespace(shadow_node.get()), ns.c_str()) == 0) {
      return shadow_node;
    }
  }

  rcl_context_t * rcl_ctx = rclcpp::contexts::get_global_default_context()->get_rcl_context().get();

  rcl_node_options_t options = rcl_node_get_default_options();
  options.enable_rosout = false;
  options.use_global_arguments = false;
  if (rcl_parse_arguments(0, nullptr, options.allocator, &(options.arguments)) != RCL_RET_OK) {
    rcl_reset_error();
    throw std::runtime_error("Failed to parse arguments while creating shadow node");
  }

  auto del = [](rcl_node_t * node) {
    if (rcl_node_is_valid(node)) {
      if (rcl_node_fini(node) != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "agnocast_bridge", "Error in destruction of shadow node: %s", rcl_get_error_string().str);
        rcl_reset_error();
      }
    }
    delete node;
  };
  auto node = std::shared_ptr<rcl_node_t>(new rcl_node_t{}, del);

  if (rcl_node_init(node.get(), name.c_str(), ns.c_str(), rcl_ctx, &options) != RCL_RET_OK) {
    rcl_reset_error();
    throw std::runtime_error("Failed to initialize shadow node");
  }

  return node;
}

}  // namespace agnocast
