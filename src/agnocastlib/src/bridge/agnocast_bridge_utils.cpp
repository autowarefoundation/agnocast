#include "agnocast/bridge/agnocast_bridge_utils.hpp"

#include "agnocast/agnocast.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

namespace agnocast
{

BridgeMode get_bridge_mode()
{
  const char * env_val = std::getenv("AGNOCAST_BRIDGE_MODE");
  if (env_val == nullptr) {
    return BridgeMode::On;
  }

  std::string val = env_val;
  std::transform(val.begin(), val.end(), val.begin(), ::tolower);

  if (val == "0" || val == "off") {
    return BridgeMode::Off;
  }
  if (val == "on") {
    return BridgeMode::On;
  }
  if (val == "1" || val == "standard") {
    RCLCPP_WARN_ONCE(logger, "AGNOCAST_BRIDGE_MODE=%s is deprecated. Fallback to ON.", env_val);
    return BridgeMode::On;
  }
  if (val == "2" || val == "performance") {
    RCLCPP_WARN_ONCE(logger, "AGNOCAST_BRIDGE_MODE=%s is deprecated. Fallback to ON.", env_val);
    return BridgeMode::On;
  }

  RCLCPP_WARN_ONCE(logger, "Unknown AGNOCAST_BRIDGE_MODE: %s. Fallback to ON.", env_val);
  return BridgeMode::On;
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

rclcpp::QoS daemon_request_qos(const MqMsgDaemonBridge & req)
{
  return rclcpp::QoS(req.qos_depth)
    .durability(
      req.qos_is_transient_local ? rclcpp::DurabilityPolicy::TransientLocal
                                 : rclcpp::DurabilityPolicy::Volatile)
    .reliability(
      req.qos_is_reliable ? rclcpp::ReliabilityPolicy::Reliable
                          : rclcpp::ReliabilityPolicy::BestEffort);
}

SubscriberCountResult get_agnocast_subscriber_count(const std::string & topic_name)
{
  union ioctl_get_subscriber_num_args args = {};
  args.topic_name = {topic_name.c_str(), topic_name.size()};
  if (ioctl(agnocast_fd, AGNOCAST_GET_SUBSCRIBER_NUM_CMD, &args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_SUBSCRIBER_NUM_CMD failed: %s", strerror(errno));
    return {-1, false};
  }

  int total_subs =
    static_cast<int>(args.ret_other_process_subscriber_num + args.ret_same_process_subscriber_num);
  if (args.ret_a2r_bridge_exist && total_subs > 0) {
    total_subs--;
  }

  return {total_subs, args.ret_a2r_bridge_exist};
}

PublisherCountResult get_agnocast_publisher_count(const std::string & topic_name)
{
  union ioctl_get_publisher_num_args args = {};
  args.topic_name = {topic_name.c_str(), topic_name.size()};
  if (ioctl(agnocast_fd, AGNOCAST_GET_PUBLISHER_NUM_CMD, &args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_PUBLISHER_NUM_CMD failed: %s", strerror(errno));
    return {-1, false};
  }

  int total_pubs = static_cast<int>(args.ret_publisher_num);
  if (args.ret_r2a_bridge_exist && total_pubs > 0) {
    total_pubs--;
  }

  return {total_pubs, args.ret_r2a_bridge_exist};
}

bool update_ros2_subscriber_num(const rclcpp::Node * node, const std::string & topic_name)
{
  if (node == nullptr) {
    return false;
  }

  size_t ros2_count = node->count_subscribers(topic_name);

  struct ioctl_set_ros2_subscriber_num_args args = {};
  args.topic_name = {topic_name.c_str(), topic_name.size()};
  args.ros2_subscriber_num = static_cast<uint32_t>(ros2_count);

  if (ioctl(agnocast_fd, AGNOCAST_SET_ROS2_SUBSCRIBER_NUM_CMD, &args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_SET_ROS2_SUBSCRIBER_NUM_CMD failed: %s", strerror(errno));
    return false;
  }
  return true;
}

bool update_ros2_publisher_num(const rclcpp::Node * node, const std::string & topic_name)
{
  if (node == nullptr) {
    return false;
  }

  size_t ros2_count = node->count_publishers(topic_name);

  struct ioctl_set_ros2_publisher_num_args args = {};
  args.topic_name = {topic_name.c_str(), topic_name.size()};
  args.ros2_publisher_num = static_cast<uint32_t>(ros2_count);

  if (ioctl(agnocast_fd, AGNOCAST_SET_ROS2_PUBLISHER_NUM_CMD, &args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_SET_ROS2_PUBLISHER_NUM_CMD failed: %s", strerror(errno));
    return false;
  }
  return true;
}

bool has_external_ros2_publisher(const rclcpp::Node * node, const std::string & topic_name)
{
  if (node == nullptr) {
    return false;
  }

  const std::string self_name = node->get_name();
  const std::string self_ns = node->get_namespace();
  const auto publishers = node->get_publishers_info_by_topic(topic_name);

  return std::any_of(
    publishers.begin(), publishers.end(), [&self_name, &self_ns](const auto & info) {
      return info.node_name() != self_name || info.node_namespace() != self_ns;
    });
}

bool has_external_ros2_subscriber(const rclcpp::Node * node, const std::string & topic_name)
{
  if (node == nullptr) {
    return false;
  }

  const std::string self_name = node->get_name();
  const std::string self_ns = node->get_namespace();
  const auto subscribers = node->get_subscriptions_info_by_topic(topic_name);

  return std::any_of(
    subscribers.begin(), subscribers.end(), [&self_name, &self_ns](const auto & info) {
      return info.node_name() != self_name || info.node_namespace() != self_ns;
    });
}

rclcpp::QoS get_service_qos(const std::string & service_name)
{
  const std::string request_topic_name = create_service_request_topic_name(service_name);

  auto topic_info_buffer = std::make_unique<std::array<topic_info_ret, 1>>();
  ioctl_topic_info_args topic_info_args = {};
  topic_info_args.topic_name = {request_topic_name.c_str(), request_topic_name.size()};
  topic_info_args.topic_info_ret_buffer_addr =
    reinterpret_cast<uint64_t>(topic_info_buffer->data());
  topic_info_args.topic_info_ret_buffer_size = 1;

  if (ioctl(agnocast_fd, AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD, &topic_info_args) < 0) {
    if (errno == ENOBUFS) {
      throw std::runtime_error("Multiple target agnocast services found");
    }
    throw std::runtime_error(
      "Failed to fetch target service information from agnocast kernel module");
  }

  if (topic_info_args.ret_topic_info_ret_num <= 0) {
    throw std::runtime_error("No target agnocast service found");
  }

  const topic_info_ret & info = (*topic_info_buffer)[0];

  // We know the durability policy is set to Volatile because this is a service.
  rclcpp::QoS qos = rclcpp::QoS(info.qos_depth)
                      .durability(rclcpp::DurabilityPolicy::Volatile)
                      .reliability(
                        info.qos_is_reliable ? rclcpp::ReliabilityPolicy::Reliable
                                             : rclcpp::ReliabilityPolicy::BestEffort);
  return qos;
}

bool is_agnocast_service_alive(const std::string & service_name, std::string & reason)
{
  // TODO(bdm-k): Add a dedicated service-liveness ioctl so we can validate target service state
  // directly without using get_service_qos() as a probe.
  try {
    (void)get_service_qos(service_name);
    return true;
  } catch (const std::exception & e) {
    reason = e.what();
    return false;
  } catch (...) {
    reason = "Unknown error";
    return false;
  }
}

BridgeRegistrationMsgBuilder::BridgeRegistrationMsgBuilder() : failed_(false)
{
}

// NOLINTBEGIN(cert-dcl50-cpp, cppcoreguidelines-pro-bounds-array-to-pointer-decay,
// hicpp-no-array-decay)
BridgeRegistrationMsgBuilder & BridgeRegistrationMsgBuilder::fail(const char * format, ...)
{
  va_list args;
  va_start(args, format);
  int n = vsnprintf(nullptr, 0, format, args);
  va_end(args);

  if (n < 0) {
    failed_ = true;
    reason_ = "Failed to format error message";
    return *this;
  }

  std::string buf(n + 1, '\0');
  va_start(args, format);
  vsnprintf(buf.data(), n + 1, format, args);
  va_end(args);
  // Drop the trailing null terminator.
  buf.resize(n);

  failed_ = true;
  reason_ = std::move(buf);
  return *this;
}

int BridgeRegistrationMsgBuilder::checked_snprintf(
  const std::string & member, char * buffer, size_t size, const char * format, ...)
{
  if (failed_) return -1;

  va_list args;
  va_start(args, format);
  int n = vsnprintf(buffer, size, format, args);
  va_end(args);

  if (n < 0) {
    fail("snprintf() for '%s' returned a negative value", member.c_str());
  } else if (static_cast<size_t>(n) >= size) {
    fail(
      "snprintf() for '%s' failed; length must be %zu characters or fewer", member.c_str(),
      size - 1);
  }

  return n;
}
// NOLINTEND(cert-dcl50-cpp, cppcoreguidelines-pro-bounds-array-to-pointer-decay,
// hicpp-no-array-decay)

BridgeRegistrationMsgBuilder & BridgeRegistrationMsgBuilder::set_direction(
  BridgeDirection direction)
{
  msg_.direction = direction;
  return *this;
}

BridgeRegistrationMsgBuilder & BridgeRegistrationMsgBuilder::set_is_service(bool is_service)
{
  msg_.is_service = is_service;
  return *this;
}

BridgeRegistrationMsgBuilder & BridgeRegistrationMsgBuilder::set_message_type(
  const char * message_type)
{
  checked_snprintf(
    "message_type", static_cast<char *>(msg_.pubsub_target.message_type), MESSAGE_TYPE_BUFFER_SIZE,
    "%s", message_type);
  return *this;
}

BridgeRegistrationMsgBuilder & BridgeRegistrationMsgBuilder::set_topic_name(const char * topic_name)
{
  checked_snprintf(
    "topic_name", static_cast<char *>(msg_.pubsub_target.topic_name), TOPIC_NAME_BUFFER_SIZE, "%s",
    topic_name);
  return *this;
}

BridgeRegistrationMsgBuilder & BridgeRegistrationMsgBuilder::set_pubsub_target_id(
  topic_local_id_t target_id)
{
  msg_.pubsub_target.target_id = target_id;
  return *this;
}

BridgeRegistrationMsgBuilder & BridgeRegistrationMsgBuilder::set_service_type(
  const char * service_type)
{
  checked_snprintf(
    "service_type", static_cast<char *>(msg_.srv_target.service_type), SERVICE_TYPE_BUFFER_SIZE,
    "%s", service_type);
  return *this;
}

BridgeRegistrationMsgBuilder & BridgeRegistrationMsgBuilder::set_service_name(
  const char * service_name)
{
  checked_snprintf(
    "service_name", static_cast<char *>(msg_.srv_target.service_name), SERVICE_NAME_BUFFER_SIZE,
    "%s", service_name);
  return *this;
}

BridgeRegistrationMsgBuilder & BridgeRegistrationMsgBuilder::set_shadow_node_identity(
  const std::optional<std::pair<std::string, std::string>> & shadow_node_identity)
{
  msg_.srv_target.create_shadow_node = shadow_node_identity.has_value();

  const char * shadow_node_namespace =
    shadow_node_identity.has_value() ? shadow_node_identity->first.c_str() : "";
  const char * shadow_node_name =
    shadow_node_identity.has_value() ? shadow_node_identity->second.c_str() : "";

  checked_snprintf(
    "shadow_node_namespace", static_cast<char *>(msg_.srv_target.shadow_node_namespace),
    NODE_NAME_BUFFER_SIZE, "%s", shadow_node_namespace);
  checked_snprintf(
    "shadow_node_name", static_cast<char *>(msg_.srv_target.shadow_node_name),
    NODE_NAME_BUFFER_SIZE, "%s", shadow_node_name);

  return *this;
}

std::pair<MqMsgPerformanceBridge, std::string>
BridgeRegistrationMsgBuilder::build_performance_message()
{
  return {msg_, failed_ ? std::move(reason_) : std::string{}};
}

}  // namespace agnocast
