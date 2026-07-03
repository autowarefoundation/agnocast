#pragma once

#include "agnocast/agnocast_client.hpp"
#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"

#include <fcntl.h>
#include <mqueue.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

namespace agnocast

{

static constexpr size_t DEFAULT_QOS_DEPTH = 10;

inline void send_performance_pubsub_bridge_registration_by_type_name(
  const std::string & topic_name, topic_local_id_t id, const std::string & message_type_name,
  BridgeDirection direction);
template <typename ServiceT>
void send_performance_service_bridge_registration(
  const std::string & service_name, BridgeDirection direction,
  const std::optional<std::pair<std::string, std::string>> & shadow_node_identity);

inline void register_pubsub_bridge_by_type_name(
  const std::string & topic_name, topic_local_id_t id, const std::string & message_type,
  BridgeDirection direction)
{
  auto bridge_mode = get_bridge_mode();
  if (bridge_mode == BridgeMode::On) {
    send_performance_pubsub_bridge_registration_by_type_name(
      topic_name, id, message_type, direction);
  }
}

template <typename ServiceT>
void register_service_bridge_core(
  const std::string & service_name, BridgeDirection direction,
  const std::optional<std::pair<std::string, std::string>> & shadow_node_identity)
{
  auto bridge_mode = get_bridge_mode();
  if (bridge_mode == BridgeMode::On) {
    send_performance_service_bridge_registration<ServiceT>(
      service_name, direction, shadow_node_identity);
  }
}

// Policy for agnocast::Service.
// Registers a bridge that forwards requests from ROS 2 to Agnocast (R2A).
struct RosToAgnocastServiceRegistrationPolicy
{
  template <typename NodeT, typename ServiceT>
  static void register_bridge(NodeT * node, const std::string & service_name)
  {
    std::optional<std::pair<std::string, std::string>> shadow_node_identity{std::nullopt};
    if constexpr (std::is_same_v<std::remove_cv_t<NodeT>, agnocast::Node>) {
      shadow_node_identity =
        std::make_pair(std::string(node->get_namespace()), std::string(node->get_name()));
    }
    register_service_bridge_core<ServiceT>(
      service_name, BridgeDirection::ROS2_TO_AGNOCAST, shadow_node_identity);
  }
};

inline void send_mq_message(
  const std::string & mq_name, const BridgeMsg & msg, size_t send_size,
  const rclcpp::Logger & logger)
{
  struct mq_attr attr = {};
  attr.mq_maxmsg = BRIDGE_MQ_MAX_MESSAGES;
  attr.mq_msgsize = BRIDGE_MQ_MESSAGE_SIZE;

  mqd_t mq =
    mq_open(mq_name.c_str(), O_CREAT | O_WRONLY | O_NONBLOCK | O_CLOEXEC, BRIDGE_MQ_PERMS, &attr);

  if (mq == (mqd_t)-1) {
    RCLCPP_ERROR(
      logger, "mq_open failed for name '%s': %s (errno: %d)", mq_name.c_str(), strerror(errno),
      errno);
    return;
  }

  constexpr int BRIDGE_MQ_SEND_MAX_RETRIES = 100;
  constexpr useconds_t BRIDGE_MQ_SEND_RETRY_INTERVAL_US = 100000;  // 100ms

  int send_result = -1;
  int last_errno = 0;
  for (int retry = 0; retry <= BRIDGE_MQ_SEND_MAX_RETRIES; ++retry) {
    send_result = mq_send(mq, reinterpret_cast<const char *>(&msg), send_size, 0);
    if (send_result == 0) break;
    last_errno = errno;
    if (last_errno != EAGAIN) break;
    if (retry < BRIDGE_MQ_SEND_MAX_RETRIES) {
      usleep(BRIDGE_MQ_SEND_RETRY_INTERVAL_US);
    }
  }
  if (send_result < 0) {
    RCLCPP_ERROR(
      logger, "mq_send failed for name '%s': %s (errno: %d)", mq_name.c_str(), strerror(last_errno),
      last_errno);
  }

  mq_close(mq);
}

inline void send_performance_pubsub_bridge_registration_by_type_name(
  const std::string & topic_name, topic_local_id_t id, const std::string & message_type_name,
  BridgeDirection direction)
{
  static const auto logger = rclcpp::get_logger("agnocast_performance_bridge_registrar");

  auto [msg, reason] = BridgeRegistrationMsgBuilder()
                         .set_direction(direction)
                         .set_is_service(false)
                         .set_message_type(message_type_name.c_str())
                         .set_topic_name(topic_name.c_str())
                         .set_pubsub_target_id(id)
                         .build_message();
  if (!reason.empty()) {
    RCLCPP_ERROR(
      logger, "Failed to build performance pubsub bridge registration: %s", reason.c_str());
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  std::string mq_name = create_mq_name_for_bridge(PERFORMANCE_BRIDGE_VIRTUAL_PID);
  send_mq_message(mq_name, msg, bridge_msg_wire_size<BridgeMsgPubSubPayload>(), logger);
}

template <typename ServiceT>
void send_performance_service_bridge_registration(
  const std::string & service_name, BridgeDirection direction,
  const std::optional<std::pair<std::string, std::string>> & shadow_node_identity)
{
  static const auto logger = rclcpp::get_logger("agnocast_performance_service_bridge_registrar");

  const std::string service_type_name = rosidl_generator_traits::name<ServiceT>();

  auto [msg, reason] = BridgeRegistrationMsgBuilder()
                         .set_direction(direction)
                         .set_is_service(true)
                         .set_service_type(service_type_name.c_str())
                         .set_service_name(service_name.c_str())
                         .set_shadow_node_identity(shadow_node_identity)
                         .build_message();
  if (!reason.empty()) {
    RCLCPP_ERROR(
      logger, "Failed to build performance service bridge registration: %s", reason.c_str());
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  std::string mq_name = create_mq_name_for_bridge(PERFORMANCE_BRIDGE_VIRTUAL_PID);
  send_mq_message(mq_name, msg, bridge_msg_wire_size<BridgeMsgServicePayload>(), logger);
}

}  // namespace agnocast
