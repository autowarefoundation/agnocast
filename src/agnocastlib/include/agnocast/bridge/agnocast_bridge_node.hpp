#pragma once

#include "agnocast/agnocast_client.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/bridge/agnocast_bridge_msg.hpp"
#include "agnocast/bridge/agnocast_bridge_uds.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"
#include "agnocast/cuda_message_tag.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"

#include <fcntl.h>
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
inline void send_performance_service_bridge_registration_by_type_name(
  const std::string & service_type_name, const std::string & service_name,
  BridgeDirection direction,
  const std::optional<std::pair<std::string, std::string>> & shadow_node_identity);

inline void register_pubsub_bridge_by_type_name(
  const std::string & topic_name, topic_local_id_t id, const std::string & message_type,
  BridgeDirection direction)
{
<<<<<<< HEAD
  // CUDA message types cannot be bridged to ROS 2 directly (GPU pointers are not serializable).
  // Bridge support for CUDA types (via cudaMemcpy D2H) is future work.
  if constexpr (is_cuda_message_v<MessageT>) {
    static const auto logger = rclcpp::get_logger("agnocast_bridge_requester");
    RCLCPP_WARN(
      logger,
      "Bridge skipped for CUDA topic '%s': GPU message types cannot be bridged to ROS 2. "
      "Use cudaMemcpy to a standard ROS message if DDS bridging is needed.",
      topic_name.c_str());
    (void)id;
    (void)direction;
    return;
  } else {
    auto bridge_mode = get_bridge_mode();
    if (bridge_mode == BridgeMode::Standard) {
      send_bridge_request<MessageT>(topic_name, id, direction);
    } else if (bridge_mode == BridgeMode::Performance) {
      send_performance_bridge_request<MessageT>(topic_name, id, direction);
    }
=======
  auto bridge_mode = get_bridge_mode();
  if (bridge_mode == BridgeMode::On) {
    send_performance_pubsub_bridge_registration_by_type_name(
      topic_name, id, message_type, direction);
>>>>>>> origin/main
  }
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

  const std::string uds_addr = create_uds_addr_for_bridge();
  send_bridge_uds_message(uds_addr, &msg, bridge_msg_wire_size<BridgeMsgPubSubPayload>(), logger);
}

inline void send_performance_service_bridge_registration_by_type_name(
  const std::string & service_type_name, const std::string & service_name,
  BridgeDirection direction,
  const std::optional<std::pair<std::string, std::string>> & shadow_node_identity)
{
  static const auto logger = rclcpp::get_logger("agnocast_performance_service_bridge_registrar");

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

  const std::string uds_addr = create_uds_addr_for_bridge();
  send_bridge_uds_message(uds_addr, &msg, bridge_msg_wire_size<BridgeMsgServicePayload>(), logger);
}

}  // namespace agnocast
