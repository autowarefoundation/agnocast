#pragma once

#include "agnocast/agnocast_ioctl.hpp"
#include "rclcpp/rclcpp.hpp"

#include <string>

namespace agnocast
{

inline constexpr std::string_view SUFFIX_R2A = "_R2A";
inline constexpr std::string_view SUFFIX_A2R = "_A2R";
inline constexpr size_t SUFFIX_LEN = SUFFIX_R2A.length();

inline constexpr pid_t PERFORMANCE_BRIDGE_VIRTUAL_PID = -1;

extern rclcpp::Logger logger;
extern int agnocast_fd;

enum class BridgeMode : int { Off = 0, Standard = 1, Performance = 2 };

inline void validate_qos(const rclcpp::QoS & qos)
{
  if (qos.history() == rclcpp::HistoryPolicy::KeepAll) {
    RCLCPP_ERROR(logger, "Agnocast does not support KeepAll history policy. Use KeepLast instead.");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  const auto & rmw_qos = qos.get_rmw_qos_profile();

  if (rmw_qos.deadline.sec != 0 || rmw_qos.deadline.nsec != 0) {
    RCLCPP_WARN(logger, "Agnocast does not support deadline QoS policy. It will be ignored.");
  }

  if (rmw_qos.lifespan.sec != 0 || rmw_qos.lifespan.nsec != 0) {
    RCLCPP_WARN(logger, "Agnocast does not support lifespan QoS policy. It will be ignored.");
  }

  if (rmw_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC) {
    RCLCPP_WARN(
      logger, "Agnocast does not support liveliness QoS policy. ManualByTopic will be ignored.");
  }

  if (rmw_qos.liveliness_lease_duration.sec != 0 || rmw_qos.liveliness_lease_duration.nsec != 0) {
    RCLCPP_WARN(
      logger,
      "Agnocast does not support liveliness_lease_duration QoS policy. It will be ignored.");
  }
}

BridgeMode get_bridge_mode();

void validate_ld_preload();
std::string create_mq_name_for_agnocast_publish(
  const std::string & topic_name, const topic_local_id_t id);
std::string create_mq_name_for_bridge(const pid_t pid);
std::string create_shm_name(const pid_t pid);
std::string create_service_request_topic_name(const std::string & service_name);
std::string create_service_response_topic_name(
  const std::string & service_name, const std::string & client_node_name);
uint64_t agnocast_get_timestamp();

}  // namespace agnocast
