#pragma once

#include "agnocast/agnocast_ioctl.hpp"
#include "rclcpp/rclcpp.hpp"

#include <string>

namespace agnocast
{

class Node;

extern rclcpp::Logger logger;
extern int agnocast_fd;
extern bool is_bridge_process;

namespace detail
{

inline void validate_qos_common(const rclcpp::QoS & qos)
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

  if (qos.depth() == 0) {
    RCLCPP_WARN(logger, "Agnocast does not support QoS depth=0. No messages will be delivered.");
  }

  if (rmw_qos.avoid_ros_namespace_conventions) {
    RCLCPP_WARN(
      logger,
      "Agnocast does not honor avoid_ros_namespace_conventions QoS policy. It will be ignored.");
  }
}

}  // namespace detail

inline void validate_publisher_qos(const rclcpp::QoS & qos)
{
  detail::validate_qos_common(qos);

  if (qos.reliability() == rclcpp::ReliabilityPolicy::BestEffort) {
    RCLCPP_WARN(
      logger,
      "Agnocast publishers do not honor the BestEffort reliability QoS policy. "
      "Messages are delivered through shared memory regardless of this setting.");
  }
}

inline void validate_subscription_qos(const rclcpp::QoS & qos)
{
  detail::validate_qos_common(qos);
}

void validate_ld_preload();
// Return the calling process's ROS_DOMAIN_ID parsed from the env var (0 if unset
// or unparsable), matching ROS 2's default. Registered with the kmod so topics
// in different domains are isolated.
uint32_t get_ros_domain_id();
std::string create_mq_name_for_agnocast_publish(
  const std::string & topic_name, const topic_local_id_t id);
std::string create_mq_name_for_bridge(const pid_t pid);
// Name of the MQ a per-NS daemon uses to ask a bridge_manager to create a
// cross-NS bridge. Standard mode keys on the owning user process's pid;
// performance mode is per-NS (see impl).
std::string create_mq_name_for_daemon_bridge(const pid_t pid);
std::string create_shm_name(const pid_t pid);
// Return the inode number of the calling process's IPC namespace
// (`/proc/self/ns/ipc`). Used by the type registry writer/reader as the
// per-namespace key for the tmpfs directory
// `${AGNOCAST_TMPFS_DIR:-/dev/shm}/agnocast_type_registry/<ipc_ns_inode>/`.
uint64_t get_self_ipc_ns_inode();
std::string create_service_request_topic_name(const std::string & service_name);
std::string create_service_response_topic_name(
  const std::string & service_name, const std::string & client_node_name);
uint64_t agnocast_get_timestamp();

// Create a dummy callback group for agnocast::Node tracepoint use.
// Defined in .cpp to avoid circular inclusion between agnocast_publisher/subscription.hpp and
// agnocast_node.hpp.
const void * get_node_base_address(agnocast::Node * node);

}  // namespace agnocast
