#pragma once

#include "agnocast/agnocast_ioctl.hpp"
#include "rclcpp/rclcpp.hpp"

#include <string>

namespace agnocast
{

inline constexpr std::string_view SUFFIX_R2A = "_R2A";
inline constexpr std::string_view SUFFIX_A2R = "_A2R";
inline constexpr size_t SUFFIX_LEN = SUFFIX_R2A.length();

extern rclcpp::Logger logger;

enum class BridgeMode : int { Off = 0, Standard = 1, Performance = 2 };

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
