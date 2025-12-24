#include "agnocast/agnocast_utils.hpp"

#include <cstring>

namespace agnocast
{

extern int agnocast_fd;

rclcpp::Logger logger = rclcpp::get_logger("Agnocast");

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

void validate_ld_preload()
{
  const char * ld_preload_cstr = getenv("LD_PRELOAD");
  if (
    ld_preload_cstr == nullptr ||
    std::strstr(ld_preload_cstr, "libagnocast_heaphook.so") == nullptr) {
    RCLCPP_ERROR(logger, "libagnocast_heaphook.so not found in LD_PRELOAD.");
    exit(EXIT_FAILURE);
  }

  std::string ld_preload(ld_preload_cstr);
  std::vector<std::string> paths;
  std::string::size_type start = 0;
  std::string::size_type end = 0;

  while ((end = ld_preload.find(':', start)) != std::string::npos) {
    paths.push_back(ld_preload.substr(start, end - start));
    start = end + 1;
  }
  paths.push_back(ld_preload.substr(start));

  if (paths.size() == 1) {
    RCLCPP_WARN(
      logger,
      "Pre-existing shared libraries in LD_PRELOAD may have been overwritten by "
      "libagnocast_heaphook.so");
  }
}

static std::string create_mq_name(
  const std::string & header, const std::string & topic_name, const topic_local_id_t id)
{
  if (topic_name.length() == 0 || topic_name[0] != '/') {
    RCLCPP_ERROR(logger, "create_mq_name failed");
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  std::string mq_name = topic_name;
  mq_name[0] = '@';
  mq_name = header + mq_name + "@" + std::to_string(id);

  // As a mq_name, '/' cannot be used
  for (size_t i = 1; i < mq_name.size(); i++) {
    if (mq_name[i] == '/') {
      mq_name[i] = '_';
    }
  }

  return mq_name;
}

std::string create_mq_name_for_agnocast_publish(
  const std::string & topic_name, const topic_local_id_t id)
{
  return create_mq_name("/agnocast", topic_name, id);
}

std::string create_mq_name_for_bridge_parent(const pid_t pid)
{
  return "/agnocast_bridge_manager_parent@" + std::to_string(pid);
}

std::string create_mq_name_for_bridge_daemon(const pid_t pid)
{
  return "/agnocast_bridge_manager_daemon@" + std::to_string(pid);
}

std::string create_shm_name(const pid_t pid)
{
  return "/agnocast@" + std::to_string(pid);
}

std::string create_service_request_topic_name(const std::string & service_name)
{
  return "/AGNOCAST_SRV_REQUEST" + service_name;
}

std::string create_service_response_topic_name(
  const std::string & service_name, const std::string & client_node_name)
{
  return "/AGNOCAST_SRV_RESPONSE" + service_name + "_SEP_" + client_node_name;
}

uint64_t agnocast_get_timestamp()
{
  auto now = std::chrono::system_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

}  // namespace agnocast
