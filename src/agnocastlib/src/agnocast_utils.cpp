#include "agnocast/agnocast_utils.hpp"

#include <cstring>

namespace agnocast
{
rclcpp::Logger logger = rclcpp::get_logger("Agnocast");

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

std::string create_mq_name_for_bridge(const pid_t pid)
{
  return "/agnocast_bridge_manager@" + std::to_string(pid);
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
