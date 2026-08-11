#include "agnocast/agnocast_utils.hpp"

#include "agnocast/node/agnocast_node.hpp"

#include <sys/stat.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <system_error>

namespace agnocast
{
rclcpp::Logger logger = rclcpp::get_logger("Agnocast");
bool is_bridge_process = false;

void validate_ld_preload()
{
  if (is_bridge_process) {
    // The bridge process is spawned with an empty LD_PRELOAD to avoid loading the heaphook library
    // in its descendant processes.
    return;
  }

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

uint32_t get_ros_domain_id()
{
  const char * domain_id_env = getenv("ROS_DOMAIN_ID");
  if (domain_id_env == nullptr || *domain_id_env == '\0') {
    return 0;
  }
  char * end = nullptr;
  errno = 0;
  const uint64_t value = std::strtoul(domain_id_env, &end, 10);
  // Out-of-range values would silently wrap into an unintended domain (e.g. 0),
  // breaking isolation, so reject them rather than truncate.
  if (*end != '\0' || errno != 0 || value > std::numeric_limits<uint32_t>::max()) {
    return 0;
  }
  return static_cast<uint32_t>(value);
}

// UDS-address suffix that scopes the per-IPC-namespace bridge listener by domain.
// The kmod keys the bridge manager on the *parsed* domain, so this must use
// get_ros_domain_id() and not the raw env string. Domain 0 takes no suffix,
// matching the Python discovery agent (bridge_decider._bridge_uds_addr).
static std::string bridge_domain_suffix()
{
  const uint32_t domain_id = get_ros_domain_id();
  if (domain_id == 0) {
    return "";
  }
  return "_d" + std::to_string(domain_id);
}

std::string create_uds_addr_for_bridge()
{
  // Abstract-namespace UDS address is prefixed with '\0' and its length is
  // scoped by the socklen_t passed to bind()/sendto() (no trailing NUL).
  std::string addr;
  addr.push_back('\0');
  addr += "agnocast_bridge_manager_";
  addr += std::to_string(get_self_ipc_ns_inode());
  addr += bridge_domain_suffix();
  return addr;
}

uint64_t get_self_ipc_ns_inode()
{
  struct stat st
  {
  };
  if (stat("/proc/self/ns/ipc", &st) != 0) {
    throw std::system_error(errno, std::generic_category(), "stat(/proc/self/ns/ipc)");
  }
  return static_cast<uint64_t>(st.st_ino);
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

const void * get_node_base_address(agnocast::Node * node)
{
  return static_cast<const void *>(node->get_node_base_interface().get());
}

const void * get_node_base_address(rclcpp::Node * node)
{
  return static_cast<const void *>(
    node->get_node_base_interface()->get_shared_rcl_node_handle().get());
}

}  // namespace agnocast
