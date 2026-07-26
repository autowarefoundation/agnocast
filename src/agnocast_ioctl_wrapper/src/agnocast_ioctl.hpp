#pragma once

#include <algorithm>
#include <cstdint>

#define MAX_PUBLISHER_NUM 1024   // Maximum number of publishers per topic
#define MAX_TOPIC_LOCAL_ID 4096  // Bitmap size for per-entry subscriber reference tracking
#define MAX_SUBSCRIBER_NUM \
  (MAX_TOPIC_LOCAL_ID - MAX_PUBLISHER_NUM)  // Maximum number of subscribers per topic

#define MAX_TOPIC_NUM 1024
#define MAX_TOPIC_INFO_RET_NUM std::max(MAX_PUBLISHER_NUM, MAX_SUBSCRIBER_NUM)

#define TOPIC_NAME_BUFFER_SIZE 256
#define NODE_NAME_BUFFER_SIZE 256

constexpr const char * AGNOCAST_DEVICE_NOT_FOUND_MSG =
  "Failed to open /dev/agnocast: Device not found. "
  "Please ensure the agnocast kernel module is installed. "
  "Run 'sudo modprobe agnocast' or 'sudo insmod <path-to-agnocast.ko>' to load the module.\n";

struct name_info
{
  const char * ptr;
  uint64_t len;
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
union ioctl_topic_list_args {
  struct
  {
    uint64_t topic_name_buffer_addr;
    // Parallel array of uint32 domain_ids (one per topic name). Pass 0 to skip.
    // Must mirror agnocast_kmod/agnocast.h so _IOWR encodes the same size.
    uint64_t domain_id_buffer_addr;
    uint32_t topic_name_buffer_size;
  };
  uint32_t ret_topic_num;
};

union ioctl_node_info_args {
  struct
  {
    struct name_info node_name;
    uint64_t topic_name_buffer_addr;
    uint32_t topic_name_buffer_size;
  };
  uint32_t ret_topic_num;
};

struct topic_info_ret
{
  char node_name[NODE_NAME_BUFFER_SIZE];
  uint32_t qos_depth;
  bool qos_is_transient_local;
  bool qos_is_reliable;
  bool is_bridge;
};

union ioctl_topic_info_args {
  struct
  {
    struct name_info topic_name;
    uint64_t topic_info_ret_buffer_addr;
    uint32_t topic_info_ret_buffer_size;
    // Which domain's endpoints to return (0 = default domain). Must mirror
    // agnocast_kmod/agnocast.h so _IOWR encodes the same size.
    uint32_t domain_id;
  };
  uint32_t ret_topic_info_ret_num;
};
#pragma GCC diagnostic pop

#define VERSION_BUFFER_LEN 32

struct ioctl_get_version_args
{
  char ret_version[VERSION_BUFFER_LEN];
};

// Mirrors agnocast_kmod/agnocast.h so _IOW encodes the same size.
struct ioctl_add_domain_bridge_args
{
  // topic_name_from / topic_name_to may differ (rename); equal for a plain bridge.
  struct name_info topic_name_from;
  struct name_info topic_name_to;
  uint32_t from_domain;
  uint32_t to_domain;
};

struct ioctl_discovery_agent_should_exit_args
{
  uint32_t domain_id;
  bool commit;
  bool ret_should_exit;
};

struct ioctl_add_discovery_agent_args
{
  uint32_t domain_id;
  bool ret_already_exists;
};

struct ioctl_discovery_agent_exists_args
{
  uint32_t domain_id;
  bool ret_exists;
};

#define AGNOCAST_GET_VERSION_CMD _IOR(0xA6, 1, struct ioctl_get_version_args)
#define AGNOCAST_DISCOVERY_AGENT_SHOULD_EXIT_CMD \
  _IOWR(0xA6, 29, struct ioctl_discovery_agent_should_exit_args)
#define AGNOCAST_ADD_DISCOVERY_AGENT_CMD _IOWR(0xA6, 30, struct ioctl_add_discovery_agent_args)
#define AGNOCAST_DISCOVERY_AGENT_EXISTS_CMD \
  _IOWR(0xA6, 31, struct ioctl_discovery_agent_exists_args)
#define AGNOCAST_GET_TOPIC_LIST_CMD _IOWR(0xA6, 20, union ioctl_topic_list_args)
#define AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD _IOWR(0xA6, 21, union ioctl_topic_info_args)
#define AGNOCAST_GET_TOPIC_PUBLISHER_INFO_CMD _IOWR(0xA6, 22, union ioctl_topic_info_args)
#define AGNOCAST_GET_NODE_SUBSCRIBER_TOPICS_CMD _IOWR(0xA6, 23, union ioctl_node_info_args)
#define AGNOCAST_GET_NODE_PUBLISHER_TOPICS_CMD _IOWR(0xA6, 24, union ioctl_node_info_args)
#define AGNOCAST_ADD_DOMAIN_BRIDGE_CMD _IOW(0xA6, 28, struct ioctl_add_domain_bridge_args)
