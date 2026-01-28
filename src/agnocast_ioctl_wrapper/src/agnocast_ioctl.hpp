#pragma once

#include <algorithm>
#include <cstdint>

#define MAX_PUBLISHER_NUM 4    // Maximum number of publishers per topic
#define MAX_SUBSCRIBER_NUM 16  // Maximum number of subscribers per topic

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
  uint64_t topic_name_buffer_addr;
  uint32_t ret_topic_num;
};

union ioctl_node_info_args {
  struct
  {
    struct name_info node_name;
    uint64_t topic_name_buffer_addr;
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
  };
  uint32_t ret_topic_info_ret_num;
};
#pragma GCC diagnostic pop

#define AGNOCAST_GET_TOPIC_LIST_CMD _IOWR(0xA6, 20, union ioctl_topic_list_args)
#define AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD _IOWR(0xA6, 21, union ioctl_topic_info_args)
#define AGNOCAST_GET_TOPIC_PUBLISHER_INFO_CMD _IOWR(0xA6, 22, union ioctl_topic_info_args)
#define AGNOCAST_GET_NODE_SUBSCRIBER_TOPICS_CMD _IOWR(0xA6, 23, union ioctl_node_info_args)
#define AGNOCAST_GET_NODE_PUBLISHER_TOPICS_CMD _IOWR(0xA6, 24, union ioctl_node_info_args)
