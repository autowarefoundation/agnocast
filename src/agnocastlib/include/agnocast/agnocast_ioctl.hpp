#pragma once

#include <sys/ioctl.h>
#include <sys/types.h>

#include <cstdint>

namespace agnocast
{

#define MAX_PUBLISHER_NUM 4    // Maximum number of publishers per topic
#define MAX_SUBSCRIBER_NUM 16  // Maximum number of subscribers per topic
#define MAX_QOS_DEPTH 10       // Maximum QoS depth for each publisher/subscriber
#define MAX_RELEASE_NUM 3      // Maximum number of entries that can be released at one ioctl
#define VERSION_BUFFER_LEN 32  // Maximum size of version number represented as a string

#define MAX_TOPIC_INFO_RET_NUM std::max(MAX_PUBLISHER_NUM, MAX_SUBSCRIBER_NUM)

#define NODE_NAME_BUFFER_SIZE 256

constexpr const char * AGNOCAST_DEVICE_NOT_FOUND_MSG =
  "Failed to open /dev/agnocast: Device not found. "
  "Please ensure the agnocast kernel module is installed. "
  "Run 'sudo modprobe agnocast' or 'sudo insmod <path-to-agnocast.ko>' to load the module.";

using topic_local_id_t = int32_t;
struct publisher_shm_info
{
  uint32_t publisher_num;
  pid_t publisher_pids[MAX_PUBLISHER_NUM];
  uint64_t shm_addrs[MAX_PUBLISHER_NUM];
  uint64_t shm_sizes[MAX_PUBLISHER_NUM];
};
struct name_info
{
  const char * ptr;
  uint64_t len;
};

struct ioctl_get_version_args
{
  char ret_version[VERSION_BUFFER_LEN];
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
union ioctl_add_process_args {
  struct
  {
    uint64_t ret_addr;
    uint64_t ret_shm_size;
    bool ret_unlink_daemon_exist;
  };
};
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
union ioctl_add_subscriber_args {
  struct
  {
    struct name_info topic_name;
    struct name_info node_name;
    uint32_t qos_depth;
    bool qos_is_transient_local;
    bool qos_is_reliable;
    bool is_take_sub;
    bool ignore_local_publications;
    bool is_bridge;
  };
  struct
  {
    topic_local_id_t ret_id;
  };
};
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
union ioctl_add_publisher_args {
  struct
  {
    struct name_info topic_name;
    struct name_info node_name;
    uint32_t qos_depth;
    bool qos_is_transient_local;
    bool is_bridge;
  };
  struct
  {
    topic_local_id_t ret_id;
  };
};
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
struct ioctl_update_entry_args
{
  struct name_info topic_name;
  topic_local_id_t pubsub_id;
  int64_t entry_id;
};
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
union ioctl_receive_msg_args {
  struct
  {
    struct name_info topic_name;
    topic_local_id_t subscriber_id;
  };
  struct
  {
    uint16_t ret_entry_num;
    int64_t ret_entry_ids[MAX_QOS_DEPTH];
    uint64_t ret_entry_addrs[MAX_QOS_DEPTH];
    struct publisher_shm_info ret_pub_shm_info;
  };
};
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
union ioctl_publish_msg_args {
  struct
  {
    struct name_info topic_name;
    topic_local_id_t publisher_id;
    uint64_t msg_virtual_address;
  };
  struct
  {
    int64_t ret_entry_id;
    uint32_t ret_subscriber_num;
    topic_local_id_t ret_subscriber_ids[MAX_SUBSCRIBER_NUM];
    uint32_t ret_released_num;
    uint64_t ret_released_addrs[MAX_RELEASE_NUM];
  };
};
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
union ioctl_take_msg_args {
  struct
  {
    struct name_info topic_name;
    topic_local_id_t subscriber_id;
    bool allow_same_message;
  };
  struct
  {
    uint64_t ret_addr;
    int64_t ret_entry_id;
    struct publisher_shm_info ret_pub_shm_info;
  };
};
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
union ioctl_get_subscriber_num_args {
  struct
  {
    struct name_info topic_name;
    bool include_ros2;
  };
  struct
  {
    uint32_t ret_subscriber_num;
    bool ret_bridge_exist;
  };
};
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
union ioctl_get_publisher_num_args {
  struct name_info topic_name;
  struct
  {
    uint32_t ret_publisher_num;
    bool ret_bridge_exist;
  };
};
#pragma GCC diagnostic pop

struct ioctl_get_exit_process_args
{
  bool ret_daemon_should_exit;
  pid_t ret_pid;
};

struct topic_info_ret
{
  char node_name[NODE_NAME_BUFFER_SIZE];
  uint32_t qos_depth;
  bool qos_is_transient_local;
  bool qos_is_reliable;
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
union ioctl_topic_info_args {
  struct
  {
    struct name_info topic_name;
    uint64_t topic_info_ret_buffer_addr;
  };
  uint32_t ret_topic_info_ret_num;
};
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
struct ioctl_get_subscriber_qos_args
{
  struct
  {
    struct name_info topic_name;
    topic_local_id_t subscriber_id;
  };
  struct
  {
    uint32_t ret_depth;
    bool ret_is_transient_local;
    bool ret_is_reliable;
  };
};
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
struct ioctl_get_publisher_qos_args
{
  struct
  {
    struct name_info topic_name;
    topic_local_id_t publisher_id;
  };
  struct
  {
    uint32_t ret_depth;
    bool ret_is_transient_local;
  };
};
#pragma GCC diagnostic pop

struct ioctl_remove_subscriber_args
{
  struct name_info topic_name;
  topic_local_id_t subscriber_id;
};

struct ioctl_remove_publisher_args
{
  struct name_info topic_name;
  topic_local_id_t publisher_id;
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
struct ioctl_add_bridge_args
{
  struct
  {
    pid_t pid;
    struct name_info topic_name;
    bool is_r2a;
  };
  struct
  {
    pid_t ret_pid;
    bool ret_has_r2a;
    bool ret_has_a2r;
  };
};
#pragma GCC diagnostic pop

struct ioctl_remove_bridge_args
{
  pid_t pid;
  struct name_info topic_name;
  bool is_r2a;
};

struct ioctl_get_process_num_args
{
  uint32_t ret_process_num;
};

struct ioctl_set_ros2_subscriber_num_args
{
  struct name_info topic_name;
  uint32_t ros2_subscriber_num;
};

#define AGNOCAST_GET_VERSION_CMD _IOR(0xA6, 1, struct ioctl_get_version_args)
#define AGNOCAST_ADD_PROCESS_CMD _IOWR(0xA6, 2, union ioctl_add_process_args)
#define AGNOCAST_ADD_SUBSCRIBER_CMD _IOWR(0xA6, 3, union ioctl_add_subscriber_args)
#define AGNOCAST_ADD_PUBLISHER_CMD _IOWR(0xA6, 4, union ioctl_add_publisher_args)
#define AGNOCAST_INCREMENT_RC_CMD _IOW(0xA6, 5, struct ioctl_update_entry_args)
#define AGNOCAST_DECREMENT_RC_CMD _IOW(0xA6, 6, struct ioctl_update_entry_args)
#define AGNOCAST_PUBLISH_MSG_CMD _IOWR(0xA6, 7, union ioctl_publish_msg_args)
#define AGNOCAST_RECEIVE_MSG_CMD _IOWR(0xA6, 8, union ioctl_receive_msg_args)
#define AGNOCAST_TAKE_MSG_CMD _IOWR(0xA6, 9, union ioctl_take_msg_args)
#define AGNOCAST_GET_SUBSCRIBER_NUM_CMD _IOWR(0xA6, 10, union ioctl_get_subscriber_num_args)
#define AGNOCAST_GET_EXIT_PROCESS_CMD _IOR(0xA6, 11, struct ioctl_get_exit_process_args)
#define AGNOCAST_GET_SUBSCRIBER_QOS_CMD _IOWR(0xA6, 12, struct ioctl_get_subscriber_qos_args)
#define AGNOCAST_GET_PUBLISHER_QOS_CMD _IOWR(0xA6, 13, struct ioctl_get_publisher_qos_args)
#define AGNOCAST_ADD_BRIDGE_CMD _IOWR(0xA6, 14, struct ioctl_add_bridge_args)
#define AGNOCAST_REMOVE_BRIDGE_CMD _IOW(0xA6, 15, struct ioctl_remove_bridge_args)
#define AGNOCAST_GET_PUBLISHER_NUM_CMD _IOWR(0xA6, 16, union ioctl_get_publisher_num_args)
#define AGNOCAST_REMOVE_SUBSCRIBER_CMD _IOW(0xA6, 17, struct ioctl_remove_subscriber_args)
#define AGNOCAST_REMOVE_PUBLISHER_CMD _IOW(0xA6, 18, struct ioctl_remove_publisher_args)
#define AGNOCAST_GET_PROCESS_NUM_CMD _IOR(0xA6, 19, struct ioctl_get_process_num_args)
#define AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD _IOWR(0xA6, 21, union ioctl_topic_info_args)
#define AGNOCAST_SET_ROS2_SUBSCRIBER_NUM_CMD \
  _IOW(0xA6, 25, struct ioctl_set_ros2_subscriber_num_args)

}  // namespace agnocast
