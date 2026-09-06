#pragma once

#include <sys/ioctl.h>
#include <sys/types.h>

#include <algorithm>
#include <cstdint>

namespace agnocast
{

#define MAX_PUBLISHER_NUM 1024   // Maximum number of publishers per topic
#define MAX_TOPIC_LOCAL_ID 4096  // Bitmap size for per-entry subscriber reference tracking
#define MAX_SUBSCRIBER_NUM \
  (MAX_TOPIC_LOCAL_ID - MAX_PUBLISHER_NUM)  // Maximum number of subscribers per topic
/* Maximum number of entries that can be received at one ioctl. This value is heuristically set to
 * balance the number of calling ioctl and the overhead of copying data between user and kernel
 * space. */
#define MAX_RECEIVE_NUM 10
#define MAX_RELEASE_NUM 3      // Maximum number of entries that can be released at one ioctl
#define VERSION_BUFFER_LEN 32  // Maximum size of version number represented as a string

#define MAX_TOPIC_INFO_RET_NUM std::max(MAX_PUBLISHER_NUM, MAX_SUBSCRIBER_NUM)

#define NODE_NAME_BUFFER_SIZE 256
#define TOPIC_NAME_BUFFER_SIZE 256
#define MAX_SUBSCRIPTION_NUM_PER_PROCESS 256

constexpr const char * AGNOCAST_DEVICE_NOT_FOUND_MSG =
  "Failed to open /dev/agnocast: Device not found. "
  "Please ensure the agnocast kernel module is installed. "
  "Run 'sudo modprobe agnocast' or 'sudo insmod <path-to-agnocast.ko>' to load the module.";

using topic_local_id_t = int32_t;
struct publisher_shm_info
{
  pid_t pid;
  uint64_t shm_addr;
  uint64_t shm_size;
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
    bool is_performance_bridge_manager;
    uint32_t domain_id;  // The process's ROS_DOMAIN_ID (0 if unset).
  };
  struct
  {
    uint64_t ret_addr;
    uint64_t ret_shm_size;
    bool ret_unlink_daemon_exist;
    bool ret_performance_bridge_daemon_exist;
    bool ret_discovery_agent_exist;
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
    // Topic name to use for the publish-notification MQ: the requested topic for a plain topic, or
    // the domain-bridge pair's canonical name for a bridged (incl. renamed) topic, so a publisher
    // and a renamed subscriber sharing one topic_struct derive the same MQ name.
    char ret_mq_topic_name[TOPIC_NAME_BUFFER_SIZE];
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
    // See ioctl_add_subscriber_args::ret_mq_topic_name.
    char ret_mq_topic_name[TOPIC_NAME_BUFFER_SIZE];
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
    // Unlike ret_* fields which are returned via the union copy, publisher shm info is written
    // directly to this user-space buffer via copy_to_user. The caller must ensure the buffer
    // remains valid until the ioctl returns.
    uint64_t pub_shm_info_addr;
    uint32_t pub_shm_info_size;
  };
  struct
  {
    uint16_t ret_entry_num;
    bool ret_call_again;
    int64_t ret_entry_ids[MAX_RECEIVE_NUM];
    uint64_t ret_entry_addrs[MAX_RECEIVE_NUM];
    uint32_t ret_pub_shm_num;
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
    // Unlike ret_* fields which are returned via the union copy, subscriber IDs are written
    // directly to this user-space buffer via copy_to_user. The caller must ensure the buffer
    // remains valid until the ioctl returns.
    uint64_t subscriber_ids_buffer_addr;
    uint32_t subscriber_ids_buffer_size;
  };
  struct
  {
    int64_t ret_entry_id;
    uint32_t ret_subscriber_num;
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
    // Unlike ret_* fields which are returned via the union copy, publisher shm info is written
    // directly to this user-space buffer via copy_to_user. The caller must ensure the buffer
    // remains valid until the ioctl returns.
    uint64_t pub_shm_info_addr;
    uint32_t pub_shm_info_size;
  };
  struct
  {
    uint64_t ret_addr;
    int64_t ret_entry_id;
    uint32_t ret_pub_shm_num;
  };
};
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
union ioctl_get_subscriber_num_args {
  struct name_info topic_name;
  struct
  {
    uint32_t ret_other_process_subscriber_num;
    uint32_t ret_same_process_subscriber_num;
    uint32_t ret_ros2_subscriber_num;
    bool ret_a2r_bridge_exist;
    bool ret_r2a_bridge_exist;
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
    uint32_t ret_ros2_publisher_num;
    bool ret_r2a_bridge_exist;
    bool ret_a2r_bridge_exist;
  };
};
#pragma GCC diagnostic pop

struct exit_subscription_mq_info
{
  char topic_name[TOPIC_NAME_BUFFER_SIZE];
  topic_local_id_t subscriber_id;
};

struct ioctl_get_exit_process_args
{
  // input: user-space buffer for subscription MQ info
  uint64_t subscription_mq_info_buffer_addr;
  uint32_t subscription_mq_info_buffer_size;
  // output
  bool ret_daemon_should_exit;
  pid_t ret_pid;
  uint32_t ret_subscription_mq_info_num;
};

struct topic_info_ret
{
  char node_name[NODE_NAME_BUFFER_SIZE];
  uint32_t qos_depth;
  bool qos_is_transient_local;
  bool qos_is_reliable;
  bool is_bridge;
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
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
  struct name_info topic_name;
  bool is_r2a;
};

struct ioctl_check_and_request_bridge_shutdown_args
{
  bool ret_should_shutdown;
};

struct ioctl_set_ros2_subscriber_num_args
{
  struct name_info topic_name;
  uint32_t ros2_subscriber_num;
};

struct ioctl_set_ros2_publisher_num_args
{
  struct name_info topic_name;
  uint32_t ros2_publisher_num;
};

#define AGNOCAST_GET_VERSION_CMD _IOR(0xA6, 1, struct ioctl_get_version_args)
#define AGNOCAST_ADD_PROCESS_CMD _IOWR(0xA6, 2, union ioctl_add_process_args)
#define AGNOCAST_ADD_SUBSCRIBER_CMD _IOWR(0xA6, 3, union ioctl_add_subscriber_args)
#define AGNOCAST_ADD_PUBLISHER_CMD _IOWR(0xA6, 4, union ioctl_add_publisher_args)
#define AGNOCAST_RELEASE_SUB_REF_CMD _IOW(0xA6, 6, struct ioctl_update_entry_args)
#define AGNOCAST_PUBLISH_MSG_CMD _IOWR(0xA6, 7, union ioctl_publish_msg_args)
#define AGNOCAST_RECEIVE_MSG_CMD _IOWR(0xA6, 8, union ioctl_receive_msg_args)
#define AGNOCAST_TAKE_MSG_CMD _IOWR(0xA6, 9, union ioctl_take_msg_args)
#define AGNOCAST_GET_SUBSCRIBER_NUM_CMD _IOWR(0xA6, 10, union ioctl_get_subscriber_num_args)
#define AGNOCAST_GET_EXIT_PROCESS_CMD _IOWR(0xA6, 11, struct ioctl_get_exit_process_args)
#define AGNOCAST_GET_SUBSCRIBER_QOS_CMD _IOWR(0xA6, 12, struct ioctl_get_subscriber_qos_args)
#define AGNOCAST_GET_PUBLISHER_QOS_CMD _IOWR(0xA6, 13, struct ioctl_get_publisher_qos_args)
#define AGNOCAST_ADD_BRIDGE_CMD _IOWR(0xA6, 14, struct ioctl_add_bridge_args)
#define AGNOCAST_REMOVE_BRIDGE_CMD _IOW(0xA6, 15, struct ioctl_remove_bridge_args)
#define AGNOCAST_GET_PUBLISHER_NUM_CMD _IOWR(0xA6, 16, union ioctl_get_publisher_num_args)
#define AGNOCAST_REMOVE_SUBSCRIBER_CMD _IOW(0xA6, 17, struct ioctl_remove_subscriber_args)
#define AGNOCAST_REMOVE_PUBLISHER_CMD _IOW(0xA6, 18, struct ioctl_remove_publisher_args)
#define AGNOCAST_CHECK_AND_REQUEST_BRIDGE_SHUTDOWN_CMD \
  _IOR(0xA6, 19, struct ioctl_check_and_request_bridge_shutdown_args)
// GPU device-memory region sharing. Mirrors agnocast_kmod/agnocast.h; the two
// copies are hand-maintained and the major.minor version gate is what stands
// between a missed edit and silent memory corruption.
//
// A publisher owns a list of regions rather than one: it grows its pool by
// adding a region instead of failing to borrow, so a message names the region it
// was written into. The kernel module stores each export without interpreting
// it: it holds the region's liveness reference so the memory survives the
// publishing process, and installs a fresh descriptor for each importer rather
// than having descriptors passed between processes.
#define GPU_DEVICE_UUID_SIZE 16
#define MAX_GPU_HANDLE_BLOB_SIZE 4096
#define MAX_GPU_REGION_NUM_PER_PUBLISHER 16

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
union ioctl_add_gpu_region_args {
  struct
  {
    struct name_info topic_name;
    topic_local_id_t publisher_id;
    uint32_t backend_type;
    uint32_t slot_size;
    uint32_t slot_count;
    uint64_t mapped_size;
    uint8_t device_uuid[GPU_DEVICE_UUID_SIZE];
    // -1 for mechanisms whose handle is not a file descriptor.
    int32_t handle_fd;
    uint64_t blob_addr;
    uint32_t blob_size;
  };
  // Globally unique for the module's lifetime. Messages carry it so a peer can
  // resolve a slot to its own mapping without knowing which topic or publisher
  // produced it.
  uint32_t ret_region_id;
};
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
union ioctl_get_gpu_region_args {
  struct
  {
    struct name_info topic_name;
    topic_local_id_t publisher_id;
    // Carried because NvSciBuf export descriptors are bound to a destination
    // endpoint. Mechanisms without that constraint serve every subscriber the
    // same descriptor.
    topic_local_id_t subscriber_id;
    // The region a message names, or 0 for "any", which is what a caller that
    // has not seen a message yet asks for.
    uint32_t region_id;
    uint64_t blob_buffer_addr;
    uint32_t blob_buffer_size;
  };
  struct
  {
    uint32_t ret_backend_type;
    uint32_t ret_slot_size;
    uint32_t ret_slot_count;
    uint64_t ret_mapped_size;
    uint8_t ret_device_uuid[GPU_DEVICE_UUID_SIZE];
    // Installed in this process for the same open file the kernel module holds,
    // or -1 for mechanisms that do not use one.
    int32_t ret_handle_fd;
    uint32_t ret_blob_size;
    uint32_t ret_region_id;
  };
};
#pragma GCC diagnostic pop

#define AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD _IOWR(0xA6, 21, union ioctl_topic_info_args)
#define AGNOCAST_SET_ROS2_SUBSCRIBER_NUM_CMD \
  _IOW(0xA6, 25, struct ioctl_set_ros2_subscriber_num_args)
#define AGNOCAST_SET_ROS2_PUBLISHER_NUM_CMD _IOW(0xA6, 26, struct ioctl_set_ros2_publisher_num_args)
#define AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD _IO(0xA6, 27)
#define AGNOCAST_ADD_GPU_REGION_CMD _IOWR(0xA6, 32, union ioctl_add_gpu_region_args)
#define AGNOCAST_GET_GPU_REGION_CMD _IOWR(0xA6, 33, union ioctl_get_gpu_region_args)

}  // namespace agnocast
