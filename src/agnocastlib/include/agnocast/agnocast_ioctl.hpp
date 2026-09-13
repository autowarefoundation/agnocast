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
#define MAX_NODE_NUM 1024  // Maximum number of node names returned by GET_NODE_NAMES
#define TOPIC_NAME_BUFFER_SIZE 256

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
union ioctl_get_node_names_args {
  struct
  {
    uint64_t node_name_buffer_addr;
    uint32_t node_name_buffer_size;
  };
  uint32_t ret_node_num;
};
#pragma GCC diagnostic pop

// Mirrors AGNOCAST_DOMAIN_ID_NONE in the kernel module.
#define AGNOCAST_DOMAIN_ID_NONE UINT32_MAX

// Mirrors enum process_role in the kernel module.
enum process_role {
  PROCESS_ROLE_APPLICATION = 0,
  PROCESS_ROLE_BRIDGE_MANAGER = 1,
  PROCESS_ROLE_UNLINK_DAEMON = 2,
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
union ioctl_add_process_args {
  struct
  {
    uint32_t role;       // enum process_role
    uint32_t domain_id;  // The process's ROS_DOMAIN_ID (0 if unset).
  };
  struct
  {
    uint64_t ret_addr;
    uint64_t ret_shm_size;
    bool ret_unlink_daemon_exist;
    bool ret_bridge_daemon_exist;
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
    int32_t eventfd;
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
  };
  struct
  {
    int64_t ret_entry_id;
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
    // Subscribers in the domain a bridge rule pairs this one with, counted only where the rule
    // delivers this way round. Disjoint from the own-domain counts above.
    uint32_t ret_other_domain_subscriber_num;
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

// Decided atomically by the kmod, which keys the singleton on the caller's IPC namespace and
// domain, and its liveness on the calling PID: the caller must be the process that becomes the
// agent.
struct ioctl_add_discovery_agent_args
{
  uint32_t domain_id;
  bool ret_owned_by_caller;
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
#define AGNOCAST_GET_EXIT_PROCESS_CMD _IOR(0xA6, 11, struct ioctl_get_exit_process_args)
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
// between a missed edit and silent memory corruption. See docs/gpu_ipc.md.
#define GPU_DEVICE_UUID_SIZE 16
// Regions a publisher may hold at once. Reaching it is not terminal: a region
// holding no message can be removed to make room for another.
#define MAX_GPU_REGION_NUM_PER_PUBLISHER 16

// Mirrors agnocast::internal::GpuMemoryBackendType for the module's check that a
// handle is the kind the declared mechanism uses. Never renumber or reuse one; 2
// is spoken for by NvSciBuf, which the module does not implement.
#define AGNOCAST_GPU_BACKEND_VMM 1

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
    int32_t handle_fd;
  };
  // Unique for the module's lifetime and never reused. The publisher records it
  // in each message written into this region, and a subscriber resolves it back
  // to its own mapping.
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
    // Who is asking: authorization, not routing. Must name a subscriber of this
    // topic belonging to the calling process.
    topic_local_id_t subscriber_id;
    // The region id read out of the message being resolved, or 0 for "any",
    // which is what a caller that has not seen a message yet asks for.
    uint32_t region_id;
  };
  struct
  {
    uint32_t ret_backend_type;
    uint32_t ret_slot_size;
    uint32_t ret_slot_count;
    uint64_t ret_mapped_size;
    uint8_t ret_device_uuid[GPU_DEVICE_UUID_SIZE];
    // Installed in this process for the same open file the kernel module holds.
    int32_t ret_handle_fd;
    uint32_t ret_region_id;
  };
};
#pragma GCC diagnostic pop

// Asks which of the named regions the module still holds. Keyed on region ids
// alone, so it needs no publisher or subscriber to authorize against -- an
// importer can still ask after the publisher it imported from is gone, which is
// exactly when it needs to.
#define MAX_GPU_REGION_QUERY_NUM 64

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
union ioctl_gpu_region_exists_args {
  struct
  {
    uint64_t region_ids_addr;
    uint32_t region_num;
  };
  // Bit i is set when the i-th id given is still registered.
  uint64_t ret_exists_bitmap;
};
#pragma GCC diagnostic pop

// Releases the module's liveness reference on one region. The caller must own
// the publisher and must already know that no message refers to the region.
struct ioctl_remove_gpu_region_args
{
  struct name_info topic_name;
  topic_local_id_t publisher_id;
  uint32_t region_id;
};

#define AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD _IOWR(0xA6, 21, union ioctl_topic_info_args)
#define AGNOCAST_SET_ROS2_SUBSCRIBER_NUM_CMD \
  _IOW(0xA6, 25, struct ioctl_set_ros2_subscriber_num_args)
#define AGNOCAST_SET_ROS2_PUBLISHER_NUM_CMD _IOW(0xA6, 26, struct ioctl_set_ros2_publisher_num_args)
#define AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD _IO(0xA6, 27)
#define AGNOCAST_ADD_DISCOVERY_AGENT_CMD _IOWR(0xA6, 30, struct ioctl_add_discovery_agent_args)
#define AGNOCAST_GET_NODE_NAMES_CMD _IOWR(0xA6, 33, union ioctl_get_node_names_args)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// Releases the caller's own entries that QoS depth no longer retains, reporting
// their addresses exactly as a publish does. A GPU publisher reclaims its slots
// by destroying the messages named here; a publish is the only other thing that
// reports them, and a publisher with no free slot has nothing to publish. See
// docs/gpu_ipc.md.
union ioctl_reclaim_msgs_args {
  struct
  {
    struct name_info topic_name;
    topic_local_id_t publisher_id;
  };
  struct
  {
    uint32_t ret_released_num;
    uint64_t ret_released_addrs[MAX_RELEASE_NUM];
  };
};
#pragma GCC diagnostic pop

#define AGNOCAST_ADD_GPU_REGION_CMD _IOWR(0xA6, 34, union ioctl_add_gpu_region_args)
#define AGNOCAST_GET_GPU_REGION_CMD _IOWR(0xA6, 35, union ioctl_get_gpu_region_args)
#define AGNOCAST_REMOVE_GPU_REGION_CMD _IOW(0xA6, 36, struct ioctl_remove_gpu_region_args)
#define AGNOCAST_RECLAIM_MSGS_CMD _IOWR(0xA6, 37, union ioctl_reclaim_msgs_args)
#define AGNOCAST_GPU_REGION_EXISTS_CMD _IOWR(0xA6, 38, union ioctl_gpu_region_exists_args)

}  // namespace agnocast
