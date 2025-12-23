#pragma once

#include <linux/ipc_namespace.h>
#include <linux/types.h>

#define MAX_PUBLISHER_NUM 4        // Maximum number of publishers per topic
#define MAX_SUBSCRIBER_NUM 16      // Maximum number of subscribers per topic
#define MAX_QOS_DEPTH 10           // Maximum QoS depth for each publisher/subscriber
#define MAX_RELEASE_NUM 3          // Maximum number of entries that can be released at one ioctl
#define NODE_NAME_BUFFER_SIZE 256  // Maximum length of node name: 256 characters
#define VERSION_BUFFER_LEN 32      // Maximum size of version number represented as a string

typedef int32_t topic_local_id_t;
struct publisher_shm_info
{
  uint32_t publisher_num;
  pid_t publisher_pids[MAX_PUBLISHER_NUM];  // Must be local PIDs, not global PIDs
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

union ioctl_add_process_args {
  struct
  {
    uint64_t ret_addr;
    uint64_t ret_shm_size;
    bool ret_unlink_daemon_exist;
  };
};

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
  };
  struct
  {
    topic_local_id_t ret_id;
  };
};

union ioctl_add_publisher_args {
  struct
  {
    struct name_info topic_name;
    struct name_info node_name;
    uint32_t qos_depth;
    bool qos_is_transient_local;
  };
  struct
  {
    topic_local_id_t ret_id;
  };
};

struct ioctl_update_entry_args
{
  struct name_info topic_name;
  topic_local_id_t pubsub_id;
  int64_t entry_id;
};

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

union ioctl_get_subscriber_num_args {
  struct name_info topic_name;
  uint32_t ret_subscriber_num;
};

union ioctl_get_publisher_num_args {
  struct name_info topic_name;
  uint32_t ret_publisher_num;
};

struct ioctl_get_exit_process_args
{
  bool ret_daemon_should_exit;
  pid_t ret_pid;
};

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

struct ioctl_remove_bridge_args
{
  pid_t pid;
  struct name_info topic_name;
  bool is_r2a;
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

// ================================================
// ros2cli ioctls

#define MAX_TOPIC_NUM 1024

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
};

union ioctl_topic_info_args {
  struct
  {
    struct name_info topic_name;
    uint64_t topic_info_ret_buffer_addr;
  };
  uint32_t ret_topic_info_ret_num;
};

#define AGNOCAST_GET_TOPIC_LIST_CMD _IOWR(0xA6, 20, union ioctl_topic_list_args)
#define AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD _IOWR(0xA6, 21, union ioctl_topic_info_args)
#define AGNOCAST_GET_TOPIC_PUBLISHER_INFO_CMD _IOWR(0xA6, 22, union ioctl_topic_info_args)
#define AGNOCAST_GET_NODE_SUBSCRIBER_TOPICS_CMD _IOWR(0xA6, 23, union ioctl_node_info_args)
#define AGNOCAST_GET_NODE_PUBLISHER_TOPICS_CMD _IOWR(0xA6, 24, union ioctl_node_info_args)

// ================================================
// public macros and functions in agnocast_main.c

// From experience, EXIT_QUEUE_SIZE_BITS should be greater than 10
#define EXIT_QUEUE_SIZE_BITS 16
#define EXIT_QUEUE_SIZE (1U << EXIT_QUEUE_SIZE_BITS)

void agnocast_init_mutexes(void);
void agnocast_init_device(void);
int agnocast_init_kthread(void);
int agnocast_init_kprobe(void);

void agnocast_exit_free_data(void);
void agnocast_exit_kthread(void);
void agnocast_exit_kprobe(void);
void agnocast_exit_device(void);

int add_subscriber(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const char * node_name,
  const pid_t subscriber_pid, const uint32_t qos_depth, const bool qos_is_transient_local,
  const bool qos_is_reliable, const bool is_take_sub, const bool ignore_local_publications,
  union ioctl_add_subscriber_args * ioctl_ret);

int add_publisher(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const char * node_name,
  const pid_t publisher_pid, const uint32_t qos_depth, const bool qos_is_transient_local,
  union ioctl_add_publisher_args * ioctl_ret);

int increment_message_entry_rc(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const topic_local_id_t pubsub_id,
  const int64_t entry_id);

int decrement_message_entry_rc(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const topic_local_id_t pubsub_id,
  const int64_t entry_id);

int receive_msg(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t subscriber_id, union ioctl_receive_msg_args * ioctl_ret);

int publish_msg(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const topic_local_id_t publisher_id,
  const uint64_t msg_virtual_address, union ioctl_publish_msg_args * ioctl_ret);

int take_msg(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t subscriber_id, bool allow_same_message,
  union ioctl_take_msg_args * ioctl_ret);

int add_process(
  const pid_t pid, const struct ipc_namespace * ipc_ns, union ioctl_add_process_args * ioctl_ret);

int get_subscriber_num(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  union ioctl_get_subscriber_num_args * ioctl_ret);

int get_publisher_num(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  union ioctl_get_publisher_num_args * ioctl_ret);

int get_topic_list(
  const struct ipc_namespace * ipc_ns, union ioctl_topic_list_args * topic_list_args);

int get_subscriber_qos(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t subscriber_id, struct ioctl_get_subscriber_qos_args * args);

int get_publisher_qos(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const topic_local_id_t publisher_id,
  struct ioctl_get_publisher_qos_args * args);

int remove_subscriber(
  const char * topic_name, const struct ipc_namespace * ipc_ns, topic_local_id_t subscriber_id);

int remove_publisher(
  const char * topic_name, const struct ipc_namespace * ipc_ns, topic_local_id_t publisher_id);

int add_bridge(
  const char * topic_name, const pid_t pid, bool is_r2a, const struct ipc_namespace * ipc_ns,
  struct ioctl_add_bridge_args * ioctl_ret);

int remove_bridge(
  const char * topic_name, const pid_t pid, bool is_r2a, const struct ipc_namespace * ipc_ns);

void process_exit_cleanup(const pid_t pid);

void enqueue_exit_pid(const pid_t pid);

// ================================================
// helper functions for KUnit test

#ifdef KUNIT_BUILD
int get_alive_proc_num(void);
bool is_proc_exited(const pid_t pid);
int get_topic_entries_num(const char * topic_name, const struct ipc_namespace * ipc_ns);
int64_t get_latest_received_entry_id(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t subscriber_id);
bool is_in_topic_entries(
  const char * topic_name, const struct ipc_namespace * ipc_ns, int64_t entry_id);
int get_entry_rc(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const int64_t entry_id,
  const topic_local_id_t pubsub_id);
bool is_in_subscriber_htable(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t subscriber_id);
bool is_in_publisher_htable(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t publisher_id);
int get_topic_num(const struct ipc_namespace * ipc_ns);
bool is_in_topic_htable(const char * topic_name, const struct ipc_namespace * ipc_ns);
bool is_in_bridge_htable(const char * topic_name, const struct ipc_namespace * ipc_ns);
pid_t get_bridge_owner_pid(const char * topic_name, const struct ipc_namespace * ipc_ns);
#endif
