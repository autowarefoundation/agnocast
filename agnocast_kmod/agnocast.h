/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once

#include "agnocast_ioctl_abi.h"

#include <linux/ipc_namespace.h>
#include <linux/types.h>

// ================================================
// public macros and functions in agnocast_main.c

// From experience, EXIT_QUEUE_SIZE_BITS should be greater than 10
#define EXIT_QUEUE_SIZE_BITS 16
#define EXIT_QUEUE_SIZE (1U << EXIT_QUEUE_SIZE_BITS)
#define EXIT_QUEUE_MASK (EXIT_QUEUE_SIZE - 1)

int agnocast_init_device(void);
int agnocast_init_kthread(void);
int agnocast_init_exit_hook(void);

void agnocast_exit_free_data(void);
void agnocast_exit_kthread(void);
void agnocast_exit_exit_hook(void);
void agnocast_exit_device(void);

int agnocast_ioctl_add_subscriber(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const char * node_name,
  const pid_t subscriber_pid, const uint32_t qos_depth, const bool qos_is_transient_local,
  const bool qos_is_reliable, const bool is_take_sub, const bool ignore_local_publications,
  const bool is_bridge, const int32_t eventfd, union ioctl_add_subscriber_args * ioctl_ret);

int agnocast_ioctl_add_publisher(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const char * node_name,
  const pid_t publisher_pid, const uint32_t qos_depth, const bool qos_is_transient_local,
  const bool is_bridge, union ioctl_add_publisher_args * ioctl_ret);

int agnocast_ioctl_release_message_entry_reference(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const topic_local_id_t pubsub_id,
  const int64_t entry_id);

int agnocast_ioctl_receive_msg(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t subscriber_id, struct publisher_shm_info * pub_shm_infos,
  uint32_t pub_shm_infos_size, union ioctl_receive_msg_args * ioctl_ret);

int agnocast_ioctl_publish_msg(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const topic_local_id_t publisher_id,
  const uint64_t msg_virtual_address, union ioctl_publish_msg_args * ioctl_ret);

int agnocast_ioctl_take_msg(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t subscriber_id, bool allow_same_message,
  struct publisher_shm_info * pub_shm_infos, uint32_t pub_shm_infos_size,
  union ioctl_take_msg_args * ioctl_ret);

int agnocast_ioctl_add_process(
  const pid_t pid, const struct ipc_namespace * ipc_ns, const enum process_role role,
  const uint32_t domain_id, union ioctl_add_process_args * ioctl_ret);

int agnocast_ioctl_get_subscriber_num(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const pid_t pid,
  union ioctl_get_subscriber_num_args * ioctl_ret);

int agnocast_ioctl_get_publisher_num(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  union ioctl_get_publisher_num_args * ioctl_ret);

int agnocast_ioctl_get_topic_list(
  const struct ipc_namespace * ipc_ns, char * topic_name_buf, uint32_t * domain_id_buf,
  const uint32_t buf_topic_num, uint32_t * ret_topic_num);

int agnocast_ioctl_get_node_names(
  const struct ipc_namespace * ipc_ns, const pid_t pid, char * buf, const uint32_t buf_node_num,
  uint32_t * ret_node_num);

int agnocast_ioctl_get_subscriber_qos(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t subscriber_id, struct ioctl_get_subscriber_qos_args * args);

int agnocast_ioctl_get_publisher_qos(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const topic_local_id_t publisher_id,
  struct ioctl_get_publisher_qos_args * args);

int agnocast_ioctl_remove_subscriber(
  const char * topic_name, const struct ipc_namespace * ipc_ns, topic_local_id_t subscriber_id);

int agnocast_ioctl_remove_publisher(
  const char * topic_name, const struct ipc_namespace * ipc_ns, topic_local_id_t publisher_id);

int agnocast_ioctl_add_bridge(
  const char * topic_name, const pid_t pid, bool is_r2a, const struct ipc_namespace * ipc_ns,
  struct ioctl_add_bridge_args * ioctl_ret);

int agnocast_ioctl_remove_bridge(
  const char * topic_name, const pid_t pid, bool is_r2a, const struct ipc_namespace * ipc_ns);

int agnocast_ioctl_add_domain_bridge(
  const char * topic_name_from, const char * topic_name_to, uint32_t from_domain,
  uint32_t to_domain, const struct ipc_namespace * ipc_ns);

int agnocast_ioctl_add_domain_bridge_prefix(
  const char * topic_name_prefix, uint32_t from_domain, uint32_t to_domain,
  const struct ipc_namespace * ipc_ns);

int agnocast_ioctl_get_version(struct ioctl_get_version_args * ioctl_ret);

int agnocast_ioctl_get_topic_subscriber_info(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  union ioctl_topic_info_args * topic_info_args);

int agnocast_ioctl_get_topic_publisher_info(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  union ioctl_topic_info_args * topic_info_args);

int agnocast_ioctl_get_node_subscriber_topics(
  const struct ipc_namespace * ipc_ns, const char * node_name, char * topic_name_buf,
  const uint32_t buf_topic_num, uint32_t * ret_topic_num);

int agnocast_ioctl_get_node_publisher_topics(
  const struct ipc_namespace * ipc_ns, const char * node_name, char * topic_name_buf,
  const uint32_t buf_topic_num, uint32_t * ret_topic_num);

int agnocast_ioctl_check_and_request_bridge_shutdown(
  const pid_t pid, const struct ipc_namespace * ipc_ns,
  struct ioctl_check_and_request_bridge_shutdown_args * ioctl_ret);

int agnocast_ioctl_set_ros2_subscriber_num(
  const char * topic_name, const struct ipc_namespace * ipc_ns, uint32_t count);

int agnocast_ioctl_set_ros2_publisher_num(
  const char * topic_name, const struct ipc_namespace * ipc_ns, uint32_t count);

int agnocast_ioctl_notify_bridge_shutdown(const pid_t pid);

int agnocast_ioctl_discovery_agent_should_exit(
  const pid_t pid, const struct ipc_namespace * ipc_ns, const uint32_t domain_id, const bool commit,
  bool * ret_should_exit);

int agnocast_ioctl_add_discovery_agent(
  const pid_t pid, const struct ipc_namespace * ipc_ns, const uint32_t domain_id,
  struct ioctl_add_discovery_agent_args * ioctl_ret);

int agnocast_ioctl_discovery_agent_exists(
  const struct ipc_namespace * ipc_ns, const uint32_t domain_id, bool * ret_exists);

// Returns the exited process's global pid, or -1 if the namespace has none.
pid_t agnocast_ioctl_get_exit_process(
  const struct ipc_namespace * ipc_ns, struct ioctl_get_exit_process_args * ioctl_ret);

void agnocast_commit_exit_process(
  const struct ipc_namespace * ipc_ns, pid_t global_pid, pid_t caller_pid,
  bool * ret_daemon_should_exit);

void agnocast_process_exit_cleanup(const pid_t pid);

void agnocast_enqueue_exit_pid(const pid_t pid);
bool is_agnocast_pid(const pid_t pid);

// ================================================
// helper functions for KUnit test

#ifdef KUNIT_BUILD
int agnocast_increment_message_entry_rc(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const topic_local_id_t pubsub_id,
  const int64_t entry_id);
int agnocast_get_alive_proc_num(void);
int agnocast_get_discovery_agent_num(void);
bool agnocast_is_proc_exited(const pid_t pid);
int agnocast_get_topic_entries_num(const char * topic_name, const struct ipc_namespace * ipc_ns);
int64_t agnocast_get_latest_received_entry_id(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t subscriber_id);
bool agnocast_is_in_topic_entries(
  const char * topic_name, const struct ipc_namespace * ipc_ns, int64_t entry_id);
int agnocast_get_entry_rc(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const int64_t entry_id,
  const topic_local_id_t pubsub_id);
bool agnocast_is_in_subscriber_htable(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t subscriber_id);
bool agnocast_is_in_publisher_htable(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t publisher_id);
int agnocast_get_topic_num(const struct ipc_namespace * ipc_ns);
bool agnocast_is_in_topic_htable(const char * topic_name, const struct ipc_namespace * ipc_ns);
bool agnocast_is_in_bridge_htable(const char * topic_name, const struct ipc_namespace * ipc_ns);
pid_t agnocast_get_bridge_owner_pid(const char * topic_name, const struct ipc_namespace * ipc_ns);
bool agnocast_get_domain_rule(
  const char * topic_name, const struct ipc_namespace * ipc_ns, uint32_t domain,
  uint32_t * domain_a, uint32_t * domain_b, bool * a_to_b, bool * b_to_a);
// Returns the shared topic_struct's wrapper refcount for the wrapper in domain_id,
// or 0 if no such wrapper exists. Used to observe domain-bridge grouping.
int agnocast_topic_wrapper_refcnt(
  const char * topic_name, const struct ipc_namespace * ipc_ns, uint32_t domain_id);
#endif
