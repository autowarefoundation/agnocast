/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once

#include "agnocast.h"
#include "agnocast_memory_allocator.h"

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/hashtable.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mm.h>  // kvzalloc, kvfree
#include <linux/module.h>
#include <linux/rwsem.h>
#include <linux/slab.h>  // kmalloc, kfree
#include <linux/tracepoint.h>
#include <linux/version.h>

extern int major;
extern struct class * agnocast_class;
extern struct device * agnocast_device;

// Locking convention:
//   Only agnocast_ioctl_ prefixed functions acquire locks. All other internal/static functions are
//   lock-free and rely on callers to hold the appropriate locks. Exceptions are
//   agnocast_process_exit_cleanup, agnocast_exit_free_data, and
//   agnocast_increment_message_entry_rc, which manage locks directly.
//
// Lock ordering (to prevent deadlocks, always acquire in this order):
//   1. global_htables_rwsem   (this file)
//   2. topic_rwsem            (per-topic, in struct topic_wrapper)
//   3. mempool_lock           (agnocast_memory_allocator.c)
//
// Global rwsem for hashtables (topic_hashtable, proc_info_htable, bridge_htable)
// - Read lock (down_read): when searching hashtables and operating within a topic
// - Write lock (down_write): when adding/removing entries from hashtables
extern struct rw_semaphore global_htables_rwsem;

// =========================================
// data structure

// hashtable size becomes 2^HASH_BITS
#define TOPIC_HASH_BITS 10
#define PUB_INFO_HASH_BITS 3
#define SUB_INFO_HASH_BITS 5
#define PROC_INFO_HASH_BITS 10

// Allocated in pre_handler_subscriber_exit(), freed in agnocast_commit_exit_process() after
// the daemon successfully copies the data to user-space.
struct exit_subscription_entry
{
  char topic_name[TOPIC_NAME_BUFFER_SIZE];
  topic_local_id_t subscriber_id;
  struct list_head list;
};

struct process_info
{
  bool exited;
  // Used to track whether this process is an alive Performance Bridge Manager.
  // Standard Bridge Manager also updates this flag for consistency, but the flag
  // is not used for Standard Bridge spawn decisions (Standard bridges are spawned
  // per-process, not per-IPC-namespace).
  bool is_performance_bridge_manager;
  pid_t global_pid;
  pid_t local_pid;
  struct mempool_entry * mempool_entry;
  const struct ipc_namespace * ipc_ns;
  struct list_head exit_subscription_list;
  uint32_t exit_subscription_count;
  struct hlist_node node;
  struct rcu_head rcu_head;
};

extern DECLARE_HASHTABLE(proc_info_htable, PROC_INFO_HASH_BITS);

struct publisher_info
{
  topic_local_id_t id;
  pid_t pid;
  char * node_name;
  uint32_t qos_depth;
  bool qos_is_transient_local;
  uint32_t entries_num;
  bool is_bridge;
  struct hlist_node node;
};

struct subscriber_info
{
  topic_local_id_t id;
  pid_t pid;
  uint32_t qos_depth;
  bool qos_is_transient_local;
  bool qos_is_reliable;
  int64_t latest_received_entry_id;
  char * node_name;
  bool is_take_sub;
  bool ignore_local_publications;
  bool need_mmap_update;
  bool is_bridge;
  struct hlist_node node;
};

// Helper to copy a name_info string from userspace to a kernel stack buffer.
// Returns 0 on success, -EINVAL if too long, -EFAULT on copy failure.
static inline long copy_name_from_user(char * dst, size_t dst_size, const struct name_info * src)
{
  if (src->len >= dst_size) return -EINVAL;
  if (copy_from_user(dst, (const char __user *)src->ptr, src->len)) return -EFAULT;
  dst[src->len] = '\0';
  return 0;
}

struct topic_struct
{
  struct rb_root entries;
  DECLARE_HASHTABLE(pub_info_htable, PUB_INFO_HASH_BITS);
  DECLARE_HASHTABLE(sub_info_htable, SUB_INFO_HASH_BITS);
  topic_local_id_t current_pubsub_id;
  int64_t current_entry_id;
  uint32_t ros2_subscriber_num;  // Updated by Bridge Manager
  uint32_t ros2_publisher_num;   // Updated by Bridge Manager
};

struct topic_wrapper
{
  const struct ipc_namespace *
    ipc_ns;  // For use in separating topic namespaces when using containers.
  char * key;
  struct rw_semaphore
    topic_rwsem;  // Per-topic rwsem: read for read-only ops, write for publish/receive/modify
  struct topic_struct topic;
  struct hlist_node node;
};

struct entry_node
{
  struct rb_node node;
  int64_t entry_id;  // rbtree key
  topic_local_id_t publisher_id;
  uint64_t msg_virtual_address;
  // Per-subscriber boolean flag: if bit is set, subscriber is holding a reference.
  // When subscriber releases the reference, the bit is cleared.
  DECLARE_BITMAP(referencing_subscribers, MAX_TOPIC_LOCAL_ID);
};

extern DECLARE_HASHTABLE(topic_hashtable, TOPIC_HASH_BITS);

struct bridge_info
{
  char * topic_name;
  pid_t pid;
  bool has_r2a;  // ROS2 -> Agnocast
  bool has_a2r;  // Agnocast -> ROS2
  const struct ipc_namespace * ipc_ns;
  struct hlist_node node;
};

extern DECLARE_HASHTABLE(bridge_htable, TOPIC_HASH_BITS);

int agnocast_get_size_sub_info_htable(struct topic_wrapper * wrapper);

int agnocast_get_size_pub_info_htable(struct topic_wrapper * wrapper);

bool agnocast_is_referenced(struct entry_node * en);

struct process_info * agnocast_find_process_info(const pid_t pid);

void agnocast_free_exit_subscription_list(struct process_info * proc_info);

void agnocast_remove_entry_node(struct topic_wrapper * wrapper, struct entry_node * en);

long agnocast_ioctl(struct file * file, unsigned int cmd, unsigned long arg);

// Ring buffer to hold exited pids.
// EXIT_QUEUE_SIZE (65536) far exceeds mempool_num (default 4096), and only Agnocast PIDs are
// enqueued (via is_agnocast_pid()), each exiting at most once, so the ring buffer cannot overflow.
extern spinlock_t pid_queue_lock;
extern pid_t exit_pid_queue[EXIT_QUEUE_SIZE];
extern uint32_t queue_head;
extern uint32_t queue_tail;

// For controlling the kernel thread
extern struct task_struct * worker_task;
extern struct wait_queue_head worker_wait;
extern int has_new_pid;

extern struct tracepoint * tp_sched_process_exit;

void agnocast_process_exit(void * data, struct task_struct * task);
