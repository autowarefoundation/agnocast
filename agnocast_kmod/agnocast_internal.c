#include "agnocast_internal.h"

int major;
struct class * agnocast_class;
struct device * agnocast_device;

// Locking convention:
//   Only ioctl_ prefixed functions acquire locks. All other internal/static functions are
//   lock-free and rely on callers to hold the appropriate locks. Exceptions are
//   agnocast_process_exit_cleanup, agnocast_exit_free_data, and increment_message_entry_rc, which
//   manage locks directly.
//
// Lock ordering (to prevent deadlocks, always acquire in this order):
//   1. global_htables_rwsem   (this file)
//   2. topic_rwsem            (per-topic, in struct topic_wrapper)
//   3. mempool_lock           (agnocast_memory_allocator.c)
//
// Global rwsem for hashtables (topic_hashtable, proc_info_htable, bridge_htable)
// - Read lock (down_read): when searching hashtables and operating within a topic
// - Write lock (down_write): when adding/removing entries from hashtables
DECLARE_RWSEM(global_htables_rwsem);

DEFINE_HASHTABLE(proc_info_htable, PROC_INFO_HASH_BITS);
DEFINE_HASHTABLE(topic_hashtable, TOPIC_HASH_BITS);
DEFINE_HASHTABLE(bridge_htable, TOPIC_HASH_BITS);

// Ring buffer to hold exited pids.
// EXIT_QUEUE_SIZE (65536) far exceeds mempool_num (default 4096), and only Agnocast PIDs are
// enqueued (via is_agnocast_pid()), each exiting at most once, so the ring buffer cannot overflow.
DEFINE_SPINLOCK(pid_queue_lock);
pid_t exit_pid_queue[EXIT_QUEUE_SIZE];
uint32_t queue_head;
uint32_t queue_tail;

// For controling the kernel thread
struct task_struct * worker_task;
DECLARE_WAIT_QUEUE_HEAD(worker_wait);
int has_new_pid = false;

struct tracepoint * tp_sched_process_exit;

int agnocast_get_size_sub_info_htable(struct topic_wrapper * wrapper)
{
  int count = 0;
  struct subscriber_info * sub_info;
  int bkt_sub_info;
  hash_for_each(wrapper->topic.sub_info_htable, bkt_sub_info, sub_info, node)
  {
    count++;
  }
  return count;
}

int agnocast_get_size_pub_info_htable(struct topic_wrapper * wrapper)
{
  int count = 0;
  struct publisher_info * pub_info;
  int bkt_pub_info;
  hash_for_each(wrapper->topic.pub_info_htable, bkt_pub_info, pub_info, node)
  {
    count++;
  }
  return count;
}

bool agnocast_is_referenced(struct entry_node * en)
{
  return !bitmap_empty(en->referencing_subscribers, MAX_TOPIC_LOCAL_ID);
}

struct process_info * agnocast_find_process_info(const pid_t pid)
{
  struct process_info * proc_info;
  uint32_t hash_val = hash_min(pid, PROC_INFO_HASH_BITS);
  hash_for_each_possible(proc_info_htable, proc_info, node, hash_val)
  {
    if (proc_info->global_pid == pid) {
      return proc_info;
    }
  }

  return NULL;
}

void agnocast_remove_entry_node(struct topic_wrapper * wrapper, struct entry_node * en)
{
  rb_erase(&en->node, &wrapper->topic.entries);
  kfree(en);
}

void agnocast_process_exit(void * data, struct task_struct * task)
{
  // Wait until all threads in the thread group have exited.
  // The thread group leader isn't always the last to exit, so instead of checking
  // pid == tgid, we check that no live threads remain in the group.
  if (atomic_read(&task->signal->live) != 0) return;

  // Skip non-Agnocast PIDs to avoid the full
  // enqueue → wake → dequeue → rwsem pipeline for unrelated exits.
  if (is_agnocast_pid(task->tgid)) agnocast_enqueue_exit_pid(task->tgid);
}
