// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_internal.h"

int major;
struct class * agnocast_class;
struct device * agnocast_device;

DECLARE_RWSEM(global_htables_rwsem);

DEFINE_HASHTABLE(proc_info_htable, PROC_INFO_HASH_BITS);
DEFINE_HASHTABLE(topic_hashtable, TOPIC_HASH_BITS);
DEFINE_HASHTABLE(bridge_htable, TOPIC_HASH_BITS);

DEFINE_SPINLOCK(pid_queue_lock);
pid_t exit_pid_queue[EXIT_QUEUE_SIZE];
uint32_t queue_head;
uint32_t queue_tail;

struct task_struct * worker_task;
DECLARE_WAIT_QUEUE_HEAD(worker_wait);
int has_new_pid = false;

struct tracepoint * tp_sched_process_exit;

static void pre_handler_subscriber_exit(
  struct topic_wrapper * wrapper, const pid_t pid, struct process_info * proc_info)
{
  struct subscriber_info * sub_info;
  int bkt_sub_info;
  struct hlist_node * tmp_sub_info;
  hash_for_each_safe(wrapper->topic.sub_info_htable, bkt_sub_info, tmp_sub_info, sub_info, node)
  {
    if (sub_info->pid != pid) continue;

    const topic_local_id_t subscriber_id = sub_info->id;

    // Save subscription info for daemon cleanup before deleting the subscriber
    if (proc_info->exit_subscription_count >= MAX_SUBSCRIPTION_NUM_PER_PROCESS) {
      dev_warn(
        agnocast_device,
        "exit_subscription_list is full for pid=%d, subscription MQ may leak. "
        "(%s)\n",
        pid, __func__);
    } else {
      struct exit_subscription_entry * exit_entry =
        kmalloc(sizeof(struct exit_subscription_entry), GFP_KERNEL);
      if (exit_entry) {
        strscpy(exit_entry->topic_name, wrapper->key, TOPIC_NAME_BUFFER_SIZE);
        exit_entry->subscriber_id = subscriber_id;
        list_add_tail(&exit_entry->list, &proc_info->exit_subscription_list);
        proc_info->exit_subscription_count++;
      } else {
        dev_warn(
          agnocast_device,
          "kmalloc failed for exit_subscription_entry, subscription MQ may leak. "
          "(%s)\n",
          __func__);
      }
    }

    hash_del(&sub_info->node);
    kfree(sub_info->node_name);
    kfree(sub_info);

    if (subscriber_id < 0 || subscriber_id >= MAX_TOPIC_LOCAL_ID) {
      dev_warn(
        agnocast_device, "subscriber_id %d out of range [0, %d). (%s)\n", subscriber_id,
        MAX_TOPIC_LOCAL_ID, __func__);
      continue;
    }

    struct rb_root * root = &wrapper->topic.entries;
    struct rb_node * node = rb_first(root);
    while (node) {
      struct entry_node * en = rb_entry(node, struct entry_node, node);
      node = rb_next(node);

      // The subscriber may not have referenced this entry, so the bit may already be 0.
      clear_bit(subscriber_id, en->referencing_subscribers);

      if (agnocast_is_referenced(en)) continue;

      bool publisher_exited = false;
      struct publisher_info * pub_info;
      uint32_t hash_val = hash_min(en->publisher_id, PUB_INFO_HASH_BITS);
      hash_for_each_possible(wrapper->topic.pub_info_htable, pub_info, node, hash_val)
      {
        if (pub_info->id == en->publisher_id) {
          const struct process_info * pub_proc_info = agnocast_find_process_info(pub_info->pid);
          if (!pub_proc_info || pub_proc_info->exited) {
            publisher_exited = true;
          }
          break;
        }
      }
      if (!publisher_exited) continue;

      agnocast_remove_entry_node(wrapper, en);

      pub_info->entries_num--;
      if (pub_info->entries_num == 0) {
        hash_del(&pub_info->node);
        kfree(pub_info->node_name);
        kfree(pub_info);
      }
    }
  }
}

static void pre_handler_publisher_exit(struct topic_wrapper * wrapper, const pid_t pid)
{
  struct publisher_info * pub_info;
  int bkt_pub_info;
  struct hlist_node * tmp_pub_info;
  hash_for_each_safe(wrapper->topic.pub_info_htable, bkt_pub_info, tmp_pub_info, pub_info, node)
  {
    if (pub_info->pid != pid) continue;

    const topic_local_id_t publisher_id = pub_info->id;

    // Publisher-side handles do not participate in reference counting, so we don't need
    // to remove publisher references. Just clean up entries that have no subscriber references.
    struct rb_root * root = &wrapper->topic.entries;
    struct rb_node * node = rb_first(root);
    while (node) {
      struct entry_node * en = rb_entry(node, struct entry_node, node);
      node = rb_next(node);

      if (en->publisher_id != publisher_id) continue;

      if (!agnocast_is_referenced(en)) {
        pub_info->entries_num--;
        agnocast_remove_entry_node(wrapper, en);
      }
    }

    if (pub_info->entries_num == 0) {
      hash_del(&pub_info->node);
      kfree(pub_info->node_name);
      kfree(pub_info);
    }
  }
}

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

void agnocast_free_exit_subscription_list(struct process_info * proc_info)
{
  struct exit_subscription_entry * entry;
  struct exit_subscription_entry * tmp_entry;
  list_for_each_entry_safe(entry, tmp_entry, &proc_info->exit_subscription_list, list)
  {
    list_del(&entry->list);
    kfree(entry);
  }
  proc_info->exit_subscription_count = 0;
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

void agnocast_enqueue_exit_pid(const pid_t pid)
{
  unsigned long flags;
  uint32_t next;

  bool need_wakeup = false;

  spin_lock_irqsave(&pid_queue_lock, flags);

  next = (queue_tail + 1) & EXIT_QUEUE_MASK;

  if (next != queue_head) {  // queue is not full
    exit_pid_queue[queue_tail] = pid;
    queue_tail = next;
    smp_store_release(&has_new_pid, 1);
    need_wakeup = true;
  }

  spin_unlock_irqrestore(&pid_queue_lock, flags);

  if (need_wakeup) {
    wake_up_interruptible(&worker_wait);
  } else {
    dev_warn(
      agnocast_device,
      "exit_pid_queue is full! consider expanding the queue size. (enqueue_exit_pid)\n");
  }
}

// RCU-protected check: returns true if pid is registered in agnocast.
bool is_agnocast_pid(const pid_t pid)
{
  struct process_info * proc_info;
  bool found = false;
  rcu_read_lock();
  hash_for_each_possible_rcu(proc_info_htable, proc_info, node, hash_min(pid, PROC_INFO_HASH_BITS))
  {
    if (proc_info->global_pid == pid) {
      found = true;
      break;
    }
  }
  rcu_read_unlock();
  return found;
}

// Called from sched_process_exit tracepoint. Not an ioctl function, so we manage locks here
// directly.
void agnocast_process_exit_cleanup(const pid_t pid)
{
  down_write(&global_htables_rwsem);

  // The PID was already filtered by is_agnocast_pid() in the kprobe handler, but the state may
  // have changed between then and now (e.g., the process was already cleaned up by a prior call).
  struct process_info * proc_info = NULL;
  uint32_t hash_val = hash_min(pid, PROC_INFO_HASH_BITS);
  hash_for_each_possible(proc_info_htable, proc_info, node, hash_val)
  {
    if (proc_info->global_pid == pid) {
      break;
    }
  }

  if (!proc_info || proc_info->global_pid != pid) {
    up_write(&global_htables_rwsem);
    return;
  }

  // This proc_info will be removed from proc_info_htable later by the unlink daemon.
  proc_info->exited = true;

  free_memory(pid);

  struct topic_wrapper * wrapper;
  struct hlist_node * node;
  struct hlist_node * tmp;
  int bkt;
  hash_for_each_safe(topic_hashtable, bkt, node, wrapper, node)
  {
    pre_handler_publisher_exit(wrapper, pid);

    pre_handler_subscriber_exit(wrapper, pid, proc_info);

    // Check if we can release the topic_wrapper
    if (
      agnocast_get_size_pub_info_htable(wrapper) == 0 &&
      agnocast_get_size_sub_info_htable(wrapper) == 0) {
      hash_del(&wrapper->node);
      if (wrapper->key) {
        kfree(wrapper->key);
      }
      kfree(wrapper);
    }
  }

  struct bridge_info * br_info;
  hash_for_each_safe(bridge_htable, bkt, tmp, br_info, node)
  {
    if (br_info->pid == pid) {
      hash_del(&br_info->node);
      if (br_info->topic_name) {
        kfree(br_info->topic_name);
      }
      kfree(br_info);
    }
  }

  up_write(&global_htables_rwsem);

#ifndef KUNIT_BUILD
  dev_info(agnocast_device, "Process (pid=%d) has exited. (%s)\n", pid, __func__);
#endif
}
