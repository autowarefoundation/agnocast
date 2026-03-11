#include "agnocast.h"
#include "agnocast_memory_allocator.h"
#include "agnocast_shared.h"

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/hashtable.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mm.h>  // kvzalloc, kvfree
#include <linux/rwsem.h>
#include <linux/slab.h>  // kmalloc, kfree
#include <linux/tracepoint.h>
#include <linux/version.h>

MODULE_LICENSE("Dual BSD/GPL");

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

#ifndef VERSION
#define VERSION "unknown"
#endif

DEFINE_HASHTABLE(proc_info_htable, PROC_INFO_HASH_BITS);

DEFINE_HASHTABLE(topic_hashtable, TOPIC_HASH_BITS);

DEFINE_HASHTABLE(bridge_htable, TOPIC_HASH_BITS);

int get_size_sub_info_htable(struct topic_wrapper * wrapper)
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

int get_size_pub_info_htable(struct topic_wrapper * wrapper)
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

bool is_referenced(struct entry_node * en)
{
  return !bitmap_empty(en->referencing_subscribers, MAX_TOPIC_LOCAL_ID);
}

struct process_info * find_process_info(const pid_t pid)
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

// =========================================
// Initialize and cleanup

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
static char * agnocast_devnode(const struct device * dev, umode_t * mode)
#else
static char * agnocast_devnode(struct device * dev, umode_t * mode)
#endif
{
  if (mode) {
    *mode = 0666;
  }
  return NULL;
}

static struct file_operations fops = {
  .owner = THIS_MODULE,
  .unlocked_ioctl = agnocast_ioctl,
};

void remove_entry_node(struct topic_wrapper * wrapper, struct entry_node * en)
{
  rb_erase(&en->node, &wrapper->topic.entries);
  kfree(en);
}

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
        "(pre_handler_subscriber_exit)\n",
        pid);
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
          "(pre_handler_subscriber_exit)\n");
      }
    }

    hash_del(&sub_info->node);
    kfree(sub_info->node_name);
    kfree(sub_info);

    if (subscriber_id < 0 || subscriber_id >= MAX_TOPIC_LOCAL_ID) {
      dev_warn(
        agnocast_device, "subscriber_id %d out of range [0, %d). (pre_handler_subscriber_exit)\n",
        subscriber_id, MAX_TOPIC_LOCAL_ID);
      continue;
    }

    struct rb_root * root = &wrapper->topic.entries;
    struct rb_node * node = rb_first(root);
    while (node) {
      struct entry_node * en = rb_entry(node, struct entry_node, node);
      node = rb_next(node);

      // The subscriber may not have referenced this entry, so the bit may already be 0.
      clear_bit(subscriber_id, en->referencing_subscribers);

      if (is_referenced(en)) continue;

      bool publisher_exited = false;
      struct publisher_info * pub_info;
      uint32_t hash_val = hash_min(en->publisher_id, PUB_INFO_HASH_BITS);
      hash_for_each_possible(wrapper->topic.pub_info_htable, pub_info, node, hash_val)
      {
        if (pub_info->id == en->publisher_id) {
          const struct process_info * pub_proc_info = find_process_info(pub_info->pid);
          if (!pub_proc_info || pub_proc_info->exited) {
            publisher_exited = true;
          }
          break;
        }
      }
      if (!publisher_exited) continue;

      remove_entry_node(wrapper, en);

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

      if (!is_referenced(en)) {
        pub_info->entries_num--;
        remove_entry_node(wrapper, en);
      }
    }

    if (pub_info->entries_num == 0) {
      hash_del(&pub_info->node);
      kfree(pub_info->node_name);
      kfree(pub_info);
    }
  }
}

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
    if (get_size_pub_info_htable(wrapper) == 0 && get_size_sub_info_htable(wrapper) == 0) {
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
  dev_info(agnocast_device, "Process (pid=%d) has exited. (agnocast_process_exit_cleanup)\n", pid);
#endif
}

static int exit_worker_thread(void * data)
{
  while (!kthread_should_stop()) {
    wait_event_interruptible(worker_wait, smp_load_acquire(&has_new_pid) || kthread_should_stop());

    if (kthread_should_stop()) break;

    // Drain all queued PIDs in a single wake-up cycle
    while (true) {
      pid_t pid;
      unsigned long flags;
      bool got_pid = false;

      spin_lock_irqsave(&pid_queue_lock, flags);

      if (queue_head != queue_tail) {
        pid = exit_pid_queue[queue_head];
        queue_head = (queue_head + 1) & EXIT_QUEUE_MASK;
        got_pid = true;
      }

      if (queue_head == queue_tail) smp_store_release(&has_new_pid, 0);

      spin_unlock_irqrestore(&pid_queue_lock, flags);

      if (!got_pid) break;

      agnocast_process_exit_cleanup(pid);
    }
  }

  return 0;
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

struct tracepoint * tp_sched_process_exit;

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

static void find_sched_process_exit_tp(struct tracepoint * tp, void * priv)
{
  if (strcmp(tp->name, "sched_process_exit") == 0) {
    tp_sched_process_exit = tp;
  }
}

int agnocast_init_device(void)
{
  int ret;

  major = register_chrdev(0, "agnocast" /*device driver name*/, &fops);
  if (major < 0) {
    pr_err("agnocast: register_chrdev failed: %d\n", major);
    return major;
  }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
  agnocast_class = class_create("agnocast_class");
#else
  agnocast_class = class_create(THIS_MODULE, "agnocast_class");
#endif
  if (IS_ERR(agnocast_class)) {
    ret = PTR_ERR(agnocast_class);
    pr_err("agnocast: class_create failed: %d\n", ret);
    goto err_unregister_chrdev;
  }

  agnocast_class->devnode = agnocast_devnode;
  agnocast_device =
    device_create(agnocast_class, NULL, MKDEV(major, 0), NULL, "agnocast" /*file name*/);
  if (IS_ERR(agnocast_device)) {
    ret = PTR_ERR(agnocast_device);
    pr_err("agnocast: device_create failed: %d\n", ret);
    goto err_class_destroy;
  }

  return 0;

err_class_destroy:
  class_destroy(agnocast_class);
err_unregister_chrdev:
  unregister_chrdev(major, "agnocast");
  return ret;
}

int agnocast_init_kthread(void)
{
  queue_head = queue_tail = 0;

  worker_task = kthread_run(exit_worker_thread, NULL, "agnocast_exit_worker");
  if (IS_ERR(worker_task)) {
    dev_warn(agnocast_device, "failed to create kernel thread. (agnocast_init_kthread)\n");
    return PTR_ERR(worker_task);
  }

  return 0;
}

int agnocast_init_exit_hook(void)
{
  int ret;

  for_each_kernel_tracepoint(find_sched_process_exit_tp, NULL);
  if (!tp_sched_process_exit) {
    dev_warn(agnocast_device, "sched_process_exit tracepoint not found\n");
    return -ENOENT;
  }

  ret = tracepoint_probe_register(tp_sched_process_exit, agnocast_process_exit, NULL);
  if (ret) {
    dev_warn(agnocast_device, "tracepoint_probe_register failed: %d\n", ret);
    return ret;
  }

  return 0;
}

#ifndef KUNIT_BUILD
static int agnocast_init(void)
{
  int ret;

  ret = agnocast_init_device();
  if (ret < 0) return ret;

  ret = agnocast_init_kthread();
  if (ret < 0) {
    agnocast_exit_device();
    return ret;
  }

  ret = agnocast_init_exit_hook();
  if (ret < 0) {
    agnocast_exit_kthread();
    agnocast_exit_device();
    return ret;
  }

  ret = init_memory_allocator();
  if (ret < 0) {
    agnocast_exit_exit_hook();
    agnocast_exit_kthread();
    agnocast_exit_device();
    return ret;
  }

  dev_info(agnocast_device, "Agnocast installed! v%s\n", VERSION);
  return 0;
}
#endif

static void remove_all_topics(void)
{
  struct topic_wrapper * wrapper;
  struct hlist_node * tmp;
  int bkt;

  hash_for_each_safe(topic_hashtable, bkt, tmp, wrapper, node)
  {
    struct rb_root * root = &wrapper->topic.entries;
    struct rb_node * node = rb_first(root);
    while (node) {
      struct entry_node * en = rb_entry(node, struct entry_node, node);
      node = rb_next(node);
      remove_entry_node(wrapper, en);
    }

    struct publisher_info * pub_info;
    int bkt_pub_info;
    struct hlist_node * tmp_pub_info;
    hash_for_each_safe(wrapper->topic.pub_info_htable, bkt_pub_info, tmp_pub_info, pub_info, node)
    {
      hash_del(&pub_info->node);
      kfree(pub_info->node_name);
      kfree(pub_info);
    }

    struct subscriber_info * sub_info;
    int bkt_sub_info;
    struct hlist_node * tmp_sub_info;
    hash_for_each_safe(wrapper->topic.sub_info_htable, bkt_sub_info, tmp_sub_info, sub_info, node)
    {
      hash_del(&sub_info->node);
      kfree(sub_info->node_name);
      kfree(sub_info);
    }

    hash_del(&wrapper->node);
    kfree(wrapper->key);
    kfree(wrapper);
  }
}

static void remove_all_process_info(void)
{
  struct process_info * proc_info;
  int bkt;
  struct hlist_node * tmp;
  hash_for_each_safe(proc_info_htable, bkt, tmp, proc_info, node)
  {
    free_exit_subscription_list(proc_info);
    hash_del_rcu(&proc_info->node);
    kfree_rcu(proc_info, rcu_head);
  }
  // No explicit synchronize_rcu() needed: kfree_rcu() defers freeing until after the grace period.
}

static void remove_all_bridge_info(void)
{
  struct bridge_info * br_info;
  int bkt;
  struct hlist_node * tmp;
  hash_for_each_safe(bridge_htable, bkt, tmp, br_info, node)
  {
    hash_del(&br_info->node);
    if (br_info->topic_name) {
      kfree(br_info->topic_name);
    }
    kfree(br_info);
  }
}

// Called during module unload. Not an ioctl function, so we manage locks here directly.
void agnocast_exit_free_data(void)
{
  down_write(&global_htables_rwsem);
  remove_all_topics();
  remove_all_process_info();
  remove_all_bridge_info();
  up_write(&global_htables_rwsem);
}

void agnocast_exit_kthread(void)
{
  wake_up_interruptible(&worker_wait);
  kthread_stop(worker_task);
}

void agnocast_exit_exit_hook(void)
{
  if (tp_sched_process_exit) {
    tracepoint_probe_unregister(tp_sched_process_exit, agnocast_process_exit, NULL);
    tracepoint_synchronize_unregister();
  }
}

void agnocast_exit_device(void)
{
  device_destroy(agnocast_class, MKDEV(major, 0));
  class_destroy(agnocast_class);
  unregister_chrdev(major, "agnocast");
}

#ifndef KUNIT_BUILD
static void agnocast_exit(void)
{
  agnocast_exit_kthread();
  agnocast_exit_exit_hook();

  agnocast_exit_free_data();
  cleanup_memory_allocator();
  dev_info(agnocast_device, "Agnocast removed!\n");
  agnocast_exit_device();
}
#endif

#ifndef KUNIT_BUILD
module_init(agnocast_init) module_exit(agnocast_exit)
#endif
