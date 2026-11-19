// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_internal.h"

int major;
struct class * agnocast_class;
struct device * agnocast_device;

// Only the functions in this file touch these two tables, so that adding, removing or marking an
// entry always updates the namespace's counts with it.
static DEFINE_HASHTABLE(proc_info_htable, PROC_INFO_HASH_BITS);
static DEFINE_HASHTABLE(discovery_agent_htable, DISCOVERY_AGENT_HASH_BITS);
// Serializes adding and removing entries in the two tables above across namespaces. Readers that
// hold no namespace lock walk them under RCU. Innermost: taken under a namespace lock, never the
// other way around.
static DEFINE_SPINLOCK(agnocast_pid_index_lock);

// Every IPC namespace that has registered something. Looked up under RCU on every ioctl; the
// mutex only orders creating and retiring.
static DEFINE_HASHTABLE(ns_htable, AGNOCAST_NS_HASH_BITS);
static DEFINE_MUTEX(ns_registry_lock);

static bool ns_is_empty(struct agnocast_ns * ns)
{
  return ns->alive_proc_num == 0 && ns->agent_num == 0 && hash_empty(ns->topic_hashtable) &&
         hash_empty(ns->bridge_htable) && hash_empty(ns->domain_rule_htable);
}

// Takes a reference, or returns NULL if the namespace is not registered or is being freed.
static struct agnocast_ns * ns_get(const struct ipc_namespace * ipc_ns)
{
  struct agnocast_ns * found = NULL;
  struct agnocast_ns * ns;
  rcu_read_lock();
  hash_for_each_possible_rcu(ns_htable, ns, node, (unsigned long)ipc_ns)
  {
    if (ns->ipc_ns == ipc_ns && refcount_inc_not_zero(&ns->ref)) {
      found = ns;
      break;
    }
  }
  rcu_read_unlock();
  return found;
}

static struct agnocast_ns * ns_get_or_create(const struct ipc_namespace * ipc_ns)
{
  struct agnocast_ns * ns = ns_get(ipc_ns);
  if (ns) return ns;

  // Allocated before taking the mutex: kvzalloc() may sleep in reclaim, and the mutex also orders
  // every other namespace's creation and retirement.
  struct agnocast_ns * new_ns = kvzalloc(sizeof(*new_ns), GFP_KERNEL);
  if (!new_ns) {
    // Another task may have created it meanwhile.
    ns = ns_get(ipc_ns);
    return ns ? ns : ERR_PTR(-ENOMEM);
  }
  // One reference for the registry, one for the caller.
  refcount_set(&new_ns->ref, 2);
  new_ns->ipc_ns = ipc_ns;
  init_rwsem(&new_ns->rwsem);
  hash_init(new_ns->topic_hashtable);
  hash_init(new_ns->bridge_htable);
  hash_init(new_ns->domain_rule_htable);

  mutex_lock(&ns_registry_lock);
  // Another task may have created it since the lookup above.
  ns = ns_get(ipc_ns);
  if (!ns) {
    hash_add_rcu(ns_htable, &new_ns->node, (unsigned long)ipc_ns);
    ns = new_ns;
    new_ns = NULL;
  }
  mutex_unlock(&ns_registry_lock);
  kvfree(new_ns);  // The loser's copy, never published.
  return ns;
}

static void ns_put(struct agnocast_ns * ns)
{
  // A lookup that raced the retirement may still be walking past the node.
  if (refcount_dec_and_test(&ns->ref)) kvfree_rcu(ns, rcu_head);
}

// Takes the lock on the reference `ns` holds, or drops both and returns false if the namespace was
// retired meanwhile.
static bool ns_lock_alive(struct agnocast_ns * ns, const bool write)
{
  if (write) {
    down_write(&ns->rwsem);
  } else {
    down_read(&ns->rwsem);
  }
  if (!ns->dead) return true;

  if (write) {
    up_write(&ns->rwsem);
  } else {
    up_read(&ns->rwsem);
  }
  ns_put(ns);
  return false;
}

// The retry ends: a dead namespace has already left the registry, so the next lookup misses it.
static struct agnocast_ns * ns_lock(
  const struct ipc_namespace * ipc_ns, const bool write, const bool create)
{
  for (;;) {
    struct agnocast_ns * ns = create ? ns_get_or_create(ipc_ns) : ns_get(ipc_ns);
    if (IS_ERR_OR_NULL(ns)) return ns;
    if (ns_lock_alive(ns, write)) return ns;
  }
}

struct agnocast_ns * agnocast_ns_read_lock(const struct ipc_namespace * ipc_ns)
{
  return ns_lock(ipc_ns, false, false);
}

struct agnocast_ns * agnocast_ns_write_lock(const struct ipc_namespace * ipc_ns)
{
  return ns_lock(ipc_ns, true, false);
}

struct agnocast_ns * agnocast_ns_write_lock_or_create(const struct ipc_namespace * ipc_ns)
{
  return ns_lock(ipc_ns, true, true);
}

void agnocast_ns_read_unlock(struct agnocast_ns * ns)
{
  if (!ns) return;
  up_read(&ns->rwsem);
  ns_put(ns);
}

void agnocast_ns_retire_and_unlock(struct agnocast_ns * ns)
{
  lockdep_assert_held_write(&ns->rwsem);
  ns->dead = true;
  mutex_lock(&ns_registry_lock);
  hash_del_rcu(&ns->node);
  mutex_unlock(&ns_registry_lock);
  up_write(&ns->rwsem);
  ns_put(ns);  // the registry's
  ns_put(ns);  // the caller's
}

void agnocast_ns_write_unlock(struct agnocast_ns * ns)
{
  if (!ns) return;
  if (ns_is_empty(ns)) {
    agnocast_ns_retire_and_unlock(ns);
    return;
  }
  up_write(&ns->rwsem);
  ns_put(ns);
}

struct agnocast_ns * agnocast_ns_write_lock_any(void)
{
  for (;;) {
    struct agnocast_ns * ns = NULL;
    struct agnocast_ns * entry;
    int bkt;
    mutex_lock(&ns_registry_lock);
    hash_for_each(ns_htable, bkt, entry, node)
    {
      // Every registered namespace holds the registry's reference, so this cannot fail.
      refcount_inc(&entry->ref);
      ns = entry;
      break;
    }
    mutex_unlock(&ns_registry_lock);
    if (!ns) return NULL;
    if (ns_lock_alive(ns, true)) return ns;
  }
}

#ifdef KUNIT_BUILD
struct agnocast_ns * agnocast_ns_peek(const struct ipc_namespace * ipc_ns)
{
  struct agnocast_ns * found = NULL;
  struct agnocast_ns * ns;
  rcu_read_lock();
  hash_for_each_possible_rcu(ns_htable, ns, node, (unsigned long)ipc_ns)
  {
    if (ns->ipc_ns == ipc_ns) {
      found = ns;
      break;
    }
  }
  rcu_read_unlock();
  return found;
}
#endif

DEFINE_RAW_SPINLOCK(pid_queue_lock);
pid_t exit_pid_queue[EXIT_QUEUE_SIZE];
uint32_t queue_head;
uint32_t queue_tail;

struct task_struct * worker_task;
DECLARE_SWAIT_QUEUE_HEAD(worker_wait);
int has_new_pid;

struct tracepoint * tp_sched_process_exit;

static void pre_handler_subscriber_exit(
  const struct agnocast_ns * ns, struct topic_wrapper * wrapper, const pid_t pid)
{
  // Unlinked as a batch so the notify lists are rebuilt once instead of once per subscriber, which
  // would repeat the same work and land on the same result.
  HLIST_HEAD(leaving);
  struct subscriber_info * sub_info;
  int bkt_sub_info;
  struct hlist_node * tmp_sub_info;
  hash_for_each_safe(wrapper->topic->sub_info_htable, bkt_sub_info, tmp_sub_info, sub_info, node)
  {
    if (sub_info->pid != pid) continue;

    hash_del(&sub_info->node);
    hlist_add_head(&sub_info->node, &leaving);
  }

  if (hlist_empty(&leaving)) return;

  // Before the frees below, so that no list is left pointing at a released context.
  agnocast_rebuild_notify_lists(wrapper);

  struct hlist_node * tmp_leaving;
  hlist_for_each_entry_safe(sub_info, tmp_leaving, &leaving, node)
  {
    const topic_local_id_t subscriber_id = sub_info->id;

    hlist_del(&sub_info->node);
    free_subscriber_info(sub_info);

    if (subscriber_id < 0 || subscriber_id >= MAX_TOPIC_LOCAL_ID) {
      dev_warn(
        agnocast_device, "subscriber_id %d out of range [0, %d). (%s)\n", subscriber_id,
        MAX_TOPIC_LOCAL_ID, __func__);
      continue;
    }

    struct rb_root * root = &wrapper->topic->entries;
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
      hash_for_each_possible(wrapper->topic->pub_info_htable, pub_info, node, hash_val)
      {
        if (pub_info->id == en->publisher_id) {
          const struct process_info * pub_proc_info = agnocast_find_process_info(ns, pub_info->pid);
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
        free_publisher_info(pub_info);
      }
    }
  }
}

static void pre_handler_publisher_exit(struct topic_wrapper * wrapper, const pid_t pid)
{
  struct publisher_info * pub_info;
  int bkt_pub_info;
  struct hlist_node * tmp_pub_info;
  hash_for_each_safe(wrapper->topic->pub_info_htable, bkt_pub_info, tmp_pub_info, pub_info, node)
  {
    if (pub_info->pid != pid) continue;

    const topic_local_id_t publisher_id = pub_info->id;

    // Publisher-side handles do not participate in reference counting, so we don't need
    // to remove publisher references. Just clean up entries that have no subscriber references.
    struct rb_root * root = &wrapper->topic->entries;
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
      free_publisher_info(pub_info);
    }
  }
}

int agnocast_get_size_sub_info_htable(struct topic_wrapper * wrapper)
{
  int count = 0;
  struct subscriber_info * sub_info;
  int bkt_sub_info;
  hash_for_each(wrapper->topic->sub_info_htable, bkt_sub_info, sub_info, node)
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
  hash_for_each(wrapper->topic->pub_info_htable, bkt_pub_info, pub_info, node)
  {
    count++;
  }
  return count;
}

bool agnocast_wrapper_has_domain_endpoints(const struct topic_wrapper * wrapper)
{
  struct publisher_info * pub_info;
  struct subscriber_info * sub_info;
  int bkt;
  hash_for_each(wrapper->topic->pub_info_htable, bkt, pub_info, node)
  {
    if (pub_info->domain_id == wrapper->domain_id) return true;
  }
  hash_for_each(wrapper->topic->sub_info_htable, bkt, sub_info, node)
  {
    if (sub_info->domain_id == wrapper->domain_id) return true;
  }
  return false;
}

void agnocast_release_topic_wrapper(struct topic_wrapper * wrapper)
{
  hash_del(&wrapper->node);
  kfree(wrapper->key);

  // Grouped domains share one topic_struct, so it (and everything it owns) is
  // torn down only when the last referencing wrapper is released. This is the
  // single place that frees the shared rbtree and pub/sub tables.
  if (--wrapper->topic->wrapper_refcnt == 0) {
    struct rb_node * node = rb_first(&wrapper->topic->entries);
    while (node) {
      struct entry_node * en = rb_entry(node, struct entry_node, node);
      node = rb_next(node);
      rb_erase(&en->node, &wrapper->topic->entries);
      kfree(en);
    }

    struct publisher_info * pub_info;
    int bkt_pub;
    struct hlist_node * tmp_pub;
    hash_for_each_safe(wrapper->topic->pub_info_htable, bkt_pub, tmp_pub, pub_info, node)
    {
      hash_del(&pub_info->node);
      free_publisher_info(pub_info);
    }

    struct subscriber_info * sub_info;
    int bkt_sub;
    struct hlist_node * tmp_sub;
    hash_for_each_safe(wrapper->topic->sub_info_htable, bkt_sub, tmp_sub, sub_info, node)
    {
      hash_del(&sub_info->node);
      free_subscriber_info(sub_info);
    }

    kfree(wrapper->topic);
  }
  kfree(wrapper);
}

bool agnocast_is_referenced(struct entry_node * en)
{
  return !bitmap_empty(en->referencing_subscribers, MAX_TOPIC_LOCAL_ID);
}

// Only an entry of `ns` is returned: a pid may name an entry of another namespace (an exited
// entry, or a reused pid), which `ns`'s lock does not keep alive. The walk is under RCU because
// other namespaces add and remove entries concurrently. What keeps the returned entry alive is the
// caller's namespace lock: entries of a namespace are removed only under its lock (write).
struct process_info * agnocast_find_process_info(const struct agnocast_ns * ns, const pid_t pid)
{
  lockdep_assert_held(&ns->rwsem);
  struct process_info * found = NULL;
  struct process_info * proc_info;
  rcu_read_lock();
  hash_for_each_possible_rcu(proc_info_htable, proc_info, node, hash_min(pid, PROC_INFO_HASH_BITS))
  {
    if (proc_info->global_pid == pid) {
      if (proc_info->ipc_ns == ns->ipc_ns) found = proc_info;
      break;
    }
  }
  rcu_read_unlock();
  return found;
}

// Host-wide, unlike agnocast_find_process_info(): a pid identifies one process across namespaces.
bool agnocast_pid_has_process_info(const pid_t pid)
{
  bool found = false;
  struct process_info * proc_info;
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

// Caller holds the namespace lock (write).
void agnocast_add_process_info(struct agnocast_ns * ns, struct process_info * proc_info)
{
  lockdep_assert_held_write(&ns->rwsem);
  INIT_HLIST_NODE(&proc_info->node);
  spin_lock(&agnocast_pid_index_lock);
  hash_add_rcu(
    proc_info_htable, &proc_info->node, hash_min(proc_info->global_pid, PROC_INFO_HASH_BITS));
  spin_unlock(&agnocast_pid_index_lock);
  ns->alive_proc_num++;
}

// Caller holds the namespace lock (write).
void agnocast_remove_process_info(struct agnocast_ns * ns, struct process_info * proc_info)
{
  lockdep_assert_held_write(&ns->rwsem);
  // An exited entry already stopped counting as alive when the exit cleanup marked it.
  if (!proc_info->exited) ns->alive_proc_num--;
  spin_lock(&agnocast_pid_index_lock);
  hash_del_rcu(&proc_info->node);
  spin_unlock(&agnocast_pid_index_lock);
  kfree_rcu(proc_info, rcu_head);
}

// It stops counting as alive now, so a namespace whose unlink daemon died with it can be retired.
// A second call (a stale entry hit again through pid reuse) leaves the count alone.
// Caller holds the namespace lock (write).
static void mark_process_exited(struct agnocast_ns * ns, struct process_info * proc_info)
{
  lockdep_assert_held_write(&ns->rwsem);
  if (!proc_info->exited) ns->alive_proc_num--;
  proc_info->exited = true;
}

// The first exited entry of this namespace, or -1. Exited entries outlive the namespace's state, so
// this does not need it.
pid_t agnocast_find_exited_process(const struct ipc_namespace * ipc_ns, pid_t * ret_local_pid)
{
  pid_t global_pid = -1;
  struct process_info * proc_info;
  int bkt;
  rcu_read_lock();
  hash_for_each_rcu(proc_info_htable, bkt, proc_info, node)
  {
    if (proc_info->ipc_ns != ipc_ns || !proc_info->exited) {
      continue;
    }

    *ret_local_pid = proc_info->local_pid;
    global_pid = proc_info->global_pid;
    break;
  }
  rcu_read_unlock();
  return global_pid;
}

// A bridge manager is per-(ipc_ns, domain): its UDS address carries
// the domain suffix, so each domain needs its own manager. Gate on the domain too,
// otherwise a manager in one domain would suppress spawning in another.
bool agnocast_has_alive_bridge_manager(const struct agnocast_ns * ns, const uint32_t domain_id)
{
  lockdep_assert_held(&ns->rwsem);
  bool found = false;
  struct process_info * proc_info;
  int bkt;
  rcu_read_lock();
  hash_for_each_rcu(proc_info_htable, bkt, proc_info, node)
  {
    if (
      ns->ipc_ns == proc_info->ipc_ns && proc_info->domain_id == domain_id &&
      proc_info->role == PROCESS_ROLE_BRIDGE_MANAGER && !proc_info->exited) {
      found = true;
      break;
    }
  }
  rcu_read_unlock();
  return found;
}

// Namespace-scoped, unlike the bridge manager. Liveness lags the exit worker: a daemon killed
// abruptly still reads as alive until the worker drains its pid, so a process registering in that
// window is not told to spawn a replacement.
bool agnocast_has_alive_unlink_daemon(const struct agnocast_ns * ns)
{
  lockdep_assert_held(&ns->rwsem);
  bool found = false;
  struct process_info * proc_info;
  int bkt;
  rcu_read_lock();
  hash_for_each_rcu(proc_info_htable, bkt, proc_info, node)
  {
    if (ns->ipc_ns == proc_info->ipc_ns && proc_info->role == PROCESS_ROLE_UNLINK_DAEMON) {
      found = true;
      break;
    }
  }
  rcu_read_unlock();
  return found;
}

// Counting the unlink daemon would keep it from ever deciding the namespace is done.
int agnocast_get_process_num_except_unlink_daemon(const struct agnocast_ns * ns)
{
  lockdep_assert_held(&ns->rwsem);
  int count = 0;
  struct process_info * proc_info;
  int bkt_proc_info;
  rcu_read_lock();
  hash_for_each_rcu(proc_info_htable, bkt_proc_info, proc_info, node)
  {
    if (ns->ipc_ns == proc_info->ipc_ns && proc_info->role != PROCESS_ROLE_UNLINK_DAEMON) {
      count++;
    }
  }
  rcu_read_unlock();
  return count;
}

int agnocast_get_process_num_in_domain_except_unlink_daemon(
  const struct agnocast_ns * ns, const uint32_t domain_id)
{
  lockdep_assert_held(&ns->rwsem);
  int count = 0;
  struct process_info * proc_info;
  int bkt_proc_info;
  rcu_read_lock();
  hash_for_each_rcu(proc_info_htable, bkt_proc_info, proc_info, node)
  {
    if (
      ns->ipc_ns == proc_info->ipc_ns && proc_info->domain_id == domain_id &&
      proc_info->role != PROCESS_ROLE_UNLINK_DAEMON) {
      count++;
    }
  }
  rcu_read_unlock();
  return count;
}

// Like agnocast_get_process_num_in_domain_except_unlink_daemon() but also excludes processes that
// have exited and are still pending cleanup. The discovery agent tracks live endpoints, so an
// exited entry that lingers until the unlink daemon drains it must not gate the agent's spawn or
// self-exit. Its domain_id reaches this unvalidated from user space, so the daemon has to be
// excluded by role: a caller passing AGNOCAST_DOMAIN_ID_NONE would otherwise match it.
int agnocast_get_alive_process_num_in_domain_except_unlink_daemon(
  const struct agnocast_ns * ns, const uint32_t domain_id)
{
  lockdep_assert_held(&ns->rwsem);
  int count = 0;
  struct process_info * proc_info;
  int bkt_proc_info;
  rcu_read_lock();
  hash_for_each_rcu(proc_info_htable, bkt_proc_info, proc_info, node)
  {
    if (
      ns->ipc_ns == proc_info->ipc_ns && proc_info->domain_id == domain_id && !proc_info->exited &&
      proc_info->role != PROCESS_ROLE_UNLINK_DAEMON) {
      count++;
    }
  }
  rcu_read_unlock();
  return count;
}

// Caller holds the namespace lock (write).
void agnocast_add_discovery_agent_info(struct agnocast_ns * ns, struct discovery_agent_info * agent)
{
  lockdep_assert_held_write(&ns->rwsem);
  INIT_HLIST_NODE(&agent->node);
  spin_lock(&agnocast_pid_index_lock);
  hash_add_rcu(
    discovery_agent_htable, &agent->node, hash_min(agent->pid, DISCOVERY_AGENT_HASH_BITS));
  spin_unlock(&agnocast_pid_index_lock);
  ns->agent_num++;
}

// Caller holds the namespace lock (read). Scans because the table is keyed by pid.
struct discovery_agent_info * agnocast_find_discovery_agent(
  const struct agnocast_ns * ns, const uint32_t domain_id)
{
  lockdep_assert_held(&ns->rwsem);
  struct discovery_agent_info * found = NULL;
  struct discovery_agent_info * agent;
  int bkt;
  rcu_read_lock();
  hash_for_each_rcu(discovery_agent_htable, bkt, agent, node)
  {
    if (agent->ipc_ns == ns->ipc_ns && agent->domain_id == domain_id) {
      found = agent;
      break;
    }
  }
  rcu_read_unlock();
  return found;
}

// Module unload: every namespace is gone, so no count needs updating.
void agnocast_remove_all_process_info(void)
{
  struct process_info * proc_info;
  int bkt;
  struct hlist_node * tmp;
  spin_lock(&agnocast_pid_index_lock);
  hash_for_each_safe(proc_info_htable, bkt, tmp, proc_info, node)
  {
    hash_del_rcu(&proc_info->node);
    kfree_rcu(proc_info, rcu_head);
  }
  spin_unlock(&agnocast_pid_index_lock);
  // No explicit synchronize_rcu() needed: kfree_rcu() defers freeing until after the grace period.
}

void agnocast_remove_all_discovery_agents(void)
{
  struct discovery_agent_info * agent;
  int bkt;
  struct hlist_node * tmp;
  spin_lock(&agnocast_pid_index_lock);
  hash_for_each_safe(discovery_agent_htable, bkt, tmp, agent, node)
  {
    hash_del_rcu(&agent->node);
    kfree_rcu(agent, rcu_head);
  }
  spin_unlock(&agnocast_pid_index_lock);
}

void agnocast_remove_entry_node(struct topic_wrapper * wrapper, struct entry_node * en)
{
  rb_erase(&en->node, &wrapper->topic->entries);
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
  uint32_t next;

  bool need_wakeup = false;

  raw_spin_lock(&pid_queue_lock);

  next = (queue_tail + 1) & EXIT_QUEUE_MASK;

  if (next != queue_head) {  // queue is not full
    exit_pid_queue[queue_tail] = pid;
    queue_tail = next;
    // Pairs with smp_load_acquire() in the worker thread; ensures the queue write
    // is visible before the worker observes has_new_pid == 1.
    smp_store_release(&has_new_pid, 1);
    need_wakeup = true;
  }

  raw_spin_unlock(&pid_queue_lock);

  if (need_wakeup) {
    swake_up_one(&worker_wait);
  } else {
    dev_warn(
      agnocast_device,
      "exit_pid_queue is full! consider expanding the queue size. (enqueue_exit_pid)\n");
  }
}

// Caller holds the namespace lock (write).
void agnocast_remove_discovery_agent_by_pid(struct agnocast_ns * ns, const pid_t pid)
{
  lockdep_assert_held_write(&ns->rwsem);
  struct discovery_agent_info * agent;
  struct hlist_node * tmp;
  spin_lock(&agnocast_pid_index_lock);
  hash_for_each_possible_safe(
    discovery_agent_htable, agent, tmp, node, hash_min(pid, DISCOVERY_AGENT_HASH_BITS))
  {
    if (agent->pid == pid && agent->ipc_ns == ns->ipc_ns) {
      hash_del_rcu(&agent->node);
      kfree_rcu(agent, rcu_head);
      ns->agent_num--;
      break;
    }
  }
  spin_unlock(&agnocast_pid_index_lock);
}

const struct ipc_namespace * agnocast_find_process_ipc_ns(const pid_t pid)
{
  const struct ipc_namespace * ipc_ns = NULL;
  struct process_info * proc_info;
  rcu_read_lock();
  hash_for_each_possible_rcu(proc_info_htable, proc_info, node, hash_min(pid, PROC_INFO_HASH_BITS))
  {
    if (proc_info->global_pid == pid) {
      ipc_ns = proc_info->ipc_ns;
      break;
    }
  }
  rcu_read_unlock();
  return ipc_ns;
}

const struct ipc_namespace * agnocast_find_discovery_agent_ipc_ns(const pid_t pid)
{
  const struct ipc_namespace * ipc_ns = NULL;
  struct discovery_agent_info * agent;
  rcu_read_lock();
  hash_for_each_possible_rcu(
    discovery_agent_htable, agent, node, hash_min(pid, DISCOVERY_AGENT_HASH_BITS))
  {
    if (agent->pid == pid) {
      ipc_ns = agent->ipc_ns;
      break;
    }
  }
  rcu_read_unlock();
  return ipc_ns;
}

// RCU-protected check: returns true if pid is a registered agnocast process or discovery agent.
bool is_agnocast_pid(const pid_t pid)
{
  return agnocast_find_process_ipc_ns(pid) || agnocast_find_discovery_agent_ipc_ns(pid);
}

// Called from sched_process_exit tracepoint. Not an ioctl function, so we manage locks here
// directly.
void agnocast_process_exit_cleanup(const pid_t pid)
{
  // The discovery agent is tracked outside proc_info and has no proc_info entry, so drain it
  // first, in its own namespace: the same pid may also have a stale exited proc_info left in
  // another namespace, whose unlink daemon died before draining it.
  const struct ipc_namespace * agent_ipc_ns = agnocast_find_discovery_agent_ipc_ns(pid);
  if (agent_ipc_ns) {
    struct agnocast_ns * agent_ns = agnocast_ns_write_lock(agent_ipc_ns);
    if (agent_ns) {
      agnocast_remove_discovery_agent_by_pid(agent_ns, pid);
      agnocast_ns_write_unlock(agent_ns);
    }
  }

  const struct ipc_namespace * ipc_ns = agnocast_find_process_ipc_ns(pid);
  if (!ipc_ns) return;

  struct agnocast_ns * ns = agnocast_ns_write_lock(ipc_ns);
  if (!ns) return;

  // The PID was already filtered by is_agnocast_pid() in the kprobe handler, but the state may
  // have changed between then and now (e.g., the process was already cleaned up by a prior call).
  struct process_info * proc_info = agnocast_find_process_info(ns, pid);
  if (!proc_info) {
    agnocast_ns_write_unlock(ns);
    return;
  }

  // No daemon can drain the daemon's own entry, so for this role registered means alive.
  if (proc_info->role == PROCESS_ROLE_UNLINK_DAEMON) {
    agnocast_remove_process_info(ns, proc_info);
  } else {
    // This proc_info will be removed from proc_info_htable later by the unlink daemon.
    mark_process_exited(ns, proc_info);
  }

  free_memory(pid);

  struct topic_wrapper * wrapper;
  struct hlist_node * node;
  struct hlist_node * tmp;
  int bkt;
  hash_for_each_safe(ns->topic_hashtable, bkt, node, wrapper, node)
  {
    pre_handler_publisher_exit(wrapper, pid);

    pre_handler_subscriber_exit(ns, wrapper, pid);

    // Release this wrapper once its own domain has no endpoints left.
    if (!agnocast_wrapper_has_domain_endpoints(wrapper)) {
      agnocast_release_topic_wrapper(wrapper);
    }
  }

  struct bridge_info * br_info;
  hash_for_each_safe(ns->bridge_htable, bkt, tmp, br_info, node)
  {
    if (br_info->pid == pid) {
      hash_del(&br_info->node);
      kfree(br_info->topic_name);
      kfree(br_info);
    }
  }

  agnocast_ns_write_unlock(ns);

#ifndef KUNIT_BUILD
  dev_info(agnocast_device, "Process (pid=%d) has exited. (%s)\n", pid, __func__);
#endif
}

#ifdef KUNIT_BUILD
int agnocast_get_alive_proc_num(void)
{
  int count = 0;
  struct process_info * proc_info;
  int bkt_proc_info;
  rcu_read_lock();
  hash_for_each_rcu(proc_info_htable, bkt_proc_info, proc_info, node)
  {
    if (!proc_info->exited) {
      count++;
    }
  }
  rcu_read_unlock();
  return count;
}

int agnocast_get_discovery_agent_num(void)
{
  int count = 0;
  struct discovery_agent_info * agent;
  int bkt;
  rcu_read_lock();
  hash_for_each_rcu(discovery_agent_htable, bkt, agent, node)
  {
    count++;
  }
  rcu_read_unlock();
  return count;
}

bool agnocast_is_proc_exited(const pid_t pid)
{
  bool exited = false;
  struct process_info * proc_info;
  rcu_read_lock();
  hash_for_each_possible_rcu(proc_info_htable, proc_info, node, hash_min(pid, PROC_INFO_HASH_BITS))
  {
    if (proc_info->global_pid == pid) {
      exited = proc_info->exited;
      break;
    }
  }
  rcu_read_unlock();
  return exited;
}
#endif
