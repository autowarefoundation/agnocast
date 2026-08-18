// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_internal.h"

#ifndef KUNIT_BUILD
// Kernel module uses global PIDs, whereas user-space and the interface between them use local PIDs.
// Thus, PIDs must be converted from global to local before they are passed from kernel to user.
static pid_t convert_pid_to_local(pid_t global_pid)
{
  rcu_read_lock();

  struct pid * pid_struct = find_pid_ns(global_pid, &init_pid_ns);
  if (!pid_struct) {
    dev_warn(
      agnocast_device, "Cannot convert global pid=%d to local pid (%s)\n", global_pid, __func__);
    rcu_read_unlock();
    return -1;
  }

  const pid_t local_pid = pid_vnr(pid_struct);

  rcu_read_unlock();

  return local_pid;
}
#endif

static bool ipc_eq(const struct ipc_namespace * ipc_ns1, const struct ipc_namespace * ipc_ns2)
{
  return ipc_ns1 == ipc_ns2;
}

static unsigned long get_topic_hash(const char * str)
{
  unsigned long hash = full_name_hash(NULL /*namespace*/, str, strlen(str));
  return hash_min(hash, TOPIC_HASH_BITS);
}

static struct topic_wrapper * find_topic(
  const char * topic_name, const struct ipc_namespace * ipc_ns, uint32_t domain_id)
{
  struct topic_wrapper * entry;
  unsigned long hash_val = get_topic_hash(topic_name);

  hash_for_each_possible(topic_hashtable, entry, node, hash_val)
  {
    if (
      ipc_eq(entry->ipc_ns, ipc_ns) && entry->domain_id == domain_id &&
      strcmp(entry->key, topic_name) == 0)
      return entry;
  }

  return NULL;
}

// A process operates in exactly one ROS_DOMAIN_ID, recorded in its process_info.
// Returns the default domain 0 if the process is not registered.
// Caller must hold global_htables_rwsem (same as agnocast_find_process_info).
static uint32_t get_process_domain_id(pid_t pid)
{
  struct process_info * proc_info = agnocast_find_process_info(pid);
  return proc_info ? proc_info->domain_id : 0;
}

static uint32_t get_current_domain_id(void)
{
  return get_process_domain_id(current->tgid);
}

// find_topic variant for operations on behalf of the calling process: matches in
// the caller's own domain. Caller holds global_htables_rwsem.
static struct topic_wrapper * find_topic_for_current(
  const char * topic_name, const struct ipc_namespace * ipc_ns)
{
  return find_topic(topic_name, ipc_ns, get_current_domain_id());
}

static struct domain_bridge_rule * find_domain_rule(
  const char * topic_name, const struct ipc_namespace * ipc_ns, uint32_t domain_id);

// If a domain bridge rule pairs this cell (topic_name, domain_id) with another
// whose wrapper already exists, return that partner's topic_struct so the new
// wrapper can share it. The partner may use a different name (rename), so look it
// up by the rule's name for the partner domain.
static struct topic_struct * find_grouped_topic_struct(
  const char * topic_name, const struct ipc_namespace * ipc_ns, uint32_t domain_id)
{
  const struct domain_bridge_rule * rule = find_domain_rule(topic_name, ipc_ns, domain_id);
  if (!rule) return NULL;

  uint32_t partner_domain;
  const char * partner_name;
  if (domain_id == rule->domain_a) {
    partner_domain = rule->domain_b;
    partner_name = rule->topic_name_b;
  } else if (domain_id == rule->domain_b) {
    partner_domain = rule->domain_a;
    partner_name = rule->topic_name_a;
  } else {
    return NULL;
  }

  struct topic_wrapper * partner = find_topic(partner_name, ipc_ns, partner_domain);
  return partner ? partner->topic : NULL;
}

// Whether a publication in pub_domain may be delivered to a subscriber in
// sub_domain within this topic_struct. Same domain is always allowed; crossing
// domains requires a rule permitting that direction (from_domain -> to_domain).
static bool domain_delivery_allowed(
  const struct topic_struct * topic, uint32_t pub_domain, uint32_t sub_domain)
{
  if (pub_domain == sub_domain) return true;

  const struct domain_bridge_rule * rule = topic->rule;
  if (!rule) return false;
  if (pub_domain == rule->domain_a && sub_domain == rule->domain_b) return rule->a_to_b;
  if (pub_domain == rule->domain_b && sub_domain == rule->domain_a) return rule->b_to_a;
  return false;
}

static int add_topic(
  const char * topic_name, const struct ipc_namespace * ipc_ns, uint32_t domain_id,
  struct topic_wrapper ** wrapper)
{
  *wrapper = find_topic(topic_name, ipc_ns, domain_id);
  if (*wrapper) {
    return 0;
  }

  *wrapper = kmalloc(sizeof(struct topic_wrapper), GFP_KERNEL);
  if (!*wrapper) {
    dev_warn(
      agnocast_device, "Failed to add a new topic (topic_name=%s) by kmalloc. (%s)\n", topic_name,
      __func__);
    return -ENOMEM;
  }

  (*wrapper)->key = kstrdup(topic_name, GFP_KERNEL);
  if (!(*wrapper)->key) {
    dev_warn(
      agnocast_device, "Failed to add a new topic (topic_name=%s) by kstrdup. (%s)\n", topic_name,
      __func__);
    kfree(*wrapper);
    return -ENOMEM;
  }
  (*wrapper)->ipc_ns = ipc_ns;
  (*wrapper)->domain_id = domain_id;

  struct topic_struct * grouped = find_grouped_topic_struct(topic_name, ipc_ns, domain_id);
  if (grouped) {
    (*wrapper)->topic = grouped;
    grouped->wrapper_refcnt++;
  } else {
    (*wrapper)->topic = kmalloc(sizeof(struct topic_struct), GFP_KERNEL);
    if (!(*wrapper)->topic) {
      dev_warn(
        agnocast_device,
        "Failed to allocate topic_struct for a new topic (topic_name=%s) by kmalloc. (%s)\n",
        topic_name, __func__);
      kfree((*wrapper)->key);
      kfree(*wrapper);
      return -ENOMEM;
    }
    init_rwsem(&(*wrapper)->topic->rwsem);
    (*wrapper)->topic->entries = RB_ROOT;
    hash_init((*wrapper)->topic->pub_info_htable);
    hash_init((*wrapper)->topic->sub_info_htable);
    (*wrapper)->topic->current_pubsub_id = 0;
    (*wrapper)->topic->current_entry_id = 0;
    (*wrapper)->topic->ros2_subscriber_num = 0;
    (*wrapper)->topic->ros2_publisher_num = 0;
    (*wrapper)->topic->wrapper_refcnt = 1;
    (*wrapper)->topic->rule = find_domain_rule(topic_name, ipc_ns, domain_id);
  }
  hash_add(topic_hashtable, &(*wrapper)->node, get_topic_hash(topic_name));

  dev_dbg(agnocast_device, "Topic (topic_name=%s) added. (%s)\n", topic_name, __func__);

  return 0;
}

// Returns true for parameter service topics, whose registration logs are
// suppressed to avoid flooding the kernel log when many agnocast nodes start up.
static bool is_parameter_service_topic(const char * key)
{
  if (strncmp(key, "/AGNOCAST_SRV_", 14) != 0) return false;
  return strstr(key, "/get_parameters") || strstr(key, "/set_parameters") ||
         strstr(key, "/get_parameter_types") || strstr(key, "/describe_parameters") ||
         strstr(key, "/list_parameters");
}

static struct subscriber_info * find_subscriber_info(
  const struct topic_wrapper * wrapper, const topic_local_id_t subscriber_id)
{
  struct subscriber_info * info;
  uint32_t hash_val = hash_min(subscriber_id, SUB_INFO_HASH_BITS);
  hash_for_each_possible(wrapper->topic->sub_info_htable, info, node, hash_val)
  {
    if (info->id == subscriber_id) {
      return info;
    }
  }

  return NULL;
}

// Take subs are excluded, so they cannot inflate every publisher's array.
static uint32_t notifiable_subscriber_num(const struct topic_wrapper * wrapper)
{
  uint32_t num = 0;
  struct subscriber_info * sub_info;
  int bkt_sub_info;
  hash_for_each(wrapper->topic->sub_info_htable, bkt_sub_info, sub_info, node)
  {
    if (sub_info->notify_ctx) num++;
  }
  return num;
}

// Caller holds global_htables_rwsem (write), so the swap below cannot race a publish: those hold
// the read side for their whole duration. The current list is carried across, so that a caller who
// fails partway through reserving has invalidated no one's list and owes no undo.
static int reserve_notify_ctxs(struct publisher_info * pub_info, const uint32_t needed)
{
  if (needed <= pub_info->notify_capacity) return 0;

  uint32_t new_capacity =
    pub_info->notify_capacity ? pub_info->notify_capacity * 2 : NOTIFY_CTXS_MIN_CAPACITY;
  if (new_capacity > MAX_SUBSCRIBER_NUM) new_capacity = MAX_SUBSCRIBER_NUM;
  // Last, so that the cap above can never leave the list short of what the fill needs.
  if (new_capacity < needed) new_capacity = needed;

  struct eventfd_ctx ** new_ctxs = kvmalloc_array(new_capacity, sizeof(*new_ctxs), GFP_KERNEL);
  if (!new_ctxs) return -ENOMEM;

  if (pub_info->notify_ctxs)
    memcpy(new_ctxs, pub_info->notify_ctxs, pub_info->notify_num * sizeof(*new_ctxs));
  kvfree(pub_info->notify_ctxs);
  pub_info->notify_ctxs = new_ctxs;
  pub_info->notify_capacity = new_capacity;
  return 0;
}

// The fallible half of a rebuild, split out so callers can do it before the change they are about
// to make, while backing out is still free.
static int reserve_all_notify_ctxs(struct topic_wrapper * wrapper, const uint32_t needed)
{
  struct publisher_info * pub_info;
  int bkt_pub_info;
  hash_for_each(wrapper->topic->pub_info_htable, bkt_pub_info, pub_info, node)
  {
    int ret = reserve_notify_ctxs(pub_info, needed);
    if (ret < 0) return ret;
  }
  return 0;
}

// None of the filters below depends on the message, so evaluating them here leaves publish with
// nothing to do but signal. Infallible: the caller reserved the capacity.
static void rebuild_notify_list(struct topic_wrapper * wrapper, struct publisher_info * pub_info)
{
  uint32_t notify_num = 0;
  struct subscriber_info * sub_info;
  int bkt_sub_info;
  hash_for_each(wrapper->topic->sub_info_htable, bkt_sub_info, sub_info, node)
  {
    // NULL exactly for take subs, which poll instead of being woken.
    if (!sub_info->notify_ctx) continue;
    if (!domain_delivery_allowed(wrapper->topic, pub_info->domain_id, sub_info->domain_id))
      continue;
    if (sub_info->ignore_local_publications && sub_info->pid == pub_info->pid) continue;

    if (WARN_ON_ONCE(notify_num == pub_info->notify_capacity)) break;
    pub_info->notify_ctxs[notify_num++] = sub_info->notify_ctx;
  }
  pub_info->notify_num = notify_num;
}

// Caller holds global_htables_rwsem (write). Infallible, so it can run once a change is already
// visible: only a join needs to grow a list, and it reserves upfront.
static void rebuild_all_notify_lists(struct topic_wrapper * wrapper)
{
  struct publisher_info * pub_info;
  int bkt_pub_info;
  hash_for_each(wrapper->topic->pub_info_htable, bkt_pub_info, pub_info, node)
  {
    rebuild_notify_list(wrapper, pub_info);
  }
}

void agnocast_rebuild_notify_lists(struct topic_wrapper * wrapper)
{
  rebuild_all_notify_lists(wrapper);
}

void agnocast_unlink_subscriber_info(
  struct topic_wrapper * wrapper, struct subscriber_info * sub_info)
{
  const bool was_notifiable = sub_info->notify_ctx;
  hash_del(&sub_info->node);

  // Take subs are in no list, so rebuilding for one would land on the same result. Otherwise this
  // must precede the release below, so that no list is left pointing at a freed context.
  if (was_notifiable) rebuild_all_notify_lists(wrapper);

  free_subscriber_info(sub_info);
}

static int insert_subscriber_info(
  struct topic_wrapper * wrapper, const char * node_name, const pid_t subscriber_pid,
  const uint32_t qos_depth, const bool qos_is_transient_local, const bool qos_is_reliable,
  const bool is_take_sub, bool ignore_local_publications, const bool is_bridge,
  struct eventfd_ctx * notify_ctx, struct subscriber_info ** new_info)
{
  // rebuild_notify_list() skips take subs by testing notify_ctx alone, so the two must agree.
  WARN_ON_ONCE(is_take_sub != (notify_ctx == NULL));

  int count = agnocast_get_size_sub_info_htable(wrapper);
  if (count == MAX_SUBSCRIBER_NUM) {
    dev_warn(
      agnocast_device,
      "The number of subscribers for the topic (topic_name=%s) reached the upper "
      "bound (MAX_SUBSCRIBER_NUM=%d), so no new subscriber can be "
      "added. (%s)\n",
      wrapper->key, MAX_SUBSCRIBER_NUM, __func__);
    return -ENOBUFS;
  }

  if (wrapper->topic->current_pubsub_id >= MAX_TOPIC_LOCAL_ID) {
    dev_warn(
      agnocast_device,
      "current_pubsub_id (%d) for the topic (topic_name=%s) reached the upper "
      "bound (MAX_TOPIC_LOCAL_ID=%d), so no new subscriber can be "
      "added. (%s)\n",
      wrapper->topic->current_pubsub_id, wrapper->key, MAX_TOPIC_LOCAL_ID, __func__);
    return -ENOSPC;
  }

  *new_info = kmalloc(sizeof(struct subscriber_info), GFP_KERNEL);
  if (!*new_info) {
    return -ENOMEM;
  }

  char * node_name_copy = kstrdup(node_name, GFP_KERNEL);
  if (!node_name_copy) {
    kfree(*new_info);
    return -ENOMEM;
  }

  // Last failure point: reserving while nothing is committed yet is what lets the rebuild below be
  // infallible, and leaves the id counters untouched on failure.
  if (notify_ctx) {
    int reserve_ret = reserve_all_notify_ctxs(wrapper, notifiable_subscriber_num(wrapper) + 1);
    if (reserve_ret < 0) {
      kfree(node_name_copy);
      kfree(*new_info);
      // The caller releases notify_ctx on the error path, so it must not be released here.
      return reserve_ret;
    }
  }

  const topic_local_id_t new_id = wrapper->topic->current_pubsub_id;
  wrapper->topic->current_pubsub_id++;

  (*new_info)->id = new_id;
  (*new_info)->domain_id = wrapper->domain_id;
  (*new_info)->pid = subscriber_pid;
  (*new_info)->qos_depth = qos_depth;
  (*new_info)->qos_is_transient_local = qos_is_transient_local;
  (*new_info)->qos_is_reliable = qos_is_reliable;
  if (qos_is_transient_local) {
    (*new_info)->latest_received_entry_id = -1;
  } else {
    (*new_info)->latest_received_entry_id = wrapper->topic->current_entry_id++;
  }
  (*new_info)->node_name = node_name_copy;
  (*new_info)->is_take_sub = is_take_sub;
  (*new_info)->ignore_local_publications = ignore_local_publications;
  (*new_info)->need_mmap_update = true;
  (*new_info)->is_bridge = is_bridge;
  (*new_info)->notify_ctx = notify_ctx;
  INIT_HLIST_NODE(&(*new_info)->node);
  uint32_t hash_val = hash_min(new_id, SUB_INFO_HASH_BITS);
  hash_add(wrapper->topic->sub_info_htable, &(*new_info)->node, hash_val);

  if (notify_ctx) rebuild_all_notify_lists(wrapper);

  if (!is_parameter_service_topic(wrapper->key)) {
    dev_info(
      agnocast_device,
      "Subscriber (topic_local_id=%d, pid=%d, node_name=%s) is added to the topic (topic_name=%s). "
      "(%s)\n",
      new_id, subscriber_pid, node_name, wrapper->key, __func__);
  }

  // Check if the topic has any volatile publishers.
  if (qos_is_transient_local) {
    struct publisher_info * pub_info;
    int bkt_pub_info;
    hash_for_each(wrapper->topic->pub_info_htable, bkt_pub_info, pub_info, node)
    {
      if (!pub_info->qos_is_transient_local) {
        dev_warn(
          agnocast_device,
          "Incompatible QoS is set for the topic (topic_name=%s): subscriber is transient local "
          "but publisher is volatile. (%s)\n",
          wrapper->key, __func__);
        break;
      }
    }
  }

  return 0;
}

static struct publisher_info * find_publisher_info(
  const struct topic_wrapper * wrapper, const topic_local_id_t publisher_id)
{
  struct publisher_info * info;
  uint32_t hash_val = hash_min(publisher_id, PUB_INFO_HASH_BITS);
  hash_for_each_possible(wrapper->topic->pub_info_htable, info, node, hash_val)
  {
    if (info->id == publisher_id) {
      return info;
    }
  }

  return NULL;
}

static int insert_publisher_info(
  struct topic_wrapper * wrapper, const char * node_name, const pid_t publisher_pid,
  const uint32_t qos_depth, const bool qos_is_transient_local, const bool is_bridge,
  struct publisher_info ** new_info)
{
  int count = agnocast_get_size_pub_info_htable(wrapper);
  if (count == MAX_PUBLISHER_NUM) {
    dev_warn(
      agnocast_device,
      "The number of publishers for the topic (topic_name=%s) reached the upper "
      "bound (MAX_PUBLISHER_NUM=%d), so no new publisher can be "
      "added. (%s)\n",
      wrapper->key, MAX_PUBLISHER_NUM, __func__);
    return -ENOBUFS;
  }

  if (wrapper->topic->current_pubsub_id >= MAX_TOPIC_LOCAL_ID) {
    dev_warn(
      agnocast_device,
      "current_pubsub_id (%d) for the topic (topic_name=%s) reached the upper "
      "bound (MAX_TOPIC_LOCAL_ID=%d), so no new publisher can be "
      "added. (%s)\n",
      wrapper->topic->current_pubsub_id, wrapper->key, MAX_TOPIC_LOCAL_ID, __func__);
    return -ENOSPC;
  }

  *new_info = kmalloc(sizeof(struct publisher_info), GFP_KERNEL);
  if (!*new_info) {
    return -ENOMEM;
  }

  char * node_name_copy = kstrdup(node_name, GFP_KERNEL);
  if (!node_name_copy) {
    kfree(*new_info);
    return -ENOMEM;
  }

  const topic_local_id_t new_id = wrapper->topic->current_pubsub_id;
  wrapper->topic->current_pubsub_id++;

  (*new_info)->id = new_id;
  (*new_info)->domain_id = wrapper->domain_id;
  (*new_info)->pid = publisher_pid;
  (*new_info)->node_name = node_name_copy;
  (*new_info)->qos_depth = qos_depth;
  (*new_info)->qos_is_transient_local = qos_is_transient_local;
  (*new_info)->entries_num = 0;
  (*new_info)->is_bridge = is_bridge;
  (*new_info)->notify_ctxs = NULL;
  (*new_info)->notify_num = 0;
  (*new_info)->notify_capacity = 0;
  INIT_HLIST_NODE(&(*new_info)->node);

  // Before hash_add, so a failure needs no more undo than the free below: the fill reads only the
  // subscriber table and the fields set above, so the publisher need not be linked yet.
  int ret = reserve_notify_ctxs(*new_info, notifiable_subscriber_num(wrapper));
  if (ret < 0) {
    free_publisher_info(*new_info);
    return ret;
  }
  rebuild_notify_list(wrapper, *new_info);

  uint32_t hash_val = hash_min(new_id, PUB_INFO_HASH_BITS);
  hash_add(wrapper->topic->pub_info_htable, &(*new_info)->node, hash_val);

  if (!is_parameter_service_topic(wrapper->key)) {
    dev_info(
      agnocast_device,
      "Publisher (topic_local_id=%d, pid=%d, node_name=%s) is added to the topic (topic_name=%s). "
      "(%s)\n",
      new_id, publisher_pid, node_name, wrapper->key, __func__);
  }

  // Check if the topic has any transient local subscribers.
  if (!qos_is_transient_local) {
    struct subscriber_info * sub_info;
    int bkt_sub_info;
    hash_for_each(wrapper->topic->sub_info_htable, bkt_sub_info, sub_info, node)
    {
      if (sub_info->qos_is_transient_local) {
        dev_warn(
          agnocast_device,
          "Incompatible QoS is set for the topic (topic_name=%s): publisher is volatile "
          "but subscriber is transient local. (%s)\n",
          wrapper->key, __func__);
        break;
      }
    }
  }

  return 0;
}

// Add subscriber reference to entry (set boolean flag to true).
// Called when subscriber first receives/takes the message.
//
// Returns -EALREADY if this subscriber already held a reference.
// Pass allow_existing=true when that is an expected outcome (a repeated take of the same entry)
// to suppress the warning. The test and the set are deliberately one atomic operation
// rather than a test_bit followed by a conditional set:
// agnocast_ioctl_release_message_entry_reference holds only the topic read lock, so it can clear
// this bit at any point and a separate test would already be stale by the time the set ran.
static int add_subscriber_reference(
  struct entry_node * en, const topic_local_id_t id, const bool allow_existing)
{
  if (id < 0 || id >= MAX_TOPIC_LOCAL_ID) {
    pr_err("subscriber id %d out of range [0, %d). (%s)\n", id, MAX_TOPIC_LOCAL_ID, __func__);
    return -EINVAL;
  }

  if (test_and_set_bit(id, en->referencing_subscribers)) {
    if (!allow_existing) {
      dev_warn(
        agnocast_device,
        "subscriber id=%d already holds a reference for entry_id=%lld. "
        "(%s)\n",
        id, en->entry_id, __func__);
    }
    return -EALREADY;
  }
  return 0;
}

static struct entry_node * find_message_entry(
  struct topic_wrapper * wrapper, const int64_t entry_id)
{
  struct rb_root * root = &wrapper->topic->entries;
  struct rb_node ** new = &(root->rb_node);

  while (*new) {
    struct entry_node * this = container_of(*new, struct entry_node, node);

    if (entry_id < this->entry_id) {
      new = &((*new)->rb_left);
    } else if (entry_id > this->entry_id) {
      new = &((*new)->rb_right);
    } else {
      return this;
    }
  }

  return NULL;
}

// Forward declaration
static int get_process_num(const struct ipc_namespace * ipc_ns);
static int get_process_num_in_domain(const struct ipc_namespace * ipc_ns, const uint32_t domain_id);
static int get_alive_process_num_in_domain(
  const struct ipc_namespace * ipc_ns, const uint32_t domain_id);

// Release subscriber reference from message entry (set boolean flag to false).
// Called when subscriber's last ipc_shared_ptr reference is destroyed.
int agnocast_ioctl_release_message_entry_reference(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const topic_local_id_t pubsub_id,
  const int64_t entry_id)
{
  int ret = 0;

  down_read(&global_htables_rwsem);

  struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (!wrapper) {
    dev_warn(agnocast_device, "Topic (topic_name=%s) not found. (%s)\n", topic_name, __func__);
    ret = -EINVAL;
    goto unlock_only_global;
  }

  down_read(&wrapper->topic->rwsem);

  struct entry_node * en = find_message_entry(wrapper, entry_id);
  if (!en) {
    dev_warn(
      agnocast_device,
      "Message entry (topic_name=%s entry_id=%lld) not found. "
      "(%s)\n",
      topic_name, entry_id, __func__);
    ret = -EINVAL;
    goto unlock_all;
  }

  if (pubsub_id < 0 || pubsub_id >= MAX_TOPIC_LOCAL_ID) {
    dev_warn(
      agnocast_device, "pubsub_id %d out of range [0, %d). (%s)\n", pubsub_id, MAX_TOPIC_LOCAL_ID,
      __func__);
    ret = -EINVAL;
    goto unlock_all;
  }

  if (!test_and_clear_bit(pubsub_id, en->referencing_subscribers)) {
    dev_warn(
      agnocast_device,
      "pubsub_id %d does not hold a reference for entry (topic_name=%s entry_id=%lld). "
      "(%s)\n",
      pubsub_id, topic_name, entry_id, __func__);
    ret = -EINVAL;
    goto unlock_all;
  }

unlock_all:
  up_read(&wrapper->topic->rwsem);
unlock_only_global:
  up_read(&global_htables_rwsem);
  return ret;
}

static int insert_message_entry(
  struct topic_wrapper * wrapper, struct publisher_info * pub_info, uint64_t msg_virtual_address,
  union ioctl_publish_msg_args * ioctl_ret)
{
  struct entry_node * new_node = kmalloc(sizeof(struct entry_node), GFP_KERNEL);
  if (!new_node) {
    return -ENOMEM;
  }

  new_node->entry_id = wrapper->topic->current_entry_id++;
  new_node->publisher_id = pub_info->id;
  new_node->msg_virtual_address = msg_virtual_address;
  // Publisher-side handles do not participate in reference counting.
  // Subscribers will add their references when they receive/take the message.
  bitmap_zero(new_node->referencing_subscribers, MAX_TOPIC_LOCAL_ID);

  struct rb_root * root = &wrapper->topic->entries;
  struct rb_node ** new = &(root->rb_node);
  struct rb_node * parent = NULL;

  while (*new) {
    const struct entry_node * this = container_of(*new, struct entry_node, node);
    parent = *new;

    if (new_node->entry_id > this->entry_id) {
      new = &((*new)->rb_right);
    } else {
      dev_warn(
        agnocast_device,
        "Unreachable: New message entry (entry_id=%lld) does not have the largest entry_id in the "
        "topic (topic_name=%s). (%s)\n",
        new_node->entry_id, wrapper->key, __func__);
      kfree(new_node);
      return -ECANCELED;
    }
  }

  rb_link_node(&new_node->node, parent, new);
  rb_insert_color(&new_node->node, root);

  pub_info->entries_num++;

  dev_dbg(
    agnocast_device,
    "Insert a message entry (topic_name=%s entry_id=%lld msg_virtual_address=%lld). "
    "(%s)\n",
    wrapper->key, new_node->entry_id, msg_virtual_address, __func__);

  ioctl_ret->ret_entry_id = new_node->entry_id;

  return 0;
}

static int set_publisher_shm_info(
  const struct topic_wrapper * wrapper, const pid_t subscriber_pid,
  struct publisher_shm_info * pub_shm_infos, uint32_t pub_shm_infos_size,
  uint32_t * ret_pub_shm_num)
{
  uint32_t publisher_num = 0;
  struct publisher_info * pub_info;
  int bkt;
  hash_for_each(wrapper->topic->pub_info_htable, bkt, pub_info, node)
  {
    if (subscriber_pid == pub_info->pid) {
      continue;
    }

    // A subscriber only reads from publishers that deliver to it; a one-way
    // bridge rule can exclude opposite-domain publishers, so skip mapping them.
    if (!domain_delivery_allowed(wrapper->topic, pub_info->domain_id, wrapper->domain_id)) {
      continue;
    }

    const struct process_info * proc_info = agnocast_find_process_info(pub_info->pid);
    if (!proc_info || proc_info->exited) {
      continue;
    }

    int ret = reference_memory(proc_info->mempool_entry, subscriber_pid);
    if (ret < 0) {
      if (ret == -EEXIST) {
        continue;
      } else if (ret == -ENOMEM) {
        dev_warn(
          agnocast_device,
          "Failed to allocate memory for mapping from pid=%d to process (pid=%d)'s memory pool. "
          "(%s)\n",
          subscriber_pid, pub_info->pid, __func__);
        return ret;
      } else {
        dev_warn(
          agnocast_device,
          "Unreachable: process (pid=%d) failed to reference memory of (pid=%d). "
          "(%s)\n",
          subscriber_pid, pub_info->pid, __func__);
        return ret;
      }
    }

    if (publisher_num == pub_shm_infos_size) {
      dev_warn(
        agnocast_device,
        "The number of publisher processes to be mapped exceeds the buffer size "
        "(pub_shm_infos_size=%u, topic_name=%s, subscriber_pid=%d). (%s)\n",
        pub_shm_infos_size, wrapper->key, subscriber_pid, __func__);
      return -ENOBUFS;
    }

#ifndef KUNIT_BUILD
    const pid_t local_pid = convert_pid_to_local(pub_info->pid);
    if (local_pid == -1) {
      return -ESRCH;
    }
    pub_shm_infos[publisher_num].pid = local_pid;
#else
    pub_shm_infos[publisher_num].pid = pub_info->pid;
#endif

    pub_shm_infos[publisher_num].shm_addr = proc_info->mempool_entry->addr;
    pub_shm_infos[publisher_num].shm_size = mempool_size_bytes;
    publisher_num++;
  }

  *ret_pub_shm_num = publisher_num;

  return 0;
}

int agnocast_ioctl_get_version(struct ioctl_get_version_args * ioctl_ret)
{
  strscpy(ioctl_ret->ret_version, VERSION, VERSION_BUFFER_LEN);

  return 0;
}

// A bridge manager is per-(ipc_ns, domain): its UDS address carries
// the domain suffix, so each domain needs its own manager. Gate on the domain too,
// otherwise a manager in one domain would suppress spawning in another.
static bool has_alive_bridge_manager(const struct ipc_namespace * ipc_ns, const uint32_t domain_id)
{
  struct process_info * proc_info;
  int bkt;
  hash_for_each(proc_info_htable, bkt, proc_info, node)
  {
    if (
      ipc_eq(ipc_ns, proc_info->ipc_ns) && proc_info->domain_id == domain_id &&
      proc_info->is_bridge_manager && !proc_info->exited) {
      return true;
    }
  }
  return false;
}

int agnocast_ioctl_add_process(
  const pid_t pid, const struct ipc_namespace * ipc_ns, const bool is_bridge_manager,
  const uint32_t domain_id, union ioctl_add_process_args * ioctl_ret)
{
  int ret = 0;

  down_write(&global_htables_rwsem);

  if (agnocast_find_process_info(pid)) {
    dev_warn(agnocast_device, "Process (pid=%d) already exists. (%s)\n", pid, __func__);
    ret = -EINVAL;
    goto unlock;
  }
  ioctl_ret->ret_unlink_daemon_exist = (get_process_num(ipc_ns) > 0);
  ioctl_ret->ret_bridge_daemon_exist = has_alive_bridge_manager(ipc_ns, domain_id);
  ioctl_ret->ret_discovery_agent_exist = (agnocast_find_discovery_agent(ipc_ns, domain_id) != NULL);

  if (is_bridge_manager && ioctl_ret->ret_bridge_daemon_exist) {
    goto unlock;
  }

  struct process_info * new_proc_info = kmalloc(sizeof(struct process_info), GFP_KERNEL);
  if (!new_proc_info) {
    ret = -ENOMEM;
    goto unlock;
  }

  new_proc_info->exited = false;
  new_proc_info->is_bridge_manager = is_bridge_manager;
  new_proc_info->global_pid = pid;
#ifndef KUNIT_BUILD
  new_proc_info->local_pid = convert_pid_to_local(pid);
#else
  new_proc_info->local_pid = pid;
#endif
  new_proc_info->mempool_entry = assign_memory(pid);
  if (!new_proc_info->mempool_entry) {
    dev_warn(agnocast_device, "Process (pid=%d) failed to allocate memory. (%s)\n", pid, __func__);
    kfree(new_proc_info);
    ret = -ENOMEM;
    goto unlock;
  }

  new_proc_info->ipc_ns = ipc_ns;
  new_proc_info->domain_id = domain_id;

  INIT_HLIST_NODE(&new_proc_info->node);
  uint32_t hash_val = hash_min(new_proc_info->global_pid, PROC_INFO_HASH_BITS);
  hash_add_rcu(proc_info_htable, &new_proc_info->node, hash_val);

  ioctl_ret->ret_addr = new_proc_info->mempool_entry->addr;
  ioctl_ret->ret_shm_size = mempool_size_bytes;

unlock:
  up_write(&global_htables_rwsem);
  return ret;
}

int agnocast_ioctl_add_subscriber(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const char * node_name,
  const pid_t subscriber_pid, const uint32_t qos_depth, const bool qos_is_transient_local,
  const bool qos_is_reliable, const bool is_take_sub, const bool ignore_local_publications,
  const bool is_bridge, const int32_t eventfd, union ioctl_add_subscriber_args * ioctl_ret)
{
  int ret;
  struct eventfd_ctx * notify_ctx = NULL;

  if (!is_take_sub) {
    notify_ctx = agnocast_eventfd_get(eventfd);
    if (IS_ERR(notify_ctx)) {
      dev_warn(agnocast_device, "Failed to get the eventfd context (eventfd=%d).\n", eventfd);
      return PTR_ERR(notify_ctx);
    }
  }

  down_write(&global_htables_rwsem);

  struct topic_wrapper * wrapper;
  ret = add_topic(topic_name, ipc_ns, get_process_domain_id(subscriber_pid), &wrapper);
  if (ret < 0) {
    goto unlock;
  }

  struct subscriber_info * sub_info;
  ret = insert_subscriber_info(
    wrapper, node_name, subscriber_pid, qos_depth, qos_is_transient_local, qos_is_reliable,
    is_take_sub, ignore_local_publications, is_bridge, notify_ctx, &sub_info);
  if (ret < 0) {
    goto unlock;
  }

  ioctl_ret->ret_id = sub_info->id;

unlock:
  up_write(&global_htables_rwsem);
  if (ret < 0 && notify_ctx) {
    agnocast_eventfd_put(notify_ctx);
  }
  return ret;
}

int agnocast_ioctl_add_publisher(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const char * node_name,
  const pid_t publisher_pid, const uint32_t qos_depth, const bool qos_is_transient_local,
  const bool is_bridge, union ioctl_add_publisher_args * ioctl_ret)
{
  int ret;

  down_write(&global_htables_rwsem);

  struct topic_wrapper * wrapper;
  ret = add_topic(topic_name, ipc_ns, get_process_domain_id(publisher_pid), &wrapper);
  if (ret < 0) {
    goto unlock;
  }

  struct publisher_info * pub_info;
  ret = insert_publisher_info(
    wrapper, node_name, publisher_pid, qos_depth, qos_is_transient_local, is_bridge, &pub_info);
  if (ret < 0) {
    goto unlock;
  }

  ioctl_ret->ret_id = pub_info->id;

  // set true to subscriber_info.need_mmap_update to notify
  struct subscriber_info * sub_info;
  int bkt_sub_info;
  hash_for_each(wrapper->topic->sub_info_htable, bkt_sub_info, sub_info, node)
  {
    sub_info->need_mmap_update = true;
  }

unlock:
  up_write(&global_htables_rwsem);
  return ret;
}

static int release_msgs_to_meet_depth(
  struct topic_wrapper * wrapper, struct publisher_info * pub_info,
  union ioctl_publish_msg_args * ioctl_ret)
{
  ioctl_ret->ret_released_num = 0;

  if (pub_info->entries_num <= pub_info->qos_depth) {
    return 0;
  }

  const uint32_t leak_warn_threshold = (pub_info->qos_depth <= 100)
                                         ? 100 + pub_info->qos_depth
                                         : pub_info->qos_depth * 2;  // This is rough value.
  if (pub_info->entries_num > leak_warn_threshold) {
    dev_warn(
      agnocast_device,
      "For some reason, the reference count hasn't been decremented, causing the number of "
      "messages for this publisher to increase. (topic_name=%s, id=%d, entries_num=%d)."
      "(%s)\n",
      wrapper->key, pub_info->id, pub_info->entries_num, __func__);
  }

  struct rb_node * node = rb_first(&wrapper->topic->entries);
  if (!node) {
    dev_warn(
      agnocast_device,
      "Unreachable: Failed to get message entries in publisher (id=%d). "
      "(%s)\n",
      pub_info->id, __func__);
    return -ENODATA;
  }

  // Number of entries exceeding qos_depth
  uint32_t num_search_entries = pub_info->entries_num - pub_info->qos_depth;

  // NOTE:
  //   The searched message is either deleted or, if a reference count remains, is not deleted.
  //   In both cases, this number of searches is sufficient, as it does not affect the Queue size of
  //   QoS.
  //
  // HACK:
  //   The current implementation only releases a maximum of MAX_RELEASE_NUM messages at a time, and
  //   if there are more messages to release, qos_depth is temporarily not met.
  //   However, it is rare for more than MAX_RELEASE_NUM messages that are out of qos_depth to be
  //   unreferenced at a specific time. If this happens, as long as the publisher's qos_depth is
  //   greater than the subscriber's qos_depth, this has little effect on system behavior.
  while (num_search_entries > 0 && ioctl_ret->ret_released_num < MAX_RELEASE_NUM) {
    struct entry_node * en = container_of(node, struct entry_node, node);
    node = rb_next(node);
    if (!node) {
      dev_warn(
        agnocast_device,
        "Unreachable: entries_num is inconsistent with actual message entry num. "
        "(%s)\n",
        __func__);
      return -ENODATA;
    }

    if (en->publisher_id != pub_info->id) continue;

    num_search_entries--;

    // This is not counted in a Queue size of QoS.
    if (agnocast_is_referenced(en)) continue;

    ioctl_ret->ret_released_addrs[ioctl_ret->ret_released_num] = en->msg_virtual_address;
    ioctl_ret->ret_released_num++;

    rb_erase(&en->node, &wrapper->topic->entries);
    kfree(en);

    pub_info->entries_num--;

    dev_dbg(
      agnocast_device,
      "Release oldest message in the publisher_info (id=$%d) of the topic "
      "(topic_name=%s) with qos_depth=%d. (%s)\n",
      pub_info->id, wrapper->key, pub_info->qos_depth, __func__);
  }

  return 0;
}

int agnocast_ioctl_publish_msg(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const topic_local_id_t publisher_id,
  const uint64_t msg_virtual_address, union ioctl_publish_msg_args * ioctl_ret)
{
  int ret = 0;

  // Declared here so the early-error `goto unlock_all` paths reach the signal loop below with
  // notify_num == 0 (a no-op).
  struct eventfd_ctx ** notify_ctxs = NULL;
  uint32_t notify_num = 0;

  down_read(&global_htables_rwsem);

  struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (!wrapper) {
    dev_warn(agnocast_device, "Topic (topic_name=%s) not found. (%s)\n", topic_name, __func__);
    ret = -EINVAL;
    goto unlock_only_global;
  }

  down_write(&wrapper->topic->rwsem);

  struct publisher_info * pub_info = find_publisher_info(wrapper, publisher_id);
  if (!pub_info) {
    dev_warn(
      agnocast_device, "Publisher (id=%d) not found in the topic (topic_name=%s). (%s)\n",
      publisher_id, topic_name, __func__);
    ret = -EINVAL;
    goto unlock_all;
  }

  struct process_info * proc_info = agnocast_find_process_info(pub_info->pid);
  if (!proc_info) {
    dev_warn(agnocast_device, "Process (pid=%d) does not exist. (%s)\n", pub_info->pid, __func__);
    ret = -EINVAL;
    goto unlock_all;
  }

  uint64_t mempool_start = proc_info->mempool_entry->addr;
  uint64_t mempool_end = mempool_start + mempool_size_bytes;
  if (msg_virtual_address < mempool_start || msg_virtual_address >= mempool_end) {
    dev_warn(agnocast_device, "msg_virtual_address is out of bounds. (%s)\n", __func__);
    ret = -EINVAL;
    goto unlock_all;
  }

  ret = insert_message_entry(wrapper, pub_info, msg_virtual_address, ioctl_ret);
  if (ret < 0) {
    goto unlock_all;
  }

  ret = release_msgs_to_meet_depth(wrapper, pub_info, ioctl_ret);
  if (ret < 0) {
    goto unlock_all;
  }

  notify_ctxs = pub_info->notify_ctxs;
  notify_num = pub_info->notify_num;

unlock_all:
  up_write(&wrapper->topic->rwsem);

  // Signal outside topic_rwsem: RECEIVE_MSG and TAKE_MSG take it for write, so holding it here
  // would block the very subscribers being woken. The list and the contexts both stay valid
  // because a context is released only once every list has stopped pointing at it, and both that
  // and any rebuild happen under global_htables_rwsem (write), which is held here for read.
  for (uint32_t i = 0; i < notify_num; i++) {
    agnocast_eventfd_signal(notify_ctxs[i]);
  }

unlock_only_global:
  up_read(&global_htables_rwsem);
  return ret;
}

// Find the first entry with entry_id >= target_entry_id
static struct rb_node * find_first_entry_ge(struct rb_root * root, const int64_t target_entry_id)
{
  struct rb_node ** curr = &(root->rb_node);
  struct rb_node * candidate = NULL;

  while (*curr) {
    const struct entry_node * en = container_of(*curr, struct entry_node, node);
    if (en->entry_id >= target_entry_id) {
      candidate = *curr;
      curr = &((*curr)->rb_left);
    } else {
      curr = &((*curr)->rb_right);
    }
  }

  return candidate;
}

static int receive_msg_core(
  struct topic_wrapper * wrapper, struct subscriber_info * sub_info,
  const topic_local_id_t subscriber_id, union ioctl_receive_msg_args * ioctl_ret)
{
  ioctl_ret->ret_entry_num = 0;
  ioctl_ret->ret_call_again = false;

  struct rb_node * newest_node = rb_last(&wrapper->topic->entries);
  if (!newest_node) {
    return 0;
  }

  const struct entry_node * newest_en = container_of(newest_node, struct entry_node, node);
  const int64_t newest_entry_id = newest_en->entry_id;

  // Calculate start_entry_id = max(newest - qos_depth + 1, latest_received_entry_id + 1)
  const int64_t latest_received_entry_id = sub_info->latest_received_entry_id;
  const int64_t qos_start = newest_entry_id - (int64_t)sub_info->qos_depth + 1;
  const int64_t start_entry_id =
    (qos_start > latest_received_entry_id) ? qos_start : (latest_received_entry_id + 1);

  struct rb_node * node = find_first_entry_ge(&wrapper->topic->entries, start_entry_id);

  for (; node; node = rb_next(node)) {
    struct entry_node * en = container_of(node, struct entry_node, node);

    if (ioctl_ret->ret_entry_num == MAX_RECEIVE_NUM) {
      ioctl_ret->ret_call_again = true;
      break;
    }

    const struct publisher_info * pub_info = find_publisher_info(wrapper, en->publisher_id);
    if (!pub_info) {
      dev_warn(
        agnocast_device,
        "Unreachable: corresponding publisher(id=%d) not found for entry(id=%lld) in "
        "topic(topic_name=%s). (%s)\n",
        en->publisher_id, en->entry_id, wrapper->key, __func__);
      return -ENODATA;
    }

    const struct process_info * proc_info = agnocast_find_process_info(pub_info->pid);
    if (!proc_info || proc_info->exited) {
      continue;
    }

    if (sub_info->ignore_local_publications && (sub_info->pid == pub_info->pid)) {
      continue;
    }

    if (!domain_delivery_allowed(wrapper->topic, pub_info->domain_id, sub_info->domain_id)) {
      continue;
    }

    int ret = add_subscriber_reference(en, subscriber_id, false);
    if (ret < 0) {
      return ret;
    }

    ioctl_ret->ret_entry_ids[ioctl_ret->ret_entry_num] = en->entry_id;
    ioctl_ret->ret_entry_addrs[ioctl_ret->ret_entry_num] = en->msg_virtual_address;
    ioctl_ret->ret_entry_num++;
  }

  if (ioctl_ret->ret_entry_num > 0) {
    sub_info->latest_received_entry_id = ioctl_ret->ret_entry_ids[ioctl_ret->ret_entry_num - 1];
  }

  return 0;
}

int agnocast_ioctl_receive_msg(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t subscriber_id, struct publisher_shm_info * pub_shm_infos,
  uint32_t pub_shm_infos_size, union ioctl_receive_msg_args * ioctl_ret)
{
  int ret = 0;

  down_read(&global_htables_rwsem);

  struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (!wrapper) {
    dev_warn(agnocast_device, "Topic (topic_name=%s) not found. (%s)\n", topic_name, __func__);
    ret = -EINVAL;
    goto unlock_only_global;
  }

  // The receive path needs only a read lock.
  // That lets subscriber processes on one topic receive concurrently
  // rather than serializing behind an exclusive lock.
  // The concurrency unit is processes, not subscribers: agnocastlib holds a process-global
  // `mmap_mtx` across the ioctl, so subscribers sharing a process serialize there regardless.
  //
  // Everything this path touches is either guarded by a lock held here, or self-synchronized:
  //
  // 1. Entries rbtree: two classes of writer mutate it, and both are excluded here.
  //      - Publish (insert_message_entry, release_msgs_to_meet_depth) takes topic->rwsem WRITE,
  //        which a read lock excludes.
  //      - Subscriber/publisher removal, process-exit cleanup and module unload take
  //        global_htables_rwsem WRITE. Those never take topic->rwsem at all: global write is what
  //        excludes them. Holding global read is therefore what keeps this very lock alive.
  //    Both locks stay held until the function finishes, so no writer can change the tree during
  //    traversal.
  //
  // 2. Per-subscriber fields (latest_received_entry_id, need_mmap_update) live in disjoint
  //    sub_info structs, so concurrent receivers of different subscribers write different memory.
  //
  //    Two receives for the *same* subscriber are the case to worry about, and agnocastlib can
  //    produce them: a Reentrant callback group on a multi-threaded executor runs one
  //    subscription's callback on several threads at once. What makes that safe is that
  //    agnocastlib holds the process-global mmap_mtx around every receive/take ioctl, so the
  //    kernel never sees two overlap. Were two to overlap, both would read the same
  //    latest_received_entry_id and walk the same entries, and the loser of the test_and_set_bit
  //    on the first of them would fail the ioctl with -EALREADY, which agnocastlib treats as fatal.
  //
  // 3. The reference bitmap update here is a single atomic test_and_set_bit. (bitmap_empty and
  //    bitmap_zero elsewhere are NOT atomic; every one of their callers holds a write lock.)
  down_read(&wrapper->topic->rwsem);

  struct subscriber_info * sub_info = find_subscriber_info(wrapper, subscriber_id);
  if (!sub_info) {
    dev_warn(
      agnocast_device,
      "Subscriber (id=%d) for the topic (topic_name=%s) not found. "
      "(%s)\n",
      subscriber_id, topic_name, __func__);
    ret = -EINVAL;
    goto unlock_all;
  }

  ret = receive_msg_core(wrapper, sub_info, subscriber_id, ioctl_ret);
  if (ret < 0) {
    goto unlock_all;
  }

  // Check if there is any publisher that need to be mmapped
  if (!sub_info->need_mmap_update) {
    ioctl_ret->ret_pub_shm_num = 0;
    goto unlock_all;
  }

  ret = set_publisher_shm_info(
    wrapper, sub_info->pid, pub_shm_infos, pub_shm_infos_size, &ioctl_ret->ret_pub_shm_num);
  if (ret < 0) {
    goto unlock_all;
  }

  sub_info->need_mmap_update = false;

unlock_all:
  up_read(&wrapper->topic->rwsem);
unlock_only_global:
  up_read(&global_htables_rwsem);
  return ret;
}

int agnocast_ioctl_take_msg(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t subscriber_id, bool allow_same_message,
  struct publisher_shm_info * pub_shm_infos, uint32_t pub_shm_infos_size,
  union ioctl_take_msg_args * ioctl_ret)
{
  int ret = 0;

  down_read(&global_htables_rwsem);

  struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (!wrapper) {
    dev_warn(agnocast_device, "Topic (topic_name=%s) not found. (%s)\n", topic_name, __func__);
    ret = -EINVAL;
    goto unlock_only_global;
  }

  // See the comment above `down_read(&wrapper->topic->rwsem)` in agnocast_ioctl_receive_msg().
  down_read(&wrapper->topic->rwsem);

  struct subscriber_info * sub_info = find_subscriber_info(wrapper, subscriber_id);
  if (!sub_info) {
    dev_warn(
      agnocast_device, "Subscriber (id=%d) for the topic (topic_name=%s) not found. (%s)\n",
      subscriber_id, topic_name, __func__);
    ret = -EINVAL;
    goto unlock_all;
  }

  // These remains 0 if no message is found to take.
  ioctl_ret->ret_addr = 0;
  ioctl_ret->ret_entry_id = -1;

  uint32_t searched_count = 0;
  struct entry_node * candidate_en = NULL;
  struct rb_node * node = rb_last(&wrapper->topic->entries);
  while (node && searched_count < sub_info->qos_depth) {
    struct entry_node * en = container_of(node, struct entry_node, node);
    node = rb_prev(node);

    if (!allow_same_message && en->entry_id == sub_info->latest_received_entry_id) {
      break;  // Don't take the same message if it's not allowed
    }

    if (en->entry_id < sub_info->latest_received_entry_id) {
      break;  // Never take any messages that are older than the most recently received
    }

    const struct publisher_info * pub_info = find_publisher_info(wrapper, en->publisher_id);
    if (!pub_info) {
      dev_warn(
        agnocast_device,
        "Unreachable: corresponding publisher(id=%d) not found for entry(id=%lld) in "
        "topic(topic_name=%s). (%s)\n",
        en->publisher_id, en->entry_id, topic_name, __func__);
      ret = -ENODATA;
      goto unlock_all;
    }

    const struct process_info * proc_info = agnocast_find_process_info(pub_info->pid);
    if (!proc_info || proc_info->exited) {
      continue;
    }

    if (sub_info->ignore_local_publications && (sub_info->pid == pub_info->pid)) {
      continue;
    }

    if (!domain_delivery_allowed(wrapper->topic, pub_info->domain_id, sub_info->domain_id)) {
      continue;
    }

    candidate_en = en;
    searched_count++;
  }

  if (candidate_en) {
    // Claim the reference. test_and_set_bit reports -EALREADY when this subscriber already holds
    // one, which allow_same_message makes an expected outcome rather than a failure: the existing
    // reference is reused instead of a second being taken.
    ret = add_subscriber_reference(candidate_en, subscriber_id, allow_same_message);
    if (ret < 0 && !(allow_same_message && ret == -EALREADY)) {
      goto unlock_all;
    }
    ret = 0;

    ioctl_ret->ret_addr = candidate_en->msg_virtual_address;
    ioctl_ret->ret_entry_id = candidate_en->entry_id;

    sub_info->latest_received_entry_id = ioctl_ret->ret_entry_id;
  }

  // Check if there is any publisher that need to be mmapped
  if (!sub_info->need_mmap_update) {
    ioctl_ret->ret_pub_shm_num = 0;
    goto unlock_all;
  }

  ret = set_publisher_shm_info(
    wrapper, sub_info->pid, pub_shm_infos, pub_shm_infos_size, &ioctl_ret->ret_pub_shm_num);
  if (ret < 0) {
    goto unlock_all;
  }

  sub_info->need_mmap_update = false;

unlock_all:
  up_read(&wrapper->topic->rwsem);
unlock_only_global:
  up_read(&global_htables_rwsem);
  return ret;
}

// Forward declaration
static struct bridge_info * find_bridge_info(
  const char * topic_name, const struct ipc_namespace * ipc_ns);

int agnocast_ioctl_get_subscriber_num(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const pid_t pid,
  union ioctl_get_subscriber_num_args * ioctl_ret)
{
  ioctl_ret->ret_other_process_subscriber_num = 0;
  ioctl_ret->ret_same_process_subscriber_num = 0;
  ioctl_ret->ret_ros2_subscriber_num = 0;
  ioctl_ret->ret_a2r_bridge_exist = false;
  ioctl_ret->ret_r2a_bridge_exist = false;

  down_read(&global_htables_rwsem);

  struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);

  if (!wrapper) {
    up_read(&global_htables_rwsem);
    return 0;
  }

  down_read(&wrapper->topic->rwsem);

  uint32_t inter_count = 0;
  uint32_t intra_count = 0;

  // Match ROS 2's get_subscription_count: report only same-domain subscribers.
  // A bridge rule still delivers cross-domain (see the publish/receive paths),
  // but a publisher does not count subscribers in another domain. Ungrouped
  // topics hold only one domain, so this is a no-op for them.
  struct subscriber_info * sub_info;
  int bkt_sub;
  hash_for_each(wrapper->topic->sub_info_htable, bkt_sub, sub_info, node)
  {
    if (sub_info->domain_id != wrapper->domain_id) {
      continue;
    }
    if (sub_info->is_bridge) {
      ioctl_ret->ret_a2r_bridge_exist = true;
    }
    if (sub_info->pid == pid) {
      intra_count++;
    } else {
      inter_count++;
    }
  }

  struct publisher_info * pub_info;
  int bkt_pub;
  hash_for_each(wrapper->topic->pub_info_htable, bkt_pub, pub_info, node)
  {
    if (pub_info->is_bridge) {
      ioctl_ret->ret_r2a_bridge_exist = true;
      break;
    }
  }

  ioctl_ret->ret_other_process_subscriber_num = inter_count;
  ioctl_ret->ret_same_process_subscriber_num = intra_count;
  ioctl_ret->ret_ros2_subscriber_num = wrapper->topic->ros2_subscriber_num;

  up_read(&wrapper->topic->rwsem);
  up_read(&global_htables_rwsem);

  return 0;
}

int agnocast_ioctl_set_ros2_subscriber_num(
  const char * topic_name, const struct ipc_namespace * ipc_ns, uint32_t count)
{
  int ret = 0;

  down_read(&global_htables_rwsem);

  struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (wrapper) {
    down_write(&wrapper->topic->rwsem);
    wrapper->topic->ros2_subscriber_num = count;
    up_write(&wrapper->topic->rwsem);
  } else {
    ret = -ENOENT;
  }

  up_read(&global_htables_rwsem);
  return ret;
}

int agnocast_ioctl_set_ros2_publisher_num(
  const char * topic_name, const struct ipc_namespace * ipc_ns, uint32_t count)
{
  int ret = 0;

  down_read(&global_htables_rwsem);

  struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (wrapper) {
    down_write(&wrapper->topic->rwsem);
    wrapper->topic->ros2_publisher_num = count;
    up_write(&wrapper->topic->rwsem);
  } else {
    ret = -ENOENT;
  }

  up_read(&global_htables_rwsem);
  return ret;
}

int agnocast_ioctl_get_publisher_num(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  union ioctl_get_publisher_num_args * ioctl_ret)
{
  ioctl_ret->ret_publisher_num = 0;
  ioctl_ret->ret_ros2_publisher_num = 0;
  ioctl_ret->ret_r2a_bridge_exist = false;
  ioctl_ret->ret_a2r_bridge_exist = false;

  down_read(&global_htables_rwsem);

  struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);

  if (!wrapper) {
    up_read(&global_htables_rwsem);
    return 0;
  }

  down_read(&wrapper->topic->rwsem);

  ioctl_ret->ret_ros2_publisher_num = wrapper->topic->ros2_publisher_num;

  // Match ROS 2's get_publisher_count: report only same-domain publishers.
  // A bridge rule still delivers cross-domain (see the publish/receive paths),
  // but a subscriber does not count publishers in another domain. Ungrouped
  // topics hold only one domain, so this is a no-op for them.
  uint32_t publisher_num = 0;
  struct publisher_info * pub_info;
  int bkt_pub;
  hash_for_each(wrapper->topic->pub_info_htable, bkt_pub, pub_info, node)
  {
    if (pub_info->domain_id != wrapper->domain_id) {
      continue;
    }
    publisher_num++;
    if (pub_info->is_bridge) {
      ioctl_ret->ret_r2a_bridge_exist = true;
    }
  }
  ioctl_ret->ret_publisher_num = publisher_num;

  struct subscriber_info * sub_info;
  int bkt_sub;
  hash_for_each(wrapper->topic->sub_info_htable, bkt_sub, sub_info, node)
  {
    if (sub_info->is_bridge) {
      ioctl_ret->ret_a2r_bridge_exist = true;
      break;
    }
  }

  up_read(&wrapper->topic->rwsem);
  up_read(&global_htables_rwsem);

  return 0;
}

// Two-phase ioctl for exit process cleanup:
//   Phase 1 (agnocast_ioctl_get_exit_process): report the exited pid, leaving proc_info in place.
//   Phase 2 (agnocast_commit_exit_process): free proc_info.
//
// Splitting the two lets the dispatch handler copy ret_pid out with no lock held, and only commit
// once that copy succeeded: a failed copy returns -EFAULT before Phase 2, so the entry is not
// dropped kernel-side.
pid_t agnocast_ioctl_get_exit_process(
  const struct ipc_namespace * ipc_ns, struct ioctl_get_exit_process_args * ioctl_ret)
{
  ioctl_ret->ret_pid = -1;
  ioctl_ret->ret_daemon_should_exit = false;
  pid_t global_pid = -1;

  down_read(&global_htables_rwsem);

  struct process_info * proc_info;
  int bkt;
  hash_for_each(proc_info_htable, bkt, proc_info, node)
  {
    if (!ipc_eq(proc_info->ipc_ns, ipc_ns) || !proc_info->exited) {
      continue;
    }

    ioctl_ret->ret_pid = proc_info->local_pid;
    global_pid = proc_info->global_pid;
    break;
  }

  up_read(&global_htables_rwsem);
  return global_pid;
}

void agnocast_commit_exit_process(
  const struct ipc_namespace * ipc_ns, pid_t global_pid, bool * ret_daemon_should_exit)
{
  down_write(&global_htables_rwsem);

  if (global_pid >= 0) {
    struct process_info * proc_info = agnocast_find_process_info(global_pid);
    if (proc_info) {
      hash_del_rcu(&proc_info->node);
      kfree_rcu(proc_info, rcu_head);
    }
  }

  *ret_daemon_should_exit = (get_process_num(ipc_ns) == 0);

  up_write(&global_htables_rwsem);
}

// Intentionally namespace-scoped, not caller-domain-scoped: this returns every
// domain's topics, each stamped with its domain_id, rather than filtering to the
// caller's ROS_DOMAIN_ID. get_topic_*_info takes a domain input and filters here;
// the list stays broad so one call serves both a per-(NS, domain) consumer (which
// filters client-side -- cheap) and a cross-domain view (e.g. a future domain-aware
// `ros2 topic list_agnocast`). Revisit by adding a domain input if a strictly
// per-domain enumeration is ever needed.
int agnocast_ioctl_get_topic_list(
  const struct ipc_namespace * ipc_ns, union ioctl_topic_list_args * topic_list_args)
{
  int ret = 0;
  uint32_t topic_num = 0;

  down_read(&global_htables_rwsem);

  struct topic_wrapper * wrapper;
  int bkt_topic;
  hash_for_each(topic_hashtable, bkt_topic, wrapper, node)
  {
    if (!ipc_eq(ipc_ns, wrapper->ipc_ns)) {
      continue;
    }

    if (topic_num >= MAX_TOPIC_NUM || topic_num >= topic_list_args->topic_name_buffer_size) {
      dev_warn(
        agnocast_device, "Topic count exceeds limit: MAX_TOPIC_NUM=%d, topic_name_buffer_size=%u\n",
        MAX_TOPIC_NUM, topic_list_args->topic_name_buffer_size);
      ret = -ENOBUFS;
      goto unlock;
    }

    if (copy_to_user(
          (char __user *)(topic_list_args->topic_name_buffer_addr +
                          topic_num * TOPIC_NAME_BUFFER_SIZE),
          wrapper->key, strlen(wrapper->key) + 1)) {
      ret = -EFAULT;
      goto unlock;
    }

    if (topic_list_args->domain_id_buffer_addr) {
      uint32_t domain_id = wrapper->domain_id;
      uint32_t __user * domain_id_buffer =
        (uint32_t __user *)u64_to_user_ptr(topic_list_args->domain_id_buffer_addr);
      if (copy_to_user(domain_id_buffer + topic_num, &domain_id, sizeof(domain_id))) {
        ret = -EFAULT;
        goto unlock;
      }
    }

    topic_num++;
  }

  topic_list_args->ret_topic_num = topic_num;

unlock:
  up_read(&global_htables_rwsem);
  return ret;
}

int agnocast_ioctl_get_node_subscriber_topics(
  const struct ipc_namespace * ipc_ns, const char * node_name,
  union ioctl_node_info_args * node_info_args)
{
  int ret = 0;
  uint32_t topic_num = 0;

  down_read(&global_htables_rwsem);

  struct topic_wrapper * wrapper;
  int bkt_topic;

  hash_for_each(topic_hashtable, bkt_topic, wrapper, node)
  {
    if (!ipc_eq(ipc_ns, wrapper->ipc_ns)) {
      continue;
    }

    down_read(&wrapper->topic->rwsem);

    struct subscriber_info * sub_info;
    int bkt_sub_info;
    bool found = false;
    hash_for_each(wrapper->topic->sub_info_htable, bkt_sub_info, sub_info, node)
    {
      if (strcmp(sub_info->node_name, node_name) == 0) {
        found = true;
        break;
      }
    }

    up_read(&wrapper->topic->rwsem);

    if (found) {
      if (topic_num >= MAX_TOPIC_NUM || topic_num >= node_info_args->topic_name_buffer_size) {
        dev_warn(
          agnocast_device,
          "Topic count exceeds limit: MAX_TOPIC_NUM=%d, topic_name_buffer_size=%u\n", MAX_TOPIC_NUM,
          node_info_args->topic_name_buffer_size);
        ret = -ENOBUFS;
        goto unlock;
      }

      if (copy_to_user(
            (char __user *)(node_info_args->topic_name_buffer_addr +
                            topic_num * TOPIC_NAME_BUFFER_SIZE),
            wrapper->key, strlen(wrapper->key) + 1)) {
        ret = -EFAULT;
        goto unlock;
      }

      topic_num++;
    }
  }

  node_info_args->ret_topic_num = topic_num;

unlock:
  up_read(&global_htables_rwsem);
  return ret;
}

int agnocast_ioctl_get_node_publisher_topics(
  const struct ipc_namespace * ipc_ns, const char * node_name,
  union ioctl_node_info_args * node_info_args)
{
  int ret = 0;
  uint32_t topic_num = 0;

  down_read(&global_htables_rwsem);

  struct topic_wrapper * wrapper;
  int bkt_topic;

  hash_for_each(topic_hashtable, bkt_topic, wrapper, node)
  {
    if (!ipc_eq(ipc_ns, wrapper->ipc_ns)) {
      continue;
    }

    down_read(&wrapper->topic->rwsem);

    struct publisher_info * pub_info;
    int bkt_pub_info;
    bool found = false;
    hash_for_each(wrapper->topic->pub_info_htable, bkt_pub_info, pub_info, node)
    {
      if (strcmp(pub_info->node_name, node_name) == 0) {
        found = true;
        break;
      }
    }

    up_read(&wrapper->topic->rwsem);

    if (found) {
      if (topic_num >= MAX_TOPIC_NUM || topic_num >= node_info_args->topic_name_buffer_size) {
        dev_warn(
          agnocast_device,
          "Topic count exceeds limit: MAX_TOPIC_NUM=%d, topic_name_buffer_size=%u\n", MAX_TOPIC_NUM,
          node_info_args->topic_name_buffer_size);
        ret = -ENOBUFS;
        goto unlock;
      }

      if (copy_to_user(
            (char __user *)(node_info_args->topic_name_buffer_addr +
                            topic_num * TOPIC_NAME_BUFFER_SIZE),
            wrapper->key, strlen(wrapper->key) + 1)) {
        ret = -EFAULT;
        goto unlock;
      }

      topic_num++;
    }
  }

  node_info_args->ret_topic_num = topic_num;

unlock:
  up_read(&global_htables_rwsem);
  return ret;
}

int agnocast_ioctl_get_topic_subscriber_info(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  union ioctl_topic_info_args * topic_info_args)
{
  int ret = 0;
  topic_info_args->ret_topic_info_ret_num = 0;

  down_read(&global_htables_rwsem);

  struct topic_wrapper * wrapper = find_topic(topic_name, ipc_ns, topic_info_args->domain_id);
  if (!wrapper) {
    up_read(&global_htables_rwsem);
    return 0;
  }

  down_read(&wrapper->topic->rwsem);

  struct subscriber_info * sub_info;
  int bkt_sub_info;

  struct topic_info_ret __user * user_buffer =
    (struct topic_info_ret __user *)topic_info_args->topic_info_ret_buffer_addr;

  // Count actual subscribers first. The htable may be shared with a bridged
  // domain, so only count endpoints in the requested domain.
  uint32_t subscriber_num = 0;
  hash_for_each(wrapper->topic->sub_info_htable, bkt_sub_info, sub_info, node)
  {
    if (sub_info->domain_id != wrapper->domain_id) continue;
    subscriber_num++;
  }

  if (subscriber_num > topic_info_args->topic_info_ret_buffer_size) {
    dev_warn(
      agnocast_device,
      "Subscriber count exceeds limit: subscriber_num=%u, "
      "topic_info_ret_buffer_size=%u\n",
      subscriber_num, topic_info_args->topic_info_ret_buffer_size);
    ret = -ENOBUFS;
    goto unlock;
  }

  struct topic_info_ret * topic_info_mem = NULL;
  if (subscriber_num > 0) {
    topic_info_mem = kvcalloc(subscriber_num, sizeof(struct topic_info_ret), GFP_KERNEL);
    if (!topic_info_mem) {
      ret = -ENOMEM;
      goto unlock;
    }
  }

  uint32_t idx = 0;
  hash_for_each(wrapper->topic->sub_info_htable, bkt_sub_info, sub_info, node)
  {
    if (sub_info->domain_id != wrapper->domain_id) continue;

    if (!sub_info->node_name) {
      kvfree(topic_info_mem);
      ret = -EFAULT;
      goto unlock;
    }

    struct topic_info_ret * temp_info = &topic_info_mem[idx];

    strscpy(temp_info->node_name, sub_info->node_name, NODE_NAME_BUFFER_SIZE);
    temp_info->qos_depth = sub_info->qos_depth;
    temp_info->qos_is_transient_local = sub_info->qos_is_transient_local;
    temp_info->qos_is_reliable = sub_info->qos_is_reliable;
    temp_info->is_bridge = sub_info->is_bridge;

    idx++;
  }

  if (
    subscriber_num > 0 &&
    copy_to_user(user_buffer, topic_info_mem, sizeof(struct topic_info_ret) * subscriber_num)) {
    kvfree(topic_info_mem);
    ret = -EFAULT;
    goto unlock;
  }

  kvfree(topic_info_mem);
  topic_info_args->ret_topic_info_ret_num = subscriber_num;

unlock:
  up_read(&wrapper->topic->rwsem);
  up_read(&global_htables_rwsem);
  return ret;
}

int agnocast_ioctl_get_topic_publisher_info(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  union ioctl_topic_info_args * topic_info_args)
{
  int ret = 0;
  topic_info_args->ret_topic_info_ret_num = 0;

  down_read(&global_htables_rwsem);

  struct topic_wrapper * wrapper = find_topic(topic_name, ipc_ns, topic_info_args->domain_id);
  if (!wrapper) {
    up_read(&global_htables_rwsem);
    return 0;
  }

  down_read(&wrapper->topic->rwsem);

  struct publisher_info * pub_info;
  int bkt_pub_info;

  struct topic_info_ret __user * user_buffer =
    (struct topic_info_ret __user *)topic_info_args->topic_info_ret_buffer_addr;

  // Count actual publishers first. The htable may be shared with a bridged
  // domain, so only count endpoints in the requested domain.
  uint32_t publisher_num = 0;
  hash_for_each(wrapper->topic->pub_info_htable, bkt_pub_info, pub_info, node)
  {
    if (pub_info->domain_id != wrapper->domain_id) continue;
    publisher_num++;
  }

  if (publisher_num > topic_info_args->topic_info_ret_buffer_size) {
    dev_warn(
      agnocast_device,
      "Publisher count exceeds limit: publisher_num=%u, "
      "topic_info_ret_buffer_size=%u\n",
      publisher_num, topic_info_args->topic_info_ret_buffer_size);
    ret = -ENOBUFS;
    goto unlock;
  }

  struct topic_info_ret * topic_info_mem = NULL;
  if (publisher_num > 0) {
    topic_info_mem = kvcalloc(publisher_num, sizeof(struct topic_info_ret), GFP_KERNEL);
    if (!topic_info_mem) {
      ret = -ENOMEM;
      goto unlock;
    }
  }

  uint32_t idx = 0;
  hash_for_each(wrapper->topic->pub_info_htable, bkt_pub_info, pub_info, node)
  {
    if (pub_info->domain_id != wrapper->domain_id) continue;

    if (!pub_info->node_name) {
      kvfree(topic_info_mem);
      ret = -EFAULT;
      goto unlock;
    }

    struct topic_info_ret * temp_info = &topic_info_mem[idx];

    strscpy(temp_info->node_name, pub_info->node_name, NODE_NAME_BUFFER_SIZE);
    temp_info->qos_depth = pub_info->qos_depth;
    temp_info->qos_is_transient_local = pub_info->qos_is_transient_local;
    temp_info->qos_is_reliable = false;  // Publishers do not have reliability QoS
    temp_info->is_bridge = pub_info->is_bridge;

    idx++;
  }

  if (
    publisher_num > 0 &&
    copy_to_user(user_buffer, topic_info_mem, sizeof(struct topic_info_ret) * publisher_num)) {
    kvfree(topic_info_mem);
    ret = -EFAULT;
    goto unlock;
  }

  kvfree(topic_info_mem);
  topic_info_args->ret_topic_info_ret_num = publisher_num;

unlock:
  up_read(&wrapper->topic->rwsem);
  up_read(&global_htables_rwsem);
  return ret;
}

int agnocast_ioctl_get_subscriber_qos(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t subscriber_id, struct ioctl_get_subscriber_qos_args * args)
{
  int ret = 0;

  down_read(&global_htables_rwsem);

  struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (!wrapper) {
    dev_dbg(agnocast_device, "Topic (topic_name=%s) not found. (%s)\n", topic_name, __func__);
    ret = -EINVAL;
    goto unlock_only_global;
  }

  down_read(&wrapper->topic->rwsem);

  const struct subscriber_info * sub_info = find_subscriber_info(wrapper, subscriber_id);
  if (!sub_info) {
    dev_dbg(
      agnocast_device,
      "Subscriber (id=%d) for the topic (topic_name=%s) not found. "
      "(%s)\n",
      subscriber_id, topic_name, __func__);
    ret = -EINVAL;
    goto unlock_all;
  }

  args->ret_depth = sub_info->qos_depth;
  args->ret_is_transient_local = sub_info->qos_is_transient_local;
  args->ret_is_reliable = sub_info->qos_is_reliable;

unlock_all:
  up_read(&wrapper->topic->rwsem);
unlock_only_global:
  up_read(&global_htables_rwsem);
  return ret;
}

int agnocast_ioctl_get_publisher_qos(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const topic_local_id_t publisher_id,
  struct ioctl_get_publisher_qos_args * args)
{
  int ret = 0;

  down_read(&global_htables_rwsem);

  struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (!wrapper) {
    dev_dbg(agnocast_device, "Topic (topic_name=%s) not found. (%s)\n", topic_name, __func__);
    ret = -EINVAL;
    goto unlock_only_global;
  }

  down_read(&wrapper->topic->rwsem);

  const struct publisher_info * pub_info = find_publisher_info(wrapper, publisher_id);
  if (!pub_info) {
    dev_dbg(
      agnocast_device,
      "Publisher (id=%d) for the topic (topic_name=%s) not found. "
      "(%s)\n",
      publisher_id, topic_name, __func__);
    ret = -EINVAL;
    goto unlock_all;
  }

  args->ret_depth = pub_info->qos_depth;
  args->ret_is_transient_local = pub_info->qos_is_transient_local;

unlock_all:
  up_read(&wrapper->topic->rwsem);
unlock_only_global:
  up_read(&global_htables_rwsem);
  return ret;
}

int agnocast_ioctl_remove_subscriber(
  const char * topic_name, const struct ipc_namespace * ipc_ns, topic_local_id_t subscriber_id)
{
  int ret = 0;

  down_write(&global_htables_rwsem);

  struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (!wrapper) {
    ret = -EINVAL;
    goto unlock;
  }

  struct subscriber_info * sub_info = find_subscriber_info(wrapper, subscriber_id);
  if (!sub_info) {
    ret = -ENODATA;
    goto unlock;
  }

  agnocast_unlink_subscriber_info(wrapper, sub_info);

  if (!is_parameter_service_topic(topic_name)) {
    dev_info(
      agnocast_device, "Subscriber (id=%d) removed from topic %s.\n", subscriber_id, topic_name);
  }

  if (subscriber_id < 0 || subscriber_id >= MAX_TOPIC_LOCAL_ID) {
    dev_warn(
      agnocast_device, "subscriber_id %d out of range [0, %d). (%s)\n", subscriber_id,
      MAX_TOPIC_LOCAL_ID, __func__);
    ret = -EINVAL;
    goto unlock;
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
        const struct process_info * proc_info = agnocast_find_process_info(pub_info->pid);
        if (!proc_info || proc_info->exited) {
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

  if (!agnocast_wrapper_has_domain_endpoints(wrapper)) {
    agnocast_release_topic_wrapper(wrapper);
    dev_dbg(agnocast_device, "Topic %s removed (empty).\n", topic_name);
  }

unlock:
  up_write(&global_htables_rwsem);
  return ret;
}

int agnocast_ioctl_remove_publisher(
  const char * topic_name, const struct ipc_namespace * ipc_ns, topic_local_id_t publisher_id)
{
  int ret = 0;

  down_write(&global_htables_rwsem);

  struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (!wrapper) {
    ret = -EINVAL;
    goto unlock;
  }

  struct publisher_info * pub_info = find_publisher_info(wrapper, publisher_id);
  if (!pub_info) {
    ret = -ENODATA;
    goto unlock;
  }

  // Publisher-side handles do not participate in reference counting, so we don't need
  // to remove publisher references. Just clean up entries that have no subscriber references.
  struct rb_root * root = &wrapper->topic->entries;
  struct rb_node * node = rb_first(root);
  struct rb_node * next_node;

  while (node) {
    next_node = rb_next(node);
    struct entry_node * en = rb_entry(node, struct entry_node, node);
    node = next_node;

    if (en->publisher_id != publisher_id) continue;

    if (!agnocast_is_referenced(en)) {
      pub_info->entries_num--;
      agnocast_remove_entry_node(wrapper, en);
    }
  }

  if (pub_info->entries_num == 0) {
    hash_del(&pub_info->node);
    free_publisher_info(pub_info);

    if (!is_parameter_service_topic(topic_name)) {
      dev_info(
        agnocast_device, "Publisher (id=%d) removed from topic %s.\n", publisher_id, topic_name);
    }
  }

  if (!agnocast_wrapper_has_domain_endpoints(wrapper)) {
    agnocast_release_topic_wrapper(wrapper);
    dev_dbg(agnocast_device, "Topic %s removed (empty).\n", topic_name);
  }

unlock:
  up_write(&global_htables_rwsem);
  return ret;
}

static struct bridge_info * find_bridge_info(
  const char * topic_name, const struct ipc_namespace * ipc_ns)
{
  struct bridge_info * br_info;
  uint32_t hash_val = full_name_hash(NULL, topic_name, strlen(topic_name));
  hash_for_each_possible(bridge_htable, br_info, node, hash_val)
  {
    if (ipc_ns == br_info->ipc_ns && strcmp(br_info->topic_name, topic_name) == 0) {
      return br_info;
    }
  }
  return NULL;
}

int agnocast_ioctl_add_bridge(
  const char * topic_name, const pid_t pid, const bool is_r2a, const struct ipc_namespace * ipc_ns,
  struct ioctl_add_bridge_args * ioctl_ret)
{
  int ret = 0;

  down_write(&global_htables_rwsem);

  struct bridge_info * existing = find_bridge_info(topic_name, ipc_ns);

  if (existing) {
    if (existing->pid != pid) {
      ioctl_ret->ret_pid = existing->pid;
      ioctl_ret->ret_has_r2a = existing->has_r2a;
      ioctl_ret->ret_has_a2r = existing->has_a2r;
      ret = -EEXIST;
      goto unlock;
    }

    // pid matches
    if (is_r2a) {
      if (!existing->has_r2a) {
        existing->has_r2a = true;
        dev_info(
          agnocast_device, "Bridge (topic=%s) r2a direction added for pid=%d.\n", topic_name, pid);
      }
    } else {
      if (!existing->has_a2r) {
        existing->has_a2r = true;
        dev_info(
          agnocast_device, "Bridge (topic=%s) a2r direction added for pid=%d.\n", topic_name, pid);
      }
    }

    ioctl_ret->ret_pid = existing->pid;
    ioctl_ret->ret_has_r2a = existing->has_r2a;
    ioctl_ret->ret_has_a2r = existing->has_a2r;
    goto unlock;
  }

  struct bridge_info * br_info = kmalloc(sizeof(*br_info), GFP_KERNEL);
  if (!br_info) {
    ret = -ENOMEM;
    goto unlock;
  }

  br_info->topic_name = kstrdup(topic_name, GFP_KERNEL);
  if (!br_info->topic_name) {
    dev_warn(
      agnocast_device, "Failed to add a new topic (topic_name=%s) by kstrdup. (ioctl_add_bridge)\n",
      topic_name);
    kfree(br_info);
    ret = -ENOMEM;
    goto unlock;
  }

  br_info->pid = pid;
  br_info->ipc_ns = ipc_ns;

  if (is_r2a) {
    br_info->has_r2a = true;
    br_info->has_a2r = false;
  } else {
    br_info->has_r2a = false;
    br_info->has_a2r = true;
  }

  if (ioctl_ret) {
    ioctl_ret->ret_pid = pid;
    ioctl_ret->ret_has_r2a = br_info->has_r2a;
    ioctl_ret->ret_has_a2r = br_info->has_a2r;
  }

  INIT_HLIST_NODE(&br_info->node);
  uint32_t hash_val = full_name_hash(NULL, topic_name, strlen(topic_name));

  hash_add(bridge_htable, &br_info->node, hash_val);

  dev_info(
    agnocast_device, "Bridge (topic=%s) added. pid=%d, r2a=%d, a2r=%d.\n", topic_name, pid,
    br_info->has_r2a, br_info->has_a2r);

unlock:
  up_write(&global_htables_rwsem);
  return ret;
}

// A rule is keyed by cell = (name, domain). Rules are few (one per bridged topic
// pair), so a full scan is cheaper than maintaining a dual-name hash.
static struct domain_bridge_rule * find_domain_rule(
  const char * topic_name, const struct ipc_namespace * ipc_ns, uint32_t domain_id)
{
  struct domain_bridge_rule * rule;
  int bkt;
  hash_for_each(domain_rule_htable, bkt, rule, node)
  {
    if (ipc_ns != rule->ipc_ns) continue;
    if (
      (domain_id == rule->domain_a && strcmp(rule->topic_name_a, topic_name) == 0) ||
      (domain_id == rule->domain_b && strcmp(rule->topic_name_b, topic_name) == 0)) {
      return rule;
    }
  }
  return NULL;
}

// The caller holds global_htables_rwsem for write and has already established that neither cell
// is taken.
static int insert_domain_rule(
  const char * name_a, const char * name_b, const struct ipc_namespace * ipc_ns,
  const uint32_t domain_a, const uint32_t domain_b, const bool from_is_a)
{
  struct domain_bridge_rule * rule = kmalloc(sizeof(*rule), GFP_KERNEL);
  if (!rule) return -ENOMEM;

  rule->topic_name_a = kstrdup(name_a, GFP_KERNEL);
  rule->topic_name_b = kstrdup(name_b, GFP_KERNEL);
  if (!rule->topic_name_a || !rule->topic_name_b) {
    kfree(rule->topic_name_a);
    kfree(rule->topic_name_b);
    kfree(rule);
    return -ENOMEM;
  }
  rule->ipc_ns = ipc_ns;
  rule->domain_a = domain_a;
  rule->domain_b = domain_b;
  rule->a_to_b = from_is_a;
  rule->b_to_a = !from_is_a;
  INIT_HLIST_NODE(&rule->node);
  // find_domain_rule scans every bucket, so the hash key is only for even distribution.
  hash_add(domain_rule_htable, &rule->node, full_name_hash(NULL, name_a, strlen(name_a)));

  // Report it the way it was declared, not in the canonical order it is stored in.
  dev_info(
    agnocast_device, "Domain bridge rule added (%s@%u -> %s@%u).\n", from_is_a ? name_a : name_b,
    from_is_a ? domain_a : domain_b, from_is_a ? name_b : name_a, from_is_a ? domain_b : domain_a);
  return 0;
}

// The caller holds global_htables_rwsem for write.
static int add_domain_rule(
  const char * name_a, const char * name_b, const struct ipc_namespace * ipc_ns,
  const uint32_t domain_a, const uint32_t domain_b, const bool from_is_a)
{
  // Invariant: each cell (name, domain) belongs to at most one rule, and a rule pairs
  // exactly two cells. r_a == r_b (non-NULL) means an existing rule already pairs exactly
  // these two cells -- a re-declaration or the reverse direction, so just OR in the
  // direction. Any other overlap (a cell already paired with a different cell) is a
  // fan-out and is rejected. This is the one place that enforces one pair per cell.
  // TODO: support >2 domains per topic by storing a domain group instead of a fixed pair.
  struct domain_bridge_rule * r_a = find_domain_rule(name_a, ipc_ns, domain_a);
  struct domain_bridge_rule * r_b = find_domain_rule(name_b, ipc_ns, domain_b);
  if (r_a || r_b) {
    if (r_a != r_b) return -EBUSY;

    r_a->a_to_b |= from_is_a;
    r_a->b_to_a |= !from_is_a;

    // Unlike the paths below, this one runs after endpoints may have registered, and the
    // direction just enabled was denied when their notify lists were built. Both cells share one
    // topic_struct once grouped, so either wrapper reaches every publisher.
    struct topic_wrapper * wrapper = find_topic(name_a, ipc_ns, domain_a);
    if (!wrapper) wrapper = find_topic(name_b, ipc_ns, domain_b);
    if (wrapper) rebuild_all_notify_lists(wrapper);
    return 0;
  }

  // Grouping merges the two domains' id and entry_id spaces, which is only safe
  // before either side has allocated any; reject if an endpoint already joined.
  if (find_topic(name_a, ipc_ns, domain_a) || find_topic(name_b, ipc_ns, domain_b)) return -EBUSY;

  return insert_domain_rule(name_a, name_b, ipc_ns, domain_a, domain_b, from_is_a);
}

int agnocast_ioctl_add_domain_bridge(
  const char * topic_name_from, const char * topic_name_to, const uint32_t from_domain,
  const uint32_t to_domain, const struct ipc_namespace * ipc_ns)
{
  if (from_domain == to_domain) return -EINVAL;

  // Store the pair canonically (domain_a < domain_b), each domain keeping its own name
  // (they differ on rename). Direction lives only in the a_to_b / b_to_a flags; grouping
  // (find_grouped_topic_struct) pairs the two cells and delivery direction is enforced by
  // domain_delivery_allowed.
  const bool from_is_a = from_domain < to_domain;
  const uint32_t domain_a = from_is_a ? from_domain : to_domain;
  const uint32_t domain_b = from_is_a ? to_domain : from_domain;
  const char * name_a = from_is_a ? topic_name_from : topic_name_to;
  const char * name_b = from_is_a ? topic_name_to : topic_name_from;

  down_write(&global_htables_rwsem);
  const int ret = add_domain_rule(name_a, name_b, ipc_ns, domain_a, domain_b, from_is_a);
  up_write(&global_htables_rwsem);
  return ret;
}

int agnocast_ioctl_remove_bridge(
  const char * topic_name, const pid_t pid, const bool is_r2a, const struct ipc_namespace * ipc_ns)
{
  int ret = 0;

  down_write(&global_htables_rwsem);

  struct bridge_info * br_info = find_bridge_info(topic_name, ipc_ns);

  if (!br_info) {
    dev_warn(agnocast_device, "Bridge (topic=%s) not found. (%s)\n", topic_name, __func__);
    ret = -ENOENT;
    goto unlock;
  }

  if (br_info->pid != pid) {
    dev_warn(
      agnocast_device, "Bridge (topic=%s) pid mismatch. Expected %d, got %d.\n", topic_name,
      br_info->pid, pid);
    ret = -EPERM;
    goto unlock;
  }

  if (is_r2a) {
    if (!br_info->has_r2a) {
      dev_warn(agnocast_device, "Bridge (topic=%s) r2a flag was already false.\n", topic_name);
    }
    br_info->has_r2a = false;
  } else {
    if (!br_info->has_a2r) {
      dev_warn(agnocast_device, "Bridge (topic=%s) a2r flag was already false.\n", topic_name);
    }
    br_info->has_a2r = false;
  }

  if (!br_info->has_r2a && !br_info->has_a2r) {
    hash_del(&br_info->node);
    kfree(br_info->topic_name);
    kfree(br_info);

    dev_info(agnocast_device, "Bridge (topic=%s) removed completely.\n", topic_name);
  } else {
    dev_info(
      agnocast_device, "Bridge (topic=%s) direction removed. Remaining: r2a=%d, a2r=%d.\n",
      topic_name, br_info->has_r2a, br_info->has_a2r);
  }

unlock:
  up_write(&global_htables_rwsem);
  return ret;
}

static int get_process_num(const struct ipc_namespace * ipc_ns)
{
  int count = 0;
  struct process_info * proc_info;
  int bkt_proc_info;
  hash_for_each(proc_info_htable, bkt_proc_info, proc_info, node)
  {
    if (ipc_eq(ipc_ns, proc_info->ipc_ns)) {
      count++;
    }
  }
  return count;
}

static int get_process_num_in_domain(const struct ipc_namespace * ipc_ns, const uint32_t domain_id)
{
  int count = 0;
  struct process_info * proc_info;
  int bkt_proc_info;
  hash_for_each(proc_info_htable, bkt_proc_info, proc_info, node)
  {
    if (ipc_eq(ipc_ns, proc_info->ipc_ns) && proc_info->domain_id == domain_id) {
      count++;
    }
  }
  return count;
}

// Like get_process_num_in_domain() but excludes processes that have exited and are still
// pending cleanup. The discovery agent tracks live endpoints, so an exited entry that lingers
// until the unlink daemon drains it must not gate the agent's spawn or self-exit.
static int get_alive_process_num_in_domain(
  const struct ipc_namespace * ipc_ns, const uint32_t domain_id)
{
  int count = 0;
  struct process_info * proc_info;
  int bkt_proc_info;
  hash_for_each(proc_info_htable, bkt_proc_info, proc_info, node)
  {
    if (
      ipc_eq(ipc_ns, proc_info->ipc_ns) && proc_info->domain_id == domain_id &&
      !proc_info->exited) {
      count++;
    }
  }
  return count;
}

int agnocast_ioctl_notify_bridge_shutdown(const pid_t pid)
{
  down_write(&global_htables_rwsem);
  struct process_info * proc_info = agnocast_find_process_info(pid);
  if (proc_info) {
    proc_info->is_bridge_manager = false;
  }
  up_write(&global_htables_rwsem);
  return 0;
}

// Caller holds global_htables_rwsem (read). Scans because the table is keyed by pid.
struct discovery_agent_info * agnocast_find_discovery_agent(
  const struct ipc_namespace * ipc_ns, const uint32_t domain_id)
{
  struct discovery_agent_info * agent;
  int bkt;
  hash_for_each(discovery_agent_htable, bkt, agent, node)
  {
    if (ipc_eq(agent->ipc_ns, ipc_ns) && agent->domain_id == domain_id) {
      return agent;
    }
  }
  return NULL;
}

// The agent is not a registered Agnocast process, so it passes its own pid + ROS_DOMAIN_ID.
//
// commit == false: read-only idle poll; userspace counts consecutive idle polls before exiting.
// commit == true: the atomic exit gate. Deregister iff the domain is truly empty, under the same
// write lock add_process takes -- so either a starting process is counted first and vetoes the
// exit (ret_should_exit = false), or the agent deregisters first and that process then spawns a
// replacement. Neither ordering leaves live processes without an agent.
int agnocast_ioctl_discovery_agent_should_exit(
  const pid_t pid, const struct ipc_namespace * ipc_ns, const uint32_t domain_id, const bool commit,
  bool * ret_should_exit)
{
  if (!commit) {
    down_read(&global_htables_rwsem);
    *ret_should_exit = (get_alive_process_num_in_domain(ipc_ns, domain_id) == 0);
    up_read(&global_htables_rwsem);
    return 0;
  }

  down_write(&global_htables_rwsem);
  if (get_alive_process_num_in_domain(ipc_ns, domain_id) == 0) {
    agnocast_remove_discovery_agent_by_pid(pid);
    *ret_should_exit = true;
  } else {
    *ret_should_exit = false;
  }
  up_write(&global_htables_rwsem);
  return 0;
}

// Atomic singleton claim; replaces the userspace flock. Reports whether the caller owns the
// (ns, domain) slot once the call returns; a caller that does not must exit.
int agnocast_ioctl_add_discovery_agent(
  const pid_t pid, const struct ipc_namespace * ipc_ns, const uint32_t domain_id,
  struct ioctl_add_discovery_agent_args * ioctl_ret)
{
  int ret = 0;
  struct discovery_agent_info * existing;
  // Not owning it is the safe answer, so every early exit including -ENOMEM leaves this false.
  ioctl_ret->ret_owned_by_caller = false;
  down_write(&global_htables_rwsem);

  existing = agnocast_find_discovery_agent(ipc_ns, domain_id);
  if (existing) {
    ioctl_ret->ret_owned_by_caller = (existing->pid == pid);
    goto unlock;
  }

  struct discovery_agent_info * agent = kmalloc(sizeof(struct discovery_agent_info), GFP_KERNEL);
  if (!agent) {
    ret = -ENOMEM;
    goto unlock;
  }
  agent->pid = pid;
  agent->ipc_ns = ipc_ns;
  agent->domain_id = domain_id;
  INIT_HLIST_NODE(&agent->node);
  hash_add_rcu(discovery_agent_htable, &agent->node, hash_min(pid, DISCOVERY_AGENT_HASH_BITS));
  ioctl_ret->ret_owned_by_caller = true;

unlock:
  up_write(&global_htables_rwsem);
  return ret;
}

// Read-only liveness query for the CLI status verb. Now that the kmod owns agent liveness, this
// is the authoritative "is the agent alive?" signal (replacing the userspace flock probe).
int agnocast_ioctl_discovery_agent_exists(
  const struct ipc_namespace * ipc_ns, const uint32_t domain_id, bool * ret_exists)
{
  down_read(&global_htables_rwsem);
  *ret_exists = (agnocast_find_discovery_agent(ipc_ns, domain_id) != NULL);
  up_read(&global_htables_rwsem);
  return 0;
}

int agnocast_ioctl_check_and_request_bridge_shutdown(
  const pid_t pid, const struct ipc_namespace * ipc_ns,
  struct ioctl_check_and_request_bridge_shutdown_args * ioctl_ret)
{
  down_write(&global_htables_rwsem);
  // A bridge manager is per (ipc_ns, domain), so it must shut down once its
  // own domain is empty -- counting the whole namespace would keep it alive while an
  // unrelated domain is busy. The manager itself is the remaining process (count == 1),
  // and poll_for_unlink is not registered here, so it is excluded.
  if (get_process_num_in_domain(ipc_ns, get_process_domain_id(pid)) <= 1) {
    struct process_info * proc_info = agnocast_find_process_info(pid);
    if (proc_info) {
      proc_info->is_bridge_manager = false;
    }
    ioctl_ret->ret_should_shutdown = true;
  } else {
    ioctl_ret->ret_should_shutdown = false;
  }
  up_write(&global_htables_rwsem);
  return 0;
}

static long get_version_cmd(struct ioctl_get_version_args __user * arg)
{
  int ret = 0;

  struct ioctl_get_version_args get_version_args;
  memset(&get_version_args, 0, sizeof(get_version_args));
  ret = agnocast_ioctl_get_version(&get_version_args);
  if (copy_to_user(arg, &get_version_args, sizeof(get_version_args))) return -EFAULT;
  return ret;
}

static long add_process_cmd(union ioctl_add_process_args __user * arg)
{
  int ret = 0;
  const pid_t pid = current->tgid;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  union ioctl_add_process_args add_process_args;
  if (copy_from_user(&add_process_args, arg, sizeof(add_process_args))) return -EFAULT;
  bool is_bridge_manager = add_process_args.is_bridge_manager;
  uint32_t domain_id = add_process_args.domain_id;
  ret = agnocast_ioctl_add_process(pid, ipc_ns, is_bridge_manager, domain_id, &add_process_args);
  if (ret == 0) {
    if (copy_to_user(arg, &add_process_args, sizeof(add_process_args))) return -EFAULT;
  }
  return ret;
}

static long add_subscriber_cmd(union ioctl_add_subscriber_args __user * arg)
{
  int ret = 0;
  const pid_t pid = current->tgid;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  union ioctl_add_subscriber_args sub_args;
  if (copy_from_user(&sub_args, arg, sizeof(sub_args))) return -EFAULT;

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  char node_name_buf[NODE_NAME_BUFFER_SIZE];
  ret = copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &sub_args.topic_name);
  if (ret) return ret;

  ret = copy_name_from_user(node_name_buf, sizeof(node_name_buf), &sub_args.node_name);
  if (ret) return ret;

  ret = agnocast_ioctl_add_subscriber(
    topic_name_buf, ipc_ns, node_name_buf, pid, sub_args.qos_depth, sub_args.qos_is_transient_local,
    sub_args.qos_is_reliable, sub_args.is_take_sub, sub_args.ignore_local_publications,
    sub_args.is_bridge, sub_args.eventfd, &sub_args);
  if (ret == 0) {
    if (copy_to_user(arg, &sub_args, sizeof(sub_args))) return -EFAULT;
  }
  return ret;
}

static long add_publisher_cmd(union ioctl_add_publisher_args __user * arg)
{
  int ret = 0;
  const pid_t pid = current->tgid;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  union ioctl_add_publisher_args pub_args;
  if (copy_from_user(&pub_args, arg, sizeof(pub_args))) return -EFAULT;

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  char node_name_buf[NODE_NAME_BUFFER_SIZE];
  ret = copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &pub_args.topic_name);
  if (ret) return ret;

  ret = copy_name_from_user(node_name_buf, sizeof(node_name_buf), &pub_args.node_name);
  if (ret) return ret;

  ret = agnocast_ioctl_add_publisher(
    topic_name_buf, ipc_ns, node_name_buf, pid, pub_args.qos_depth, pub_args.qos_is_transient_local,
    pub_args.is_bridge, &pub_args);
  if (ret == 0) {
    if (copy_to_user(arg, &pub_args, sizeof(pub_args))) return -EFAULT;
  }
  return ret;
}

static long release_sub_ref_cmd(struct ioctl_update_entry_args __user * arg)
{
  int ret = 0;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  struct ioctl_update_entry_args entry_args;
  if (copy_from_user(&entry_args, arg, sizeof(entry_args))) return -EFAULT;

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  ret = copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &entry_args.topic_name);
  if (ret) return ret;

  ret = agnocast_ioctl_release_message_entry_reference(
    topic_name_buf, ipc_ns, entry_args.pubsub_id, entry_args.entry_id);
  return ret;
}

static long receive_msg_cmd(union ioctl_receive_msg_args __user * arg)
{
  int ret = 0;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  union ioctl_receive_msg_args receive_msg_args;
  if (copy_from_user(&receive_msg_args, arg, sizeof(receive_msg_args))) return -EFAULT;

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  ret = copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &receive_msg_args.topic_name);
  if (ret) return ret;

  uint64_t pub_shm_info_addr = receive_msg_args.pub_shm_info_addr;
  uint32_t pub_shm_info_size = receive_msg_args.pub_shm_info_size;
  if (pub_shm_info_size > MAX_PUBLISHER_NUM) {
    return -EINVAL;
  }

  struct publisher_shm_info * pub_shm_infos =
    kcalloc(pub_shm_info_size, sizeof(struct publisher_shm_info), GFP_KERNEL);
  if (!pub_shm_infos) {
    return -ENOMEM;
  }

  ret = agnocast_ioctl_receive_msg(
    topic_name_buf, ipc_ns, receive_msg_args.subscriber_id, pub_shm_infos, pub_shm_info_size,
    &receive_msg_args);

  if (ret == 0 && receive_msg_args.ret_pub_shm_num > 0) {
    if (copy_to_user(
          (struct publisher_shm_info __user *)pub_shm_info_addr, pub_shm_infos,
          receive_msg_args.ret_pub_shm_num * sizeof(struct publisher_shm_info))) {
      kfree(pub_shm_infos);
      return -EFAULT;
    }
  }
  kfree(pub_shm_infos);

  if (ret == 0) {
    if (copy_to_user(arg, &receive_msg_args, sizeof(receive_msg_args))) return -EFAULT;
  }
  return ret;
}

static long publish_msg_cmd(union ioctl_publish_msg_args __user * arg)
{
  int ret = 0;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  union ioctl_publish_msg_args publish_msg_args;
  if (copy_from_user(&publish_msg_args, arg, sizeof(publish_msg_args))) return -EFAULT;

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  ret = copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &publish_msg_args.topic_name);
  if (ret) return ret;

  ret = agnocast_ioctl_publish_msg(
    topic_name_buf, ipc_ns, publish_msg_args.publisher_id, publish_msg_args.msg_virtual_address,
    &publish_msg_args);

  // NOTE: the entry is already inserted and every subscriber eventfd already signalled, so
  // -EFAULT here means the publication happened and only its results failed to reach the
  // publisher; the woken subscribers still receive a valid entry. The signalling cannot be
  // deferred past this copy because the contexts are only valid under global_htables_rwsem,
  // which agnocast_ioctl_publish_msg() drops on return.
  if (ret == 0) {
    if (copy_to_user(arg, &publish_msg_args, sizeof(publish_msg_args))) return -EFAULT;
  }
  return ret;
}

static long take_msg_cmd(union ioctl_take_msg_args __user * arg)
{
  int ret = 0;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  union ioctl_take_msg_args take_args;
  if (copy_from_user(&take_args, arg, sizeof(take_args))) return -EFAULT;

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  ret = copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &take_args.topic_name);
  if (ret) return ret;

  uint64_t pub_shm_info_addr = take_args.pub_shm_info_addr;
  uint32_t pub_shm_info_size = take_args.pub_shm_info_size;
  if (pub_shm_info_size > MAX_PUBLISHER_NUM) {
    return -EINVAL;
  }

  struct publisher_shm_info * pub_shm_infos =
    kcalloc(pub_shm_info_size, sizeof(struct publisher_shm_info), GFP_KERNEL);
  if (!pub_shm_infos) {
    return -ENOMEM;
  }

  ret = agnocast_ioctl_take_msg(
    topic_name_buf, ipc_ns, take_args.subscriber_id, take_args.allow_same_message, pub_shm_infos,
    pub_shm_info_size, &take_args);

  if (ret == 0 && take_args.ret_pub_shm_num > 0) {
    if (copy_to_user(
          (struct publisher_shm_info __user *)pub_shm_info_addr, pub_shm_infos,
          take_args.ret_pub_shm_num * sizeof(struct publisher_shm_info))) {
      kfree(pub_shm_infos);
      return -EFAULT;
    }
  }
  kfree(pub_shm_infos);

  if (ret == 0) {
    if (copy_to_user(arg, &take_args, sizeof(take_args))) return -EFAULT;
  }
  return ret;
}

static long get_subscriber_num_cmd(union ioctl_get_subscriber_num_args __user * arg)
{
  int ret = 0;
  const pid_t pid = current->tgid;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  union ioctl_get_subscriber_num_args get_subscriber_num_args;
  if (copy_from_user(&get_subscriber_num_args, arg, sizeof(get_subscriber_num_args)))
    return -EFAULT;

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  ret = copy_name_from_user(
    topic_name_buf, sizeof(topic_name_buf), &get_subscriber_num_args.topic_name);
  if (ret) return ret;

  ret = agnocast_ioctl_get_subscriber_num(topic_name_buf, ipc_ns, pid, &get_subscriber_num_args);
  if (copy_to_user(arg, &get_subscriber_num_args, sizeof(get_subscriber_num_args))) return -EFAULT;
  return ret;
}

static long get_publisher_num_cmd(union ioctl_get_publisher_num_args __user * arg)
{
  int ret = 0;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  union ioctl_get_publisher_num_args get_publisher_num_args;
  if (copy_from_user(&get_publisher_num_args, arg, sizeof(get_publisher_num_args))) return -EFAULT;

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  ret =
    copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &get_publisher_num_args.topic_name);
  if (ret) return ret;

  ret = agnocast_ioctl_get_publisher_num(topic_name_buf, ipc_ns, &get_publisher_num_args);
  if (copy_to_user(arg, &get_publisher_num_args, sizeof(get_publisher_num_args))) return -EFAULT;
  return ret;
}

static long get_exit_process_cmd(struct ioctl_get_exit_process_args __user * arg)
{
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  struct ioctl_get_exit_process_args get_exit_process_args = {};

  const pid_t global_pid = agnocast_ioctl_get_exit_process(ipc_ns, &get_exit_process_args);

  // Copy ret_pid to user-space BEFORE commit.
  // ret_daemon_should_exit is not yet known and will be patched after commit.
  if (copy_to_user(arg, &get_exit_process_args, sizeof(get_exit_process_args))) return -EFAULT;

  // Commit: free proc_info. Safe because user-space already has ret_pid.
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(ipc_ns, global_pid, &daemon_should_exit);

  // Patch ret_daemon_should_exit. Not fatal: when a pid was returned, its proc_info has already
  // been committed, so -EFAULT would make the daemon exit while discarding the ret_pid whose shm
  // needs unlinking; the flag is advisory and re-derived on the next poll.
  if (copy_to_user(&arg->ret_daemon_should_exit, &daemon_should_exit, sizeof(daemon_should_exit))) {
    dev_warn(agnocast_device, "Failed to report the daemon exit flag. (%s)\n", __func__);
  }
  return 0;
}

static long get_topic_list_cmd(union ioctl_topic_list_args __user * arg)
{
  int ret = 0;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  union ioctl_topic_list_args topic_list_args;
  if (copy_from_user(&topic_list_args, arg, sizeof(topic_list_args))) return -EFAULT;
  ret = agnocast_ioctl_get_topic_list(ipc_ns, &topic_list_args);
  if (ret == 0) {
    if (copy_to_user(arg, &topic_list_args, sizeof(topic_list_args))) return -EFAULT;
  }
  return ret;
}

static long get_node_subscriber_topics_cmd(union ioctl_node_info_args __user * arg)
{
  int ret = 0;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  union ioctl_node_info_args node_info_sub_args;
  if (copy_from_user(&node_info_sub_args, arg, sizeof(node_info_sub_args))) return -EFAULT;

  char node_name_buf[NODE_NAME_BUFFER_SIZE];
  ret = copy_name_from_user(node_name_buf, sizeof(node_name_buf), &node_info_sub_args.node_name);
  if (ret) return ret;

  ret = agnocast_ioctl_get_node_subscriber_topics(ipc_ns, node_name_buf, &node_info_sub_args);
  if (ret == 0) {
    if (copy_to_user(arg, &node_info_sub_args, sizeof(node_info_sub_args))) return -EFAULT;
  }
  return ret;
}

static long get_node_publisher_topics_cmd(union ioctl_node_info_args __user * arg)
{
  int ret = 0;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  union ioctl_node_info_args node_info_pub_args;
  if (copy_from_user(&node_info_pub_args, arg, sizeof(node_info_pub_args))) return -EFAULT;

  char node_name_buf[NODE_NAME_BUFFER_SIZE];
  ret = copy_name_from_user(node_name_buf, sizeof(node_name_buf), &node_info_pub_args.node_name);
  if (ret) return ret;

  ret = agnocast_ioctl_get_node_publisher_topics(ipc_ns, node_name_buf, &node_info_pub_args);
  if (ret == 0) {
    if (copy_to_user(arg, &node_info_pub_args, sizeof(node_info_pub_args))) return -EFAULT;
  }
  return ret;
}

static long get_topic_subscriber_info_cmd(union ioctl_topic_info_args __user * arg)
{
  int ret = 0;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  union ioctl_topic_info_args topic_info_sub_args;
  if (copy_from_user(&topic_info_sub_args, arg, sizeof(topic_info_sub_args))) return -EFAULT;

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  ret =
    copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &topic_info_sub_args.topic_name);
  if (ret) return ret;

  ret = agnocast_ioctl_get_topic_subscriber_info(topic_name_buf, ipc_ns, &topic_info_sub_args);
  if (ret == 0) {
    if (copy_to_user(arg, &topic_info_sub_args, sizeof(topic_info_sub_args))) return -EFAULT;
  }
  return ret;
}

static long get_topic_publisher_info_cmd(union ioctl_topic_info_args __user * arg)
{
  int ret = 0;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  union ioctl_topic_info_args topic_info_pub_args;
  if (copy_from_user(&topic_info_pub_args, arg, sizeof(topic_info_pub_args))) return -EFAULT;

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  ret =
    copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &topic_info_pub_args.topic_name);
  if (ret) return ret;

  ret = agnocast_ioctl_get_topic_publisher_info(topic_name_buf, ipc_ns, &topic_info_pub_args);
  if (ret == 0) {
    if (copy_to_user(arg, &topic_info_pub_args, sizeof(topic_info_pub_args))) return -EFAULT;
  }
  return ret;
}

static long get_subscriber_qos_cmd(struct ioctl_get_subscriber_qos_args __user * arg)
{
  int ret = 0;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  struct ioctl_get_subscriber_qos_args get_sub_qos_args;
  if (copy_from_user(&get_sub_qos_args, arg, sizeof(get_sub_qos_args))) return -EFAULT;

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  ret = copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &get_sub_qos_args.topic_name);
  if (ret) return ret;

  ret = agnocast_ioctl_get_subscriber_qos(
    topic_name_buf, ipc_ns, get_sub_qos_args.subscriber_id, &get_sub_qos_args);
  if (ret == 0) {
    if (copy_to_user(arg, &get_sub_qos_args, sizeof(get_sub_qos_args))) return -EFAULT;
  }
  return ret;
}

static long get_publisher_qos_cmd(struct ioctl_get_publisher_qos_args __user * arg)
{
  int ret = 0;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  struct ioctl_get_publisher_qos_args get_pub_qos_args;
  if (copy_from_user(&get_pub_qos_args, arg, sizeof(get_pub_qos_args))) return -EFAULT;

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  ret = copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &get_pub_qos_args.topic_name);
  if (ret) return ret;

  ret = agnocast_ioctl_get_publisher_qos(
    topic_name_buf, ipc_ns, get_pub_qos_args.publisher_id, &get_pub_qos_args);
  if (ret == 0) {
    if (copy_to_user(arg, &get_pub_qos_args, sizeof(get_pub_qos_args))) return -EFAULT;
  }
  return ret;
}

static long remove_subscriber_cmd(struct ioctl_remove_subscriber_args __user * arg)
{
  int ret = 0;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  struct ioctl_remove_subscriber_args remove_subscriber_args;
  if (copy_from_user(&remove_subscriber_args, arg, sizeof(remove_subscriber_args))) {
    return -EFAULT;
  }

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  ret =
    copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &remove_subscriber_args.topic_name);
  if (ret) return ret;

  ret =
    agnocast_ioctl_remove_subscriber(topic_name_buf, ipc_ns, remove_subscriber_args.subscriber_id);
  return ret;
}

static long remove_publisher_cmd(struct ioctl_remove_publisher_args __user * arg)
{
  int ret = 0;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  struct ioctl_remove_publisher_args remove_publisher_args;
  if (copy_from_user(&remove_publisher_args, arg, sizeof(remove_publisher_args))) {
    return -EFAULT;
  }

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  ret =
    copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &remove_publisher_args.topic_name);
  if (ret) return ret;

  ret = agnocast_ioctl_remove_publisher(topic_name_buf, ipc_ns, remove_publisher_args.publisher_id);
  return ret;
}

static long add_bridge_cmd(struct ioctl_add_bridge_args __user * arg)
{
  int ret = 0;
  const pid_t pid = current->tgid;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  struct ioctl_add_bridge_args bridge_args;
  if (copy_from_user(&bridge_args, arg, sizeof(bridge_args))) return -EFAULT;

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  ret = copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &bridge_args.topic_name);
  if (ret) return ret;

  ret = agnocast_ioctl_add_bridge(topic_name_buf, pid, bridge_args.is_r2a, ipc_ns, &bridge_args);
  if (ret == 0 || ret == -EEXIST) {
    if (copy_to_user((struct ioctl_add_bridge_args __user *)arg, &bridge_args, sizeof(bridge_args)))
      return -EFAULT;
  }
  return ret;
}

static long add_domain_bridge_cmd(struct ioctl_add_domain_bridge_args __user * arg)
{
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  struct ioctl_add_domain_bridge_args domain_bridge_args;
  if (copy_from_user(&domain_bridge_args, arg, sizeof(domain_bridge_args))) return -EFAULT;

  char from_name_buf[TOPIC_NAME_BUFFER_SIZE];
  char to_name_buf[TOPIC_NAME_BUFFER_SIZE];
  int ret =
    copy_name_from_user(from_name_buf, sizeof(from_name_buf), &domain_bridge_args.topic_name_from);
  if (ret) return ret;
  ret = copy_name_from_user(to_name_buf, sizeof(to_name_buf), &domain_bridge_args.topic_name_to);
  if (ret) return ret;

  return agnocast_ioctl_add_domain_bridge(
    from_name_buf, to_name_buf, domain_bridge_args.from_domain, domain_bridge_args.to_domain,
    ipc_ns);
}

static long remove_bridge_cmd(struct ioctl_remove_bridge_args __user * arg)
{
  int ret = 0;
  const pid_t pid = current->tgid;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  struct ioctl_remove_bridge_args remove_bridge_args;
  if (copy_from_user(&remove_bridge_args, arg, sizeof(remove_bridge_args))) return -EFAULT;

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  ret = copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &remove_bridge_args.topic_name);
  if (ret) return ret;

  ret = agnocast_ioctl_remove_bridge(topic_name_buf, pid, remove_bridge_args.is_r2a, ipc_ns);
  return ret;
}

static long check_and_request_bridge_shutdown_cmd(
  struct ioctl_check_and_request_bridge_shutdown_args __user * arg)
{
  int ret = 0;
  const pid_t pid = current->tgid;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  struct ioctl_check_and_request_bridge_shutdown_args shutdown_args;
  memset(&shutdown_args, 0, sizeof(shutdown_args));
  ret = agnocast_ioctl_check_and_request_bridge_shutdown(pid, ipc_ns, &shutdown_args);
  if (copy_to_user(
        (struct ioctl_check_and_request_bridge_shutdown_args __user *)arg, &shutdown_args,
        sizeof(shutdown_args)))
    return -EFAULT;
  return ret;
}

static long set_ros2_subscriber_num_cmd(struct ioctl_set_ros2_subscriber_num_args __user * arg)
{
  int ret = 0;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  struct ioctl_set_ros2_subscriber_num_args set_ros2_sub_args;
  if (copy_from_user(&set_ros2_sub_args, arg, sizeof(set_ros2_sub_args))) return -EFAULT;

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  ret = copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &set_ros2_sub_args.topic_name);
  if (ret) return ret;

  ret = agnocast_ioctl_set_ros2_subscriber_num(
    topic_name_buf, ipc_ns, set_ros2_sub_args.ros2_subscriber_num);
  return ret;
}

static long set_ros2_publisher_num_cmd(struct ioctl_set_ros2_publisher_num_args __user * arg)
{
  int ret = 0;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  struct ioctl_set_ros2_publisher_num_args set_ros2_pub_args;
  if (copy_from_user(&set_ros2_pub_args, arg, sizeof(set_ros2_pub_args))) return -EFAULT;

  char topic_name_buf[TOPIC_NAME_BUFFER_SIZE];
  ret = copy_name_from_user(topic_name_buf, sizeof(topic_name_buf), &set_ros2_pub_args.topic_name);
  if (ret) return ret;

  ret = agnocast_ioctl_set_ros2_publisher_num(
    topic_name_buf, ipc_ns, set_ros2_pub_args.ros2_publisher_num);
  return ret;
}

static long notify_bridge_shutdown_cmd(void)
{
  int ret = 0;
  const pid_t pid = current->tgid;

  ret = agnocast_ioctl_notify_bridge_shutdown(pid);
  return ret;
}

static long discovery_agent_should_exit_cmd(
  struct ioctl_discovery_agent_should_exit_args __user * arg)
{
  const pid_t pid = current->tgid;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  struct ioctl_discovery_agent_should_exit_args args;
  if (copy_from_user(&args, arg, sizeof(args))) return -EFAULT;

  int ret = agnocast_ioctl_discovery_agent_should_exit(
    pid, ipc_ns, args.domain_id, args.commit, &args.ret_should_exit);
  if (copy_to_user(arg, &args, sizeof(args))) return -EFAULT;
  return ret;
}

static long add_discovery_agent_cmd(struct ioctl_add_discovery_agent_args __user * arg)
{
  const pid_t pid = current->tgid;
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  struct ioctl_add_discovery_agent_args args;
  if (copy_from_user(&args, arg, sizeof(args))) return -EFAULT;

  int ret = agnocast_ioctl_add_discovery_agent(pid, ipc_ns, args.domain_id, &args);
  if (copy_to_user(arg, &args, sizeof(args))) return -EFAULT;
  return ret;
}

static long discovery_agent_exists_cmd(struct ioctl_discovery_agent_exists_args __user * arg)
{
  const struct ipc_namespace * ipc_ns = current->nsproxy->ipc_ns;

  struct ioctl_discovery_agent_exists_args args;
  if (copy_from_user(&args, arg, sizeof(args))) return -EFAULT;

  int ret = agnocast_ioctl_discovery_agent_exists(ipc_ns, args.domain_id, &args.ret_exists);
  if (copy_to_user(arg, &args, sizeof(args))) return -EFAULT;
  return ret;
}

long agnocast_ioctl(struct file * file, unsigned int cmd, unsigned long arg)
{
  switch (cmd) {
    case AGNOCAST_GET_VERSION_CMD:
      return get_version_cmd((struct ioctl_get_version_args __user *)arg);
    case AGNOCAST_ADD_PROCESS_CMD:
      return add_process_cmd((union ioctl_add_process_args __user *)arg);
    case AGNOCAST_ADD_SUBSCRIBER_CMD:
      return add_subscriber_cmd((union ioctl_add_subscriber_args __user *)arg);
    case AGNOCAST_ADD_PUBLISHER_CMD:
      return add_publisher_cmd((union ioctl_add_publisher_args __user *)arg);
    case AGNOCAST_RELEASE_SUB_REF_CMD:
      return release_sub_ref_cmd((struct ioctl_update_entry_args __user *)arg);
    case AGNOCAST_RECEIVE_MSG_CMD:
      return receive_msg_cmd((union ioctl_receive_msg_args __user *)arg);
    case AGNOCAST_PUBLISH_MSG_CMD:
      return publish_msg_cmd((union ioctl_publish_msg_args __user *)arg);
    case AGNOCAST_TAKE_MSG_CMD:
      return take_msg_cmd((union ioctl_take_msg_args __user *)arg);
    case AGNOCAST_GET_SUBSCRIBER_NUM_CMD:
      return get_subscriber_num_cmd((union ioctl_get_subscriber_num_args __user *)arg);
    case AGNOCAST_GET_PUBLISHER_NUM_CMD:
      return get_publisher_num_cmd((union ioctl_get_publisher_num_args __user *)arg);
    case AGNOCAST_GET_EXIT_PROCESS_CMD:
      return get_exit_process_cmd((struct ioctl_get_exit_process_args __user *)arg);
    case AGNOCAST_GET_TOPIC_LIST_CMD:
      return get_topic_list_cmd((union ioctl_topic_list_args __user *)arg);
    case AGNOCAST_GET_NODE_SUBSCRIBER_TOPICS_CMD:
      return get_node_subscriber_topics_cmd((union ioctl_node_info_args __user *)arg);
    case AGNOCAST_GET_NODE_PUBLISHER_TOPICS_CMD:
      return get_node_publisher_topics_cmd((union ioctl_node_info_args __user *)arg);
    case AGNOCAST_GET_TOPIC_SUBSCRIBER_INFO_CMD:
      return get_topic_subscriber_info_cmd((union ioctl_topic_info_args __user *)arg);
    case AGNOCAST_GET_TOPIC_PUBLISHER_INFO_CMD:
      return get_topic_publisher_info_cmd((union ioctl_topic_info_args __user *)arg);
    case AGNOCAST_GET_SUBSCRIBER_QOS_CMD:
      return get_subscriber_qos_cmd((struct ioctl_get_subscriber_qos_args __user *)arg);
    case AGNOCAST_GET_PUBLISHER_QOS_CMD:
      return get_publisher_qos_cmd((struct ioctl_get_publisher_qos_args __user *)arg);
    case AGNOCAST_REMOVE_SUBSCRIBER_CMD:
      return remove_subscriber_cmd((struct ioctl_remove_subscriber_args __user *)arg);
    case AGNOCAST_REMOVE_PUBLISHER_CMD:
      return remove_publisher_cmd((struct ioctl_remove_publisher_args __user *)arg);
    case AGNOCAST_ADD_BRIDGE_CMD:
      return add_bridge_cmd((struct ioctl_add_bridge_args __user *)arg);
    case AGNOCAST_REMOVE_BRIDGE_CMD:
      return remove_bridge_cmd((struct ioctl_remove_bridge_args __user *)arg);
    case AGNOCAST_ADD_DOMAIN_BRIDGE_CMD:
      return add_domain_bridge_cmd((struct ioctl_add_domain_bridge_args __user *)arg);
    case AGNOCAST_CHECK_AND_REQUEST_BRIDGE_SHUTDOWN_CMD:
      return check_and_request_bridge_shutdown_cmd(
        (struct ioctl_check_and_request_bridge_shutdown_args __user *)arg);
    case AGNOCAST_SET_ROS2_SUBSCRIBER_NUM_CMD:
      return set_ros2_subscriber_num_cmd((struct ioctl_set_ros2_subscriber_num_args __user *)arg);
    case AGNOCAST_SET_ROS2_PUBLISHER_NUM_CMD:
      return set_ros2_publisher_num_cmd((struct ioctl_set_ros2_publisher_num_args __user *)arg);
    case AGNOCAST_NOTIFY_BRIDGE_SHUTDOWN_CMD:
      return notify_bridge_shutdown_cmd();
    case AGNOCAST_DISCOVERY_AGENT_SHOULD_EXIT_CMD:
      return discovery_agent_should_exit_cmd(
        (struct ioctl_discovery_agent_should_exit_args __user *)arg);
    case AGNOCAST_ADD_DISCOVERY_AGENT_CMD:
      return add_discovery_agent_cmd((struct ioctl_add_discovery_agent_args __user *)arg);
    case AGNOCAST_DISCOVERY_AGENT_EXISTS_CMD:
      return discovery_agent_exists_cmd((struct ioctl_discovery_agent_exists_args __user *)arg);
    default:
      return -EINVAL;
  }
}

// =========================================
// helper functions for KUnit test

#ifdef KUNIT_BUILD

// Add subscriber reference to message entry (set boolean flag to true).
// Called when subscriber first receives/takes the message.
int agnocast_increment_message_entry_rc(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const topic_local_id_t pubsub_id,
  const int64_t entry_id)
{
  int ret = 0;

  down_read(&global_htables_rwsem);

  struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (!wrapper) {
    dev_warn(
      agnocast_device, "Topic (topic_name=%s) not found. (increment_message_entry_rc)\n",
      topic_name);
    ret = -EINVAL;
    goto unlock_only_global;
  }

  down_read(&wrapper->topic->rwsem);

  struct entry_node * en = find_message_entry(wrapper, entry_id);
  if (!en) {
    dev_warn(
      agnocast_device,
      "Message entry (topic_name=%s entry_id=%lld) not found. "
      "(increment_message_entry_rc)\n",
      topic_name, entry_id);
    ret = -EINVAL;
    goto unlock_all;
  }

  // Adding reference is allowed only for subscribers
  if (!find_subscriber_info(wrapper, pubsub_id)) {
    dev_warn(
      agnocast_device,
      "Subscriber (id=%d) not found in the topic (topic_name=%s). (increment_message_entry_rc)\n",
      pubsub_id, wrapper->key);
    ret = -EINVAL;
    goto unlock_all;
  }

  ret = add_subscriber_reference(en, pubsub_id, false);
  if (ret < 0) {
    goto unlock_all;
  }

unlock_all:
  up_read(&wrapper->topic->rwsem);
unlock_only_global:
  up_read(&global_htables_rwsem);
  return ret;
}

// No locking needed for the following KUnit helper functions.
// These are only called from single-threaded KUnit context.

int agnocast_get_alive_proc_num(void)
{
  int count = 0;
  struct process_info * proc_info;
  int bkt_proc_info;
  hash_for_each(proc_info_htable, bkt_proc_info, proc_info, node)
  {
    if (!proc_info->exited) {
      count++;
    }
  }
  return count;
}

int agnocast_get_discovery_agent_num(void)
{
  int count = 0;
  struct discovery_agent_info * agent;
  int bkt;
  // Serialize against the exit path's hash_del_rcu()+kfree_rcu(), which runs under the write lock.
  down_read(&global_htables_rwsem);
  hash_for_each(discovery_agent_htable, bkt, agent, node)
  {
    count++;
  }
  up_read(&global_htables_rwsem);
  return count;
}

bool agnocast_is_proc_exited(const pid_t pid)
{
  struct process_info * proc_info;
  hash_for_each_possible(proc_info_htable, proc_info, node, hash_min(pid, PROC_INFO_HASH_BITS))
  {
    if (proc_info->global_pid == pid) {
      if (proc_info->exited) {
        return true;
      } else {
        return false;
      }
    }
  }
  return false;
}

int agnocast_get_topic_entries_num(const char * topic_name, const struct ipc_namespace * ipc_ns)
{
  struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (!wrapper) {
    return 0;
  }

  struct rb_root * root = &wrapper->topic->entries;
  struct rb_node * node;
  int count = 0;
  for (node = rb_first(root); node; node = rb_next(node)) {
    count++;
  }
  return count;
}

bool agnocast_is_in_topic_entries(
  const char * topic_name, const struct ipc_namespace * ipc_ns, int64_t entry_id)
{
  struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (!wrapper) {
    return false;
  }
  const struct entry_node * en = find_message_entry(wrapper, entry_id);
  if (!en) {
    return false;
  }

  return true;
}

// Returns 1 if subscriber is holding a reference to the entry, 0 otherwise.
// Returns -1 if topic or entry not found.
int agnocast_get_entry_rc(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const int64_t entry_id,
  const topic_local_id_t pubsub_id)
{
  struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (!wrapper) {
    return -1;
  }

  const struct entry_node * en = find_message_entry(wrapper, entry_id);
  if (!en) {
    return -1;
  }

  if (pubsub_id < 0 || pubsub_id >= MAX_TOPIC_LOCAL_ID) {
    return -1;
  }

  return test_bit(pubsub_id, en->referencing_subscribers) ? 1 : 0;
}

int64_t agnocast_get_latest_received_entry_id(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t subscriber_id)
{
  const struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (!wrapper) {
    return -1;
  }
  const struct subscriber_info * sub_info = find_subscriber_info(wrapper, subscriber_id);
  if (!sub_info) {
    return -1;
  }

  return sub_info->latest_received_entry_id;
}

bool agnocast_is_in_subscriber_htable(
  const char * topic_name, const struct ipc_namespace * ipc_ns,
  const topic_local_id_t subscriber_id)
{
  const struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (!wrapper) {
    return false;
  }
  const struct subscriber_info * sub_info = find_subscriber_info(wrapper, subscriber_id);
  if (!sub_info) {
    return false;
  }
  return true;
}

bool agnocast_is_in_publisher_htable(
  const char * topic_name, const struct ipc_namespace * ipc_ns, const topic_local_id_t publisher_id)
{
  const struct topic_wrapper * wrapper = find_topic_for_current(topic_name, ipc_ns);
  if (!wrapper) {
    return false;
  }
  const struct publisher_info * pub_info = find_publisher_info(wrapper, publisher_id);
  if (!pub_info) {
    return false;
  }
  return true;
}

int agnocast_get_topic_num(const struct ipc_namespace * ipc_ns)
{
  int count = 0;
  struct topic_wrapper * wrapper;
  int bkt_wrapper;
  hash_for_each(topic_hashtable, bkt_wrapper, wrapper, node)
  {
    if (ipc_eq(wrapper->ipc_ns, ipc_ns)) {
      count++;
    }
  }
  return count;
}

bool agnocast_is_in_topic_htable(const char * topic_name, const struct ipc_namespace * ipc_ns)
{
  return find_topic_for_current(topic_name, ipc_ns) != NULL;
}

bool agnocast_is_in_bridge_htable(const char * topic_name, const struct ipc_namespace * ipc_ns)
{
  const struct bridge_info * br_info = find_bridge_info(topic_name, ipc_ns);
  return (br_info != NULL);
}

pid_t agnocast_get_bridge_owner_pid(const char * topic_name, const struct ipc_namespace * ipc_ns)
{
  const struct bridge_info * br_info = find_bridge_info(topic_name, ipc_ns);
  if (br_info) {
    return br_info->pid;
  }
  return -1;
}

bool agnocast_get_domain_rule(
  const char * topic_name, const struct ipc_namespace * ipc_ns, uint32_t domain,
  uint32_t * domain_a, uint32_t * domain_b, bool * a_to_b, bool * b_to_a)
{
  down_read(&global_htables_rwsem);
  const struct domain_bridge_rule * rule = find_domain_rule(topic_name, ipc_ns, domain);
  bool found = rule != NULL;
  if (found) {
    *domain_a = rule->domain_a;
    *domain_b = rule->domain_b;
    *a_to_b = rule->a_to_b;
    *b_to_a = rule->b_to_a;
  }
  up_read(&global_htables_rwsem);
  return found;
}

int agnocast_topic_wrapper_refcnt(
  const char * topic_name, const struct ipc_namespace * ipc_ns, uint32_t domain_id)
{
  down_read(&global_htables_rwsem);
  const struct topic_wrapper * wrapper = find_topic(topic_name, ipc_ns, domain_id);
  int refcnt = wrapper ? (int)wrapper->topic->wrapper_refcnt : 0;
  up_read(&global_htables_rwsem);
  return refcnt;
}

#endif
