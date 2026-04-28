// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_internal.h"

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
      agnocast_remove_entry_node(wrapper, en);
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
    agnocast_free_exit_subscription_list(proc_info);
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
