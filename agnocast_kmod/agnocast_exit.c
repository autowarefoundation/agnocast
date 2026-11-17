// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_internal.h"

static void remove_all_topics(struct agnocast_ns * ns)
{
  struct topic_wrapper * wrapper;
  struct hlist_node * tmp;
  int bkt;

  // release frees the shared topic_struct (rbtree + pub/sub tables) once its
  // last wrapper goes; for a grouped pair that is the second wrapper.
  hash_for_each_safe(ns->topic_hashtable, bkt, tmp, wrapper, node)
  {
    agnocast_release_topic_wrapper(wrapper);
  }
}

static void remove_all_bridge_info(struct agnocast_ns * ns)
{
  struct bridge_info * br_info;
  int bkt;
  struct hlist_node * tmp;
  hash_for_each_safe(ns->bridge_htable, bkt, tmp, br_info, node)
  {
    hash_del(&br_info->node);
    kfree(br_info->topic_name);
    kfree(br_info);
  }
}

static void remove_all_domain_rules(struct agnocast_ns * ns)
{
  struct domain_bridge_rule * rule;
  int bkt;
  struct hlist_node * tmp;
  hash_for_each_safe(ns->domain_rule_htable, bkt, tmp, rule, node)
  {
    hash_del(&rule->node);
    kfree(rule->topic_name_a);
    kfree(rule->topic_name_b);
    kfree(rule);
  }
}

// No explicit synchronize_rcu() needed below: kfree_rcu() defers freeing until after the grace
// period.
static void remove_all_process_info(void)
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
}

static void remove_all_discovery_agents(void)
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

// Called during module unload. Not an ioctl function, so we manage locks here directly.
void agnocast_exit_free_data(void)
{
  struct agnocast_ns * ns;
  while ((ns = agnocast_ns_write_lock_any()) != NULL) {
    remove_all_topics(ns);
    remove_all_bridge_info(ns);
    remove_all_domain_rules(ns);
    agnocast_ns_retire_and_unlock(ns);
  }
  // Every namespace is retired, so an exit cleanup still running finds none to lock.
  remove_all_process_info();
  remove_all_discovery_agents();
}

void agnocast_exit_kthread(void)
{
  // kthread_stop() wakes the worker itself; the wait condition includes kthread_should_stop().
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
