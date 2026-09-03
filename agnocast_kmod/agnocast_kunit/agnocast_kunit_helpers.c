// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_helpers.h"

uint64_t agnocast_kunit_setup_process(
  struct kunit * test, const pid_t pid, const uint32_t domain_id)
{
  union ioctl_add_process_args add_process_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, domain_id, &add_process_args),
    0);
  return add_process_args.ret_addr;
}

topic_local_id_t agnocast_kunit_setup_publisher(
  struct kunit * test, const char * topic_name, const char * node_name, const pid_t pid,
  const uint32_t qos_depth, const bool qos_is_transient_local, const bool is_bridge)
{
  union ioctl_add_publisher_args add_publisher_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_publisher(
      topic_name, current->nsproxy->ipc_ns, node_name, pid, qos_depth, qos_is_transient_local,
      is_bridge, &add_publisher_args),
    0);
  return add_publisher_args.ret_id;
}

topic_local_id_t agnocast_kunit_setup_subscriber(
  struct kunit * test, const char * topic_name, const char * node_name, const pid_t pid,
  const uint32_t qos_depth, const bool qos_is_transient_local, const bool qos_is_reliable,
  const bool is_take_sub, const bool ignore_local_publications, const bool is_bridge,
  const int eventfd)
{
  union ioctl_add_subscriber_args add_subscriber_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_subscriber(
      topic_name, current->nsproxy->ipc_ns, node_name, pid, qos_depth, qos_is_transient_local,
      qos_is_reliable, is_take_sub, ignore_local_publications, is_bridge, eventfd,
      &add_subscriber_args),
    0);
  return add_subscriber_args.ret_id;
}

topic_local_id_t agnocast_kunit_setup_one_publisher(
  struct kunit * test, const char * topic_name, const char * node_name, const pid_t pid,
  const uint32_t qos_depth, const bool qos_is_transient_local, const bool is_bridge,
  const uint32_t domain_id, uint64_t * ret_addr)
{
  uint64_t addr = agnocast_kunit_setup_process(test, pid, domain_id);
  if (ret_addr) *ret_addr = addr;
  return agnocast_kunit_setup_publisher(
    test, topic_name, node_name, pid, qos_depth, qos_is_transient_local, is_bridge);
}

topic_local_id_t agnocast_kunit_setup_one_subscriber(
  struct kunit * test, const char * topic_name, const char * node_name, const pid_t pid,
  const uint32_t qos_depth, const bool qos_is_transient_local, const bool qos_is_reliable,
  const bool is_take_sub, const bool ignore_local_publications, const bool is_bridge,
  const int eventfd, const uint32_t domain_id)
{
  agnocast_kunit_setup_process(test, pid, domain_id);
  return agnocast_kunit_setup_subscriber(
    test, topic_name, node_name, pid, qos_depth, qos_is_transient_local, qos_is_reliable,
    is_take_sub, ignore_local_publications, is_bridge, eventfd);
}
