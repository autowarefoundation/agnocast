// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_domain_bridge.h"

#include "../agnocast.h"

#include <kunit/test.h>

static const char * TOPIC_NAME = "/kunit_test_domain_bridge_topic";

static void setup_process_in_domain(struct kunit * test, const pid_t pid, const uint32_t domain_id)
{
  union ioctl_add_process_args args;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_process(pid, current->nsproxy->ipc_ns, false, domain_id, &args), 0);
}

static topic_local_id_t add_publisher_for(struct kunit * test, const pid_t pid)
{
  union ioctl_add_publisher_args args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_publisher(
      TOPIC_NAME, current->nsproxy->ipc_ns, "/kunit_node", pid, 1, false, false, &args),
    0);
  return args.ret_id;
}

static topic_local_id_t add_subscriber_for(struct kunit * test, const pid_t pid)
{
  union ioctl_add_subscriber_args args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_subscriber(
      TOPIC_NAME, current->nsproxy->ipc_ns, "/kunit_node", pid, 1, false, true, false, false, false,
      &args),
    0);
  return args.ret_id;
}

void test_case_add_domain_bridge_normal(struct kunit * test)
{
  int ret = agnocast_ioctl_add_domain_bridge(TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns);

  KUNIT_EXPECT_EQ(test, ret, 0);

  uint32_t domain_a = 0, domain_b = 0;
  bool a_to_b = false, b_to_a = false;
  KUNIT_ASSERT_TRUE(
    test, agnocast_get_domain_rule(
            TOPIC_NAME, current->nsproxy->ipc_ns, &domain_a, &domain_b, &a_to_b, &b_to_a));
  KUNIT_EXPECT_EQ(test, domain_a, 1);
  KUNIT_EXPECT_EQ(test, domain_b, 2);
  KUNIT_EXPECT_TRUE(test, a_to_b);
  KUNIT_EXPECT_FALSE(test, b_to_a);
}

void test_case_add_domain_bridge_same_domain_rejected(struct kunit * test)
{
  int ret = agnocast_ioctl_add_domain_bridge(TOPIC_NAME, 3, 3, current->nsproxy->ipc_ns);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_add_domain_bridge_reverse_direction(struct kunit * test)
{
  // Re-declaring the same pair in reverse records the reverse direction on the
  // existing rule rather than creating a second one.
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns), 0);
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, 2, 1, current->nsproxy->ipc_ns), 0);

  uint32_t domain_a = 0, domain_b = 0;
  bool a_to_b = false, b_to_a = false;
  KUNIT_ASSERT_TRUE(
    test, agnocast_get_domain_rule(
            TOPIC_NAME, current->nsproxy->ipc_ns, &domain_a, &domain_b, &a_to_b, &b_to_a));
  KUNIT_EXPECT_TRUE(test, a_to_b);
  KUNIT_EXPECT_TRUE(test, b_to_a);
}

void test_case_add_domain_bridge_third_domain_rejected(struct kunit * test)
{
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns), 0);
  // A different domain pair on the same topic is rejected (v1 is pair-only).
  int ret = agnocast_ioctl_add_domain_bridge(TOPIC_NAME, 1, 3, current->nsproxy->ipc_ns);
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
}

void test_case_add_domain_bridge_rejected_when_endpoint_exists(struct kunit * test)
{
  setup_process_in_domain(test, 1000, 1);
  add_publisher_for(test, 1000);

  // The topic's domain-1 id space is already allocated, so grouping is unsafe.
  int ret = agnocast_ioctl_add_domain_bridge(TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns);
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
}

void test_case_domain_bridge_groups_wrappers(struct kunit * test)
{
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns), 0);

  setup_process_in_domain(test, 1000, 1);
  add_publisher_for(test, 1000);
  setup_process_in_domain(test, 1001, 2);
  add_subscriber_for(test, 1001);

  // Both domains' wrappers point at one shared topic_struct (refcnt 2).
  KUNIT_EXPECT_EQ(test, agnocast_topic_wrapper_refcnt(TOPIC_NAME, current->nsproxy->ipc_ns, 1), 2);
  KUNIT_EXPECT_EQ(test, agnocast_topic_wrapper_refcnt(TOPIC_NAME, current->nsproxy->ipc_ns, 2), 2);
}
