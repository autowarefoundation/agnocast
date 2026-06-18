// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_domain_bridge.h"

#include "../agnocast.h"

#include <kunit/test.h>

static const char * TOPIC_NAME = "/kunit_test_domain_bridge_topic";

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
