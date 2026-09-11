// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_add_domain_bridge_prefix.h"

#include "../agnocast.h"

#include <kunit/test.h>

// A prefix rule bridges every name under PFX, pairing each with the identical name in the other
// domain. PFX_A stands for a caller's response topic; OUTSIDE lies beyond the prefix.
static const char * PFX = "/kunit_test_add_domain_bridge_prefix_";
static const char * PFX_A = "/kunit_test_add_domain_bridge_prefix_clientA";
static const char * OUTSIDE = "/kunit_test_add_domain_bridge_outside";

static void setup_process_in_domain(struct kunit * test, const pid_t pid, const uint32_t domain_id)
{
  union ioctl_add_process_args args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, domain_id, &args),
    0);
}

static void add_publisher_named(struct kunit * test, const pid_t pid, const char * topic_name)
{
  union ioctl_add_publisher_args args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_publisher(
      topic_name, current->nsproxy->ipc_ns, "/kunit_node", pid, 1, false, false, &args),
    0);
}

void test_case_add_domain_bridge_prefix_normal(struct kunit * test)
{
  // Act
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 2, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  uint32_t domain_a = 0, domain_b = 0;
  bool a_to_b = false, b_to_a = false;
  KUNIT_ASSERT_TRUE(
    test, agnocast_get_domain_rule(
            PFX_A, current->nsproxy->ipc_ns, 1, &domain_a, &domain_b, &a_to_b, &b_to_a));
  KUNIT_EXPECT_EQ(test, domain_a, 1);
  KUNIT_EXPECT_EQ(test, domain_b, 2);
  KUNIT_EXPECT_TRUE(test, a_to_b);
  KUNIT_EXPECT_FALSE(test, b_to_a);
}

void test_case_add_domain_bridge_prefix_same_domain_rejected(struct kunit * test)
{
  // Act
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX, 3, 3, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_add_domain_bridge_prefix_empty_rejected(struct kunit * test)
{
  // Act
  const int ret = agnocast_ioctl_add_domain_bridge_prefix("", 1, 2, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_add_domain_bridge_prefix_root_rejected(struct kunit * test)
{
  // Act
  const int ret = agnocast_ioctl_add_domain_bridge_prefix("/", 1, 2, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_add_domain_bridge_prefix_relative_rejected(struct kunit * test)
{
  // Act: the same prefix without its leading '/'.
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX + 1, 1, 2, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_add_domain_bridge_prefix_redeclaration_is_idempotent(struct kunit * test)
{
  // Arrange: a covered endpoint has joined.
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 2, current->nsproxy->ipc_ns), 0);
  setup_process_in_domain(test, 1000, 1);
  add_publisher_named(test, 1000, PFX_A);

  // Act
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 2, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
}

void test_case_add_domain_bridge_prefix_reverse_direction(struct kunit * test)
{
  // Arrange: no endpoint yet, so the reverse direction is still free to be added.
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 2, current->nsproxy->ipc_ns), 0);

  // Act
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX, 2, 1, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  uint32_t domain_a = 0, domain_b = 0;
  bool a_to_b = false, b_to_a = false;
  KUNIT_ASSERT_TRUE(
    test, agnocast_get_domain_rule(
            PFX_A, current->nsproxy->ipc_ns, 1, &domain_a, &domain_b, &a_to_b, &b_to_a));
  KUNIT_EXPECT_TRUE(test, a_to_b);
  KUNIT_EXPECT_TRUE(test, b_to_a);
}

void test_case_add_domain_bridge_prefix_accepted_beside_a_disjoint_prefix(struct kunit * test)
{
  // Arrange: neither prefix is a prefix of the other.
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge_prefix(OUTSIDE, 1, 2, current->nsproxy->ipc_ns), 0);

  // Act
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 2, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
}

void test_case_add_domain_bridge_prefix_accepted_beside_an_exact_rule(struct kunit * test)
{
  // Arrange: an exact rule on a name outside the prefix.
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(OUTSIDE, OUTSIDE, 1, 2, current->nsproxy->ipc_ns), 0);

  // Act
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 2, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
}

void test_case_add_domain_bridge_prefix_accepted_with_a_topic_outside_it(struct kunit * test)
{
  // Arrange: an endpoint has joined, but on a name the prefix does not cover.
  setup_process_in_domain(test, 1000, 1);
  add_publisher_named(test, 1000, OUTSIDE);

  // Act
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 2, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
}

void test_case_add_domain_bridge_prefix_accepted_with_a_covered_topic_elsewhere(struct kunit * test)
{
  // Arrange: a covered name has an endpoint, but in a domain this rule does not bridge.
  setup_process_in_domain(test, 1000, 3);
  add_publisher_named(test, 1000, PFX_A);

  // Act
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 2, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
}

void test_case_add_domain_bridge_prefix_nested_over_disjoint_domains(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 2, current->nsproxy->ipc_ns), 0);

  // Act
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX_A, 3, 4, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
}

void test_case_add_domain_bridge_prefix_repointed_pair_rejected(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 2, current->nsproxy->ipc_ns), 0);

  // Act: the same prefix, pointed at a different partner domain.
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 3, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
}

void test_case_add_domain_bridge_prefix_nested_rejected(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 2, current->nsproxy->ipc_ns), 0);

  // Act: a longer prefix, nesting inside the one already registered.
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX_A, 1, 2, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
}

void test_case_add_domain_bridge_prefix_nested_rejected_either_order(struct kunit * test)
{
  // Arrange: the longer prefix first, so the shorter one is the newcomer.
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge_prefix(PFX_A, 1, 2, current->nsproxy->ipc_ns), 0);

  // Act
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 2, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
}

void test_case_add_domain_bridge_prefix_late_reverse_direction_rejected(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 2, current->nsproxy->ipc_ns), 0);
  setup_process_in_domain(test, 1000, 1);
  add_publisher_named(test, 1000, PFX_A);

  // Act
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX, 2, 1, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
}

void test_case_add_domain_bridge_prefix_rejected_when_covered_endpoint_exists(struct kunit * test)
{
  // Arrange
  setup_process_in_domain(test, 1000, 1);
  add_publisher_named(test, 1000, PFX_A);

  // Act
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 2, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
}

void test_case_add_domain_bridge_prefix_over_exact_rejected(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(PFX_A, PFX_A, 1, 2, current->nsproxy->ipc_ns), 0);

  // Act
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 2, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
}

void test_case_add_domain_bridge_prefix_over_exact_rename_rejected(struct kunit * test)
{
  // Arrange: only the rule's destination cell falls under the prefix.
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(OUTSIDE, PFX_A, 1, 2, current->nsproxy->ipc_ns), 0);

  // Act
  const int ret = agnocast_ioctl_add_domain_bridge_prefix(PFX, 1, 2, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
}
