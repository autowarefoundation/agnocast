// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_add_subscriber.h"

#include "../agnocast.h"
#include "../agnocast_memory_allocator.h"

#include <kunit/test.h>

static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * NODE_NAME = "/kunit_test_node";
static const bool QOS_IS_TRANSIENT_LOCAL = false;
static const bool QOS_IS_RELIABLE = true;
static const bool IS_TAKE_SUB = false;
static const bool IGNORE_LOCAL_PUBLICATIONS = false;
static const bool IS_BRIDGE = false;

static void setup_process(struct kunit * test, const pid_t pid)
{
  union ioctl_add_process_args add_process_args;
  int ret = agnocast_ioctl_add_process(pid, current->nsproxy->ipc_ns, false, 0, &add_process_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

static void setup_process_domain(struct kunit * test, const pid_t pid, const uint32_t domain_id)
{
  union ioctl_add_process_args add_process_args;
  int ret =
    agnocast_ioctl_add_process(pid, current->nsproxy->ipc_ns, false, domain_id, &add_process_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

static int add_pubsub_pair(
  struct kunit * test, const pid_t pub_pid, const pid_t sub_pid, const uint32_t qos_depth)
{
  union ioctl_add_publisher_args add_publisher_args;
  int ret = agnocast_ioctl_add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, pub_pid, qos_depth, QOS_IS_TRANSIENT_LOCAL,
    IS_BRIDGE, &add_publisher_args);
  if (ret < 0) return ret;

  union ioctl_add_subscriber_args add_subscriber_args;
  return agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, sub_pid, qos_depth, QOS_IS_TRANSIENT_LOCAL,
    QOS_IS_RELIABLE, IS_TAKE_SUB, IGNORE_LOCAL_PUBLICATIONS, IS_BRIDGE, &add_subscriber_args);
}

// A publisher and subscriber with the same topic name but different ROS_DOMAIN_ID
// must produce two distinct topic wrappers (ROS 2 domain isolation).
void test_case_add_subscriber_domain_isolation(struct kunit * test)
{
  const pid_t pub_pid = 2000;
  const pid_t sub_pid = 2001;
  setup_process_domain(test, pub_pid, 0);
  setup_process_domain(test, sub_pid, 1);

  KUNIT_ASSERT_EQ(test, add_pubsub_pair(test, pub_pid, sub_pid, 1), 0);

  // Two wrappers: (TOPIC_NAME, ipc_ns, domain=0) and (TOPIC_NAME, ipc_ns, domain=1).
  KUNIT_EXPECT_EQ(test, agnocast_get_topic_num(current->nsproxy->ipc_ns), 2);
}

// A publisher and subscriber in the same domain on the same topic share one
// wrapper (unchanged behavior).
void test_case_add_subscriber_same_domain_shared(struct kunit * test)
{
  const pid_t pub_pid = 2002;
  const pid_t sub_pid = 2003;
  setup_process_domain(test, pub_pid, 0);
  setup_process_domain(test, sub_pid, 0);

  KUNIT_ASSERT_EQ(test, add_pubsub_pair(test, pub_pid, sub_pid, 1), 0);

  KUNIT_EXPECT_EQ(test, agnocast_get_topic_num(current->nsproxy->ipc_ns), 1);
}

void test_case_add_subscriber_normal(struct kunit * test)
{
  // Arrange
  union ioctl_add_subscriber_args add_subscriber_args;
  const pid_t subscriber_pid = 1000;
  const uint32_t qos_depth = 1;
  setup_process(test, subscriber_pid);
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 1);
  KUNIT_ASSERT_FALSE(test, agnocast_is_proc_exited(subscriber_pid));

  // Act
  int ret = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, subscriber_pid, qos_depth,
    QOS_IS_TRANSIENT_LOCAL, QOS_IS_RELIABLE, IS_TAKE_SUB, IGNORE_LOCAL_PUBLICATIONS, IS_BRIDGE,
    &add_subscriber_args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  union ioctl_get_subscriber_num_args get_subscriber_num_args;
  agnocast_ioctl_get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  KUNIT_EXPECT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 1);
  KUNIT_EXPECT_EQ(test, add_subscriber_args.ret_id, 0);
  KUNIT_EXPECT_TRUE(
    test, agnocast_is_in_subscriber_htable(
            TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args.ret_id));
  KUNIT_EXPECT_EQ(test, agnocast_get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_TRUE(test, agnocast_is_in_topic_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
}

void test_case_add_subscriber_too_many_subscribers(struct kunit * test)
{
  // Arrange
  union ioctl_add_subscriber_args add_subscriber_args;
  const uint32_t qos_depth = 1;
  const pid_t subscriber_pid = 1000;
  setup_process(test, subscriber_pid);
  for (uint32_t i = 0; i < MAX_SUBSCRIBER_NUM; i++) {
    union ioctl_add_subscriber_args add_subscriber_args;
    agnocast_ioctl_add_subscriber(
      TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, subscriber_pid, qos_depth,
      QOS_IS_TRANSIENT_LOCAL, QOS_IS_RELIABLE, IS_TAKE_SUB, IGNORE_LOCAL_PUBLICATIONS, IS_BRIDGE,
      &add_subscriber_args);
  }

  // Act
  int ret = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, subscriber_pid, qos_depth,
    QOS_IS_TRANSIENT_LOCAL, QOS_IS_RELIABLE, IS_TAKE_SUB, IGNORE_LOCAL_PUBLICATIONS, IS_BRIDGE,
    &add_subscriber_args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -ENOBUFS);
}
