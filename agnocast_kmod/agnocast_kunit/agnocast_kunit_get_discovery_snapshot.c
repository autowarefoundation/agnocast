// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_get_discovery_snapshot.h"

#include "../agnocast.h"

#include <kunit/test.h>

static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * NODE_NAME = "/kunit_test_node";
static const pid_t PID = 1000;
static const uint32_t QOS_DEPTH = 1;
static const bool IS_BRIDGE = false;

static void setup_process(struct kunit * test, const pid_t pid)
{
  union ioctl_add_process_args add_process_args;
  int ret = agnocast_ioctl_add_process(pid, current->nsproxy->ipc_ns, false, &add_process_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

static void setup_pub_and_sub(struct kunit * test)
{
  union ioctl_add_publisher_args add_pub_args;
  union ioctl_add_subscriber_args add_sub_args;
  int ret;

  setup_process(test, PID);

  ret = agnocast_ioctl_add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, PID, QOS_DEPTH, false, IS_BRIDGE,
    &add_pub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  ret = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, PID, QOS_DEPTH, false, false, false, false,
    IS_BRIDGE, &add_sub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

// Empty namespace: no topics, returns 0 with zero counts (no copy_to_user reached).
void test_case_get_discovery_snapshot_empty(struct kunit * test)
{
  struct ioctl_discovery_snapshot_args args = {0};

  int ret = agnocast_ioctl_get_discovery_snapshot(current->nsproxy->ipc_ns, &args);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, args.ret_topic_num, (uint32_t)0);
  KUNIT_EXPECT_EQ(test, args.ret_endpoint_num, (uint32_t)0);
}

// Probe (zero-capacity buffers): reports required counts via -ENOBUFS without touching
// copy_to_user. One topic with one publisher and one subscriber => 1 topic, 2 endpoints.
void test_case_get_discovery_snapshot_probe_counts(struct kunit * test)
{
  struct ioctl_discovery_snapshot_args args = {0};

  setup_pub_and_sub(test);

  int ret = agnocast_ioctl_get_discovery_snapshot(current->nsproxy->ipc_ns, &args);
  KUNIT_EXPECT_EQ(test, ret, -ENOBUFS);
  KUNIT_EXPECT_EQ(test, args.ret_topic_num, (uint32_t)1);
  KUNIT_EXPECT_EQ(test, args.ret_endpoint_num, (uint32_t)2);
}

// Fill path: with capacity available, the handler reaches copy_to_user, which returns -EFAULT
// in KUnit (kernel thread) context. Reaching it confirms the topic/endpoints were laid out.
void test_case_get_discovery_snapshot_fill(struct kunit * test)
{
  struct ioctl_discovery_snapshot_args args = {0};

  setup_pub_and_sub(test);

  args.topic_buffer_size = MAX_TOPIC_NUM;
  args.endpoint_buffer_size = MAX_PUBLISHER_NUM + MAX_SUBSCRIBER_NUM;
  // Buffer addresses left 0: copy_to_user(NULL, ...) returns -EFAULT under KUnit.

  int ret = agnocast_ioctl_get_discovery_snapshot(current->nsproxy->ipc_ns, &args);
  KUNIT_EXPECT_TRUE(
    test, ret == -EFAULT || (ret == 0 && args.ret_topic_num == 1 && args.ret_endpoint_num == 2));
}
