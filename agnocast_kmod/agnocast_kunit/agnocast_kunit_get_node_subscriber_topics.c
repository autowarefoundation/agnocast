// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_get_node_subscriber_topics.h"

#include "../agnocast.h"

#include <kunit/test.h>

static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * NODE_NAME = "/kunit_test_node";
static const char * NODE_NAME_WITH_SUFFIX = "/kunit_test_node_extra";
static const pid_t PID = 1000;
static const uint32_t QOS_DEPTH = 1;
static const bool IS_BRIDGE = false;

static void setup_process(struct kunit * test, const pid_t pid)
{
  union ioctl_add_process_args add_process_args;
  int ret = agnocast_ioctl_add_process(
    pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &add_process_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

void test_case_get_node_sub_topics_exact_match(struct kunit * test)
{
  union ioctl_add_subscriber_args add_sub_args;
  char buf[1][TOPIC_NAME_BUFFER_SIZE];
  uint32_t topic_num = UINT_MAX;
  int ret;

  setup_process(test, PID);

  ret = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, PID, QOS_DEPTH, false, false, false, false,
    IS_BRIDGE, -1, &add_sub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  ret = agnocast_ioctl_get_node_subscriber_topics(
    current->nsproxy->ipc_ns, NODE_NAME, (char *)buf, ARRAY_SIZE(buf), &topic_num);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, topic_num, 1);
  KUNIT_EXPECT_STREQ(test, buf[0], TOPIC_NAME);
}

void test_case_get_node_sub_topics_prefix_no_match(struct kunit * test)
{
  union ioctl_add_subscriber_args add_sub_args;
  char buf[1][TOPIC_NAME_BUFFER_SIZE];
  uint32_t topic_num = UINT_MAX;
  int ret;

  setup_process(test, PID);

  ret = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME_WITH_SUFFIX, PID, QOS_DEPTH, false, false,
    false, false, IS_BRIDGE, -1, &add_sub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  ret = agnocast_ioctl_get_node_subscriber_topics(
    current->nsproxy->ipc_ns, NODE_NAME, (char *)buf, ARRAY_SIZE(buf), &topic_num);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ_MSG(
    test, topic_num, (uint32_t)0, "Prefix of node name should not match (strcmp, not strncmp)");
}

void test_case_get_node_sub_topics_buffer_size_exceeded(struct kunit * test)
{
  union ioctl_add_subscriber_args add_sub_args;
  char buf[1][TOPIC_NAME_BUFFER_SIZE];
  uint32_t topic_num = UINT_MAX;
  int ret;

  setup_process(test, PID);

  ret = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, PID, QOS_DEPTH, false, false, false, false,
    IS_BRIDGE, -1, &add_sub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  ret = agnocast_ioctl_get_node_subscriber_topics(
    current->nsproxy->ipc_ns, NODE_NAME, (char *)buf, 0, &topic_num);
  KUNIT_EXPECT_EQ(test, ret, -ENOBUFS);
  KUNIT_EXPECT_EQ(test, topic_num, UINT_MAX);
}
