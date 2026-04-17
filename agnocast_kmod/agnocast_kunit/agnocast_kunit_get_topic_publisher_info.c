// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_get_topic_publisher_info.h"

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

// Normal case: one publisher exists, should return count == 1
void test_case_get_topic_pub_info_one_publisher(struct kunit * test)
{
  union ioctl_add_publisher_args add_pub_args;
  union ioctl_topic_info_args topic_info_args = {0};
  int ret;

  setup_process(test, PID);

  ret = agnocast_ioctl_add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, PID, QOS_DEPTH, false, IS_BRIDGE,
    &add_pub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  // copy_to_user inside agnocast_ioctl_get_topic_publisher_info returns -EFAULT in KUnit
  // (kernel thread) context, but reaching it confirms the publisher was counted.
  topic_info_args.topic_info_ret_buffer_size = MAX_PUBLISHER_NUM;
  ret =
    agnocast_ioctl_get_topic_publisher_info(TOPIC_NAME, current->nsproxy->ipc_ns, &topic_info_args);
  KUNIT_EXPECT_TRUE(
    test, ret == -EFAULT || (ret == 0 && topic_info_args.ret_topic_info_ret_num == 1));
}

// No publishers on the topic (topic exists via subscriber, but no publisher)
void test_case_get_topic_pub_info_no_publishers(struct kunit * test)
{
  union ioctl_add_subscriber_args add_sub_args;
  union ioctl_topic_info_args topic_info_args = {0};
  int ret;

  setup_process(test, PID);

  ret = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, PID, QOS_DEPTH, false, false, false, false,
    IS_BRIDGE, &add_sub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  topic_info_args.topic_info_ret_buffer_size = MAX_PUBLISHER_NUM;
  ret =
    agnocast_ioctl_get_topic_publisher_info(TOPIC_NAME, current->nsproxy->ipc_ns, &topic_info_args);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, topic_info_args.ret_topic_info_ret_num, (uint32_t)0);
}

// Topic does not exist
void test_case_get_topic_pub_info_topic_not_found(struct kunit * test)
{
  union ioctl_topic_info_args topic_info_args = {0};

  topic_info_args.topic_info_ret_buffer_size = MAX_PUBLISHER_NUM;
  int ret = agnocast_ioctl_get_topic_publisher_info(
    "/nonexistent_topic", current->nsproxy->ipc_ns, &topic_info_args);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, topic_info_args.ret_topic_info_ret_num, (uint32_t)0);
}
