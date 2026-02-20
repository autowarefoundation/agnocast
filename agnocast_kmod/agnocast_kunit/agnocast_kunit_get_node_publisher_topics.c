#include "agnocast_kunit_get_node_publisher_topics.h"

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
  int ret = ioctl_add_process(pid, current->nsproxy->ipc_ns, &add_process_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

void test_case_get_node_pub_topics_exact_match(struct kunit * test)
{
  union ioctl_add_publisher_args add_pub_args;
  union ioctl_node_info_args node_info_args = {0};
  int ret;

  setup_process(test, PID);

  ret = ioctl_add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, PID, QOS_DEPTH, false, IS_BRIDGE,
    &add_pub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  // copy_to_user inside ioctl_get_node_publisher_topics returns -EFAULT in KUnit (kernel thread)
  // context, but reaching it confirms that the node name match was found.
  ret = ioctl_get_node_publisher_topics(current->nsproxy->ipc_ns, NODE_NAME, &node_info_args);
  KUNIT_EXPECT_TRUE(test, ret == -EFAULT || (ret == 0 && node_info_args.ret_topic_num == 1));
}

void test_case_get_node_pub_topics_prefix_no_match(struct kunit * test)
{
  union ioctl_add_publisher_args add_pub_args;
  union ioctl_node_info_args node_info_args = {0};
  int ret;

  setup_process(test, PID);

  ret = ioctl_add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME_WITH_SUFFIX, PID, QOS_DEPTH, false, IS_BRIDGE,
    &add_pub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  ret = ioctl_get_node_publisher_topics(current->nsproxy->ipc_ns, NODE_NAME, &node_info_args);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ_MSG(
    test, node_info_args.ret_topic_num, (uint32_t)0,
    "Prefix of node name should not match (strcmp, not strncmp)");
}
