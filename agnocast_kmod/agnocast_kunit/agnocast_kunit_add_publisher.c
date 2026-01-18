#include "agnocast_kunit_add_publisher.h"

#include "../agnocast.h"

#include <kunit/test.h>

static const char * topic_name = "/kunit_test_topic";
static const char * node_name = "/kunit_test_node";
static const pid_t publisher_pid = 1000;
static const uint32_t qos_depth = 1;
static const bool qos_is_transient_local = false;
static const bool is_bridge = false;

void test_case_add_publisher_normal(struct kunit * test)
{
  union ioctl_get_publisher_num_args get_publisher_num_args;
  int ret = get_publisher_num(topic_name, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 0);
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 0);

  union ioctl_add_publisher_args add_publisher_args;
  int ret1 = add_publisher(
    topic_name, current->nsproxy->ipc_ns, node_name, publisher_pid, qos_depth,
    qos_is_transient_local, is_bridge, &add_publisher_args);

  KUNIT_EXPECT_EQ(test, ret1, 0);
  int ret2 = get_publisher_num(topic_name, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 1);
  KUNIT_EXPECT_EQ(test, add_publisher_args.ret_id, 0);
  KUNIT_EXPECT_TRUE(
    test, is_in_publisher_htable(topic_name, current->nsproxy->ipc_ns, add_publisher_args.ret_id));
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_TRUE(test, is_in_topic_htable(topic_name, current->nsproxy->ipc_ns));
}

void test_case_add_publisher_many(struct kunit * test)
{
  union ioctl_get_publisher_num_args get_publisher_num_args;
  int ret = get_publisher_num(topic_name, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 0);
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 0);

  const int publisher_num = MAX_PUBLISHER_NUM;
  int ret1;
  union ioctl_add_publisher_args add_publisher_args;
  for (int i = 0; i < publisher_num; i++) {
    ret1 = add_publisher(
      topic_name, current->nsproxy->ipc_ns, node_name, publisher_pid, qos_depth,
      qos_is_transient_local, is_bridge, &add_publisher_args);
  }

  KUNIT_EXPECT_EQ(test, ret1, 0);
  KUNIT_EXPECT_EQ(test, add_publisher_args.ret_id, publisher_num - 1);
  int ret2 = get_publisher_num(topic_name, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, publisher_num);
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_TRUE(test, is_in_topic_htable(topic_name, current->nsproxy->ipc_ns));
}

void test_case_add_publisher_too_many(struct kunit * test)
{
  union ioctl_get_publisher_num_args get_publisher_num_args;
  int ret = get_publisher_num(topic_name, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 0);
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 0);

  const int publisher_num = MAX_PUBLISHER_NUM + 1;
  int ret1 = 0;
  for (int i = 0; i < publisher_num; i++) {
    union ioctl_add_publisher_args add_publisher_args;
    ret1 = add_publisher(
      topic_name, current->nsproxy->ipc_ns, node_name, publisher_pid, qos_depth,
      qos_is_transient_local, is_bridge, &add_publisher_args);
  }

  KUNIT_EXPECT_EQ(test, ret1, -ENOBUFS);
  int ret2 = get_publisher_num(topic_name, current->nsproxy->ipc_ns, &get_publisher_num_args);
  KUNIT_ASSERT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, MAX_PUBLISHER_NUM);
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_TRUE(test, is_in_topic_htable(topic_name, current->nsproxy->ipc_ns));
}
