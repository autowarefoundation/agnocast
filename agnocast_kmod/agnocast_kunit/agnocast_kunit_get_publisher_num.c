#include "agnocast_kunit_get_publisher_num.h"

#include "../agnocast.h"

static char * node_name = "/kunit_test_node";
static uint32_t qos_depth = 10;
static bool qos_is_transient_local = false;
static bool qos_is_reliable = true;
static pid_t subscriber_pid = 1000;
static pid_t publisher_pid = 2000;
static bool is_take_sub = false;
static bool ignore_local_publications = false;

static void setup_one_subscriber(struct kunit * test, char * topic_name)
{
  subscriber_pid++;

  union ioctl_add_process_args add_process_args;
  int ret1 = add_process(subscriber_pid, current->nsproxy->ipc_ns, &add_process_args);

  union ioctl_add_subscriber_args add_subscriber_args;
  int ret2 = add_subscriber(
    topic_name, current->nsproxy->ipc_ns, node_name, subscriber_pid, qos_depth,
    qos_is_transient_local, qos_is_reliable, is_take_sub, ignore_local_publications,
    &add_subscriber_args);

  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ret2, 0);
}

static void setup_one_publisher(struct kunit * test, char * topic_name)
{
  publisher_pid++;

  union ioctl_add_process_args add_process_args;
  int ret1 = add_process(publisher_pid, current->nsproxy->ipc_ns, &add_process_args);

  union ioctl_add_publisher_args add_publisher_args;
  int ret2 = add_publisher(
    topic_name, current->nsproxy->ipc_ns, node_name, publisher_pid, qos_depth,
    qos_is_transient_local, &add_publisher_args);

  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ret2, 0);
}

void test_case_get_publisher_num_normal(struct kunit * test)
{
  char * topic_name = "/kunit_test_topic";
  setup_one_publisher(test, topic_name);

  union ioctl_get_publisher_num_args publisher_num_args;
  int ret = get_publisher_num(topic_name, current->nsproxy->ipc_ns, &publisher_num_args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, publisher_num_args.ret_publisher_num, 1);
}

void test_case_get_publisher_num_many(struct kunit * test)
{
  char * topic_name = "/kunit_test_topic";
  for (int i = 0; i < MAX_PUBLISHER_NUM; i++) {
    setup_one_publisher(test, topic_name);
  }

  union ioctl_get_publisher_num_args publisher_num_args;
  int ret = get_publisher_num(topic_name, current->nsproxy->ipc_ns, &publisher_num_args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, publisher_num_args.ret_publisher_num, MAX_PUBLISHER_NUM);
}

void test_case_get_publisher_num_different_topic(struct kunit * test)
{
  char * topic_name1 = "/kunit_test_topic1";
  char * topic_name2 = "/kunit_test_topic2";
  setup_one_publisher(test, topic_name1);
  setup_one_publisher(test, topic_name2);

  union ioctl_get_publisher_num_args publisher_num_args1;
  union ioctl_get_publisher_num_args publisher_num_args2;
  int ret1 = get_publisher_num(topic_name1, current->nsproxy->ipc_ns, &publisher_num_args1);
  int ret2 = get_publisher_num(topic_name2, current->nsproxy->ipc_ns, &publisher_num_args2);

  KUNIT_EXPECT_EQ(test, ret1, 0);
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, publisher_num_args1.ret_publisher_num, 1);
  KUNIT_EXPECT_EQ(test, publisher_num_args2.ret_publisher_num, 1);
}

void test_case_get_publisher_num_with_exit(struct kunit * test)
{
  char * topic_name = "/kunit_test_topic";
  setup_one_publisher(test, topic_name);

  union ioctl_get_publisher_num_args publisher_num_args;
  process_exit_cleanup(publisher_pid);
  int ret = get_publisher_num(topic_name, current->nsproxy->ipc_ns, &publisher_num_args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, publisher_num_args.ret_publisher_num, 0);
}

void test_case_get_publisher_num_no_publisher(struct kunit * test)
{
  char * topic_name = "/kunit_test_topic";
  setup_one_subscriber(test, topic_name);

  union ioctl_get_publisher_num_args publisher_num_args;
  int ret = get_publisher_num(topic_name, current->nsproxy->ipc_ns, &publisher_num_args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, publisher_num_args.ret_publisher_num, 0);
}

void test_case_get_publisher_num_bridge_exist(struct kunit * test)
{
  char * topic_name = "/kunit_test_topic";
  pid_t bridge_owner_pid = 9000;
  setup_one_publisher(test, topic_name);

  union ioctl_get_publisher_num_args publisher_num_args;
  int ret1 = get_publisher_num(topic_name, current->nsproxy->ipc_ns, &publisher_num_args);
  KUNIT_EXPECT_EQ(test, ret1, 0);
  KUNIT_EXPECT_FALSE(test, publisher_num_args.ret_bridge_exist);

  struct ioctl_add_bridge_args add_bridge_args = {0};
  int ret2 =
    add_bridge(topic_name, bridge_owner_pid, true, current->nsproxy->ipc_ns, &add_bridge_args);
  KUNIT_ASSERT_EQ(test, ret2, 0);

  int ret3 = get_publisher_num(topic_name, current->nsproxy->ipc_ns, &publisher_num_args);
  KUNIT_EXPECT_EQ(test, ret3, 0);
  KUNIT_EXPECT_TRUE(test, publisher_num_args.ret_bridge_exist);
}
