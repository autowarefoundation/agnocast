#include "agnocast_kunit_get_subscriber_num.h"

#include "../agnocast.h"

char * node_name = "/kunit_test_node";
uint32_t qos_depth = 10;
bool qos_is_transient_local = false;
bool qos_is_reliable = true;
pid_t subscriber_pid = 1000;
pid_t publisher_pid = 2000;
bool is_take_sub = false;
bool ignore_local_publications = false;
bool is_bridge = false;

static void setup_one_subscriber(struct kunit * test, char * topic_name)
{
  subscriber_pid++;

  union ioctl_add_process_args add_process_args;
  int ret1 = add_process(subscriber_pid, current->nsproxy->ipc_ns, &add_process_args);

  union ioctl_add_subscriber_args add_subscriber_args;
  int ret2 = add_subscriber(
    topic_name, current->nsproxy->ipc_ns, node_name, subscriber_pid, qos_depth,
    qos_is_transient_local, qos_is_reliable, is_take_sub, ignore_local_publications, is_bridge,
    &add_subscriber_args);

  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ret2, 0);
}

static void setup_one_subscriber_with_bridge(struct kunit * test, char * topic_name)
{
  subscriber_pid++;

  union ioctl_add_process_args add_process_args;
  int ret1 = add_process(subscriber_pid, current->nsproxy->ipc_ns, &add_process_args);

  union ioctl_add_subscriber_args add_subscriber_args;
  int ret2 = add_subscriber(
    topic_name, current->nsproxy->ipc_ns, node_name, subscriber_pid, qos_depth,
    qos_is_transient_local, qos_is_reliable, is_take_sub, ignore_local_publications, true,
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
    qos_is_transient_local, is_bridge, &add_publisher_args);

  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ret2, 0);
}

void test_case_get_subscriber_num_normal(struct kunit * test)
{
  char * topic_name = "/kunit_test_topic";
  setup_one_subscriber(test, topic_name);

  union ioctl_get_subscriber_num_args subscriber_num_args;
  int ret = get_subscriber_num(topic_name, current->nsproxy->ipc_ns, false, &subscriber_num_args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, subscriber_num_args.ret_subscriber_num, 1);
}

void test_case_get_subscriber_num_many(struct kunit * test)
{
  char * topic_name = "/kunit_test_topic";
  for (int i = 0; i < MAX_SUBSCRIBER_NUM; i++) {
    setup_one_subscriber(test, topic_name);
  }

  union ioctl_get_subscriber_num_args subscriber_num_args;
  int ret = get_subscriber_num(topic_name, current->nsproxy->ipc_ns, false, &subscriber_num_args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, subscriber_num_args.ret_subscriber_num, MAX_SUBSCRIBER_NUM);
}

void test_case_get_subscriber_num_different_topic(struct kunit * test)
{
  char * topic_name1 = "/kunit_test_topic1";
  char * topic_name2 = "/kunit_test_topic2";
  setup_one_subscriber(test, topic_name1);
  setup_one_subscriber(test, topic_name2);

  union ioctl_get_subscriber_num_args subscriber_num_args1;
  union ioctl_get_subscriber_num_args subscriber_num_args2;
  int ret1 =
    get_subscriber_num(topic_name1, current->nsproxy->ipc_ns, false, &subscriber_num_args1);
  int ret2 =
    get_subscriber_num(topic_name2, current->nsproxy->ipc_ns, false, &subscriber_num_args2);

  KUNIT_EXPECT_EQ(test, ret1, 0);
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, subscriber_num_args1.ret_subscriber_num, 1);
  KUNIT_EXPECT_EQ(test, subscriber_num_args2.ret_subscriber_num, 1);
}

void test_case_get_subscriber_num_with_exit(struct kunit * test)
{
  char * topic_name = "/kunit_test_topic";
  setup_one_subscriber(test, topic_name);

  union ioctl_get_subscriber_num_args subscriber_num_args;
  process_exit_cleanup(subscriber_pid);
  int ret = get_subscriber_num(topic_name, current->nsproxy->ipc_ns, false, &subscriber_num_args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, subscriber_num_args.ret_subscriber_num, 0);
}

void test_case_get_subscriber_num_no_subscriber(struct kunit * test)
{
  char * topic_name = "/kunit_test_topic";
  setup_one_publisher(test, topic_name);

  union ioctl_get_subscriber_num_args subscriber_num_args;
  int ret = get_subscriber_num(topic_name, current->nsproxy->ipc_ns, false, &subscriber_num_args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, subscriber_num_args.ret_subscriber_num, 0);
}

void test_case_get_subscriber_num_include_ros2(struct kunit * test)
{
  char * topic_name = "/kunit_test_topic";
  setup_one_subscriber(test, topic_name);

  int ret1 = set_ros2_subscriber_num(topic_name, current->nsproxy->ipc_ns, 3);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  union ioctl_get_subscriber_num_args subscriber_num_args;
  int ret2 = get_subscriber_num(topic_name, current->nsproxy->ipc_ns, false, &subscriber_num_args);
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, subscriber_num_args.ret_subscriber_num, 1);

  int ret3 = get_subscriber_num(topic_name, current->nsproxy->ipc_ns, true, &subscriber_num_args);
  KUNIT_EXPECT_EQ(test, ret3, 0);
  KUNIT_EXPECT_EQ(test, subscriber_num_args.ret_subscriber_num, 4);
}

void test_case_get_subscriber_num_bridge_exist(struct kunit * test)
{
  char * topic_name = "/kunit_test_topic";
  setup_one_subscriber(test, topic_name);

  union ioctl_get_subscriber_num_args subscriber_num_args;
  int ret1 = get_subscriber_num(topic_name, current->nsproxy->ipc_ns, false, &subscriber_num_args);
  KUNIT_EXPECT_EQ(test, ret1, 0);
  KUNIT_EXPECT_FALSE(test, subscriber_num_args.ret_bridge_exist);

  setup_one_subscriber_with_bridge(test, topic_name);

  int ret3 = get_subscriber_num(topic_name, current->nsproxy->ipc_ns, false, &subscriber_num_args);
  KUNIT_EXPECT_EQ(test, ret3, 0);
  KUNIT_EXPECT_TRUE(test, subscriber_num_args.ret_bridge_exist);
}
