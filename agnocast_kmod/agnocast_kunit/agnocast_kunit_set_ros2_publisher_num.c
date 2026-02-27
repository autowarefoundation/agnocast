#include "agnocast_kunit_set_ros2_publisher_num.h"

#include "../agnocast.h"

static char * node_name = "/kunit_test_node";
static uint32_t qos_depth = 10;
static bool qos_is_transient_local = false;
static pid_t publisher_pid = 3000;
static bool is_bridge = false;

static void setup_one_publisher(struct kunit * test, char * topic_name)
{
  publisher_pid++;

  union ioctl_add_process_args add_process_args;
  int ret1 = agnocast_ioctl_add_process(publisher_pid, current->nsproxy->ipc_ns, &add_process_args);

  union ioctl_add_publisher_args add_publisher_args;
  int ret2 = agnocast_ioctl_add_publisher(
    topic_name, current->nsproxy->ipc_ns, node_name, publisher_pid, qos_depth,
    qos_is_transient_local, is_bridge, &add_publisher_args);

  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ret2, 0);
}

void test_case_set_ros2_publisher_num_normal(struct kunit * test)
{
  char * topic_name = "/kunit_test_topic";
  setup_one_publisher(test, topic_name);

  int ret = agnocast_ioctl_set_ros2_publisher_num(topic_name, current->nsproxy->ipc_ns, 5);
  KUNIT_EXPECT_EQ(test, ret, 0);

  union ioctl_get_publisher_num_args publisher_num_args;
  int ret2 =
    agnocast_ioctl_get_publisher_num(topic_name, current->nsproxy->ipc_ns, &publisher_num_args);
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, publisher_num_args.ret_publisher_num, 1);
  KUNIT_EXPECT_EQ(test, publisher_num_args.ret_ros2_publisher_num, 5);
}

void test_case_set_ros2_publisher_num_topic_not_exist(struct kunit * test)
{
  char * topic_name = "/kunit_nonexistent_topic";

  int ret = agnocast_ioctl_set_ros2_publisher_num(topic_name, current->nsproxy->ipc_ns, 5);
  KUNIT_EXPECT_EQ(test, ret, -ENOENT);
}

void test_case_set_ros2_publisher_num_update(struct kunit * test)
{
  char * topic_name = "/kunit_test_topic";
  setup_one_publisher(test, topic_name);

  int ret1 = agnocast_ioctl_set_ros2_publisher_num(topic_name, current->nsproxy->ipc_ns, 3);
  KUNIT_EXPECT_EQ(test, ret1, 0);

  union ioctl_get_publisher_num_args publisher_num_args;
  int ret2 =
    agnocast_ioctl_get_publisher_num(topic_name, current->nsproxy->ipc_ns, &publisher_num_args);
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, publisher_num_args.ret_publisher_num, 1);
  KUNIT_EXPECT_EQ(test, publisher_num_args.ret_ros2_publisher_num, 3);

  // Update to new value
  int ret3 = agnocast_ioctl_set_ros2_publisher_num(topic_name, current->nsproxy->ipc_ns, 7);
  KUNIT_EXPECT_EQ(test, ret3, 0);

  int ret4 =
    agnocast_ioctl_get_publisher_num(topic_name, current->nsproxy->ipc_ns, &publisher_num_args);
  KUNIT_EXPECT_EQ(test, ret4, 0);
  KUNIT_EXPECT_EQ(test, publisher_num_args.ret_publisher_num, 1);
  KUNIT_EXPECT_EQ(test, publisher_num_args.ret_ros2_publisher_num, 7);
}
