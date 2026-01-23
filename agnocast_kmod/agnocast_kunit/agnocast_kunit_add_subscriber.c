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
  int ret = add_process(pid, current->nsproxy->ipc_ns, &add_process_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

void test_case_add_subscriber_normal(struct kunit * test)
{
  // Arrange
  union ioctl_add_subscriber_args add_subscriber_args;
  const pid_t subscriber_pid = 1000;
  const uint32_t qos_depth = 1;
  setup_process(test, subscriber_pid);
  KUNIT_ASSERT_EQ(test, get_alive_proc_num(), 1);
  KUNIT_ASSERT_FALSE(test, is_proc_exited(subscriber_pid));

  // Act
  int ret = add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, subscriber_pid, qos_depth,
    QOS_IS_TRANSIENT_LOCAL, QOS_IS_RELIABLE, IS_TAKE_SUB, IGNORE_LOCAL_PUBLICATIONS, IS_BRIDGE,
    &add_subscriber_args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  union ioctl_get_subscriber_num_args get_subscriber_num_args;
  get_subscriber_num(TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &get_subscriber_num_args);
  KUNIT_EXPECT_EQ(test, get_subscriber_num_args.ret_other_process_subscriber_num, 1);
  KUNIT_EXPECT_EQ(test, add_subscriber_args.ret_id, 0);
  KUNIT_EXPECT_TRUE(
    test,
    is_in_subscriber_htable(TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args.ret_id));
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_TRUE(test, is_in_topic_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
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
    add_subscriber(
      TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, subscriber_pid, qos_depth,
      QOS_IS_TRANSIENT_LOCAL, QOS_IS_RELIABLE, IS_TAKE_SUB, IGNORE_LOCAL_PUBLICATIONS, IS_BRIDGE,
      &add_subscriber_args);
  }

  // Act
  int ret = add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, subscriber_pid, qos_depth,
    QOS_IS_TRANSIENT_LOCAL, QOS_IS_RELIABLE, IS_TAKE_SUB, IGNORE_LOCAL_PUBLICATIONS, IS_BRIDGE,
    &add_subscriber_args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -ENOBUFS);
}
