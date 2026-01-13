#include "agnocast_kunit_remove_subscriber.h"

#include "../agnocast.h"
#include "../agnocast_memory_allocator.h"

#include <kunit/test.h>
#include <linux/delay.h>

static const pid_t PID_BASE = 1000;
static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * NODE_NAME = "/kunit_test_node";
static const uint32_t QOS_DEPTH = 1;
static const bool QOS_IS_TRANSIENT_LOCAL = false;
static const bool QOS_IS_RELIABLE = true;
static const bool IS_TAKE_SUB = false;
static const bool IGNORE_LOCAL_PUBLICATIONS = false;

static uint64_t setup_one_process(struct kunit * test, const pid_t pid)
{
  union ioctl_add_process_args ioctl_ret;
  int ret = add_process(pid, current->nsproxy->ipc_ns, &ioctl_ret);

  KUNIT_ASSERT_EQ(test, ret, 0);
  return ioctl_ret.ret_addr;
}

static topic_local_id_t setup_one_publisher(struct kunit * test, const pid_t publisher_pid)
{
  union ioctl_add_publisher_args add_publisher_args;
  int ret = add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, publisher_pid, QOS_DEPTH,
    QOS_IS_TRANSIENT_LOCAL, &add_publisher_args);

  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_TRUE(test, is_in_topic_htable(TOPIC_NAME, current->nsproxy->ipc_ns));

  return add_publisher_args.ret_id;
}

static topic_local_id_t setup_one_subscriber(struct kunit * test, const pid_t subscriber_pid)
{
  union ioctl_add_subscriber_args add_subscriber_args;
  int ret = add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, subscriber_pid, QOS_DEPTH,
    QOS_IS_TRANSIENT_LOCAL, QOS_IS_RELIABLE, IS_TAKE_SUB, IGNORE_LOCAL_PUBLICATIONS,
    &add_subscriber_args);

  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_TRUE(test, is_in_topic_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
  KUNIT_ASSERT_TRUE(
    test,
    is_in_subscriber_htable(TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args.ret_id));

  return add_subscriber_args.ret_id;
}

static uint64_t setup_one_entry(
  struct kunit * test, const topic_local_id_t publisher_id, const uint64_t msg_virtual_address)
{
  union ioctl_publish_msg_args publish_msg_args;
  int ret = publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, msg_virtual_address, &publish_msg_args);

  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_TRUE(
    test, is_in_topic_entries(TOPIC_NAME, current->nsproxy->ipc_ns, publish_msg_args.ret_entry_id));

  return publish_msg_args.ret_entry_id;
}

void test_case_remove_subscriber_basic(struct kunit * test)
{
  // Arrange
  const pid_t subscriber_pid = PID_BASE;
  setup_one_process(test, subscriber_pid);
  const topic_local_id_t subscriber_id = setup_one_subscriber(test, subscriber_pid);

  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  union ioctl_get_subscriber_num_args get_sub_args;
  int ret = get_subscriber_num(TOPIC_NAME, current->nsproxy->ipc_ns, false, &get_sub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_EQ(test, get_sub_args.ret_subscriber_num, 1);

  // Act
  ret = remove_subscriber(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 0);
  KUNIT_EXPECT_FALSE(test, is_in_topic_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
  KUNIT_EXPECT_FALSE(
    test, is_in_subscriber_htable(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id));
}

void test_case_remove_subscriber_keeps_topic_with_publisher(struct kunit * test)
{
  // Arrange
  const pid_t pid = PID_BASE;
  setup_one_process(test, pid);
  setup_one_publisher(test, pid);
  const topic_local_id_t sub_id = setup_one_subscriber(test, pid);

  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  union ioctl_get_subscriber_num_args get_sub_args;
  get_subscriber_num(TOPIC_NAME, current->nsproxy->ipc_ns, false, &get_sub_args);
  KUNIT_ASSERT_EQ(test, get_sub_args.ret_subscriber_num, 1);

  // Act
  int ret = remove_subscriber(TOPIC_NAME, current->nsproxy->ipc_ns, sub_id);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_EXPECT_TRUE(test, is_in_topic_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
  get_subscriber_num(TOPIC_NAME, current->nsproxy->ipc_ns, false, &get_sub_args);
  KUNIT_EXPECT_EQ(test, get_sub_args.ret_subscriber_num, 0);
  union ioctl_get_publisher_num_args get_pub_args;
  get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_pub_args);
  KUNIT_EXPECT_EQ(test, get_pub_args.ret_publisher_num, 1);
}

void test_case_remove_subscriber_clears_references(struct kunit * test)
{
  // Arrange
  const pid_t pid = PID_BASE;
  const uint64_t msg_addr = setup_one_process(test, pid);
  const topic_local_id_t pub_id = setup_one_publisher(test, pid);
  const topic_local_id_t sub_id = setup_one_subscriber(test, pid);
  const uint64_t entry_id = setup_one_entry(test, pub_id, msg_addr);

  int ret = increment_message_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, sub_id, entry_id);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_EQ(test, get_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id, pub_id), 1);
  KUNIT_ASSERT_EQ(test, get_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id, sub_id), 1);

  // Act
  ret = remove_subscriber(TOPIC_NAME, current->nsproxy->ipc_ns, sub_id);
  KUNIT_ASSERT_EQ(test, ret, 0);

  // Assert
  KUNIT_EXPECT_TRUE(test, is_in_topic_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
  KUNIT_EXPECT_TRUE(test, is_in_topic_entries(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id));
  KUNIT_EXPECT_EQ(test, get_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id, pub_id), 1);
  KUNIT_EXPECT_EQ(test, get_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id, sub_id), 0);
}

void test_case_remove_subscriber_triggers_gc(struct kunit * test)
{
  // Arrange
  const pid_t pub_pid = PID_BASE;
  const pid_t sub_pid = PID_BASE + 1;
  const uint64_t msg_addr = setup_one_process(test, pub_pid);
  setup_one_process(test, sub_pid);

  const topic_local_id_t pub_id = setup_one_publisher(test, pub_pid);
  const topic_local_id_t sub_id = setup_one_subscriber(test, sub_pid);
  const uint64_t entry_id = setup_one_entry(test, pub_id, msg_addr);

  int ret = increment_message_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, sub_id, entry_id);
  KUNIT_ASSERT_EQ(test, ret, 0);

  enqueue_exit_pid(pub_pid);
  msleep(10);  // wait exit

  KUNIT_ASSERT_TRUE(test, is_proc_exited(pub_pid));
  KUNIT_ASSERT_TRUE(test, is_in_topic_entries(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id));

  // Act
  ret = remove_subscriber(TOPIC_NAME, current->nsproxy->ipc_ns, sub_id);
  KUNIT_ASSERT_EQ(test, ret, 0);

  // Assert
  KUNIT_EXPECT_FALSE(test, is_in_topic_entries(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id));
  KUNIT_EXPECT_FALSE(test, is_in_topic_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
}

void test_case_remove_subscriber_shared_ref_gc(struct kunit * test)
{
  // Arrange
  const pid_t pub_pid = PID_BASE;
  const pid_t sub1_pid = PID_BASE + 1;
  const pid_t sub2_pid = PID_BASE + 2;
  const uint64_t msg_addr = setup_one_process(test, pub_pid);
  setup_one_process(test, sub1_pid);
  setup_one_process(test, sub2_pid);

  const topic_local_id_t pub_id = setup_one_publisher(test, pub_pid);
  const topic_local_id_t sub1_id = setup_one_subscriber(test, sub1_pid);
  const topic_local_id_t sub2_id = setup_one_subscriber(test, sub2_pid);
  const uint64_t entry_id = setup_one_entry(test, pub_id, msg_addr);

  increment_message_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, sub1_id, entry_id);
  increment_message_entry_rc(TOPIC_NAME, current->nsproxy->ipc_ns, sub2_id, entry_id);

  enqueue_exit_pid(pub_pid);
  msleep(10);

  // Act
  remove_subscriber(TOPIC_NAME, current->nsproxy->ipc_ns, sub1_id);
  // Assert
  KUNIT_EXPECT_TRUE(test, is_in_topic_entries(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id));
  remove_subscriber(TOPIC_NAME, current->nsproxy->ipc_ns, sub2_id);
  KUNIT_EXPECT_FALSE(test, is_in_topic_entries(TOPIC_NAME, current->nsproxy->ipc_ns, entry_id));
}
