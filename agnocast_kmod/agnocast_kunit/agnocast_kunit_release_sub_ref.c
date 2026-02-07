#include "agnocast_kunit_release_sub_ref.h"

#include "../agnocast.h"

#include <kunit/test.h>

static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * NODE_NAME = "/kunit_test_node";
static const bool QOS_IS_TRANSIENT_LOCAL = true;
static const bool QOS_IS_RELIABLE = true;
static const bool IGNORE_LOCAL_PUBLICATIONS = false;
static const uint32_t QOS_DEPTH = 1;
static const bool IS_BRIDGE = false;

static void setup_one_publisher(
  struct kunit * test, topic_local_id_t * ret_publisher_id, uint64_t * ret_addr)
{
  const pid_t PUBLISHER_PID = 2000;

  union ioctl_add_process_args add_process_args;
  int ret1 = add_process(PUBLISHER_PID, current->nsproxy->ipc_ns, &add_process_args);
  union ioctl_add_publisher_args add_publisher_args;
  int ret2 = add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, PUBLISHER_PID, QOS_DEPTH,
    QOS_IS_TRANSIENT_LOCAL, IS_BRIDGE, &add_publisher_args);

  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ret2, 0);

  *ret_addr = add_process_args.ret_addr;
  *ret_publisher_id = add_publisher_args.ret_id;
}

void test_case_release_sub_ref_no_topic(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 0);
  KUNIT_EXPECT_EQ(
    test, release_message_entry_reference(TOPIC_NAME, current->nsproxy->ipc_ns, 0, 0), -EINVAL);
}

void test_case_release_sub_ref_no_message(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 0);

  // Arrange
  topic_local_id_t ret_publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &ret_publisher_id, &ret_addr);

  // Act
  int ret =
    release_message_entry_reference(TOPIC_NAME, current->nsproxy->ipc_ns, ret_publisher_id, 0);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_release_sub_ref_no_pubsub_id(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 0);

  // Arrange
  topic_local_id_t ret_publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &ret_publisher_id, &ret_addr);

  union ioctl_publish_msg_args publish_msg_args;
  int ret0 = publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, ret_publisher_id, ret_addr, &publish_msg_args);
  KUNIT_ASSERT_EQ(test, ret0, 0);

  // Act: Attempt to release a reference using the publisher's local ID.
  // This should fail with -EINVAL because only subscriber IDs are tracked
  // in the referencing_ids array for reference counting, not publisher IDs.
  int ret_sut = release_message_entry_reference(
    TOPIC_NAME, current->nsproxy->ipc_ns, ret_publisher_id, publish_msg_args.ret_entry_id);

  // Assert
  KUNIT_EXPECT_EQ(test, ret_sut, -EINVAL);
}

void test_case_release_sub_ref_last_reference(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 0);

  // Arrange: Publisher publishes a message, subscriber adds reference then releases it.
  topic_local_id_t ret_publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &ret_publisher_id, &ret_addr);

  union ioctl_publish_msg_args publish_msg_args;
  int ret = publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, ret_publisher_id, ret_addr, &publish_msg_args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  const pid_t subscriber_pid = 1000;
  union ioctl_add_process_args add_process_args;
  int ret2 = add_process(subscriber_pid, current->nsproxy->ipc_ns, &add_process_args);
  KUNIT_ASSERT_EQ(test, ret2, 0);

  union ioctl_add_subscriber_args add_subscriber_args;
  int ret3 = add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, subscriber_pid, QOS_DEPTH,
    QOS_IS_TRANSIENT_LOCAL, QOS_IS_RELIABLE, false, IGNORE_LOCAL_PUBLICATIONS, IS_BRIDGE,
    &add_subscriber_args);
  KUNIT_ASSERT_EQ(test, ret3, 0);

  int ret4 = increment_message_entry_rc(
    TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args.ret_id,
    publish_msg_args.ret_entry_id);
  KUNIT_ASSERT_EQ(test, ret4, 0);

  // Act: Subscriber releases its reference.
  int ret_sut = release_message_entry_reference(
    TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args.ret_id,
    publish_msg_args.ret_entry_id);

  // Assert
  KUNIT_EXPECT_EQ(test, ret_sut, 0);
  KUNIT_EXPECT_EQ(
    test,
    get_entry_rc(
      TOPIC_NAME, current->nsproxy->ipc_ns, publish_msg_args.ret_entry_id,
      add_subscriber_args.ret_id),
    0);
}

void test_case_release_sub_ref_multi_reference(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, get_topic_num(current->nsproxy->ipc_ns), 0);

  // Arrange: Publisher publishes a message, two subscribers add references.
  topic_local_id_t ret_publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &ret_publisher_id, &ret_addr);

  union ioctl_publish_msg_args publish_msg_args;
  int ret1 = publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, ret_publisher_id, ret_addr, &publish_msg_args);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  // First subscriber
  const pid_t subscriber_pid1 = 1000;
  union ioctl_add_process_args add_process_args1;
  int ret2 = add_process(subscriber_pid1, current->nsproxy->ipc_ns, &add_process_args1);
  KUNIT_ASSERT_EQ(test, ret2, 0);

  union ioctl_add_subscriber_args add_subscriber_args1;
  int ret3 = add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, subscriber_pid1, QOS_DEPTH,
    QOS_IS_TRANSIENT_LOCAL, QOS_IS_RELIABLE, false, IGNORE_LOCAL_PUBLICATIONS, IS_BRIDGE,
    &add_subscriber_args1);
  KUNIT_ASSERT_EQ(test, ret3, 0);

  int ret4 = increment_message_entry_rc(
    TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args1.ret_id,
    publish_msg_args.ret_entry_id);
  KUNIT_ASSERT_EQ(test, ret4, 0);

  // Second subscriber
  const pid_t subscriber_pid2 = 1001;
  union ioctl_add_process_args add_process_args2;
  int ret5 = add_process(subscriber_pid2, current->nsproxy->ipc_ns, &add_process_args2);
  KUNIT_ASSERT_EQ(test, ret5, 0);

  union ioctl_add_subscriber_args add_subscriber_args2;
  int ret6 = add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, subscriber_pid2, QOS_DEPTH,
    QOS_IS_TRANSIENT_LOCAL, QOS_IS_RELIABLE, false, IGNORE_LOCAL_PUBLICATIONS, IS_BRIDGE,
    &add_subscriber_args2);
  KUNIT_ASSERT_EQ(test, ret6, 0);

  int ret7 = increment_message_entry_rc(
    TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args2.ret_id,
    publish_msg_args.ret_entry_id);
  KUNIT_ASSERT_EQ(test, ret7, 0);

  // Act: First subscriber releases its reference.
  int ret_sut = release_message_entry_reference(
    TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args1.ret_id,
    publish_msg_args.ret_entry_id);

  // Assert: First subscriber's reference is now released, second subscriber still has reference.
  KUNIT_EXPECT_EQ(test, ret_sut, 0);
  KUNIT_EXPECT_EQ(
    test,
    get_entry_rc(
      TOPIC_NAME, current->nsproxy->ipc_ns, publish_msg_args.ret_entry_id,
      add_subscriber_args1.ret_id),
    0);
  KUNIT_EXPECT_EQ(
    test,
    get_entry_rc(
      TOPIC_NAME, current->nsproxy->ipc_ns, publish_msg_args.ret_entry_id,
      add_subscriber_args2.ret_id),
    1);
}
