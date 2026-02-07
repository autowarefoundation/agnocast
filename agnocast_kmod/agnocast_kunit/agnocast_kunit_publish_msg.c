#include "agnocast_kunit_publish_msg.h"

#include "../agnocast.h"

#include <kunit/test.h>

static char * topic_name = "/kunit_test_topic";
static char * node_name = "/kunit_test_node";
static uint32_t qos_depth = 1;
static bool qos_is_transient_local = false;
static bool qos_is_reliable = true;
static pid_t subscriber_pid = 1000;
static pid_t publisher_pid = 2000;
static pid_t common_pid = 3000;
static bool is_take_sub = false;
static bool is_bridge = false;

static void setup_one_subscriber(
  struct kunit * test, topic_local_id_t * subscriber_id, bool ignore_local_publications)
{
  subscriber_pid++;

  union ioctl_add_process_args add_process_args;
  int ret1 = add_process(subscriber_pid, current->nsproxy->ipc_ns, &add_process_args);

  union ioctl_add_subscriber_args add_subscriber_args;
  int ret2 = add_subscriber(
    topic_name, current->nsproxy->ipc_ns, node_name, subscriber_pid, qos_depth,
    qos_is_transient_local, qos_is_reliable, is_take_sub, ignore_local_publications, is_bridge,
    &add_subscriber_args);
  *subscriber_id = add_subscriber_args.ret_id;

  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ret2, 0);
}

static void setup_one_publisher(
  struct kunit * test, topic_local_id_t * publisher_id, uint64_t * ret_addr)
{
  publisher_pid++;

  union ioctl_add_process_args add_process_args;
  int ret1 = add_process(publisher_pid, current->nsproxy->ipc_ns, &add_process_args);
  *ret_addr = add_process_args.ret_addr;

  union ioctl_add_publisher_args add_publisher_args;
  int ret2 = add_publisher(
    topic_name, current->nsproxy->ipc_ns, node_name, publisher_pid, qos_depth,
    qos_is_transient_local, is_bridge, &add_publisher_args);
  *publisher_id = add_publisher_args.ret_id;

  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ret2, 0);
}

static void setup_pub_sub_same_process(
  struct kunit * test, topic_local_id_t * publisher_id, topic_local_id_t * subscriber_id,
  bool ignore_local_publications, uint64_t * ret_addr)
{
  common_pid++;

  union ioctl_add_process_args add_process_args;
  int ret_proc = add_process(common_pid, current->nsproxy->ipc_ns, &add_process_args);

  if (ret_addr) {
    *ret_addr = add_process_args.ret_addr;
  }

  union ioctl_add_publisher_args add_publisher_args;
  int ret_pub = add_publisher(
    topic_name, current->nsproxy->ipc_ns, node_name, common_pid, qos_depth, qos_is_transient_local,
    is_bridge, &add_publisher_args);

  if (publisher_id) {
    *publisher_id = add_publisher_args.ret_id;
  }

  union ioctl_add_subscriber_args add_subscriber_args;
  int ret_sub = add_subscriber(
    topic_name, current->nsproxy->ipc_ns, node_name, common_pid, qos_depth, qos_is_transient_local,
    qos_is_reliable, is_take_sub, ignore_local_publications, is_bridge, &add_subscriber_args);

  if (subscriber_id) {
    *subscriber_id = add_subscriber_args.ret_id;
  }

  KUNIT_ASSERT_EQ(test, ret_proc, 0);
  KUNIT_ASSERT_EQ(test, ret_pub, 0);
  KUNIT_ASSERT_EQ(test, ret_sub, 0);
}

// Expect to fail at find_topic()
void test_case_publish_msg_no_topic(struct kunit * test)
{
  // Arrange
  topic_local_id_t publisher_id = 0;
  uint64_t msg_virtual_address = 0x40000000000;
  union ioctl_publish_msg_args ioctl_publish_ret;

  // Act
  int ret = publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, msg_virtual_address, &ioctl_publish_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

// Expect to fail at find_publisher_info
void test_case_publish_msg_no_publisher(struct kunit * test)
{
  // Arrange
  topic_local_id_t subscriber_id;
  bool ignore_local_publications = false;
  setup_one_subscriber(test, &subscriber_id, ignore_local_publications);

  topic_local_id_t publisher_id = 0;
  uint64_t msg_virtual_address = 0x40000000000;
  union ioctl_publish_msg_args ioctl_publish_msg_ret;

  // Act
  int ret = publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, msg_virtual_address,
    &ioctl_publish_msg_ret);

  // Assert
  KUNIT_ASSERT_EQ(test, ret, -EINVAL);
}

void test_case_publish_msg_simple_publish_without_any_release(struct kunit * test)
{
  // Arrange
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);

  union ioctl_publish_msg_args ioctl_publish_msg_ret;

  // Act
  int ret = publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_released_num, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_subscriber_num, 0);
  KUNIT_EXPECT_EQ(
    test,
    is_in_topic_entries(topic_name, current->nsproxy->ipc_ns, ioctl_publish_msg_ret.ret_entry_id),
    true);
  KUNIT_EXPECT_EQ(test, get_topic_entries_num(topic_name, current->nsproxy->ipc_ns), 1);
}

void test_case_publish_msg_different_publisher_no_release(struct kunit * test)
{
  // Arrange: Two different publishers each publish one message.
  // Publishers do not hold reference counts, but GC only releases entries from the same publisher
  // when qos_depth is exceeded. Since each publisher has only one entry, nothing is released.
  topic_local_id_t publisher_id1, publisher_id2;
  uint64_t ret_addr1, ret_addr2;
  setup_one_publisher(test, &publisher_id1, &ret_addr1);
  setup_one_publisher(test, &publisher_id2, &ret_addr2);

  union ioctl_publish_msg_args ioctl_publish_msg_ret1;
  int ret1 = publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id1, ret_addr1, &ioctl_publish_msg_ret1);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  union ioctl_publish_msg_args ioctl_publish_msg_ret2;

  // Act
  int ret2 = publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id2, ret_addr2, &ioctl_publish_msg_ret2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret2.ret_released_num, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret2.ret_subscriber_num, 0);
  KUNIT_EXPECT_EQ(
    test,
    is_in_topic_entries(topic_name, current->nsproxy->ipc_ns, ioctl_publish_msg_ret1.ret_entry_id),
    true);
  KUNIT_EXPECT_EQ(
    test,
    is_in_topic_entries(topic_name, current->nsproxy->ipc_ns, ioctl_publish_msg_ret2.ret_entry_id),
    true);
  KUNIT_EXPECT_EQ(test, get_topic_entries_num(topic_name, current->nsproxy->ipc_ns), 2);
}

void test_case_publish_msg_referenced_node_not_released(struct kunit * test)
{
  // Arrange: A subscriber holds a reference to entry1, preventing it from being GC'd
  // when entry2 is published (even though qos_depth=1 would normally trigger GC).
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);

  topic_local_id_t subscriber_id;
  setup_one_subscriber(test, &subscriber_id, false);

  union ioctl_publish_msg_args ioctl_publish_msg_ret1;
  int ret1 = publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret1);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  // Subscriber takes a reference to entry1
  int ret_inc = increment_message_entry_rc(
    topic_name, current->nsproxy->ipc_ns, subscriber_id, ioctl_publish_msg_ret1.ret_entry_id);
  KUNIT_ASSERT_EQ(test, ret_inc, 0);

  union ioctl_publish_msg_args ioctl_publish_msg_ret2;

  // Act
  int ret2 = publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr + 1, &ioctl_publish_msg_ret2);

  // Assert: entry1 is not released because subscriber holds a reference
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret2.ret_released_num, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret2.ret_subscriber_num, 1);
  KUNIT_EXPECT_EQ(
    test,
    is_in_topic_entries(topic_name, current->nsproxy->ipc_ns, ioctl_publish_msg_ret1.ret_entry_id),
    true);
  KUNIT_EXPECT_EQ(
    test,
    is_in_topic_entries(topic_name, current->nsproxy->ipc_ns, ioctl_publish_msg_ret2.ret_entry_id),
    true);
  KUNIT_EXPECT_EQ(test, get_topic_entries_num(topic_name, current->nsproxy->ipc_ns), 2);
}

void test_case_publish_msg_single_release_return(struct kunit * test)
{
  // Arrange: With qos_depth=1 and no subscriber references, entry1 is automatically released
  // when entry2 is published (GC is triggered to meet qos_depth).
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);

  union ioctl_publish_msg_args ioctl_publish_msg_ret1;
  int ret1 = publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret1);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  union ioctl_publish_msg_args ioctl_publish_msg_ret2;

  // Act: entry1 should be released to meet qos_depth=1
  int ret2 = publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr + 1, &ioctl_publish_msg_ret2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret2.ret_released_num, 1);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret2.ret_released_addrs[0], ret_addr);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret2.ret_subscriber_num, 0);
  KUNIT_EXPECT_EQ(
    test,
    is_in_topic_entries(topic_name, current->nsproxy->ipc_ns, ioctl_publish_msg_ret1.ret_entry_id),
    false);
  KUNIT_EXPECT_EQ(
    test,
    is_in_topic_entries(topic_name, current->nsproxy->ipc_ns, ioctl_publish_msg_ret2.ret_entry_id),
    true);
  KUNIT_EXPECT_EQ(test, get_topic_entries_num(topic_name, current->nsproxy->ipc_ns), 1);
}

void test_case_publish_msg_excessive_release_count(struct kunit * test)
{
  // Arrange: Test that GC is limited to MAX_RELEASE_NUM entries per publish call.
  // We use a subscriber to hold references to entries, then release them all before final publish.
  // Note: With qos_depth=1, entries without references are released immediately.
  // We need the subscriber to hold references during the initial publish loop.
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);

  topic_local_id_t subscriber_id;
  setup_one_subscriber(test, &subscriber_id, false);

  int64_t entry_ids[MAX_RELEASE_NUM + 1];
  for (int i = 0; i < MAX_RELEASE_NUM + 1; i++) {
    union ioctl_publish_msg_args ioctl_publish_msg_ret;
    int ret = publish_msg(
      topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr + i, &ioctl_publish_msg_ret);
    entry_ids[i] = ioctl_publish_msg_ret.ret_entry_id;
    KUNIT_ASSERT_EQ(test, ret, 0);

    // Subscriber holds a reference to each entry to prevent immediate GC
    ret =
      increment_message_entry_rc(topic_name, current->nsproxy->ipc_ns, subscriber_id, entry_ids[i]);
    KUNIT_ASSERT_EQ(test, ret, 0);
  }

  // Release all subscriber references so entries become eligible for GC
  for (int i = 0; i < MAX_RELEASE_NUM + 1; i++) {
    int ret = release_message_entry_reference(
      topic_name, current->nsproxy->ipc_ns, subscriber_id, entry_ids[i]);
    KUNIT_ASSERT_EQ(test, ret, 0);
  }

  union ioctl_publish_msg_args ioctl_publish_msg_ret;

  // Act: Publish one more message; GC should release up to MAX_RELEASE_NUM entries
  int ret = publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert: GC is limited to MAX_RELEASE_NUM entries per publish call
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_released_num, MAX_RELEASE_NUM);
  // Remaining entries: (MAX_RELEASE_NUM + 1) - MAX_RELEASE_NUM + 1 (new) = 2
  KUNIT_EXPECT_EQ(test, get_topic_entries_num(topic_name, current->nsproxy->ipc_ns), 2);
}

void test_case_publish_msg_ret_one_subscriber(struct kunit * test)
{
  // Arrange
  topic_local_id_t publisher_id, subscriber_id;
  bool ignore_local_publications = false;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);
  setup_one_subscriber(test, &subscriber_id, ignore_local_publications);

  union ioctl_publish_msg_args ioctl_publish_msg_ret;

  // Act
  int ret = publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_released_num, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_subscriber_num, 1);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_subscriber_ids[0], subscriber_id);
}

void test_case_publish_msg_ret_many_subscribers(struct kunit * test)
{
  // Arrange
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);

  bool ignore_local_publications = false;

  for (int i = 0; i < MAX_SUBSCRIBER_NUM; i++) {
    topic_local_id_t subscriber_id;
    setup_one_subscriber(test, &subscriber_id, ignore_local_publications);
  }

  union ioctl_publish_msg_args ioctl_publish_msg_ret;

  // Act
  int ret = publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_released_num, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_subscriber_num, MAX_SUBSCRIBER_NUM);
}

void test_case_ignore_local_same_pid_enabled(struct kunit * test)
{
  // Arrange
  topic_local_id_t publisher_id;
  topic_local_id_t subscriber_id;
  uint64_t ret_addr;

  bool ignore_local_publications = true;
  setup_pub_sub_same_process(
    test, &publisher_id, &subscriber_id, ignore_local_publications, &ret_addr);

  union ioctl_publish_msg_args ioctl_publish_msg_ret = {0};

  // Act
  int ret = publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_subscriber_num, 0);
}

void test_case_ignore_local_same_pid_disabled(struct kunit * test)
{
  // Arrange
  topic_local_id_t publisher_id;
  topic_local_id_t subscriber_id;
  uint64_t ret_addr;
  bool ignore_local_publications = false;

  setup_pub_sub_same_process(
    test, &publisher_id, &subscriber_id, ignore_local_publications, &ret_addr);

  union ioctl_publish_msg_args ioctl_publish_msg_ret = {0};

  // Act
  int ret = publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_subscriber_num, 1);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_subscriber_ids[0], subscriber_id);
}

void test_case_ignore_local_diff_pid_enabled(struct kunit * test)
{
  // Arrange
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);

  topic_local_id_t subscriber_id;
  bool ignore_local_publications = true;
  setup_one_subscriber(test, &subscriber_id, ignore_local_publications);

  union ioctl_publish_msg_args ioctl_publish_msg_ret = {0};

  // Act
  int ret = publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_subscriber_num, 1);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_subscriber_ids[0], subscriber_id);
}

void test_case_ignore_local_diff_pid_disabled(struct kunit * test)
{
  // Arrange
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);

  topic_local_id_t subscriber_id;
  bool ignore_local_publications = false;
  setup_one_subscriber(test, &subscriber_id, ignore_local_publications);

  union ioctl_publish_msg_args ioctl_publish_msg_ret = {0};

  // Act
  int ret = publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_subscriber_num, 1);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_subscriber_ids[0], subscriber_id);
}
