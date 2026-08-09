// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_publish_msg.h"

#include "../agnocast.h"
#include "../agnocast_memory_allocator.h"
#include "agnocast_kunit_eventfd.h"

#include <kunit/test.h>
#include <linux/delay.h>

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
  int ret1 = agnocast_ioctl_add_process(
    subscriber_pid, current->nsproxy->ipc_ns, false, 0, &add_process_args);

  union ioctl_add_subscriber_args add_subscriber_args;
  int ret2 = agnocast_ioctl_add_subscriber(
    topic_name, current->nsproxy->ipc_ns, node_name, subscriber_pid, qos_depth,
    qos_is_transient_local, qos_is_reliable, is_take_sub, ignore_local_publications, is_bridge, -1,
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
  int ret1 = agnocast_ioctl_add_process(
    publisher_pid, current->nsproxy->ipc_ns, false, 0, &add_process_args);
  *ret_addr = add_process_args.ret_addr;

  union ioctl_add_publisher_args add_publisher_args;
  int ret2 = agnocast_ioctl_add_publisher(
    topic_name, current->nsproxy->ipc_ns, node_name, publisher_pid, qos_depth,
    qos_is_transient_local, is_bridge, &add_publisher_args);
  *publisher_id = add_publisher_args.ret_id;

  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ret2, 0);
}

// The setup helpers above pass -1, which the fake maps to its unobserved slot: the subscriber is
// still collected and signaled on publish, but no assertion can see it. Cases that assert on
// delivery need a real fd instead.
static void add_subscriber_with_eventfd(
  struct kunit * test, const pid_t pid, const int eventfd, const bool ignore_local_publications)
{
  union ioctl_add_subscriber_args add_subscriber_args;
  int ret = agnocast_ioctl_add_subscriber(
    topic_name, current->nsproxy->ipc_ns, node_name, pid, qos_depth, qos_is_transient_local,
    qos_is_reliable, is_take_sub, ignore_local_publications, is_bridge, eventfd,
    &add_subscriber_args);

  KUNIT_ASSERT_EQ(test, ret, 0);
}

// Same, but in a process of its own.
static void setup_one_subscriber_with_eventfd(
  struct kunit * test, const int eventfd, const bool ignore_local_publications)
{
  subscriber_pid++;

  union ioctl_add_process_args add_process_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      subscriber_pid, current->nsproxy->ipc_ns, false, 0, &add_process_args),
    0);
  add_subscriber_with_eventfd(test, subscriber_pid, eventfd, ignore_local_publications);
}

// One process holding both, which is what makes ignore_local_publications observable.
static void setup_pub_sub_same_process_with_eventfd(
  struct kunit * test, topic_local_id_t * publisher_id, const int eventfd,
  const bool ignore_local_publications, uint64_t * ret_addr)
{
  common_pid++;

  union ioctl_add_process_args add_process_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(common_pid, current->nsproxy->ipc_ns, false, 0, &add_process_args),
    0);
  *ret_addr = add_process_args.ret_addr;

  union ioctl_add_publisher_args add_publisher_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_publisher(
      topic_name, current->nsproxy->ipc_ns, node_name, common_pid, qos_depth,
      qos_is_transient_local, is_bridge, &add_publisher_args),
    0);
  *publisher_id = add_publisher_args.ret_id;

  add_subscriber_with_eventfd(test, common_pid, eventfd, ignore_local_publications);
}

static uint32_t signal_count_of(const int eventfd)
{
  const struct agnocast_kunit_eventfd_slot * slot = agnocast_kunit_eventfd_slot_of(eventfd);

  // An fd outside the fake table fails the caller's comparison instead of dereferencing NULL.
  return slot ? slot->signal_count : U32_MAX;
}

// Expect to fail at find_topic()
void test_case_publish_msg_no_topic(struct kunit * test)
{
  // Arrange
  topic_local_id_t publisher_id = 0;
  uint64_t msg_virtual_address = 0x40000000000;
  union ioctl_publish_msg_args ioctl_publish_ret;

  // Act
  int ret = agnocast_ioctl_publish_msg(
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
  int ret = agnocast_ioctl_publish_msg(
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
  int ret = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_released_num, 0);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_is_in_topic_entries(
      topic_name, current->nsproxy->ipc_ns, ioctl_publish_msg_ret.ret_entry_id),
    true);
  KUNIT_EXPECT_EQ(test, agnocast_get_topic_entries_num(topic_name, current->nsproxy->ipc_ns), 1);
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
  int ret1 = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id1, ret_addr1, &ioctl_publish_msg_ret1);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  union ioctl_publish_msg_args ioctl_publish_msg_ret2;

  // Act
  int ret2 = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id2, ret_addr2, &ioctl_publish_msg_ret2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret2.ret_released_num, 0);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_is_in_topic_entries(
      topic_name, current->nsproxy->ipc_ns, ioctl_publish_msg_ret1.ret_entry_id),
    true);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_is_in_topic_entries(
      topic_name, current->nsproxy->ipc_ns, ioctl_publish_msg_ret2.ret_entry_id),
    true);
  KUNIT_EXPECT_EQ(test, agnocast_get_topic_entries_num(topic_name, current->nsproxy->ipc_ns), 2);
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
  int ret1 = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret1);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  // Subscriber takes a reference to entry1
  int ret_inc = agnocast_increment_message_entry_rc(
    topic_name, current->nsproxy->ipc_ns, subscriber_id, ioctl_publish_msg_ret1.ret_entry_id);
  KUNIT_ASSERT_EQ(test, ret_inc, 0);

  union ioctl_publish_msg_args ioctl_publish_msg_ret2;

  // Act
  int ret2 = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr + 1, &ioctl_publish_msg_ret2);

  // Assert: entry1 is not released because subscriber holds a reference
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret2.ret_released_num, 0);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_is_in_topic_entries(
      topic_name, current->nsproxy->ipc_ns, ioctl_publish_msg_ret1.ret_entry_id),
    true);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_is_in_topic_entries(
      topic_name, current->nsproxy->ipc_ns, ioctl_publish_msg_ret2.ret_entry_id),
    true);
  KUNIT_EXPECT_EQ(test, agnocast_get_topic_entries_num(topic_name, current->nsproxy->ipc_ns), 2);
}

void test_case_publish_msg_single_release_return(struct kunit * test)
{
  // Arrange: With qos_depth=1 and no subscriber references, entry1 is automatically released
  // when entry2 is published (GC is triggered to meet qos_depth).
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);

  union ioctl_publish_msg_args ioctl_publish_msg_ret1;
  int ret1 = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret1);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  union ioctl_publish_msg_args ioctl_publish_msg_ret2;

  // Act: entry1 should be released to meet qos_depth=1
  int ret2 = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr + 1, &ioctl_publish_msg_ret2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret2.ret_released_num, 1);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret2.ret_released_addrs[0], ret_addr);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_is_in_topic_entries(
      topic_name, current->nsproxy->ipc_ns, ioctl_publish_msg_ret1.ret_entry_id),
    false);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_is_in_topic_entries(
      topic_name, current->nsproxy->ipc_ns, ioctl_publish_msg_ret2.ret_entry_id),
    true);
  KUNIT_EXPECT_EQ(test, agnocast_get_topic_entries_num(topic_name, current->nsproxy->ipc_ns), 1);
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
    int ret = agnocast_ioctl_publish_msg(
      topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr + i, &ioctl_publish_msg_ret);
    entry_ids[i] = ioctl_publish_msg_ret.ret_entry_id;
    KUNIT_ASSERT_EQ(test, ret, 0);

    // Subscriber holds a reference to each entry to prevent immediate GC
    ret = agnocast_increment_message_entry_rc(
      topic_name, current->nsproxy->ipc_ns, subscriber_id, entry_ids[i]);
    KUNIT_ASSERT_EQ(test, ret, 0);
  }

  // Release all subscriber references so entries become eligible for GC
  for (int i = 0; i < MAX_RELEASE_NUM + 1; i++) {
    int ret = agnocast_ioctl_release_message_entry_reference(
      topic_name, current->nsproxy->ipc_ns, subscriber_id, entry_ids[i]);
    KUNIT_ASSERT_EQ(test, ret, 0);
  }

  union ioctl_publish_msg_args ioctl_publish_msg_ret;

  // Act: Publish one more message; GC should release up to MAX_RELEASE_NUM entries
  int ret = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert: GC is limited to MAX_RELEASE_NUM entries per publish call
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_publish_msg_ret.ret_released_num, MAX_RELEASE_NUM);
  // Remaining entries: (MAX_RELEASE_NUM + 1) - MAX_RELEASE_NUM + 1 (new) = 2
  KUNIT_EXPECT_EQ(test, agnocast_get_topic_entries_num(topic_name, current->nsproxy->ipc_ns), 2);
}

// ignore_local_publications drops a subscriber only when it shares the publisher's process. The
// flag-off case short-circuits before the pid comparison, so it needs no diff-pid counterpart.
void test_case_ignore_local_same_pid_enabled(struct kunit * test)
{
  // Arrange
  agnocast_kunit_eventfd_reset();
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const int eventfd = 0;
  setup_pub_sub_same_process_with_eventfd(test, &publisher_id, eventfd, true, &ret_addr);

  union ioctl_publish_msg_args ioctl_publish_msg_ret = {0};

  // Act
  int ret = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, signal_count_of(eventfd), 0);
}

void test_case_ignore_local_same_pid_disabled(struct kunit * test)
{
  // Arrange
  agnocast_kunit_eventfd_reset();
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const int eventfd = 0;
  setup_pub_sub_same_process_with_eventfd(test, &publisher_id, eventfd, false, &ret_addr);

  union ioctl_publish_msg_args ioctl_publish_msg_ret = {0};

  // Act
  int ret = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, signal_count_of(eventfd), 1);
}

void test_case_ignore_local_diff_pid_enabled(struct kunit * test)
{
  // Arrange
  agnocast_kunit_eventfd_reset();
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);

  const int eventfd = 0;
  setup_one_subscriber_with_eventfd(test, eventfd, true);

  union ioctl_publish_msg_args ioctl_publish_msg_ret = {0};

  // Act
  int ret = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert: the flag only suppresses publications from the subscriber's own process.
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, signal_count_of(eventfd), 1);
}

void test_case_publish_msg_signals_all_subscribers(struct kunit * test)
{
  // Arrange
  agnocast_kunit_eventfd_reset();
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);

  const int subscriber_num = 3;
  for (int eventfd = 0; eventfd < subscriber_num; eventfd++) {
    setup_one_subscriber_with_eventfd(test, eventfd, false);
  }

  union ioctl_publish_msg_args ioctl_publish_msg_ret = {0};

  // Act
  int ret = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  for (int eventfd = 0; eventfd < subscriber_num; eventfd++) {
    KUNIT_EXPECT_EQ(test, signal_count_of(eventfd), 1);
  }
}

void test_case_publish_msg_does_not_signal_take_sub(struct kunit * test)
{
  // Arrange
  agnocast_kunit_eventfd_reset();
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);

  subscriber_pid++;
  union ioctl_add_process_args add_process_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      subscriber_pid, current->nsproxy->ipc_ns, false, 0, &add_process_args),
    0);

  const int take_eventfd = 0;
  const int notify_eventfd = 1;
  union ioctl_add_subscriber_args take_sub_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_subscriber(
      topic_name, current->nsproxy->ipc_ns, node_name, subscriber_pid, qos_depth,
      qos_is_transient_local, qos_is_reliable, true /* is_take_sub */, false, is_bridge,
      take_eventfd, &take_sub_args),
    0);
  add_subscriber_with_eventfd(test, subscriber_pid, notify_eventfd, false);

  union ioctl_publish_msg_args ioctl_publish_msg_ret = {0};

  // Act
  int ret = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert: a take subscriber polls for messages, so PUBLISH must not wake it.
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, signal_count_of(take_eventfd), 0);
  KUNIT_EXPECT_EQ(test, signal_count_of(notify_eventfd), 1);
}

void test_case_publish_msg_signals_large_fanout(struct kunit * test)
{
  // Arrange: a fan-out well past what the e2e tests reach, to exercise the collection PUBLISH
  // does at scale.
  agnocast_kunit_eventfd_reset();
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);

  const int subscriber_num = 100;
  subscriber_pid++;
  union ioctl_add_process_args add_process_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      subscriber_pid, current->nsproxy->ipc_ns, false, 0, &add_process_args),
    0);
  for (int eventfd = 0; eventfd < subscriber_num; eventfd++) {
    add_subscriber_with_eventfd(test, subscriber_pid, eventfd, false);
  }

  union ioctl_publish_msg_args ioctl_publish_msg_ret = {0};

  // Act
  int ret = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert: every subscriber woken exactly once, none missed and none woken twice.
  KUNIT_EXPECT_EQ(test, ret, 0);
  for (int eventfd = 0; eventfd < subscriber_num; eventfd++) {
    KUNIT_EXPECT_EQ(test, signal_count_of(eventfd), 1);
  }
}

void test_case_publish_msg_signals_once_per_publish(struct kunit * test)
{
  // Arrange
  agnocast_kunit_eventfd_reset();
  const int publish_num = 5;
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);

  const int eventfd = 0;
  setup_one_subscriber_with_eventfd(test, eventfd, false);

  // Act
  for (int i = 0; i < publish_num; i++) {
    union ioctl_publish_msg_args ioctl_publish_msg_ret = {0};
    int ret = agnocast_ioctl_publish_msg(
      topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);
    KUNIT_ASSERT_EQ(test, ret, 0);
  }

  // Assert: one signal per publish, and publish never re-acquires the context.
  KUNIT_EXPECT_EQ(test, signal_count_of(eventfd), publish_num);
  const struct agnocast_kunit_eventfd_slot * slot = agnocast_kunit_eventfd_slot_of(eventfd);
  KUNIT_ASSERT_NOT_NULL(test, slot);
  KUNIT_EXPECT_EQ(test, slot->get_count, 1);
}

// The publisher's message must live in that publisher process's own mempool. Both sides of the
// bounds check are rejected, which is what keeps a caller from handing the kernel an address it
// does not own.
void test_case_publish_msg_address_below_mempool(struct kunit * test)
{
  // Arrange
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);

  union ioctl_publish_msg_args ioctl_publish_msg_ret;

  // Act
  int ret = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr - 1, &ioctl_publish_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
  KUNIT_EXPECT_EQ(test, agnocast_get_topic_entries_num(topic_name, current->nsproxy->ipc_ns), 0);
}

void test_case_publish_msg_address_above_mempool(struct kunit * test)
{
  // Arrange
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);

  union ioctl_publish_msg_args ioctl_publish_msg_ret;

  // Act: mempool_end is exclusive, so the first address past the pool must be rejected.
  int ret = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr + mempool_size_bytes,
    &ioctl_publish_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
  KUNIT_EXPECT_EQ(test, agnocast_get_topic_entries_num(topic_name, current->nsproxy->ipc_ns), 0);
}

// A publisher outlives its process when a subscriber still references one of its entries: the
// exit handler keeps publisher_info alive, but the daemon reaps process_info. Publishing on that
// stale id must be rejected rather than dereferencing the missing mempool.
void test_case_publish_msg_no_process(struct kunit * test)
{
  // Arrange
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_one_publisher(test, &publisher_id, &ret_addr);
  const pid_t exiting_pid = publisher_pid;  // setup_one_publisher() advanced it to the pid it used

  topic_local_id_t subscriber_id;
  setup_one_subscriber(test, &subscriber_id, false);

  union ioctl_publish_msg_args first_publish_ret;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_publish_msg(
      topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &first_publish_ret),
    0);
  KUNIT_ASSERT_EQ(
    test,
    agnocast_increment_message_entry_rc(
      topic_name, current->nsproxy->ipc_ns, subscriber_id, first_publish_ret.ret_entry_id),
    0);

  // The referenced entry keeps publisher_info alive through the exit handler.
  agnocast_enqueue_exit_pid(exiting_pid);
  msleep(20);
  KUNIT_ASSERT_TRUE(test, agnocast_is_proc_exited(exiting_pid));

  // The unlink daemon then reaps process_info, leaving the publisher without one.
  struct ioctl_get_exit_process_args get_exit_args;
  memset(&get_exit_args, 0, sizeof(get_exit_args));
  struct exit_subscription_mq_info mq_info_buf[1];
  pid_t global_pid = -1;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_get_exit_process(
      current->nsproxy->ipc_ns, &get_exit_args, mq_info_buf, ARRAY_SIZE(mq_info_buf), &global_pid),
    0);
  KUNIT_ASSERT_EQ(test, get_exit_args.ret_pid, exiting_pid);
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(
    current->nsproxy->ipc_ns, global_pid, get_exit_args.ret_subscription_mq_info_num,
    &daemon_should_exit);

  union ioctl_publish_msg_args ioctl_publish_msg_ret;

  // Act
  int ret = agnocast_ioctl_publish_msg(
    topic_name, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}
