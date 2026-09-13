// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_reclaim_msgs.h"

#include "../agnocast.h"

#include <kunit/test.h>

static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * NODE_NAME = "/kunit_test_node";
static const uint32_t QOS_DEPTH = 1;
// A reclaim releases what publishing would have released, so a publisher needs
// more entries than its depth retains before there is anything to reclaim.
static const uint32_t PUBLISH_NUM = 3;
#define IS_BRIDGE false
#define KUNIT_PUB_SHM_BUF_SIZE 4

static pid_t publisher_pid = 4000;

// At file scope, as in the publish_msg suite: MAX_SUBSCRIBER_NUM entries do not
// fit a kernel stack frame.
static topic_local_id_t subscriber_ids_buf[MAX_SUBSCRIBER_NUM];

static void setup_publisher(
  struct kunit * test, pid_t * pid, topic_local_id_t * publisher_id, uint64_t * ret_addr)
{
  publisher_pid++;
  *pid = publisher_pid;

  union ioctl_add_process_args add_process_args;
  int ret = agnocast_ioctl_add_process(*pid, current->nsproxy->ipc_ns, false, 0, &add_process_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  *ret_addr = add_process_args.ret_addr;

  union ioctl_add_publisher_args add_publisher_args;
  ret = agnocast_ioctl_add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, *pid, QOS_DEPTH, false, IS_BRIDGE,
    &add_publisher_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  *publisher_id = add_publisher_args.ret_id;
}

static void publish_once(
  struct kunit * test, const topic_local_id_t publisher_id, const uint64_t addr)
{
  union ioctl_publish_msg_args publish_args;
  const int ret = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, addr, subscriber_ids_buf,
    MAX_SUBSCRIBER_NUM, &publish_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

// With no subscriber referencing them, publishing already released everything
// beyond the depth, so a reclaim that follows finds nothing left to do. The
// point is that it succeeds and reports zero rather than releasing an entry the
// depth is still meant to retain.
void test_case_reclaim_msgs_nothing_to_release(struct kunit * test)
{
  pid_t pid;
  topic_local_id_t publisher_id;
  uint64_t addr;
  setup_publisher(test, &pid, &publisher_id, &addr);

  for (uint32_t i = 0; i < PUBLISH_NUM; i++) {
    publish_once(test, publisher_id, addr + i * 1024);
  }

  union ioctl_reclaim_msgs_args reclaim_args;
  memset(&reclaim_args, 0, sizeof(reclaim_args));
  const int ret = agnocast_ioctl_reclaim_msgs(
    TOPIC_NAME, current->nsproxy->ipc_ns, pid, publisher_id, &reclaim_args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, reclaim_args.ret_released_num, 0u);
}

// A subscriber holding the oldest entry is what makes publishing unable to
// release it, which is the state a GPU publisher runs out of slots in. Once the
// reference is dropped nothing publishes again, so the reclaim is the only thing
// that can hand the address back.
void test_case_reclaim_msgs_releases_what_publish_could_not(struct kunit * test)
{
  pid_t pid;
  topic_local_id_t publisher_id;
  uint64_t addr;
  setup_publisher(test, &pid, &publisher_id, &addr);

  union ioctl_add_subscriber_args add_subscriber_args;
  int ret = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, pid, QOS_DEPTH, false, true, false, false,
    IS_BRIDGE, &add_subscriber_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  const topic_local_id_t subscriber_id = add_subscriber_args.ret_id;

  publish_once(test, publisher_id, addr);

  // Take the reference the publisher below will not be able to release past.
  union ioctl_receive_msg_args receive_args;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};
  ret = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &receive_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_ASSERT_EQ(test, receive_args.ret_entry_num, 1);
  const int64_t held_entry_id = receive_args.ret_entry_ids[0];

  // Beyond the depth now, but the oldest entry is referenced, so this releases
  // nothing.
  for (uint32_t i = 1; i < PUBLISH_NUM; i++) {
    publish_once(test, publisher_id, addr + i * 1024);
  }

  union ioctl_reclaim_msgs_args reclaim_args;
  memset(&reclaim_args, 0, sizeof(reclaim_args));
  ret = agnocast_ioctl_reclaim_msgs(
    TOPIC_NAME, current->nsproxy->ipc_ns, pid, publisher_id, &reclaim_args);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, reclaim_args.ret_released_num, 0u);

  // The subscriber lets go. Nothing publishes after this, which is exactly the
  // case a publisher with no free slot is in.
  ret = agnocast_ioctl_release_message_entry_reference(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, held_entry_id);
  KUNIT_ASSERT_EQ(test, ret, 0);

  memset(&reclaim_args, 0, sizeof(reclaim_args));
  ret = agnocast_ioctl_reclaim_msgs(
    TOPIC_NAME, current->nsproxy->ipc_ns, pid, publisher_id, &reclaim_args);
  // Exactly one: entries_num is 2 and the depth retains 1, so the release must
  // take the oldest and stop. A weaker bound would pass while over-releasing --
  // freeing a message the depth is still meant to hold.
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, reclaim_args.ret_released_num, 1u);
  KUNIT_EXPECT_EQ(test, reclaim_args.ret_released_addrs[0], addr);
}

// The addresses reported are passed straight to delete by the caller, so only
// the owning process may ask.
void test_case_reclaim_msgs_rejects_a_foreign_process(struct kunit * test)
{
  pid_t pid;
  topic_local_id_t publisher_id;
  uint64_t addr;
  setup_publisher(test, &pid, &publisher_id, &addr);

  union ioctl_reclaim_msgs_args reclaim_args;
  memset(&reclaim_args, 0, sizeof(reclaim_args));
  const int ret = agnocast_ioctl_reclaim_msgs(
    TOPIC_NAME, current->nsproxy->ipc_ns, pid + 1, publisher_id, &reclaim_args);

  KUNIT_EXPECT_EQ(test, ret, -EPERM);
}

void test_case_reclaim_msgs_topic_not_found(struct kunit * test)
{
  union ioctl_reclaim_msgs_args reclaim_args;
  memset(&reclaim_args, 0, sizeof(reclaim_args));
  const int ret =
    agnocast_ioctl_reclaim_msgs("/no_such_topic", current->nsproxy->ipc_ns, 1, 0, &reclaim_args);

  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_reclaim_msgs_publisher_not_found(struct kunit * test)
{
  pid_t pid;
  topic_local_id_t publisher_id;
  uint64_t addr;
  setup_publisher(test, &pid, &publisher_id, &addr);

  union ioctl_reclaim_msgs_args reclaim_args;
  memset(&reclaim_args, 0, sizeof(reclaim_args));
  const int ret = agnocast_ioctl_reclaim_msgs(
    TOPIC_NAME, current->nsproxy->ipc_ns, pid, publisher_id + 100, &reclaim_args);

  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}
