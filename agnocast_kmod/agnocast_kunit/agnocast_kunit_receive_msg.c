// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_receive_msg.h"

#include "../agnocast.h"
#include "../agnocast_memory_allocator.h"
#include "agnocast_kunit_helpers.h"

#include <kunit/test.h>
#include <linux/delay.h>

static char * TOPIC_NAME = "/kunit_test_topic";
static char * NODE_NAME = "/kunit_test_node";
static bool IS_TAKE_SUB = false;
static bool IS_RELIABLE = true;
static bool IGNORE_LOCAL_PUBLICATIONS = false;
static bool IS_BRIDGE = false;

// Small buffer size for KUnit tests to avoid exceeding kernel stack frame limits.
#define KUNIT_PUB_SHM_BUF_SIZE 4

static void setup_subscriber_impl(
  struct kunit * test, const pid_t subscriber_pid, const uint32_t qos_depth,
  const bool is_transient_local, const uint32_t domain_id, const bool sub_is_bridge,
  topic_local_id_t * subscriber_id)
{
  *subscriber_id = agnocast_kunit_setup_one_subscriber(
    test, TOPIC_NAME, NODE_NAME, subscriber_pid, qos_depth, is_transient_local, IS_RELIABLE,
    IS_TAKE_SUB, IGNORE_LOCAL_PUBLICATIONS, sub_is_bridge, -1, domain_id);
}

static void setup_one_subscriber(
  struct kunit * test, pid_t subscriber_pid, uint32_t qos_depth, bool is_transient_local,
  topic_local_id_t * subscriber_id)
{
  setup_subscriber_impl(
    test, subscriber_pid, qos_depth, is_transient_local, 0, IS_BRIDGE, subscriber_id);
}

// RECEIVE resolves the wrapper by the caller's domain, so the subscriber of a cross-domain case
// has to be current->tgid.
static void setup_current_subscriber_in_domain(
  struct kunit * test, const uint32_t domain_id, const bool sub_is_bridge,
  topic_local_id_t * subscriber_id)
{
  setup_subscriber_impl(test, current->tgid, 1, false, domain_id, sub_is_bridge, subscriber_id);
}

static void setup_publisher_impl(
  struct kunit * test, const pid_t publisher_pid, const uint32_t qos_depth,
  const bool is_transient_local, const uint32_t domain_id, const bool pub_is_bridge,
  topic_local_id_t * publisher_id, uint64_t * ret_addr)
{
  *publisher_id = agnocast_kunit_setup_one_publisher(
    test, TOPIC_NAME, NODE_NAME, publisher_pid, qos_depth, is_transient_local, pub_is_bridge,
    domain_id, ret_addr);
}

static void setup_one_publisher(
  struct kunit * test, pid_t publisher_pid, uint32_t qos_depth, bool is_transient_local,
  topic_local_id_t * publisher_id, uint64_t * ret_addr)
{
  setup_publisher_impl(
    test, publisher_pid, qos_depth, is_transient_local, 0, IS_BRIDGE, publisher_id, ret_addr);
}

static void setup_publisher_in_domain(
  struct kunit * test, const pid_t publisher_pid, const uint32_t domain_id,
  const bool pub_is_bridge, topic_local_id_t * publisher_id, uint64_t * ret_addr)
{
  setup_publisher_impl(
    test, publisher_pid, 1, false, domain_id, pub_is_bridge, publisher_id, ret_addr);
}

void test_case_receive_msg_no_topic_when_receive(struct kunit * test)
{
  // Arrange
  topic_local_id_t subscriber_id = 0;
  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_receive_msg_no_subscriber_when_receive(struct kunit * test)
{
  // Arrange
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const pid_t publisher_pid = 1000;
  const uint32_t publisher_qos_depth = 1;
  bool publisher_transient_local = false;
  setup_one_publisher(
    test, publisher_pid, publisher_qos_depth, publisher_transient_local, &publisher_id, &ret_addr);

  topic_local_id_t subscriber_id = 0;
  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_receive_msg_no_publish_nothing_to_receive(struct kunit * test)
{
  // Arrange
  topic_local_id_t subscriber_id;
  const pid_t subscriber_pid = 2000;
  const uint32_t subscriber_qos_depth = 1;
  bool subscriber_transient_local = false;
  setup_one_subscriber(
    test, subscriber_pid, subscriber_qos_depth, subscriber_transient_local, &subscriber_id);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 0);
}

void test_case_receive_msg_receive_one(struct kunit * test)
{
  // Arrange
  bool is_transient_local = false;
  const uint32_t qos_depth = 1;

  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const pid_t publisher_pid = 1000;
  setup_one_publisher(test, publisher_pid, qos_depth, is_transient_local, &publisher_id, &ret_addr);
  topic_local_id_t subscriber_id;
  const pid_t subscriber_pid = 2000;
  setup_one_subscriber(test, subscriber_pid, qos_depth, is_transient_local, &subscriber_id);

  union ioctl_publish_msg_args ioctl_publish_msg_ret;
  int ret1 = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret2 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 1);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_ids[0], ioctl_publish_msg_ret.ret_entry_id);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_get_latest_received_entry_id(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id),
    ioctl_receive_msg_ret.ret_entry_ids[0]);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 1);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].pid, publisher_pid);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].shm_addr, ret_addr);
}

void test_case_receive_msg_sub_qos_smaller_than_publish_num_smaller_than_pub_qos(
  struct kunit * test)
{
  // Arrange
  bool is_transient_local = false;

  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const pid_t publisher_pid = 1000;
  const uint32_t publisher_qos_depth = 10;
  setup_one_publisher(
    test, publisher_pid, publisher_qos_depth, is_transient_local, &publisher_id, &ret_addr);
  topic_local_id_t subscriber_id;
  const pid_t subscriber_pid = 2000;
  const uint32_t subscriber_qos_depth = 1;
  setup_one_subscriber(
    test, subscriber_pid, subscriber_qos_depth, is_transient_local, &subscriber_id);

  union ioctl_publish_msg_args ioctl_publish_msg_ret1;
  int ret1 = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret1);
  KUNIT_ASSERT_EQ(test, ret1, 0);
  union ioctl_publish_msg_args ioctl_publish_msg_ret2;
  int ret2 = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr + 1, &ioctl_publish_msg_ret2);
  KUNIT_ASSERT_EQ(test, ret2, 0);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret3 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret3, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 1);
  KUNIT_EXPECT_EQ(
    test, ioctl_receive_msg_ret.ret_entry_ids[0], ioctl_publish_msg_ret2.ret_entry_id);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_get_latest_received_entry_id(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id),
    ioctl_receive_msg_ret.ret_entry_ids[0]);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 1);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].pid, publisher_pid);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].shm_addr, ret_addr);
}

void test_case_receive_msg_publish_num_smaller_than_sub_qos_smaller_than_pub_qos(
  struct kunit * test)
{
  // Arrange
  bool is_transient_local = false;

  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const pid_t publisher_pid = 1000;
  const uint32_t publisher_qos_depth = 10;
  setup_one_publisher(
    test, publisher_pid, publisher_qos_depth, is_transient_local, &publisher_id, &ret_addr);
  topic_local_id_t subscriber_id;
  const pid_t subscriber_pid = 2000;
  const uint32_t subscriber_qos_depth = 2;
  setup_one_subscriber(
    test, subscriber_pid, subscriber_qos_depth, is_transient_local, &subscriber_id);

  union ioctl_publish_msg_args ioctl_publish_msg_ret;
  int ret1 = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret2 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 1);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_ids[0], ioctl_publish_msg_ret.ret_entry_id);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_get_latest_received_entry_id(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id),
    ioctl_receive_msg_ret.ret_entry_ids[0]);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 1);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].pid, publisher_pid);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].shm_addr, ret_addr);
}

void test_case_receive_msg_sub_qos_smaller_than_pub_qos_smaller_than_publish_num(
  struct kunit * test)
{
  // Arrange
  bool is_transient_local = false;

  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const pid_t publisher_pid = 1000;
  const uint32_t publisher_qos_depth = 10;
  setup_one_publisher(
    test, publisher_pid, publisher_qos_depth, is_transient_local, &publisher_id, &ret_addr);
  topic_local_id_t subscriber_id;
  const pid_t subscriber_pid = 2000;
  const uint32_t subscriber_qos_depth = 1;
  setup_one_subscriber(
    test, subscriber_pid, subscriber_qos_depth, is_transient_local, &subscriber_id);

  for (int i = 0; i < publisher_qos_depth; i++) {
    union ioctl_publish_msg_args ioctl_publish_msg_ret;
    int ret = agnocast_ioctl_publish_msg(
      TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr + i, &ioctl_publish_msg_ret);
    KUNIT_ASSERT_EQ(test, ret, 0);
  }
  union ioctl_publish_msg_args ioctl_publish_msg_ret;
  int ret1 = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr + publisher_qos_depth + 1,
    &ioctl_publish_msg_ret);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret2 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 1);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_ids[0], ioctl_publish_msg_ret.ret_entry_id);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_get_latest_received_entry_id(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id),
    ioctl_receive_msg_ret.ret_entry_ids[0]);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 1);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].pid, publisher_pid);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].shm_addr, ret_addr);
}

void test_case_receive_msg_publish_num_and_sub_qos_and_pub_qos_are_all_max_receive_num(
  struct kunit * test)
{
  // Arrange
  bool is_transient_local = false;

  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const pid_t publisher_pid = 1000;
  const uint32_t publisher_qos_depth = MAX_RECEIVE_NUM;
  setup_one_publisher(
    test, publisher_pid, publisher_qos_depth, is_transient_local, &publisher_id, &ret_addr);
  topic_local_id_t subscriber_id;
  const pid_t subscriber_pid = 2000;
  const uint32_t subscriber_qos_depth = MAX_RECEIVE_NUM;
  setup_one_subscriber(
    test, subscriber_pid, subscriber_qos_depth, is_transient_local, &subscriber_id);

  for (int i = 0; i < MAX_RECEIVE_NUM - 1; i++) {
    union ioctl_publish_msg_args ioctl_publish_msg_ret;
    int ret = agnocast_ioctl_publish_msg(
      TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);
    KUNIT_ASSERT_EQ(test, ret, 0);
  }
  union ioctl_publish_msg_args ioctl_publish_msg_ret;
  int ret = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);
  KUNIT_ASSERT_EQ(test, ret, 0);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret3 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret3, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, MAX_RECEIVE_NUM);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_get_latest_received_entry_id(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id),
    ioctl_publish_msg_ret.ret_entry_id);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 1);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].pid, publisher_pid);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].shm_addr, ret_addr);
}

void test_case_receive_msg_qos_depth_larger_than_max_receive_num(struct kunit * test)
{
  // Arrange
  bool is_transient_local = false;
  const uint32_t qos_depth = MAX_RECEIVE_NUM * 2;

  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const pid_t publisher_pid = 1000;
  setup_one_publisher(test, publisher_pid, qos_depth, is_transient_local, &publisher_id, &ret_addr);
  topic_local_id_t subscriber_id;
  const pid_t subscriber_pid = 2000;
  setup_one_subscriber(test, subscriber_pid, qos_depth, is_transient_local, &subscriber_id);

  // Publish qos_depth messages
  int64_t first_entry_id = -1;
  int64_t last_entry_id = -1;
  for (uint32_t i = 0; i < qos_depth; i++) {
    union ioctl_publish_msg_args ioctl_publish_msg_ret;
    int ret = agnocast_ioctl_publish_msg(
      TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr + i, &ioctl_publish_msg_ret);
    KUNIT_ASSERT_EQ(test, ret, 0);
    if (i == 0) {
      first_entry_id = ioctl_publish_msg_ret.ret_entry_id;
    }
    last_entry_id = ioctl_publish_msg_ret.ret_entry_id;
  }

  // Act & Assert - First ioctl_receive_msg call
  union ioctl_receive_msg_args ioctl_receive_msg_ret1;
  struct publisher_shm_info pub_shm_infos1[KUNIT_PUB_SHM_BUF_SIZE] = {0};
  int ret1 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos1, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret1);

  KUNIT_EXPECT_EQ(test, ret1, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret1.ret_entry_num, MAX_RECEIVE_NUM);
  KUNIT_EXPECT_TRUE(test, ioctl_receive_msg_ret1.ret_call_again);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret1.ret_entry_ids[0], first_entry_id);
  KUNIT_EXPECT_EQ(
    test, ioctl_receive_msg_ret1.ret_entry_ids[MAX_RECEIVE_NUM - 1],
    first_entry_id + MAX_RECEIVE_NUM - 1);

  // Act & Assert - Second ioctl_receive_msg call
  union ioctl_receive_msg_args ioctl_receive_msg_ret2;
  struct publisher_shm_info pub_shm_infos2[KUNIT_PUB_SHM_BUF_SIZE] = {0};
  int ret2 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos2, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret2);

  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret2.ret_entry_num, MAX_RECEIVE_NUM);
  KUNIT_EXPECT_FALSE(test, ioctl_receive_msg_ret2.ret_call_again);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret2.ret_entry_ids[0], first_entry_id + MAX_RECEIVE_NUM);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret2.ret_entry_ids[MAX_RECEIVE_NUM - 1], last_entry_id);

  // Verify latest_received_entry_id is updated to the last entry
  KUNIT_EXPECT_EQ(
    test,
    agnocast_get_latest_received_entry_id(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id),
    last_entry_id);
}

void test_case_receive_msg_transient_sub_qos_and_pub_qos_and_publish_num_are_all_equal(
  struct kunit * test)
{
  // Arrange
  const bool is_transient_local = true;

  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const pid_t publisher_pid = 1000;
  const uint32_t publisher_qos_depth = 1;
  setup_one_publisher(
    test, publisher_pid, publisher_qos_depth, is_transient_local, &publisher_id, &ret_addr);

  union ioctl_publish_msg_args ioctl_publish_msg_ret;
  int ret1 = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  topic_local_id_t subscriber_id;
  const pid_t subscriber_pid = 2000;
  const uint32_t subscriber_qos_depth = 1;
  setup_one_subscriber(
    test, subscriber_pid, subscriber_qos_depth, is_transient_local, &subscriber_id);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret2 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 1);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_ids[0], ioctl_publish_msg_ret.ret_entry_id);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_get_latest_received_entry_id(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id),
    ioctl_receive_msg_ret.ret_entry_ids[0]);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 1);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].pid, publisher_pid);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].shm_addr, ret_addr);
}

void test_case_receive_msg_transient_sub_qos_smaller_than_pub_qos_smaller_than_publish_num(
  struct kunit * test)
{
  // Arrange
  const bool is_transient_local = true;

  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const pid_t publisher_pid = 1000;
  const uint32_t publisher_qos_depth = 2;
  setup_one_publisher(
    test, publisher_pid, publisher_qos_depth, is_transient_local, &publisher_id, &ret_addr);

  union ioctl_publish_msg_args ioctl_publish_msg_ret1;
  int ret1 = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret1);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  union ioctl_publish_msg_args ioctl_publish_msg_ret2;
  int ret2 = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr + 1, &ioctl_publish_msg_ret2);
  KUNIT_ASSERT_EQ(test, ret2, 0);

  union ioctl_publish_msg_args ioctl_publish_msg_ret3;
  int ret3 = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret3);
  KUNIT_ASSERT_EQ(test, ret3, 0);

  topic_local_id_t subscriber_id;
  const pid_t subscriber_pid = 2000;
  const uint32_t subscriber_qos_depth = 1;
  setup_one_subscriber(
    test, subscriber_pid, subscriber_qos_depth, is_transient_local, &subscriber_id);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret4 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret4, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 1);
  KUNIT_EXPECT_EQ(
    test, ioctl_receive_msg_ret.ret_entry_ids[0], ioctl_publish_msg_ret3.ret_entry_id);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_get_latest_received_entry_id(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id),
    ioctl_receive_msg_ret.ret_entry_ids[0]);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 1);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].pid, publisher_pid);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].shm_addr, ret_addr);
}

void test_case_receive_msg_transient_sub_qos_smaller_than_publish_num_smaller_than_pub_qos(
  struct kunit * test)
{
  // Arrange
  const bool is_transient_local = true;

  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const pid_t publisher_pid = 1000;
  const uint32_t publisher_qos_depth = 10;
  setup_one_publisher(
    test, publisher_pid, publisher_qos_depth, is_transient_local, &publisher_id, &ret_addr);

  union ioctl_publish_msg_args ioctl_publish_msg_ret1;
  int ret1 = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret1);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  union ioctl_publish_msg_args ioctl_publish_msg_ret2;
  int ret2 = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr + 1, &ioctl_publish_msg_ret2);
  KUNIT_ASSERT_EQ(test, ret2, 0);

  topic_local_id_t subscriber_id;
  const pid_t subscriber_pid = 2000;
  const uint32_t subscriber_qos_depth = 1;
  setup_one_subscriber(
    test, subscriber_pid, subscriber_qos_depth, is_transient_local, &subscriber_id);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret4 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret4, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 1);
  KUNIT_EXPECT_EQ(
    test, ioctl_receive_msg_ret.ret_entry_ids[0], ioctl_publish_msg_ret2.ret_entry_id);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_get_latest_received_entry_id(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id),
    ioctl_receive_msg_ret.ret_entry_ids[0]);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 1);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].pid, publisher_pid);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].shm_addr, ret_addr);
}

void test_case_receive_msg_transient_publish_num_smaller_than_sub_qos_smaller_than_pub_qos(
  struct kunit * test)
{
  // Arrange
  const bool is_transient_local = true;

  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const pid_t publisher_pid = 1000;
  const uint32_t publisher_qos_depth = 10;
  setup_one_publisher(
    test, publisher_pid, publisher_qos_depth, is_transient_local, &publisher_id, &ret_addr);

  union ioctl_publish_msg_args ioctl_publish_msg_ret1;
  int ret1 = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret1);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  union ioctl_publish_msg_args ioctl_publish_msg_ret2;
  int ret2 = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr + 1, &ioctl_publish_msg_ret2);
  KUNIT_ASSERT_EQ(test, ret2, 0);

  topic_local_id_t subscriber_id;
  const pid_t subscriber_pid = 2000;
  const uint32_t subscriber_qos_depth = 3;
  setup_one_subscriber(
    test, subscriber_pid, subscriber_qos_depth, is_transient_local, &subscriber_id);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret4 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret4, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 2);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_get_latest_received_entry_id(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id),
    ioctl_publish_msg_ret2.ret_entry_id);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 1);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].pid, publisher_pid);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].shm_addr, ret_addr);
}

// qos_depth counts the messages the topic holds, not a span of entry_ids. entry_id is topic-wide
// while release_msgs_to_meet_depth() drops one publisher's oldest entries and leaves the others
// alone, so the ids a topic holds have gaps in them, and a depth of 3 still reaches 3 messages
// across those gaps.
void test_case_receive_msg_qos_depth_counts_entries_not_entry_ids(struct kunit * test)
{
  // Arrange
  const bool is_transient_local = true;

  topic_local_id_t publisher_id1;
  uint64_t ret_addr1;
  const uint32_t publisher1_qos_depth = 1;
  setup_one_publisher(
    test, 1000, publisher1_qos_depth, is_transient_local, &publisher_id1, &ret_addr1);
  union ioctl_publish_msg_args publish_msg_ret;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_publish_msg(
      TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id1, ret_addr1, &publish_msg_ret),
    0);
  const int64_t publisher1_entry_id = publish_msg_ret.ret_entry_id;

  // Publishes once more than its depth, so its oldest entry is released and that id is left as a
  // hole above the entry the other publisher holds.
  topic_local_id_t publisher_id2;
  uint64_t ret_addr2;
  const uint32_t publisher2_qos_depth = 2;
  setup_one_publisher(
    test, 1001, publisher2_qos_depth, is_transient_local, &publisher_id2, &ret_addr2);
  int64_t publisher2_older_entry_id = -1;
  int64_t publisher2_newer_entry_id = -1;
  for (uint32_t i = 0; i < publisher2_qos_depth + 1; i++) {
    KUNIT_ASSERT_EQ(
      test,
      agnocast_ioctl_publish_msg(
        TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id2, ret_addr2 + i, &publish_msg_ret),
      0);
    publisher2_older_entry_id = publisher2_newer_entry_id;
    publisher2_newer_entry_id = publish_msg_ret.ret_entry_id;
  }

  topic_local_id_t subscriber_id;
  const uint32_t subscriber_qos_depth = 3;
  setup_one_subscriber(test, 2000, subscriber_qos_depth, is_transient_local, &subscriber_id);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 3);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_ids[0], publisher1_entry_id);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_ids[1], publisher2_older_entry_id);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_ids[2], publisher2_newer_entry_id);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_get_latest_received_entry_id(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id),
    publisher2_newer_entry_id);
}

void test_case_receive_msg_zero_qos_depth_receives_nothing(struct kunit * test)
{
  // Arrange
  const bool is_transient_local = true;

  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const pid_t publisher_pid = 1000;
  const uint32_t publisher_qos_depth = 1;
  setup_one_publisher(
    test, publisher_pid, publisher_qos_depth, is_transient_local, &publisher_id, &ret_addr);
  union ioctl_publish_msg_args publish_msg_ret;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_publish_msg(
      TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr, &publish_msg_ret),
    0);

  topic_local_id_t subscriber_id;
  const uint32_t subscriber_qos_depth = 0;
  setup_one_subscriber(test, 2000, subscriber_qos_depth, is_transient_local, &subscriber_id);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_call_again, false);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_get_latest_received_entry_id(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id), -1);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 1);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].pid, publisher_pid);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].shm_addr, ret_addr);
}

void test_case_receive_msg_no_call_again_when_only_undeliverable_entries_remain(struct kunit * test)
{
  // Arrange
  const bool is_transient_local = true;
  const uint32_t qos_depth = MAX_RECEIVE_NUM;
  const pid_t subscriber_pid = 2000;

  topic_local_id_t remote_publisher_id;
  uint64_t remote_ret_addr;
  setup_one_publisher(
    test, 1000, qos_depth, is_transient_local, &remote_publisher_id, &remote_ret_addr);
  union ioctl_publish_msg_args publish_msg_ret;
  int64_t remote_newest_entry_id = -1;
  for (uint32_t i = 0; i < qos_depth; i++) {
    KUNIT_ASSERT_EQ(
      test,
      agnocast_ioctl_publish_msg(
        TOPIC_NAME, current->nsproxy->ipc_ns, remote_publisher_id, remote_ret_addr + i,
        &publish_msg_ret),
      0);
    remote_newest_entry_id = publish_msg_ret.ret_entry_id;
  }

  // The newest entry belongs to the subscriber's own process, so the walk that delivers the
  // MAX_RECEIVE_NUM entries below it ends on one it has to skip.
  topic_local_id_t local_publisher_id;
  uint64_t local_ret_addr;
  setup_one_publisher(
    test, subscriber_pid, 1, is_transient_local, &local_publisher_id, &local_ret_addr);
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_publish_msg(
      TOPIC_NAME, current->nsproxy->ipc_ns, local_publisher_id, local_ret_addr, &publish_msg_ret),
    0);

  const bool ignore_local_publications = true;
  union ioctl_add_subscriber_args add_subscriber_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_subscriber(
      TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, subscriber_pid, qos_depth,
      is_transient_local, IS_RELIABLE, IS_TAKE_SUB, ignore_local_publications, IS_BRIDGE, -1,
      &add_subscriber_args),
    0);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args.ret_id, pub_shm_infos,
    KUNIT_PUB_SHM_BUF_SIZE, &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, MAX_RECEIVE_NUM);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_call_again, false);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_get_latest_received_entry_id(
      TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args.ret_id),
    remote_newest_entry_id);
}

// qos_depth counts messages the subscriber can be handed: an entry it has to discard (here its
// own, under ignore_local_publications) does not use up a slot, even as the newest on the topic.
void test_case_receive_msg_discarded_message_does_not_consume_qos_depth(struct kunit * test)
{
  // Arrange
  const bool is_transient_local = true;
  const uint32_t qos_depth = 1;
  const pid_t subscriber_pid = 2000;

  topic_local_id_t remote_publisher_id;
  uint64_t remote_ret_addr;
  setup_one_publisher(
    test, 1000, qos_depth, is_transient_local, &remote_publisher_id, &remote_ret_addr);
  union ioctl_publish_msg_args remote_publish_ret;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_publish_msg(
      TOPIC_NAME, current->nsproxy->ipc_ns, remote_publisher_id, remote_ret_addr,
      &remote_publish_ret),
    0);

  // Published after the remote one, from the subscriber's own process, so it is the newest entry
  // and the one the subscriber has to skip.
  topic_local_id_t local_publisher_id;
  uint64_t local_ret_addr;
  setup_one_publisher(
    test, subscriber_pid, qos_depth, is_transient_local, &local_publisher_id, &local_ret_addr);
  union ioctl_publish_msg_args local_publish_ret;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_publish_msg(
      TOPIC_NAME, current->nsproxy->ipc_ns, local_publisher_id, local_ret_addr, &local_publish_ret),
    0);

  const bool ignore_local_publications = true;
  union ioctl_add_subscriber_args add_subscriber_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_subscriber(
      TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, subscriber_pid, qos_depth,
      is_transient_local, IS_RELIABLE, IS_TAKE_SUB, ignore_local_publications, IS_BRIDGE, -1,
      &add_subscriber_args),
    0);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args.ret_id, pub_shm_infos,
    KUNIT_PUB_SHM_BUF_SIZE, &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 1);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_ids[0], remote_publish_ret.ret_entry_id);
  KUNIT_EXPECT_EQ(
    test,
    agnocast_get_latest_received_entry_id(
      TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args.ret_id),
    remote_publish_ret.ret_entry_id);
}

// ================================================
// Tests for set_publisher_shm_info

void test_case_receive_msg_one_new_pub(struct kunit * test)
{
  // Arrange
  const bool is_transient_local = false;

  topic_local_id_t subscriber_id;
  const pid_t subscriber_pid = 2000;
  const uint32_t subscriber_qos_depth = 10;
  setup_one_subscriber(
    test, subscriber_pid, subscriber_qos_depth, is_transient_local, &subscriber_id);
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const pid_t publisher_pid = 1000;
  const uint32_t publisher_qos_depth = 10;
  setup_one_publisher(
    test, publisher_pid, publisher_qos_depth, is_transient_local, &publisher_id, &ret_addr);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 1);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].pid, publisher_pid);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].shm_addr, ret_addr);
}

void test_case_receive_msg_pubsub_in_same_process(struct kunit * test)
{
  // Arrange
  const bool is_transient_local = false;

  union ioctl_add_process_args add_process_args;
  const pid_t pid = 1000;
  int ret1 = agnocast_ioctl_add_process(
    pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &add_process_args);
  union ioctl_add_subscriber_args add_subscriber_args;
  const uint32_t subscriber_qos_depth = 10;
  int ret2 = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, pid, subscriber_qos_depth, is_transient_local,
    IS_RELIABLE, IS_TAKE_SUB, IGNORE_LOCAL_PUBLICATIONS, IS_BRIDGE, -1, &add_subscriber_args);
  union ioctl_add_publisher_args add_publisher_args;
  const uint32_t publisher_qos_depth = 10;
  int ret3 = agnocast_ioctl_add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, pid, publisher_qos_depth, is_transient_local,
    IS_BRIDGE, &add_publisher_args);
  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ret2, 0);
  KUNIT_ASSERT_EQ(test, ret3, 0);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret4 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args.ret_id, pub_shm_infos,
    KUNIT_PUB_SHM_BUF_SIZE, &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret4, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 0);
}

void test_case_receive_msg_2pub_in_same_process(struct kunit * test)
{
  // Arrange
  const bool is_transient_local = false;

  topic_local_id_t subscriber_id;
  const pid_t subscriber_pid = 2000;
  const uint32_t subscriber_qos_depth = 10;
  setup_one_subscriber(
    test, subscriber_pid, subscriber_qos_depth, is_transient_local, &subscriber_id);

  union ioctl_add_process_args add_process_args;
  const pid_t publisher_pid = 1000;
  int ret1 = agnocast_ioctl_add_process(
    publisher_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &add_process_args);
  union ioctl_add_publisher_args add_publisher_args1;
  const uint32_t publisher_qos_depth = 10;
  int ret2 = agnocast_ioctl_add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, publisher_pid, publisher_qos_depth,
    is_transient_local, IS_BRIDGE, &add_publisher_args1);
  union ioctl_add_publisher_args add_publisher_args2;
  int ret3 = agnocast_ioctl_add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, publisher_pid, publisher_qos_depth,
    is_transient_local, IS_BRIDGE, &add_publisher_args2);
  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ret2, 0);
  KUNIT_ASSERT_EQ(test, ret3, 0);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret4 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret4, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 1);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].pid, publisher_pid);
  KUNIT_EXPECT_EQ(test, pub_shm_infos[0].shm_addr, add_process_args.ret_addr);
}

void test_case_receive_msg_2sub_in_same_process(struct kunit * test)
{
  // Arrange
  const bool is_transient_local = false;

  union ioctl_add_process_args add_process_args;
  const pid_t subscriber_pid = 2000;
  int ret1 = agnocast_ioctl_add_process(
    subscriber_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &add_process_args);
  union ioctl_add_subscriber_args add_subscriber_args1;
  const uint32_t subscriber_qos_depth1 = 10;
  int ret2 = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, subscriber_pid, subscriber_qos_depth1,
    is_transient_local, IS_RELIABLE, IS_TAKE_SUB, IGNORE_LOCAL_PUBLICATIONS, IS_BRIDGE, -1,
    &add_subscriber_args1);
  union ioctl_add_subscriber_args add_subscriber_args2;
  const uint32_t subscriber_qos_depth2 = 1;
  int ret3 = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, subscriber_pid, subscriber_qos_depth2,
    is_transient_local, IS_RELIABLE, IS_TAKE_SUB, IGNORE_LOCAL_PUBLICATIONS, IS_BRIDGE, -1,
    &add_subscriber_args2);
  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ret2, 0);
  KUNIT_ASSERT_EQ(test, ret3, 0);

  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const pid_t publisher_pid = 1000;
  const uint32_t publisher_qos_depth = 10;
  setup_one_publisher(
    test, publisher_pid, publisher_qos_depth, is_transient_local, &publisher_id, &ret_addr);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};
  int ret4 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args1.ret_id, pub_shm_infos,
    KUNIT_PUB_SHM_BUF_SIZE, &ioctl_receive_msg_ret);
  KUNIT_ASSERT_EQ(test, ret4, 0);
  KUNIT_ASSERT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 0);
  KUNIT_ASSERT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 1);

  // Act
  int ret5 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args2.ret_id, pub_shm_infos,
    KUNIT_PUB_SHM_BUF_SIZE, &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret5, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 0);
}

void test_case_receive_msg_twice(struct kunit * test)
{
  // Arrange
  const bool is_transient_local = false;

  topic_local_id_t subscriber_id;
  const pid_t subscriber_pid = 2000;
  const uint32_t subscriber_qos_depth = 10;
  setup_one_subscriber(
    test, subscriber_pid, subscriber_qos_depth, is_transient_local, &subscriber_id);

  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const pid_t publisher_pid = 1000;
  const uint32_t publisher_qos_depth = 10;
  setup_one_publisher(
    test, publisher_pid, publisher_qos_depth, is_transient_local, &publisher_id, &ret_addr);

  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};
  int ret1 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);
  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 0);
  KUNIT_ASSERT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 1);
  KUNIT_ASSERT_EQ(test, pub_shm_infos[0].pid, publisher_pid);
  KUNIT_ASSERT_EQ(test, pub_shm_infos[0].shm_addr, ret_addr);

  // Act
  int ret2 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 0);
}

void test_case_receive_msg_with_exited_publisher(struct kunit * test)
{
  // Arrange
  const uint32_t qos_depth = 1;
  const bool is_transient_local = true;

  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  const pid_t publisher_pid = 1000;
  setup_one_publisher(test, publisher_pid, qos_depth, is_transient_local, &publisher_id, &ret_addr);

  uint64_t msg_addr = ret_addr;
  union ioctl_publish_msg_args ioctl_publish_msg_ret;
  int ret1 = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, msg_addr, &ioctl_publish_msg_ret);

  topic_local_id_t subscriber_id1;
  const pid_t subscriber_pid1 = 2000;
  setup_one_subscriber(test, subscriber_pid1, qos_depth, is_transient_local, &subscriber_id1);

  union ioctl_receive_msg_args ioctl_receive_msg_ret1;
  struct publisher_shm_info pub_shm_infos1[KUNIT_PUB_SHM_BUF_SIZE] = {0};
  int ret2 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id1, pub_shm_infos1, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret1);

  agnocast_enqueue_exit_pid(publisher_pid);

  // wait for exit_worker_thread to handle process exit
  msleep(10);

  topic_local_id_t subscriber_id2;
  const pid_t subscriber_pid2 = 2001;
  setup_one_subscriber(test, subscriber_pid2, qos_depth, is_transient_local, &subscriber_id2);

  union ioctl_get_subscriber_num_args ioctl_get_subscriber_num_ret;
  int ret3 = agnocast_ioctl_get_subscriber_num(
    TOPIC_NAME, current->nsproxy->ipc_ns, current->tgid, &ioctl_get_subscriber_num_ret);
  union ioctl_get_publisher_num_args get_publisher_num_args;
  int ret4 =
    agnocast_ioctl_get_publisher_num(TOPIC_NAME, current->nsproxy->ipc_ns, &get_publisher_num_args);

  KUNIT_ASSERT_EQ(test, ret1, 0);
  KUNIT_ASSERT_EQ(test, ret2, 0);
  KUNIT_ASSERT_EQ(test, ret3, 0);
  KUNIT_ASSERT_EQ(test, ret4, 0);
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 2);
  KUNIT_ASSERT_TRUE(test, agnocast_is_proc_exited(publisher_pid));
  KUNIT_ASSERT_FALSE(test, agnocast_is_proc_exited(subscriber_pid1));
  KUNIT_ASSERT_FALSE(test, agnocast_is_proc_exited(subscriber_pid2));
  KUNIT_ASSERT_EQ(test, agnocast_get_topic_num(current->nsproxy->ipc_ns), 1);
  KUNIT_ASSERT_TRUE(test, agnocast_is_in_topic_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
  KUNIT_EXPECT_EQ(test, get_publisher_num_args.ret_publisher_num, 1);
  KUNIT_ASSERT_TRUE(
    test, agnocast_is_in_publisher_htable(TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id));
  KUNIT_ASSERT_EQ(test, ioctl_get_subscriber_num_ret.ret_other_process_subscriber_num, 2);
  KUNIT_ASSERT_TRUE(
    test, agnocast_is_in_subscriber_htable(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id1));
  KUNIT_ASSERT_TRUE(
    test, agnocast_is_in_subscriber_htable(TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id2));
  KUNIT_ASSERT_EQ(test, agnocast_get_topic_entries_num(TOPIC_NAME, current->nsproxy->ipc_ns), 1);
  KUNIT_ASSERT_EQ(
    test,
    agnocast_get_entry_rc(
      TOPIC_NAME, current->nsproxy->ipc_ns, ioctl_publish_msg_ret.ret_entry_id, publisher_id),
    0);
  KUNIT_ASSERT_EQ(
    test,
    agnocast_get_entry_rc(
      TOPIC_NAME, current->nsproxy->ipc_ns, ioctl_publish_msg_ret.ret_entry_id, subscriber_id1),
    1);
  KUNIT_ASSERT_EQ(
    test,
    agnocast_get_entry_rc(
      TOPIC_NAME, current->nsproxy->ipc_ns, ioctl_publish_msg_ret.ret_entry_id, subscriber_id2),
    0);
  KUNIT_ASSERT_EQ(test, ioctl_receive_msg_ret1.ret_entry_num, 1);
  KUNIT_ASSERT_EQ(
    test, ioctl_receive_msg_ret1.ret_entry_ids[0], ioctl_publish_msg_ret.ret_entry_id);
  KUNIT_ASSERT_EQ(test, ioctl_receive_msg_ret1.ret_entry_addrs[0], msg_addr);
  KUNIT_ASSERT_EQ(test, ioctl_receive_msg_ret1.ret_pub_shm_num, 1);
  KUNIT_ASSERT_EQ(test, pub_shm_infos1[0].pid, publisher_pid);
  KUNIT_ASSERT_EQ(test, pub_shm_infos1[0].shm_addr, ret_addr);

  // Act
  union ioctl_receive_msg_args ioctl_receive_msg_ret2;
  struct publisher_shm_info pub_shm_infos2[KUNIT_PUB_SHM_BUF_SIZE] = {0};
  int ret5 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id2, pub_shm_infos2, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret5, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret2.ret_entry_num, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret2.ret_pub_shm_num, 0);
}

void test_case_receive_msg_pub_shm_info_buffer_too_small(struct kunit * test)
{
  // Arrange: create a subscriber and 2 publishers from different processes,
  // then call ioctl_receive_msg with pub_shm_infos_size=1 so that
  // set_publisher_shm_info returns -ENOBUFS.
  const bool is_transient_local = false;
  const uint32_t qos_depth = 10;

  topic_local_id_t subscriber_id;
  const pid_t subscriber_pid = 3000;
  setup_one_subscriber(test, subscriber_pid, qos_depth, is_transient_local, &subscriber_id);

  topic_local_id_t publisher_id1;
  uint64_t ret_addr1;
  const pid_t publisher_pid1 = 1000;
  setup_one_publisher(
    test, publisher_pid1, qos_depth, is_transient_local, &publisher_id1, &ret_addr1);

  topic_local_id_t publisher_id2;
  uint64_t ret_addr2;
  const pid_t publisher_pid2 = 1001;
  setup_one_publisher(
    test, publisher_pid2, qos_depth, is_transient_local, &publisher_id2, &ret_addr2);

  // Act: pass a buffer of size 1, but there are 2 publishers to map
  struct publisher_shm_info pub_shm_infos[1] = {0};
  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  int ret = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, 1, &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -ENOBUFS);
}

// ================================================
// Tests for ignore_local_publications

void test_case_receive_msg_ignore_local_same_pid_enabled(struct kunit * test)
{
  // Arrange: publisher and subscriber in the same process with ignore_local_publications=true
  const bool is_transient_local = false;
  const uint32_t qos_depth = 10;
  const pid_t pid = 1000;

  union ioctl_add_process_args add_process_args;
  int ret1 = agnocast_ioctl_add_process(
    pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &add_process_args);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  union ioctl_add_publisher_args add_publisher_args;
  int ret2 = agnocast_ioctl_add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, pid, qos_depth, is_transient_local, IS_BRIDGE,
    &add_publisher_args);
  KUNIT_ASSERT_EQ(test, ret2, 0);

  const bool ignore_local_publications = true;
  union ioctl_add_subscriber_args add_subscriber_args;
  int ret3 = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, pid, qos_depth, is_transient_local,
    IS_RELIABLE, IS_TAKE_SUB, ignore_local_publications, IS_BRIDGE, -1, &add_subscriber_args);
  KUNIT_ASSERT_EQ(test, ret3, 0);

  // Publish a message
  union ioctl_publish_msg_args ioctl_publish_msg_ret;
  int ret4 = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, add_publisher_args.ret_id, add_process_args.ret_addr,
    &ioctl_publish_msg_ret);
  KUNIT_ASSERT_EQ(test, ret4, 0);

  // Act: receive_msg should not return the message from the same process
  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};
  int ret5 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args.ret_id, pub_shm_infos,
    KUNIT_PUB_SHM_BUF_SIZE, &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret5, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 0);
}

void test_case_receive_msg_ignore_local_same_pid_disabled(struct kunit * test)
{
  // Arrange: publisher and subscriber in the same process with ignore_local_publications=false
  const bool is_transient_local = false;
  const uint32_t qos_depth = 10;
  const pid_t pid = 1000;

  union ioctl_add_process_args add_process_args;
  int ret1 = agnocast_ioctl_add_process(
    pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &add_process_args);
  KUNIT_ASSERT_EQ(test, ret1, 0);

  union ioctl_add_publisher_args add_publisher_args;
  int ret2 = agnocast_ioctl_add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, pid, qos_depth, is_transient_local, IS_BRIDGE,
    &add_publisher_args);
  KUNIT_ASSERT_EQ(test, ret2, 0);

  const bool ignore_local_publications = false;
  union ioctl_add_subscriber_args add_subscriber_args;
  int ret3 = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, pid, qos_depth, is_transient_local,
    IS_RELIABLE, IS_TAKE_SUB, ignore_local_publications, IS_BRIDGE, -1, &add_subscriber_args);
  KUNIT_ASSERT_EQ(test, ret3, 0);

  // Publish a message
  union ioctl_publish_msg_args ioctl_publish_msg_ret;
  int ret4 = agnocast_ioctl_publish_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, add_publisher_args.ret_id, add_process_args.ret_addr,
    &ioctl_publish_msg_ret);
  KUNIT_ASSERT_EQ(test, ret4, 0);

  // Act: receive_msg should return the message from the same process
  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};
  int ret5 = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, add_subscriber_args.ret_id, pub_shm_infos,
    KUNIT_PUB_SHM_BUF_SIZE, &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret5, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 1);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_ids[0], ioctl_publish_msg_ret.ret_entry_id);
}

void test_case_receive_msg_bridge_subscriber_in_other_domain_receives_nothing(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns),
    0);
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_publisher_in_domain(test, 1000, 1, IS_BRIDGE, &publisher_id, &ret_addr);
  topic_local_id_t subscriber_id;
  setup_current_subscriber_in_domain(test, 2, true, &subscriber_id);
  union ioctl_publish_msg_args ioctl_publish_msg_ret;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_publish_msg(
      TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret),
    0);
  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 0);
}

void test_case_receive_msg_bridge_publisher_in_other_domain_delivers_nothing(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(TOPIC_NAME, TOPIC_NAME, 1, 2, current->nsproxy->ipc_ns),
    0);
  topic_local_id_t publisher_id;
  uint64_t ret_addr;
  setup_publisher_in_domain(test, 1000, 1, true, &publisher_id, &ret_addr);
  topic_local_id_t subscriber_id;
  setup_current_subscriber_in_domain(test, 2, IS_BRIDGE, &subscriber_id);
  union ioctl_publish_msg_args ioctl_publish_msg_ret;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_publish_msg(
      TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, ret_addr, &ioctl_publish_msg_ret),
    0);
  union ioctl_receive_msg_args ioctl_receive_msg_ret;
  struct publisher_shm_info pub_shm_infos[KUNIT_PUB_SHM_BUF_SIZE] = {0};

  // Act
  int ret = agnocast_ioctl_receive_msg(
    TOPIC_NAME, current->nsproxy->ipc_ns, subscriber_id, pub_shm_infos, KUNIT_PUB_SHM_BUF_SIZE,
    &ioctl_receive_msg_ret);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_entry_num, 0);
  KUNIT_EXPECT_EQ(test, ioctl_receive_msg_ret.ret_pub_shm_num, 0);
}
