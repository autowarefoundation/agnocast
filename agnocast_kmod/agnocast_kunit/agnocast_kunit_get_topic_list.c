// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_get_topic_list.h"

#include "../agnocast.h"

#include <kunit/test.h>

static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * TOPIC_NAME2 = "/kunit_test_topic2";
static const char * SRV_REQUEST_TOPIC_NAME = "/AGNOCAST_SRV_REQUEST/kunit_test_service";
static const char * SRV_RESPONSE_TOPIC_NAME =
  "/AGNOCAST_SRV_RESPONSE/kunit_test_service_SEP_/kunit_test_node_SEP_0";
static const char * NODE_NAME = "/kunit_test_node";
static const pid_t PID = 1000;
static const pid_t PID2 = 2000;
static const uint32_t QOS_DEPTH = 1;
static const uint32_t DOMAIN_ID = 1;
static const uint32_t OTHER_DOMAIN_ID = 2;

static void setup_process(struct kunit * test, const pid_t pid, const uint32_t domain_id)
{
  union ioctl_add_process_args add_process_args;
  int ret = agnocast_ioctl_add_process(
    pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, domain_id, &add_process_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

static void add_subscriber(struct kunit * test, const char * topic_name, const pid_t pid)
{
  union ioctl_add_subscriber_args add_sub_args;
  int ret = agnocast_ioctl_add_subscriber(
    topic_name, current->nsproxy->ipc_ns, NODE_NAME, pid, QOS_DEPTH, false, true, false, false,
    false, -1, &add_sub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

// Returns the index of `name` in `buf`, or `num` when it is absent.
static uint32_t index_of(const char * buf, const uint32_t num, const char * name)
{
  uint32_t i;

  for (i = 0; i < num; i++) {
    if (strcmp(&buf[i * TOPIC_NAME_BUFFER_SIZE], name) == 0) return i;
  }

  return num;
}

void test_case_get_topic_list_no_topic(struct kunit * test)
{
  char buf[1][TOPIC_NAME_BUFFER_SIZE];
  uint32_t domain_ids[1];
  uint32_t topic_num = UINT_MAX;

  // Act
  int ret = agnocast_ioctl_get_topic_list(
    current->nsproxy->ipc_ns, (char *)buf, domain_ids, ARRAY_SIZE(buf), &topic_num);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, topic_num, 0);
}

// No room for a single topic, so a response topic that reached the buffer bound at all would fail
// with -ENOBUFS, whichever one the walk happens to reach first.
void test_case_get_topic_list_skips_service_response_topic(struct kunit * test)
{
  char buf[1][TOPIC_NAME_BUFFER_SIZE];
  uint32_t domain_ids[1];
  uint32_t topic_num = UINT_MAX;

  // Arrange
  setup_process(test, PID, DOMAIN_ID);
  add_subscriber(test, SRV_RESPONSE_TOPIC_NAME, PID);

  // Act
  int ret =
    agnocast_ioctl_get_topic_list(current->nsproxy->ipc_ns, (char *)buf, domain_ids, 0, &topic_num);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, topic_num, 0);
}

// A comparison shorter than the whole response prefix would skip this topic too.
void test_case_get_topic_list_lists_service_request_topic(struct kunit * test)
{
  char buf[1][TOPIC_NAME_BUFFER_SIZE];
  uint32_t domain_ids[1];
  uint32_t topic_num = 0;

  // Arrange
  setup_process(test, PID, DOMAIN_ID);
  add_subscriber(test, SRV_REQUEST_TOPIC_NAME, PID);

  // Act
  int ret = agnocast_ioctl_get_topic_list(
    current->nsproxy->ipc_ns, (char *)buf, domain_ids, ARRAY_SIZE(buf), &topic_num);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, topic_num, 1);
  KUNIT_EXPECT_STREQ(test, buf[0], SRV_REQUEST_TOPIC_NAME);
}

// One list covers every domain in the namespace, so a name only means something paired with the
// domain id at the same index.
void test_case_get_topic_list_pairs_each_topic_with_its_domain_id(struct kunit * test)
{
  char buf[2][TOPIC_NAME_BUFFER_SIZE];
  uint32_t domain_ids[2] = {UINT_MAX, UINT_MAX};
  uint32_t topic_num = 0;
  uint32_t i;
  uint32_t j;

  // Arrange
  setup_process(test, PID, DOMAIN_ID);
  setup_process(test, PID2, OTHER_DOMAIN_ID);
  add_subscriber(test, TOPIC_NAME, PID);
  add_subscriber(test, TOPIC_NAME2, PID2);

  // Act
  int ret = agnocast_ioctl_get_topic_list(
    current->nsproxy->ipc_ns, (char *)buf, domain_ids, ARRAY_SIZE(buf), &topic_num);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_ASSERT_EQ(test, topic_num, 2);
  i = index_of((const char *)buf, topic_num, TOPIC_NAME);
  j = index_of((const char *)buf, topic_num, TOPIC_NAME2);
  KUNIT_ASSERT_LT(test, i, topic_num);
  KUNIT_ASSERT_LT(test, j, topic_num);
  KUNIT_EXPECT_EQ(test, domain_ids[i], DOMAIN_ID);
  KUNIT_EXPECT_EQ(test, domain_ids[j], OTHER_DOMAIN_ID);
}

void test_case_get_topic_list_fails_when_buffer_is_full(struct kunit * test)
{
  char buf[1][TOPIC_NAME_BUFFER_SIZE];
  uint32_t domain_ids[1];
  uint32_t topic_num = UINT_MAX;

  // Arrange
  setup_process(test, PID, DOMAIN_ID);
  add_subscriber(test, TOPIC_NAME, PID);
  add_subscriber(test, TOPIC_NAME2, PID);

  // Act
  int ret = agnocast_ioctl_get_topic_list(
    current->nsproxy->ipc_ns, (char *)buf, domain_ids, ARRAY_SIZE(buf), &topic_num);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -ENOBUFS);
  KUNIT_EXPECT_EQ(test, topic_num, UINT_MAX);
}
