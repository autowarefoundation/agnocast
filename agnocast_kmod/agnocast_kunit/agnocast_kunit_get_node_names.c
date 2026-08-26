// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_get_node_names.h"

#include "../agnocast.h"

#include <kunit/test.h>

static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * TOPIC_NAME2 = "/kunit_test_topic2";
static const char * NODE_NAME = "/kunit_test_node";
static const char * NODE_NAME2 = "/kunit_test_node2";
static const pid_t PID = 1000;
static const pid_t PID2 = 2000;
static const uint32_t QOS_DEPTH = 1;
static const uint32_t DOMAIN_ID = 1;
static const uint32_t OTHER_DOMAIN_ID = 2;

static void setup_process(struct kunit * test, const pid_t pid, const uint32_t domain_id)
{
  union ioctl_add_process_args add_process_args;
  int ret =
    agnocast_ioctl_add_process(pid, current->nsproxy->ipc_ns, false, domain_id, &add_process_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

static void add_publisher(
  struct kunit * test, const char * topic_name, const char * node_name, const pid_t pid,
  const bool is_bridge)
{
  union ioctl_add_publisher_args add_pub_args;
  int ret = agnocast_ioctl_add_publisher(
    topic_name, current->nsproxy->ipc_ns, node_name, pid, QOS_DEPTH, false, is_bridge,
    &add_pub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

static void add_subscriber(
  struct kunit * test, const char * topic_name, const char * node_name, const pid_t pid,
  const bool is_bridge)
{
  union ioctl_add_subscriber_args add_sub_args;
  int ret = agnocast_ioctl_add_subscriber(
    topic_name, current->nsproxy->ipc_ns, node_name, pid, QOS_DEPTH, false, true, false, false,
    is_bridge, -1, &add_sub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

// Returns how many of the `num` names in `buf` equal `name`.
static uint32_t count_name(const char * buf, const uint32_t num, const char * name)
{
  uint32_t count = 0;
  uint32_t i;

  for (i = 0; i < num; i++) {
    if (strcmp(&buf[i * NODE_NAME_BUFFER_SIZE], name) == 0) count++;
  }

  return count;
}

void test_case_get_node_names_no_node(struct kunit * test)
{
  char buf[1][NODE_NAME_BUFFER_SIZE];
  uint32_t node_num = UINT_MAX;

  // Act
  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, (char *)buf, ARRAY_SIZE(buf), &node_num);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 0);
}

void test_case_get_node_names_multiple_nodes(struct kunit * test)
{
  char buf[2][NODE_NAME_BUFFER_SIZE];
  uint32_t node_num = 0;

  // Arrange
  setup_process(test, PID, DOMAIN_ID);
  add_publisher(test, TOPIC_NAME, NODE_NAME, PID, false);
  add_subscriber(test, TOPIC_NAME2, NODE_NAME2, PID, false);

  // Act
  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, (char *)buf, ARRAY_SIZE(buf), &node_num);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 2);
  KUNIT_EXPECT_EQ(test, count_name((const char *)buf, node_num, NODE_NAME), 1);
  KUNIT_EXPECT_EQ(test, count_name((const char *)buf, node_num, NODE_NAME2), 1);
}

// A node owning several endpoints is reported exactly once.
void test_case_get_node_names_deduplicates(struct kunit * test)
{
  char buf[2][NODE_NAME_BUFFER_SIZE];
  uint32_t node_num = 0;

  // Arrange
  setup_process(test, PID, DOMAIN_ID);
  add_publisher(test, TOPIC_NAME, NODE_NAME, PID, false);
  add_publisher(test, TOPIC_NAME2, NODE_NAME, PID, false);
  add_subscriber(test, TOPIC_NAME, NODE_NAME, PID, false);

  // Act
  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, (char *)buf, ARRAY_SIZE(buf), &node_num);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 1);
  KUNIT_EXPECT_STREQ(test, buf[0], NODE_NAME);
}

// Two processes running a node of the same name are two nodes, matching what rclcpp reports for
// the DDS graph. Only the dedup within one process may collapse them.
void test_case_get_node_names_same_name_in_two_processes(struct kunit * test)
{
  char buf[2][NODE_NAME_BUFFER_SIZE];
  uint32_t node_num = 0;

  // Arrange
  setup_process(test, PID, DOMAIN_ID);
  setup_process(test, PID2, DOMAIN_ID);
  add_publisher(test, TOPIC_NAME, NODE_NAME, PID, false);
  add_publisher(test, TOPIC_NAME, NODE_NAME, PID2, false);

  // Act
  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, (char *)buf, ARRAY_SIZE(buf), &node_num);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 2);
  KUNIT_EXPECT_EQ(test, count_name((const char *)buf, node_num, NODE_NAME), 2);
}

// Endpoints created by a bridge do not introduce a node of their own.
void test_case_get_node_names_excludes_bridge_publisher(struct kunit * test)
{
  char buf[2][NODE_NAME_BUFFER_SIZE];
  uint32_t node_num = 0;

  // Arrange
  setup_process(test, PID, DOMAIN_ID);
  add_subscriber(test, TOPIC_NAME, NODE_NAME, PID, false);
  add_publisher(test, TOPIC_NAME, NODE_NAME2, PID, true);

  // Act
  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, (char *)buf, ARRAY_SIZE(buf), &node_num);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 1);
  KUNIT_EXPECT_STREQ(test, buf[0], NODE_NAME);
}

void test_case_get_node_names_excludes_bridge_subscriber(struct kunit * test)
{
  char buf[2][NODE_NAME_BUFFER_SIZE];
  uint32_t node_num = 0;

  // Arrange
  setup_process(test, PID, DOMAIN_ID);
  add_publisher(test, TOPIC_NAME, NODE_NAME, PID, false);
  add_subscriber(test, TOPIC_NAME, NODE_NAME2, PID, true);

  // Act
  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, (char *)buf, ARRAY_SIZE(buf), &node_num);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 1);
  KUNIT_EXPECT_STREQ(test, buf[0], NODE_NAME);
}

// Nodes belonging to another ROS_DOMAIN_ID are not visible.
void test_case_get_node_names_other_domain(struct kunit * test)
{
  char buf[2][NODE_NAME_BUFFER_SIZE];
  uint32_t node_num = 0;

  // Arrange
  setup_process(test, PID, DOMAIN_ID);
  add_publisher(test, TOPIC_NAME, NODE_NAME, PID, false);

  // Act
  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, OTHER_DOMAIN_ID, (char *)buf, ARRAY_SIZE(buf), &node_num);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 0);
}

// A domain bridge puts both domains' endpoints in one shared topic_struct, so filtering on the
// wrapper alone would report the partner domain's nodes too.
void test_case_get_node_names_excludes_the_bridged_domain(struct kunit * test)
{
  char buf[2][NODE_NAME_BUFFER_SIZE];
  uint32_t node_num = 0;

  // Arrange
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_domain_bridge(
      TOPIC_NAME, TOPIC_NAME, DOMAIN_ID, OTHER_DOMAIN_ID, current->nsproxy->ipc_ns),
    0);
  setup_process(test, PID, DOMAIN_ID);
  setup_process(test, PID2, OTHER_DOMAIN_ID);
  add_publisher(test, TOPIC_NAME, NODE_NAME, PID, false);
  add_publisher(test, TOPIC_NAME, NODE_NAME2, PID2, false);

  // Act
  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, (char *)buf, ARRAY_SIZE(buf), &node_num);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 1);
  KUNIT_EXPECT_STREQ(test, buf[0], NODE_NAME);
}

// More nodes than the caller's buffer holds is an error, not a truncated graph.
void test_case_get_node_names_buffer_too_small(struct kunit * test)
{
  char buf[1][NODE_NAME_BUFFER_SIZE];
  uint32_t node_num = 0;

  // Arrange
  setup_process(test, PID, DOMAIN_ID);
  add_subscriber(test, TOPIC_NAME, NODE_NAME, PID, false);
  add_subscriber(test, TOPIC_NAME, NODE_NAME2, PID, false);

  // Act
  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, (char *)buf, ARRAY_SIZE(buf), &node_num);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -ENOBUFS);
}
