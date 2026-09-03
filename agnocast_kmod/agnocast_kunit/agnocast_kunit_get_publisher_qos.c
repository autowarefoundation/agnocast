// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_get_publisher_qos.h"

#include "../agnocast.h"
#include "agnocast_kunit_helpers.h"

#include <kunit/test.h>

static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * NODE_NAME = "/kunit_test_node";
static const pid_t PUBLISHER_PID = 1000;
static const uint32_t QOS_DEPTH = 10;
static const bool IS_BRIDGE = false;

static void verify_publisher_qos(struct kunit * test, bool is_transient)
{
  union ioctl_add_publisher_args add_pub_args;
  struct ioctl_get_publisher_qos_args get_qos_args;
  int ret;

  agnocast_kunit_setup_process(test, PUBLISHER_PID, 0);

  ret = agnocast_ioctl_add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, PUBLISHER_PID, QOS_DEPTH, is_transient,
    IS_BRIDGE, &add_pub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  ret = agnocast_ioctl_get_publisher_qos(
    TOPIC_NAME, current->nsproxy->ipc_ns, add_pub_args.ret_id, &get_qos_args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ_MSG(test, get_qos_args.ret_depth, QOS_DEPTH, "Depth mismatch");

  KUNIT_EXPECT_EQ_MSG(
    test, (bool)get_qos_args.ret_is_transient_local, is_transient, "Transient Local mismatch");
}

void test_case_qos_volatile(struct kunit * test)
{
  verify_publisher_qos(test, false);
}

void test_case_qos_transient(struct kunit * test)
{
  verify_publisher_qos(test, true);
}

void test_case_pub_error_topic_not_found(struct kunit * test)
{
  struct ioctl_get_publisher_qos_args get_qos_args;
  topic_local_id_t dummy_id;
  int ret;

  agnocast_kunit_setup_process(test, PUBLISHER_PID, 0);

  dummy_id = 0;

  ret = agnocast_ioctl_get_publisher_qos(
    "/non_existent_topic", current->nsproxy->ipc_ns, dummy_id, &get_qos_args);

  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_error_publisher_not_found(struct kunit * test)
{
  union ioctl_add_publisher_args add_pub_args;
  struct ioctl_get_publisher_qos_args get_qos_args;
  int ret;

  agnocast_kunit_setup_process(test, PUBLISHER_PID, 0);

  ret = agnocast_ioctl_add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, PUBLISHER_PID, QOS_DEPTH, false, IS_BRIDGE,
    &add_pub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  topic_local_id_t invalid_id = add_pub_args.ret_id + 999;

  ret = agnocast_ioctl_get_publisher_qos(
    TOPIC_NAME, current->nsproxy->ipc_ns, invalid_id, &get_qos_args);

  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}
