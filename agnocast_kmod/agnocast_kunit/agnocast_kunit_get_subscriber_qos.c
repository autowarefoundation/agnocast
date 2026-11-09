// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_get_subscriber_qos.h"

#include "../agnocast.h"
#include "agnocast_kunit_helpers.h"

#include <kunit/test.h>

static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * NODE_NAME = "/kunit_test_node";
static const pid_t SUBSCRIBER_PID = 1000;
static const uint32_t QOS_DEPTH = 10;
static const bool IS_BRIDGE = false;

static void verify_subscriber_qos(struct kunit * test, bool is_transient, bool is_reliable)
{
  struct ioctl_get_subscriber_qos_args get_qos_args;
  int ret;

  agnocast_kunit_setup_process(test, SUBSCRIBER_PID, 0);
  topic_local_id_t sub_id = agnocast_kunit_setup_subscriber(
    test, TOPIC_NAME, NODE_NAME, SUBSCRIBER_PID, QOS_DEPTH, is_transient, is_reliable, false, false,
    IS_BRIDGE, -1);

  ret =
    agnocast_ioctl_get_subscriber_qos(TOPIC_NAME, current->nsproxy->ipc_ns, sub_id, &get_qos_args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ_MSG(test, get_qos_args.ret_depth, QOS_DEPTH, "Depth mismatch");

  KUNIT_EXPECT_EQ_MSG(
    test, (bool)get_qos_args.ret_is_transient_local, is_transient, "Transient Local mismatch");
  KUNIT_EXPECT_EQ_MSG(test, (bool)get_qos_args.ret_is_reliable, is_reliable, "Reliable mismatch");
}

void test_case_qos_volatile_best_effort(struct kunit * test)
{
  verify_subscriber_qos(test, false, false);
}

void test_case_qos_volatile_reliable(struct kunit * test)
{
  verify_subscriber_qos(test, false, true);
}

void test_case_qos_transient_best_effort(struct kunit * test)
{
  verify_subscriber_qos(test, true, false);
}

void test_case_qos_transient_reliable(struct kunit * test)
{
  verify_subscriber_qos(test, true, true);
}

void test_case_sub_error_topic_not_found(struct kunit * test)
{
  struct ioctl_get_subscriber_qos_args get_qos_args;
  topic_local_id_t dummy_id;
  int ret;

  agnocast_kunit_setup_process(test, SUBSCRIBER_PID, 0);

  dummy_id = 0;

  ret = agnocast_ioctl_get_subscriber_qos(
    "/non_existent_topic", current->nsproxy->ipc_ns, dummy_id, &get_qos_args);

  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_error_subscriber_not_found(struct kunit * test)
{
  struct ioctl_get_subscriber_qos_args get_qos_args;
  int ret;

  agnocast_kunit_setup_process(test, SUBSCRIBER_PID, 0);
  topic_local_id_t sub_id = agnocast_kunit_setup_subscriber(
    test, TOPIC_NAME, NODE_NAME, SUBSCRIBER_PID, QOS_DEPTH, false, false, false, false, IS_BRIDGE,
    -1);

  topic_local_id_t invalid_id = sub_id + 999;

  ret = agnocast_ioctl_get_subscriber_qos(
    TOPIC_NAME, current->nsproxy->ipc_ns, invalid_id, &get_qos_args);

  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

// Refused rather than narrowed: agnocastlib turns the failure into an RCLCPP_ERROR and exits, so
// the depth the subscriber asked for is never silently different from the one it gets.
void test_case_qos_depth_above_max_is_rejected(struct kunit * test)
{
  union ioctl_add_subscriber_args add_sub_args;

  setup_process(test, SUBSCRIBER_PID);

  int ret = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, SUBSCRIBER_PID, MAX_QOS_DEPTH + 1, false, true,
    false, false, IS_BRIDGE, -1, &add_sub_args);

  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

// The bound itself is honored as asked, so the check cannot be off by one.
void test_case_qos_depth_at_max_is_kept(struct kunit * test)
{
  union ioctl_add_subscriber_args add_sub_args;
  struct ioctl_get_subscriber_qos_args get_qos_args;
  int ret;

  setup_process(test, SUBSCRIBER_PID);

  ret = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, SUBSCRIBER_PID, MAX_QOS_DEPTH, false, true,
    false, false, IS_BRIDGE, -1, &add_sub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  ret = agnocast_ioctl_get_subscriber_qos(
    TOPIC_NAME, current->nsproxy->ipc_ns, add_sub_args.ret_id, &get_qos_args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, get_qos_args.ret_depth, MAX_QOS_DEPTH);
}
