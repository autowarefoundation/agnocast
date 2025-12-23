#include "agnocast_kunit_add_bridge.h"

#include "../agnocast.h"

#include <kunit/test.h>

static const char * TOPIC_NAME = "/kunit_test_bridge_topic";
static const pid_t BRIDGE_OWNER_PID = 1000;
static const pid_t OTHER_PID = 1001;

void test_case_add_bridge_normal(struct kunit * test)
{
  // Arrange
  struct ioctl_add_bridge_args args = {0};

  int ret = add_bridge(TOPIC_NAME, BRIDGE_OWNER_PID, true, current->nsproxy->ipc_ns, &args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_TRUE(test, is_in_bridge_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
  KUNIT_EXPECT_EQ(test, args.ret_pid, BRIDGE_OWNER_PID);
  KUNIT_EXPECT_TRUE(test, args.ret_has_r2a);
  KUNIT_EXPECT_FALSE(test, args.ret_has_a2r);
}

void test_case_add_bridge_update_flags(struct kunit * test)
{
  struct ioctl_add_bridge_args args = {0};
  int ret = add_bridge(TOPIC_NAME, BRIDGE_OWNER_PID, true, current->nsproxy->ipc_ns, &args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_EXPECT_TRUE(test, args.ret_has_r2a);
  KUNIT_EXPECT_FALSE(test, args.ret_has_a2r);

  int ret1 = add_bridge(TOPIC_NAME, BRIDGE_OWNER_PID, false, current->nsproxy->ipc_ns, &args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret1, 0);
  KUNIT_EXPECT_EQ(test, args.ret_pid, BRIDGE_OWNER_PID);
  KUNIT_EXPECT_TRUE(test, args.ret_has_r2a);
  KUNIT_EXPECT_TRUE(test, args.ret_has_a2r);
}

void test_case_add_bridge_already_exists_diff_pid(struct kunit * test)
{
  struct ioctl_add_bridge_args args = {0};
  int ret = add_bridge(TOPIC_NAME, BRIDGE_OWNER_PID, true, current->nsproxy->ipc_ns, &args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  memset(&args, 0, sizeof(args));
  int ret1 = add_bridge(TOPIC_NAME, OTHER_PID, false, current->nsproxy->ipc_ns, &args);
  // Assert
  KUNIT_EXPECT_EQ(test, ret1, -EEXIST);
  KUNIT_EXPECT_EQ(test, args.ret_pid, BRIDGE_OWNER_PID);
  KUNIT_EXPECT_TRUE(test, args.ret_has_r2a);
  KUNIT_EXPECT_FALSE(test, args.ret_has_a2r);
}
