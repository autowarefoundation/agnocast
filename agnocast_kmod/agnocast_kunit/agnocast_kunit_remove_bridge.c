#include "agnocast_kunit_remove_bridge.h"

#include "../agnocast.h"

#include <kunit/test.h>

static const char * TOPIC_NAME = "/kunit_test_bridge_remove_topic";
static const pid_t BRIDGE_OWNER_PID = 1000;
static const pid_t OTHER_PID = 1001;

void test_case_remove_bridge_normal(struct kunit * test)
{
  // Arrange
  struct ioctl_add_bridge_args args = {0};
  int ret_setup = add_bridge(TOPIC_NAME, BRIDGE_OWNER_PID, true, current->nsproxy->ipc_ns, &args);
  KUNIT_ASSERT_EQ(test, ret_setup, 0);

  // Act
  int ret = remove_bridge(TOPIC_NAME, BRIDGE_OWNER_PID, true, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_FALSE(test, is_in_bridge_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
}

void test_case_remove_bridge_partial(struct kunit * test)
{
  // Arrange
  struct ioctl_add_bridge_args args = {0};
  add_bridge(TOPIC_NAME, BRIDGE_OWNER_PID, true, current->nsproxy->ipc_ns, &args);
  add_bridge(TOPIC_NAME, BRIDGE_OWNER_PID, false, current->nsproxy->ipc_ns, &args);

  // Act
  int ret1 = remove_bridge(TOPIC_NAME, BRIDGE_OWNER_PID, true, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret1, 0);
  KUNIT_EXPECT_TRUE(test, is_in_bridge_htable(TOPIC_NAME, current->nsproxy->ipc_ns));

  // Act
  int ret2 = remove_bridge(TOPIC_NAME, BRIDGE_OWNER_PID, false, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret2, 0);
  KUNIT_EXPECT_FALSE(test, is_in_bridge_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
}

void test_case_remove_bridge_not_found(struct kunit * test)
{
  // Arrange
  const char * NON_EXISTENT_TOPIC = "/kunit_non_existent_topic";

  // Act
  int ret = remove_bridge(NON_EXISTENT_TOPIC, BRIDGE_OWNER_PID, true, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -ENOENT);
}

void test_case_remove_bridge_pid_mismatch(struct kunit * test)
{
  // Arrange
  struct ioctl_add_bridge_args args = {0};
  int ret_setup = add_bridge(TOPIC_NAME, BRIDGE_OWNER_PID, true, current->nsproxy->ipc_ns, &args);
  KUNIT_ASSERT_EQ(test, ret_setup, 0);

  // Act
  int ret = remove_bridge(TOPIC_NAME, OTHER_PID, true, current->nsproxy->ipc_ns);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EPERM);
  KUNIT_EXPECT_TRUE(test, is_in_bridge_htable(TOPIC_NAME, current->nsproxy->ipc_ns));
  KUNIT_EXPECT_EQ(
    test, get_bridge_owner_pid(TOPIC_NAME, current->nsproxy->ipc_ns), BRIDGE_OWNER_PID);

  // Clean-up
  remove_bridge(TOPIC_NAME, BRIDGE_OWNER_PID, true, current->nsproxy->ipc_ns);
}