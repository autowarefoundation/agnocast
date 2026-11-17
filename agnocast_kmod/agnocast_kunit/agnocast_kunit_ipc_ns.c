// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_ipc_ns.h"

#include "../agnocast.h"
#include "../agnocast_internal.h"
#include "agnocast_kunit_helpers.h"

#include <kunit/test.h>

static const pid_t PID = 1000;
static const pid_t OTHER_PID = 1001;
static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * NODE_NAME = "/kunit_test_node";

// Only ever used as a key, never dereferenced.
static struct ipc_namespace other_ipc_ns;

static topic_local_id_t setup_publisher_in(
  struct kunit * test, const struct ipc_namespace * ipc_ns, const pid_t pid)
{
  union ioctl_add_process_args add_process_args;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_process(pid, ipc_ns, PROCESS_ROLE_APPLICATION, 0, &add_process_args),
    0);

  union ioctl_add_publisher_args add_publisher_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_publisher(
      TOPIC_NAME, ipc_ns, NODE_NAME, pid, 1, false, false, &add_publisher_args),
    0);
  return add_publisher_args.ret_id;
}

void test_case_ipc_ns_same_topic_name_is_separate(struct kunit * test)
{
  // Arrange
  setup_publisher_in(test, current->nsproxy->ipc_ns, PID);
  const topic_local_id_t other_id = setup_publisher_in(test, &other_ipc_ns, OTHER_PID);

  // Act
  const int ret = agnocast_ioctl_remove_publisher(TOPIC_NAME, &other_ipc_ns, other_id);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, agnocast_get_topic_num(&other_ipc_ns), 0);
  KUNIT_EXPECT_EQ(test, agnocast_get_topic_num(current->nsproxy->ipc_ns), 1);
}

void test_case_ipc_ns_exit_cleans_up_only_own_namespace(struct kunit * test)
{
  // Arrange
  setup_publisher_in(test, current->nsproxy->ipc_ns, PID);
  setup_publisher_in(test, &other_ipc_ns, OTHER_PID);

  // Act
  agnocast_process_exit_cleanup(OTHER_PID);

  // Assert
  KUNIT_EXPECT_TRUE(test, agnocast_is_proc_exited(OTHER_PID));
  KUNIT_EXPECT_EQ(test, agnocast_get_topic_num(&other_ipc_ns), 0);
  KUNIT_EXPECT_FALSE(test, agnocast_is_proc_exited(PID));
  KUNIT_EXPECT_EQ(test, agnocast_get_topic_num(current->nsproxy->ipc_ns), 1);
}

// The point of the per-namespace lock: one namespace registering endpoints must not hold up
// another's publish/receive.
void test_case_ipc_ns_write_lock_leaves_other_namespace_free(struct kunit * test)
{
  // Arrange
  setup_publisher_in(test, current->nsproxy->ipc_ns, PID);
  setup_publisher_in(test, &other_ipc_ns, OTHER_PID);
  struct agnocast_ns * ns = agnocast_ns_write_lock(current->nsproxy->ipc_ns);
  KUNIT_ASSERT_NOT_NULL(test, ns);
  struct agnocast_ns * other_ns = agnocast_ns_peek(&other_ipc_ns);
  KUNIT_ASSERT_NOT_NULL(test, other_ns);

  // Act
  const bool other_free = down_read_trylock(&other_ns->rwsem);

  // Assert
  KUNIT_EXPECT_PTR_NE(test, ns, other_ns);
  KUNIT_EXPECT_TRUE(test, other_free);

  if (other_free) up_read(&other_ns->rwsem);
  agnocast_ns_write_unlock(ns);
}

void test_case_ipc_ns_retired_once_empty(struct kunit * test)
{
  // Arrange
  setup_publisher_in(test, &other_ipc_ns, OTHER_PID);
  // The exited process stays registered until its namespace's unlink daemon drains it.
  agnocast_process_exit_cleanup(OTHER_PID);
  KUNIT_ASSERT_NOT_NULL(test, agnocast_ns_peek(&other_ipc_ns));
  struct ioctl_get_exit_process_args get_exit_process_args = {};
  const pid_t global_pid = agnocast_ioctl_get_exit_process(&other_ipc_ns, &get_exit_process_args);
  KUNIT_ASSERT_EQ(test, global_pid, OTHER_PID);

  // Act
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(&other_ipc_ns, global_pid, -1, &daemon_should_exit);

  // Assert
  KUNIT_EXPECT_TRUE(test, daemon_should_exit);
  KUNIT_EXPECT_NULL(test, agnocast_ns_peek(&other_ipc_ns));
}
