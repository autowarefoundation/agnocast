// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_ipc_ns.h"

#include "../agnocast.h"
#include "../agnocast_internal.h"
#include "agnocast_kunit_helpers.h"

#include <kunit/test.h>

static const pid_t PID = 1000;
static const pid_t OTHER_PID = 1001;
static const pid_t DAEMON_PID = 1002;
static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * NODE_NAME = "/kunit_test_node";

// Only ever used as a key, never dereferenced.
static struct ipc_namespace other_ipc_ns;

static void setup_process_in(
  struct kunit * test, const struct ipc_namespace * ipc_ns, const pid_t pid,
  const enum process_role role)
{
  union ioctl_add_process_args add_process_args;
  KUNIT_ASSERT_EQ(test, agnocast_ioctl_add_process(pid, ipc_ns, role, 0, &add_process_args), 0);
}

static topic_local_id_t setup_publisher_in(
  struct kunit * test, const struct ipc_namespace * ipc_ns, const pid_t pid)
{
  setup_process_in(test, ipc_ns, pid, PROCESS_ROLE_APPLICATION);

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

// The usual end of a namespace: its unlink daemon drains the exited process, then leaves.
void test_case_ipc_ns_retired_once_unlink_daemon_leaves(struct kunit * test)
{
  // Arrange
  setup_process_in(test, &other_ipc_ns, DAEMON_PID, PROCESS_ROLE_UNLINK_DAEMON);
  setup_publisher_in(test, &other_ipc_ns, OTHER_PID);
  agnocast_process_exit_cleanup(OTHER_PID);
  // The live unlink daemon keeps the namespace, so it can drain the exited process.
  KUNIT_ASSERT_NOT_NULL(test, agnocast_ns_peek(&other_ipc_ns));
  struct ioctl_get_exit_process_args get_exit_process_args = {};
  const pid_t global_pid = agnocast_ioctl_get_exit_process(&other_ipc_ns, &get_exit_process_args);
  KUNIT_ASSERT_EQ(test, global_pid, OTHER_PID);
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(&other_ipc_ns, global_pid, -1, &daemon_should_exit);
  KUNIT_ASSERT_TRUE(test, daemon_should_exit);
  KUNIT_ASSERT_NOT_NULL(test, agnocast_ns_peek(&other_ipc_ns));

  // Act: the daemon's idle poll deregisters it.
  daemon_should_exit = false;
  agnocast_commit_exit_process(&other_ipc_ns, -1, DAEMON_PID, &daemon_should_exit);

  // Assert
  KUNIT_EXPECT_TRUE(test, daemon_should_exit);
  KUNIT_EXPECT_NULL(test, agnocast_ns_peek(&other_ipc_ns));
}

// A namespace killed as a whole: nothing will drain the exited process, so the namespace must not
// wait for it.
void test_case_ipc_ns_retired_when_unlink_daemon_dies(struct kunit * test)
{
  // Arrange
  setup_process_in(test, &other_ipc_ns, DAEMON_PID, PROCESS_ROLE_UNLINK_DAEMON);
  setup_publisher_in(test, &other_ipc_ns, OTHER_PID);
  agnocast_process_exit_cleanup(OTHER_PID);
  KUNIT_ASSERT_NOT_NULL(test, agnocast_ns_peek(&other_ipc_ns));

  // Act
  agnocast_process_exit_cleanup(DAEMON_PID);

  // Assert: the namespace is gone; the undrained exited process stays, as it always did.
  KUNIT_EXPECT_NULL(test, agnocast_ns_peek(&other_ipc_ns));
  KUNIT_EXPECT_TRUE(test, agnocast_is_proc_exited(OTHER_PID));
}

void test_case_ipc_ns_exited_entry_drained_after_retirement(struct kunit * test)
{
  // Arrange
  setup_publisher_in(test, &other_ipc_ns, OTHER_PID);
  agnocast_process_exit_cleanup(OTHER_PID);
  KUNIT_ASSERT_NULL(test, agnocast_ns_peek(&other_ipc_ns));

  // Act
  struct ioctl_get_exit_process_args get_exit_process_args = {};
  const pid_t global_pid = agnocast_ioctl_get_exit_process(&other_ipc_ns, &get_exit_process_args);
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(&other_ipc_ns, global_pid, -1, &daemon_should_exit);

  // Assert
  KUNIT_EXPECT_EQ(test, global_pid, OTHER_PID);
  KUNIT_EXPECT_FALSE(test, is_agnocast_pid(OTHER_PID));
  KUNIT_EXPECT_TRUE(test, daemon_should_exit);
  KUNIT_EXPECT_NULL(test, agnocast_ns_peek(&other_ipc_ns));
}

// A discovery agent in one namespace reuses the pid of an exited, undrained process left in
// another. Its exit must still remove the agent from its own namespace.
void test_case_ipc_ns_agent_reusing_stale_pid_is_removed(struct kunit * test)
{
  // Arrange
  setup_publisher_in(test, current->nsproxy->ipc_ns, PID);
  agnocast_process_exit_cleanup(PID);
  KUNIT_ASSERT_TRUE(test, agnocast_is_proc_exited(PID));
  struct ioctl_add_discovery_agent_args add_agent_args = {};
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_discovery_agent(PID, &other_ipc_ns, 0, &add_agent_args), 0);
  KUNIT_ASSERT_TRUE(test, add_agent_args.ret_owned_by_caller);

  // Act
  agnocast_process_exit_cleanup(PID);

  // Assert
  bool exists = true;
  KUNIT_EXPECT_EQ(test, agnocast_ioctl_discovery_agent_exists(&other_ipc_ns, 0, &exists), 0);
  KUNIT_EXPECT_FALSE(test, exists);
  KUNIT_EXPECT_EQ(test, agnocast_get_discovery_agent_num(), 0);
  KUNIT_EXPECT_NULL(test, agnocast_ns_peek(&other_ipc_ns));
}
