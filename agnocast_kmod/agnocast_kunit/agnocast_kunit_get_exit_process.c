// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_get_exit_process.h"

#include "../agnocast.h"

#include <kunit/test.h>

static const pid_t PID = 1000;

// get_exit_process_cmd tolerates a failed ret_daemon_should_exit copy_to_user because the flag is
// re-derived on the next poll. That fallback only works if the idle poll, where Phase 1 found
// nothing and passes global_pid == -1, is the one that derives it. These cases pin that down so
// neither gating the derivation behind global_pid >= 0 nor dropping the idle poll's own
// derivation can silently strand the daemon.

static void setup_one_process(struct kunit * test, const pid_t pid)
{
  union ioctl_add_process_args ioctl_ret;
  int ret = agnocast_ioctl_add_process(
    pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &ioctl_ret);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

void test_case_get_exit_process_idle_poll_empty_namespace(struct kunit * test)
{
  // Arrange
  struct ioctl_get_exit_process_args get_exit_process_args = {};

  // Act
  const pid_t global_pid =
    agnocast_ioctl_get_exit_process(current->nsproxy->ipc_ns, &get_exit_process_args);

  bool daemon_should_exit = false;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, global_pid, -1, &daemon_should_exit);

  // Assert
  KUNIT_EXPECT_EQ(test, global_pid, -1);
  KUNIT_EXPECT_EQ(test, get_exit_process_args.ret_pid, -1);
  KUNIT_EXPECT_TRUE(test, daemon_should_exit);
}

void test_case_get_exit_process_idle_poll_process_remains(struct kunit * test)
{
  // Arrange
  setup_one_process(test, PID);
  struct ioctl_get_exit_process_args get_exit_process_args = {};

  // Act
  const pid_t global_pid =
    agnocast_ioctl_get_exit_process(current->nsproxy->ipc_ns, &get_exit_process_args);

  bool daemon_should_exit = true;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, global_pid, -1, &daemon_should_exit);

  // Assert
  KUNIT_EXPECT_EQ(test, global_pid, -1);
  KUNIT_EXPECT_EQ(test, get_exit_process_args.ret_pid, -1);
  KUNIT_EXPECT_FALSE(test, daemon_should_exit);
}

void test_case_get_exit_process_commit_last_process(struct kunit * test)
{
  // Arrange
  setup_one_process(test, PID);
  agnocast_process_exit_cleanup(PID);

  // Act
  struct ioctl_get_exit_process_args get_exit_process_args = {};
  const pid_t global_pid =
    agnocast_ioctl_get_exit_process(current->nsproxy->ipc_ns, &get_exit_process_args);

  bool daemon_should_exit = false;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, global_pid, -1, &daemon_should_exit);

  struct ioctl_get_exit_process_args next_args = {};
  const pid_t next_global_pid =
    agnocast_ioctl_get_exit_process(current->nsproxy->ipc_ns, &next_args);

  bool next_daemon_should_exit = false;
  agnocast_commit_exit_process(
    current->nsproxy->ipc_ns, next_global_pid, -1, &next_daemon_should_exit);

  // Assert: the commit that returned a pid says nothing; the idle poll that follows does.
  KUNIT_EXPECT_EQ(test, global_pid, PID);
  KUNIT_EXPECT_FALSE(test, daemon_should_exit);
  KUNIT_EXPECT_EQ(test, next_global_pid, -1);
  KUNIT_EXPECT_TRUE(test, next_daemon_should_exit);
}
