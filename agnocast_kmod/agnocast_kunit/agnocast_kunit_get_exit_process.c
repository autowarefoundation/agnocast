// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_get_exit_process.h"

#include "../agnocast.h"
#include "agnocast_kunit_helpers.h"

#include <kunit/test.h>

static const pid_t PID = 1000;

// get_exit_process_cmd tolerates a failed ret_daemon_should_exit copy_to_user because the flag is
// re-derived on the next poll. That fallback only works if agnocast_commit_exit_process() derives
// the flag unconditionally, including on an idle poll where Phase 1 found nothing and passes
// global_pid == -1. These cases pin that down so gating the derivation behind global_pid >= 0
// cannot silently strand the daemon.

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
  agnocast_kunit_setup_process(test, PID, 0);
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
  agnocast_kunit_setup_process(test, PID, 0);
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

  // Assert: the flag is still derived on the idle poll that follows the commit.
  KUNIT_EXPECT_EQ(test, global_pid, PID);
  KUNIT_EXPECT_TRUE(test, daemon_should_exit);
  KUNIT_EXPECT_EQ(test, next_global_pid, -1);
  KUNIT_EXPECT_TRUE(test, next_daemon_should_exit);
}
