// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_get_exit_process.h"

#include "../agnocast.h"

#include <kunit/test.h>

static const pid_t PID = 1000;

// get_exit_process_cmd tolerates a failed ret_daemon_should_exit copy_to_user because the flag is
// re-derived on the next poll. That fallback only works if agnocast_commit_exit_process() derives
// the flag unconditionally, including on an idle poll where Phase 1 found nothing and passes
// global_pid == -1. These cases pin that down so gating the derivation behind global_pid >= 0
// cannot silently strand the daemon.

static void setup_one_process(struct kunit * test, const pid_t pid)
{
  union ioctl_add_process_args ioctl_ret;
  int ret = agnocast_ioctl_add_process(pid, current->nsproxy->ipc_ns, false, 0, &ioctl_ret);
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
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, global_pid, &daemon_should_exit);

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
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, global_pid, &daemon_should_exit);

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
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, global_pid, &daemon_should_exit);

  struct ioctl_get_exit_process_args next_args = {};
  const pid_t next_global_pid =
    agnocast_ioctl_get_exit_process(current->nsproxy->ipc_ns, &next_args);

  bool next_daemon_should_exit = false;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, next_global_pid, &next_daemon_should_exit);

  // Assert: the flag is still derived on the idle poll that follows the commit.
  KUNIT_EXPECT_EQ(test, global_pid, PID);
  KUNIT_EXPECT_TRUE(test, daemon_should_exit);
  KUNIT_EXPECT_EQ(test, next_global_pid, -1);
  KUNIT_EXPECT_TRUE(test, next_daemon_should_exit);
}

// Phase 1 breaks after the first match, so several exited processes are drained one per poll. This
// is what poll_for_unlink()'s inner do-while loop relies on: it keeps calling the ioctl until
// ret_pid stops being positive, unlinking one shm per iteration.
void test_case_get_exit_process_drains_one_per_poll(struct kunit * test)
{
  // Arrange
  const int process_num = 3;
  for (int i = 0; i < process_num; i++) {
    setup_one_process(test, PID + i);
    agnocast_process_exit_cleanup(PID + i);
  }

  // Act
  pid_t drained[3] = {-1, -1, -1};
  bool daemon_should_exit[3] = {true, true, true};
  pid_t drained_ret_pid[3] = {-1, -1, -1};
  for (int i = 0; i < process_num; i++) {
    struct ioctl_get_exit_process_args args = {};
    drained[i] = agnocast_ioctl_get_exit_process(current->nsproxy->ipc_ns, &args);
    drained_ret_pid[i] = args.ret_pid;

    agnocast_commit_exit_process(current->nsproxy->ipc_ns, drained[i], &daemon_should_exit[i]);
  }

  struct ioctl_get_exit_process_args final_args = {};
  const pid_t final_global_pid =
    agnocast_ioctl_get_exit_process(current->nsproxy->ipc_ns, &final_args);

  // Assert: every pid is drained exactly once, and only the last commit empties the namespace.
  bool seen[3] = {false, false, false};
  for (int i = 0; i < process_num; i++) {
    KUNIT_ASSERT_GE(test, drained[i], PID);
    KUNIT_ASSERT_LT(test, drained[i], PID + process_num);
    KUNIT_EXPECT_FALSE(test, seen[drained[i] - PID]);
    seen[drained[i] - PID] = true;

    // ret_pid is what user-space unlinks; under KUNIT_BUILD local_pid == global_pid.
    KUNIT_EXPECT_EQ(test, drained_ret_pid[i], drained[i]);
    KUNIT_EXPECT_EQ(test, daemon_should_exit[i], i == process_num - 1);
  }

  KUNIT_EXPECT_EQ(test, final_global_pid, -1);
  KUNIT_EXPECT_EQ(test, final_args.ret_pid, -1);
}

// Phase 2 looks the pid up again under the write lock rather than trusting the caller, so a pid
// that is no longer in the htable must be tolerated instead of dereferenced.
void test_case_get_exit_process_commit_unknown_pid(struct kunit * test)
{
  // Arrange
  const pid_t unknown_pid = PID + 999;

  // Act
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, unknown_pid, &daemon_should_exit);

  // Assert
  KUNIT_EXPECT_TRUE(test, daemon_should_exit);
}

void test_case_get_exit_process_commit_unknown_pid_last_alive(struct kunit * test)
{
  // Arrange
  setup_one_process(test, PID);
  const pid_t unknown_pid = PID + 999;

  // Act
  bool daemon_should_exit = true;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, unknown_pid, &daemon_should_exit);

  struct ioctl_get_exit_process_args args = {};
  const pid_t global_pid = agnocast_ioctl_get_exit_process(current->nsproxy->ipc_ns, &args);

  // Assert: the live process is not dropped by a commit for an unrelated pid.
  KUNIT_EXPECT_FALSE(test, daemon_should_exit);
  KUNIT_EXPECT_EQ(test, global_pid, -1);
}
