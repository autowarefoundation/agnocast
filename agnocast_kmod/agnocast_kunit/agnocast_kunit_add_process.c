// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_add_process.h"

#include "../agnocast.h"
#include "../agnocast_memory_allocator.h"

#include <kunit/test.h>
#include <linux/delay.h>

static pid_t pid = 1000;
void test_case_add_process_normal(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  uint64_t local_pid = pid++;
  union ioctl_add_process_args args;
  int ret = agnocast_ioctl_add_process(local_pid, current->nsproxy->ipc_ns, false, 0, &args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 1);
  KUNIT_EXPECT_FALSE(test, agnocast_is_proc_exited(local_pid));
}

void test_case_add_process_many(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  // ================================================
  // Act

  pid_t local_pid_start = pid;
  for (int i = 0; i < mempool_num - 1; i++) {
    uint64_t local_pid = pid++;
    union ioctl_add_process_args args;
    agnocast_ioctl_add_process(local_pid, current->nsproxy->ipc_ns, false, 0, &args);
  }

  uint64_t local_pid = pid++;
  union ioctl_add_process_args args;
  int ret = agnocast_ioctl_add_process(local_pid, current->nsproxy->ipc_ns, false, 0, &args);

  // ================================================
  // Assert

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), mempool_num);
  for (int i = 0; i < mempool_num; i++) {
    KUNIT_EXPECT_FALSE(test, agnocast_is_proc_exited(local_pid_start + i));
  }
}

void test_case_add_process_twice(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  pid_t local_pid = pid++;
  union ioctl_add_process_args args;
  int ret1 = agnocast_ioctl_add_process(local_pid, current->nsproxy->ipc_ns, false, 0, &args);
  int ret2 = agnocast_ioctl_add_process(local_pid, current->nsproxy->ipc_ns, false, 0, &args);

  KUNIT_EXPECT_EQ(test, ret1, 0);
  KUNIT_EXPECT_EQ(test, ret2, -EINVAL);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 1);
  KUNIT_EXPECT_FALSE(test, agnocast_is_proc_exited(local_pid));
}

// A performance bridge manager is gated per-(ipc_ns, domain): a manager in one
// domain must not suppress spawning a manager in another domain, while a second
// manager in the same domain is suppressed.
void test_case_add_process_perf_manager_per_domain(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  union ioctl_add_process_args args_d0;
  int ret_d0 = agnocast_ioctl_add_process(pid++, current->nsproxy->ipc_ns, true, 0, &args_d0);
  KUNIT_EXPECT_EQ(test, ret_d0, 0);
  KUNIT_EXPECT_FALSE(test, args_d0.ret_performance_bridge_daemon_exist);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 1);

  // Different domain: not suppressed, so it is added and sees no existing manager.
  union ioctl_add_process_args args_d1;
  int ret_d1 = agnocast_ioctl_add_process(pid++, current->nsproxy->ipc_ns, true, 1, &args_d1);
  KUNIT_EXPECT_EQ(test, ret_d1, 0);
  KUNIT_EXPECT_FALSE(test, args_d1.ret_performance_bridge_daemon_exist);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 2);

  // Same domain as the first: a manager already exists, so it is suppressed.
  union ioctl_add_process_args args_d0_again;
  int ret_d0_again =
    agnocast_ioctl_add_process(pid++, current->nsproxy->ipc_ns, true, 0, &args_d0_again);
  KUNIT_EXPECT_EQ(test, ret_d0_again, 0);
  KUNIT_EXPECT_TRUE(test, args_d0_again.ret_performance_bridge_daemon_exist);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 2);
}

// Spawn gate is per-(ipc_ns, domain): first process sees none, a later one sees
// it, a first process in another domain sees none again.
void test_case_add_process_discovery_agent_per_domain(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  union ioctl_add_process_args first_d0;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_process(pid++, current->nsproxy->ipc_ns, false, 0, &first_d0), 0);
  KUNIT_EXPECT_FALSE(test, first_d0.ret_discovery_agent_exist);

  union ioctl_add_process_args second_d0;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_process(pid++, current->nsproxy->ipc_ns, false, 0, &second_d0), 0);
  KUNIT_EXPECT_TRUE(test, second_d0.ret_discovery_agent_exist);

  union ioctl_add_process_args first_d1;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_process(pid++, current->nsproxy->ipc_ns, false, 1, &first_d1), 0);
  KUNIT_EXPECT_FALSE(test, first_d1.ret_discovery_agent_exist);
}

// The agent self-exit query is true only when its (ipc_ns, domain) has no process.
void test_case_discovery_agent_should_exit(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  bool should_exit = false;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_discovery_agent_should_exit(current->nsproxy->ipc_ns, 7, &should_exit), 0);
  KUNIT_EXPECT_TRUE(test, should_exit);  // empty domain

  union ioctl_add_process_args args;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_process(pid++, current->nsproxy->ipc_ns, false, 7, &args), 0);

  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_discovery_agent_should_exit(current->nsproxy->ipc_ns, 7, &should_exit), 0);
  KUNIT_EXPECT_FALSE(test, should_exit);  // domain 7 now has a process

  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_discovery_agent_should_exit(current->nsproxy->ipc_ns, 8, &should_exit), 0);
  KUNIT_EXPECT_TRUE(test, should_exit);  // a different, empty domain
}

// A domain whose only process has exited but is not yet drained must be treated as empty:
// the exited entry lingers in proc_info_htable until the unlink daemon reaps it, and it must
// neither keep the discovery agent alive nor block a fresh agent from spawning.
void test_case_discovery_agent_ignores_exited_process(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  const pid_t exited_pid = pid++;
  union ioctl_add_process_args args;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_process(exited_pid, current->nsproxy->ipc_ns, false, 9, &args), 0);
  KUNIT_EXPECT_FALSE(test, args.ret_discovery_agent_exist);  // first live process in domain 9

  agnocast_enqueue_exit_pid(exited_pid);
  msleep(10);  // let exit_worker_thread mark the process exited
  KUNIT_ASSERT_TRUE(test, agnocast_is_proc_exited(exited_pid));

  bool should_exit = false;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_discovery_agent_should_exit(current->nsproxy->ipc_ns, 9, &should_exit), 0);
  KUNIT_EXPECT_TRUE(test, should_exit);  // exited-but-not-drained must not keep the agent alive

  union ioctl_add_process_args next_args;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_process(pid++, current->nsproxy->ipc_ns, false, 9, &next_args), 0);
  KUNIT_EXPECT_FALSE(
    test, next_args.ret_discovery_agent_exist);  // exited predecessor is not counted
}

void test_case_add_process_too_many(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  // ================================================
  // Act

  for (int i = 0; i < mempool_num; i++) {
    uint64_t local_pid = pid++;
    union ioctl_add_process_args args;
    agnocast_ioctl_add_process(local_pid, current->nsproxy->ipc_ns, false, 0, &args);
  }
  uint64_t local_pid = pid++;
  union ioctl_add_process_args args;
  int ret = agnocast_ioctl_add_process(local_pid, current->nsproxy->ipc_ns, false, 0, &args);

  // ================================================
  // Assert

  KUNIT_EXPECT_EQ(test, ret, -ENOMEM);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), mempool_num);
  KUNIT_EXPECT_FALSE(test, agnocast_is_proc_exited(local_pid));
}
