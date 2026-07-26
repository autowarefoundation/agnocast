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

// The first agent to claim a (ns, domain) wins and is recorded.
void test_case_discovery_agent_register_first_wins(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_discovery_agent_num(), 0);

  struct ioctl_add_discovery_agent_args reg;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_discovery_agent(pid++, current->nsproxy->ipc_ns, 10, &reg), 0);
  KUNIT_EXPECT_FALSE(test, reg.ret_already_exists);
  KUNIT_EXPECT_EQ(test, agnocast_get_discovery_agent_num(), 1);
}

// A second claim on the same (ns, domain) loses; the registry keeps exactly one agent.
void test_case_discovery_agent_register_duplicate_loses(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_discovery_agent_num(), 0);

  struct ioctl_add_discovery_agent_args first;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_discovery_agent(pid++, current->nsproxy->ipc_ns, 11, &first), 0);
  KUNIT_EXPECT_FALSE(test, first.ret_already_exists);

  struct ioctl_add_discovery_agent_args second;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_discovery_agent(pid++, current->nsproxy->ipc_ns, 11, &second), 0);
  KUNIT_EXPECT_TRUE(test, second.ret_already_exists);
  KUNIT_EXPECT_EQ(test, agnocast_get_discovery_agent_num(), 1);
}

// The fork gate reflects agent registration, not the process count: a live process alone does
// not make the agent "exist"; registering one does.
void test_case_discovery_agent_exist_reflects_registration(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_discovery_agent_num(), 0);

  union ioctl_add_process_args before;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_process(pid++, current->nsproxy->ipc_ns, false, 12, &before), 0);
  KUNIT_EXPECT_FALSE(test, before.ret_discovery_agent_exist);  // process alive, no agent yet

  struct ioctl_add_discovery_agent_args reg;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_discovery_agent(pid++, current->nsproxy->ipc_ns, 12, &reg), 0);

  union ioctl_add_process_args after;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_process(pid++, current->nsproxy->ipc_ns, false, 12, &after), 0);
  KUNIT_EXPECT_TRUE(test, after.ret_discovery_agent_exist);  // now an agent is registered
}

// commit on an empty domain deregisters the agent and tells it to exit.
void test_case_discovery_agent_commit_exit_when_idle(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_discovery_agent_num(), 0);

  const pid_t agent_pid = pid++;
  struct ioctl_add_discovery_agent_args reg;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_discovery_agent(agent_pid, current->nsproxy->ipc_ns, 13, &reg), 0);

  bool should_exit = false;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_discovery_agent_should_exit(
      agent_pid, current->nsproxy->ipc_ns, 13, true, &should_exit),
    0);
  KUNIT_EXPECT_TRUE(test, should_exit);
  KUNIT_EXPECT_EQ(test, agnocast_get_discovery_agent_num(), 0);  // deregistered
}

// commit is vetoed while the domain still has a live process; the agent stays registered.
void test_case_discovery_agent_commit_exit_vetoed_when_busy(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_discovery_agent_num(), 0);

  const pid_t agent_pid = pid++;
  struct ioctl_add_discovery_agent_args reg;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_discovery_agent(agent_pid, current->nsproxy->ipc_ns, 14, &reg), 0);

  union ioctl_add_process_args proc;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_process(pid++, current->nsproxy->ipc_ns, false, 14, &proc), 0);

  bool should_exit = true;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_discovery_agent_should_exit(
      agent_pid, current->nsproxy->ipc_ns, 14, true, &should_exit),
    0);
  KUNIT_EXPECT_FALSE(test, should_exit);
  KUNIT_EXPECT_EQ(test, agnocast_get_discovery_agent_num(), 1);  // retained
}

// The agent has no proc_info, so its liveness must be wired into the exit pipeline: its pid is an
// agnocast pid while registered, and the exit cleanup drains its registry entry.
void test_case_discovery_agent_reaped_on_exit(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_discovery_agent_num(), 0);

  const pid_t agent_pid = pid++;
  struct ioctl_add_discovery_agent_args reg;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_discovery_agent(agent_pid, current->nsproxy->ipc_ns, 15, &reg), 0);
  KUNIT_EXPECT_TRUE(test, is_agnocast_pid(agent_pid));

  agnocast_enqueue_exit_pid(agent_pid);
  msleep(20);  // let exit_worker_thread run agnocast_process_exit_cleanup
  KUNIT_EXPECT_EQ(test, agnocast_get_discovery_agent_num(), 0);
  KUNIT_EXPECT_FALSE(test, is_agnocast_pid(agent_pid));
}

// The orphan race the whole registry exists to close: the agent's final exit (commit) and a new
// process's add_process are serialized, so the domain never ends up with live processes and no
// agent. Here commit wins first, then a new process spawns a replacement that registers cleanly.
void test_case_discovery_agent_orphan_race(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_discovery_agent_num(), 0);

  const pid_t agent_a = pid++;
  struct ioctl_add_discovery_agent_args reg_a;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_discovery_agent(agent_a, current->nsproxy->ipc_ns, 16, &reg_a), 0);

  bool should_exit = false;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_discovery_agent_should_exit(
      agent_a, current->nsproxy->ipc_ns, 16, true, &should_exit),
    0);
  KUNIT_ASSERT_TRUE(test, should_exit);  // idle domain -> A deregisters

  union ioctl_add_process_args p2;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_process(pid++, current->nsproxy->ipc_ns, false, 16, &p2), 0);
  KUNIT_EXPECT_FALSE(test, p2.ret_discovery_agent_exist);  // A is gone -> P2 must spawn one

  const pid_t agent_b = pid++;
  struct ioctl_add_discovery_agent_args reg_b;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_discovery_agent(agent_b, current->nsproxy->ipc_ns, 16, &reg_b), 0);
  KUNIT_EXPECT_FALSE(test, reg_b.ret_already_exists);            // replacement wins
  KUNIT_EXPECT_EQ(test, agnocast_get_discovery_agent_num(), 1);  // exactly one agent, no orphan
}

// The read-only liveness ioctl (backing the CLI status verb) reflects registration per domain.
void test_case_discovery_agent_exists_ioctl(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_discovery_agent_num(), 0);

  bool exists = true;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_discovery_agent_exists(current->nsproxy->ipc_ns, 17, &exists), 0);
  KUNIT_EXPECT_FALSE(test, exists);  // none registered

  struct ioctl_add_discovery_agent_args reg;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_discovery_agent(pid++, current->nsproxy->ipc_ns, 17, &reg), 0);

  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_discovery_agent_exists(current->nsproxy->ipc_ns, 17, &exists), 0);
  KUNIT_EXPECT_TRUE(test, exists);  // registered in domain 17

  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_discovery_agent_exists(current->nsproxy->ipc_ns, 18, &exists), 0);
  KUNIT_EXPECT_FALSE(test, exists);  // a different domain
}

// An exited-but-not-yet-drained process must not veto the commit: it lingers in proc_info_htable
// until the unlink daemon reaps it, but get_alive_process_num_in_domain() skips ->exited entries,
// so the domain reads empty and the agent is allowed to exit.
void test_case_discovery_agent_commit_ignores_exited_process(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_discovery_agent_num(), 0);

  const pid_t agent_pid = pid++;
  struct ioctl_add_discovery_agent_args reg;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_discovery_agent(agent_pid, current->nsproxy->ipc_ns, 19, &reg), 0);

  const pid_t app_pid = pid++;
  union ioctl_add_process_args app;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_process(app_pid, current->nsproxy->ipc_ns, false, 19, &app), 0);

  agnocast_enqueue_exit_pid(app_pid);
  msleep(20);  // let exit_worker_thread mark it exited (still present, not yet drained)
  KUNIT_ASSERT_TRUE(test, agnocast_is_proc_exited(app_pid));

  bool should_exit = false;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_discovery_agent_should_exit(
      agent_pid, current->nsproxy->ipc_ns, 19, true, &should_exit),
    0);
  KUNIT_EXPECT_TRUE(test, should_exit);                          // exited process does not veto
  KUNIT_EXPECT_EQ(test, agnocast_get_discovery_agent_num(), 0);  // agent deregistered
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
