// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_add_process.h"

#include "../agnocast.h"
#include "../agnocast_memory_allocator.h"

#include <kunit/test.h>

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
