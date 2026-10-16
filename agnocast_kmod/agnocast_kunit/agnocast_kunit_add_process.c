// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_add_process.h"

#include "../agnocast_internal.h"

#include <kunit/test.h>
#include <linux/delay.h>

static pid_t pid = 1000;
void test_case_add_process_normal(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  uint64_t local_pid = pid++;
  union ioctl_add_process_args args;
  int ret = agnocast_ioctl_add_process(
    local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 1);
}

void test_case_add_process_many(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  // ================================================
  // Act

  for (int i = 0; i < mempool_num - 1; i++) {
    uint64_t local_pid = pid++;
    union ioctl_add_process_args args;
    agnocast_ioctl_add_process(
      local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &args);
  }

  uint64_t local_pid = pid++;
  union ioctl_add_process_args args;
  int ret = agnocast_ioctl_add_process(
    local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &args);

  // ================================================
  // Assert

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), mempool_num);
}

void test_case_add_process_twice(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  pid_t local_pid = pid++;
  union ioctl_add_process_args args;
  int ret1 = agnocast_ioctl_add_process(
    local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &args);
  int ret2 = agnocast_ioctl_add_process(
    local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &args);

  KUNIT_EXPECT_EQ(test, ret1, 0);
  KUNIT_EXPECT_EQ(test, ret2, -EINVAL);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 1);
}

// A bridge manager is gated per-(ipc_ns, domain): a manager in one
// domain must not suppress spawning a manager in another domain, while a second
// manager in the same domain is suppressed.
void test_case_add_process_bridge_manager_per_domain(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  union ioctl_add_process_args args_d0;
  int ret_d0 = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_BRIDGE_MANAGER, 0, &args_d0);
  KUNIT_EXPECT_EQ(test, ret_d0, 0);
  KUNIT_EXPECT_FALSE(test, args_d0.ret_bridge_daemon_exist);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 1);

  // Different domain: not suppressed, so it is added and sees no existing manager.
  union ioctl_add_process_args args_d1;
  int ret_d1 = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_BRIDGE_MANAGER, 1, &args_d1);
  KUNIT_EXPECT_EQ(test, ret_d1, 0);
  KUNIT_EXPECT_FALSE(test, args_d1.ret_bridge_daemon_exist);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 2);

  // Same domain as the first: a manager already exists, so it is suppressed.
  union ioctl_add_process_args args_d0_again;
  int ret_d0_again = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_BRIDGE_MANAGER, 0, &args_d0_again);
  KUNIT_EXPECT_EQ(test, ret_d0_again, 0);
  KUNIT_EXPECT_TRUE(test, args_d0_again.ret_bridge_daemon_exist);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 2);
}

void test_case_add_process_rejects_unknown_role(struct kunit * test)
{
  // Arrange
  union ioctl_add_process_args args;

  // Act
  int ret = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, (enum process_role)(PROCESS_ROLE_BRIDGE_MANAGER + 1), 0,
    &args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 0);
}

void test_case_add_process_too_many(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  // ================================================
  // Act

  for (int i = 0; i < mempool_num; i++) {
    uint64_t local_pid = pid++;
    union ioctl_add_process_args args;
    agnocast_ioctl_add_process(
      local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &args);
  }
  uint64_t local_pid = pid++;
  union ioctl_add_process_args args;
  int ret = agnocast_ioctl_add_process(
    local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &args);

  // ================================================
  // Assert

  KUNIT_EXPECT_EQ(test, ret, -ENOMEM);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), mempool_num);
}

void test_case_register_shm_invalid_fd(struct kunit * test)
{
  pid_t local_pid = pid++;
  union ioctl_add_process_args args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &args),
    0);

  KUNIT_EXPECT_EQ(test, agnocast_ioctl_register_process_shm(local_pid, -1), -EBADF);
}

void test_case_register_shm_normal(struct kunit * test)
{
  pid_t local_pid = pid++;
  union ioctl_add_process_args args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &args),
    0);

  KUNIT_ASSERT_EQ(test, agnocast_register_publisher_memf(local_pid, NULL), 0);
}

void test_case_register_shm_duplicate(struct kunit * test)
{
  // Arrange
  pid_t local_pid = pid++;
  union ioctl_add_process_args args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &args),
    0);

  struct file * original_memf;
  KUNIT_ASSERT_EQ(test, agnocast_register_publisher_memf(local_pid, &original_memf), 0);

  // Act & Assert:
  //
  // 1. -EEXIST should be returned for duplicate registration
  // 2. A depulicate attempt should not change the already-registered memory file
  KUNIT_EXPECT_EQ(test, agnocast_register_publisher_memf(local_pid, NULL), -EEXIST);

  // KUnit is executed in single-threaded context; skip locking.
  struct process_info * proc_info = agnocast_find_process_info(local_pid);
  struct file * memf = proc_info ? proc_info->mempool_entry->memf : NULL;
  KUNIT_EXPECT_PTR_EQ(test, memf, original_memf);
}
