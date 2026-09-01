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
  int ret = agnocast_ioctl_add_process(
    local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &args);

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
  for (int i = 0; i < mempool_num; i++) {
    KUNIT_EXPECT_FALSE(test, agnocast_is_proc_exited(local_pid_start + i));
  }
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
  KUNIT_EXPECT_FALSE(test, agnocast_is_proc_exited(local_pid));
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

// ret_unlink_daemon_exist reports a registered daemon, not a non-empty namespace.
void test_case_add_process_unlink_daemon_registration(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);
  union ioctl_add_process_args app_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &app_args),
    0);
  KUNIT_ASSERT_FALSE(test, app_args.ret_unlink_daemon_exist);

  // Act
  union ioctl_add_process_args daemon_args;
  int ret = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, &daemon_args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_FALSE(test, daemon_args.ret_unlink_daemon_exist);
  union ioctl_add_process_args later_args;
  KUNIT_EXPECT_EQ(
    test,
    agnocast_ioctl_add_process(
      pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &later_args),
    0);
  KUNIT_EXPECT_TRUE(test, later_args.ret_unlink_daemon_exist);
}

// Two processes starting at once can both decide to spawn a daemon; only one may register.
void test_case_add_process_unlink_daemon_duplicate_refused(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);
  union ioctl_add_process_args first_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, &first_args),
    0);
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 1);

  // Act
  union ioctl_add_process_args second_args;
  int ret = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, &second_args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_TRUE(test, second_args.ret_unlink_daemon_exist);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 1);
}

// A dead daemon is observable, so the exited entries it left behind must not stand in for it.
void test_case_add_process_unlink_daemon_respawns_after_death(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);
  const pid_t app_pid = pid++;
  union ioctl_add_process_args app_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      app_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &app_args),
    0);
  const pid_t daemon_pid = pid++;
  union ioctl_add_process_args daemon_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      daemon_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, &daemon_args),
    0);
  agnocast_process_exit_cleanup(app_pid);
  agnocast_process_exit_cleanup(daemon_pid);
  KUNIT_ASSERT_TRUE(test, agnocast_is_proc_exited(app_pid));

  // Act
  union ioctl_add_process_args next_args;
  int ret = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &next_args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_FALSE(test, next_args.ret_unlink_daemon_exist);
}

// The daemon is what drains the table, so on its own it has to be told the namespace is done.
void test_case_add_process_unlink_daemon_is_not_counted_as_work(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);
  union ioctl_add_process_args daemon_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, &daemon_args),
    0);

  // Act
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, -1, -1, &daemon_should_exit);

  // Assert
  KUNIT_EXPECT_TRUE(test, daemon_should_exit);
}

// An application's entry is left pending for the daemon to drain; the daemon's own is not.
void test_case_add_process_unlink_daemon_removed_on_death(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);
  const pid_t app_pid = pid++;
  union ioctl_add_process_args app_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      app_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &app_args),
    0);
  const pid_t daemon_pid = pid++;
  union ioctl_add_process_args daemon_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      daemon_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, &daemon_args),
    0);

  // Act
  agnocast_process_exit_cleanup(app_pid);
  agnocast_process_exit_cleanup(daemon_pid);

  // Assert
  struct ioctl_get_exit_process_args exit_args = {};
  KUNIT_EXPECT_EQ(
    test, agnocast_ioctl_get_exit_process(current->nsproxy->ipc_ns, &exit_args), app_pid);
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, app_pid, -1, &daemon_should_exit);
  KUNIT_EXPECT_EQ(test, agnocast_ioctl_get_exit_process(current->nsproxy->ipc_ns, &exit_args), -1);
}

// poll_for_unlink()'s drain loop discards the flag from a commit that returned a pid, so
// deregistering there would leave the daemon polling on as an unregistered ghost.
void test_case_add_process_unlink_daemon_stays_registered_while_draining(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);
  const pid_t app_pid = pid++;
  union ioctl_add_process_args app_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      app_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &app_args),
    0);
  const pid_t daemon_pid = pid++;
  union ioctl_add_process_args daemon_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      daemon_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, &daemon_args),
    0);
  agnocast_process_exit_cleanup(app_pid);

  // Act
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, app_pid, daemon_pid, &daemon_should_exit);

  // Assert
  KUNIT_EXPECT_FALSE(test, daemon_should_exit);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 1);
  union ioctl_add_process_args next_args;
  KUNIT_EXPECT_EQ(
    test,
    agnocast_ioctl_add_process(
      pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &next_args),
    0);
  KUNIT_EXPECT_TRUE(test, next_args.ret_unlink_daemon_exist);
}

// A process starting between the exit decision and the daemon's death spawns a replacement.
void test_case_add_process_unlink_daemon_deregisters_when_told_to_exit(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);
  const pid_t daemon_pid = pid++;
  union ioctl_add_process_args daemon_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      daemon_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, &daemon_args),
    0);

  // Act
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, -1, daemon_pid, &daemon_should_exit);

  // Assert
  KUNIT_EXPECT_TRUE(test, daemon_should_exit);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 0);
  // The daemon process is still running at this point.
  union ioctl_add_process_args next_args;
  KUNIT_EXPECT_EQ(
    test,
    agnocast_ioctl_add_process(
      pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &next_args),
    0);
  KUNIT_EXPECT_FALSE(test, next_args.ret_unlink_daemon_exist);
}

// Deregistering the daemon early must release its mempool slot too, which nothing else will:
// its exit no longer reaches agnocast_process_exit_cleanup().
void test_case_add_process_unlink_daemon_deregistration_frees_its_mempool_slot(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);
  const pid_t daemon_pid = pid++;
  union ioctl_add_process_args daemon_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      daemon_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, &daemon_args),
    0);
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, -1, daemon_pid, &daemon_should_exit);
  KUNIT_ASSERT_TRUE(test, daemon_should_exit);

  // Act
  int ret = 0;
  for (int i = 0; i < mempool_num; i++) {
    union ioctl_add_process_args args;
    ret = agnocast_ioctl_add_process(
      pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &args);
    if (ret != 0) break;
  }

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), mempool_num);
}

void test_case_add_process_rejects_unknown_role(struct kunit * test)
{
  // Arrange
  union ioctl_add_process_args args;

  // Act
  int ret = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, (enum process_role)(PROCESS_ROLE_UNLINK_DAEMON + 1), 0, &args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 0);
}

void test_case_add_process_rejects_the_daemon_domain_id(struct kunit * test)
{
  // Arrange
  union ioctl_add_process_args args;

  // Act
  int ret = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, AGNOCAST_DOMAIN_ID_NONE, &args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 0);
}

// The daemon's domain_id is assigned by role, so whatever it sends is accepted and ignored.
void test_case_add_process_daemon_may_send_the_daemon_domain_id(struct kunit * test)
{
  // Arrange
  union ioctl_add_process_args args;

  // Act
  int ret = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, AGNOCAST_DOMAIN_ID_NONE, &args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 1);
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
  KUNIT_EXPECT_FALSE(test, agnocast_is_proc_exited(local_pid));
}
