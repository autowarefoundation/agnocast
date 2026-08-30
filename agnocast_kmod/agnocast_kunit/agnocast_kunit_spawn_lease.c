// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_spawn_lease.h"

#include "../agnocast.h"

#include <kunit/test.h>

// The fd that owns a lease in production cannot be created from KUnit, so these exercise the
// decision and the bookkeeping directly. What the fd adds -- that dropping the last reference
// releases the lease -- is covered by the kernel module test suite instead.
static pid_t pid = 2000;

// A namespace with no daemon and no lease grants the lease to whoever asks first.
void test_case_spawn_lease_first_acquire_wins(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_spawn_lease_num(), 0);

  struct spawn_lease * lease = NULL;
  int ret = agnocast_ioctl_acquire_spawn_lease(
    current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, AGNOCAST_DOMAIN_ID_NONE, &lease);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_NOT_NULL(test, lease);
  KUNIT_EXPECT_EQ(test, agnocast_get_spawn_lease_num(), 1);

  agnocast_ioctl_release_spawn_lease(lease);
}

// The whole point: while one process is forking a daemon, every other process that starts is told
// not to. This is what turns N forks into one.
void test_case_spawn_lease_refused_while_held(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_spawn_lease_num(), 0);

  struct spawn_lease * first = NULL;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_acquire_spawn_lease(
      current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, AGNOCAST_DOMAIN_ID_NONE, &first),
    0);
  KUNIT_ASSERT_NOT_NULL(test, first);

  struct spawn_lease * second = NULL;
  int ret = agnocast_ioctl_acquire_spawn_lease(
    current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, AGNOCAST_DOMAIN_ID_NONE, &second);

  // Refused, not an error: the caller simply has nothing to do.
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_NULL(test, second);
  KUNIT_EXPECT_EQ(test, agnocast_get_spawn_lease_num(), 1);

  agnocast_ioctl_release_spawn_lease(first);
}

// A child that dies before it registers drops the lease, and the next process to start must then
// be told to spawn a replacement. Releasing is the only thing that has to happen for that -- no
// deadline has to expire first.
void test_case_spawn_lease_granted_again_after_release(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_spawn_lease_num(), 0);

  struct spawn_lease * first = NULL;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_acquire_spawn_lease(
      current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, AGNOCAST_DOMAIN_ID_NONE, &first),
    0);
  KUNIT_ASSERT_NOT_NULL(test, first);
  agnocast_ioctl_release_spawn_lease(first);
  KUNIT_ASSERT_EQ(test, agnocast_get_spawn_lease_num(), 0);

  struct spawn_lease * second = NULL;
  int ret = agnocast_ioctl_acquire_spawn_lease(
    current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, AGNOCAST_DOMAIN_ID_NONE, &second);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_NOT_NULL(test, second);

  agnocast_ioctl_release_spawn_lease(second);
}

// Once a daemon is registered it is the registration, not the lease, that tells a starting process
// a daemon exists -- so a lease is never handed out for a role that is already filled.
void test_case_spawn_lease_refused_when_daemon_registered(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_spawn_lease_num(), 0);

  union ioctl_add_process_args daemon_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, AGNOCAST_DOMAIN_ID_NONE,
      &daemon_args),
    0);

  struct spawn_lease * lease = NULL;
  int ret = agnocast_ioctl_acquire_spawn_lease(
    current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, AGNOCAST_DOMAIN_ID_NONE, &lease);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_NULL(test, lease);
  KUNIT_EXPECT_EQ(test, agnocast_get_spawn_lease_num(), 0);
}

// The daemon spawned under a lease still has to register, so holding the lease must not stand in
// the way of the very registration it exists to protect.
void test_case_spawn_lease_does_not_block_daemon_registration(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_spawn_lease_num(), 0);

  struct spawn_lease * lease = NULL;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_acquire_spawn_lease(
      current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, AGNOCAST_DOMAIN_ID_NONE, &lease),
    0);
  KUNIT_ASSERT_NOT_NULL(test, lease);

  union ioctl_add_process_args daemon_args;
  int ret = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, AGNOCAST_DOMAIN_ID_NONE,
    &daemon_args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_FALSE(test, daemon_args.ret_unlink_daemon_exist);

  agnocast_ioctl_release_spawn_lease(lease);
}

// The bridge manager is per (namespace, domain), like has_alive_bridge_manager(), so a lease held
// for one domain must not stop another domain from starting its own.
void test_case_spawn_lease_bridge_is_per_domain(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_spawn_lease_num(), 0);

  struct spawn_lease * domain0 = NULL;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_acquire_spawn_lease(
      current->nsproxy->ipc_ns, PROCESS_ROLE_BRIDGE_MANAGER, 0, &domain0),
    0);
  KUNIT_ASSERT_NOT_NULL(test, domain0);

  struct spawn_lease * domain1 = NULL;
  KUNIT_EXPECT_EQ(
    test,
    agnocast_ioctl_acquire_spawn_lease(
      current->nsproxy->ipc_ns, PROCESS_ROLE_BRIDGE_MANAGER, 1, &domain1),
    0);
  KUNIT_EXPECT_NOT_NULL(test, domain1);

  struct spawn_lease * domain0_again = NULL;
  KUNIT_EXPECT_EQ(
    test,
    agnocast_ioctl_acquire_spawn_lease(
      current->nsproxy->ipc_ns, PROCESS_ROLE_BRIDGE_MANAGER, 0, &domain0_again),
    0);
  KUNIT_EXPECT_NULL(test, domain0_again);
  KUNIT_EXPECT_EQ(test, agnocast_get_spawn_lease_num(), 2);

  agnocast_ioctl_release_spawn_lease(domain0);
  agnocast_ioctl_release_spawn_lease(domain1);
}

// The unlink daemon is namespace-scoped and belongs to no domain, so callers running under
// different ROS_DOMAIN_IDs must contend for the same lease rather than get one each.
void test_case_spawn_lease_unlink_ignores_domain(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_spawn_lease_num(), 0);

  struct spawn_lease * from_domain0 = NULL;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_acquire_spawn_lease(
      current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, &from_domain0),
    0);
  KUNIT_ASSERT_NOT_NULL(test, from_domain0);

  struct spawn_lease * from_domain7 = NULL;
  KUNIT_EXPECT_EQ(
    test,
    agnocast_ioctl_acquire_spawn_lease(
      current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 7, &from_domain7),
    0);
  KUNIT_EXPECT_NULL(test, from_domain7);
  KUNIT_EXPECT_EQ(test, agnocast_get_spawn_lease_num(), 1);

  agnocast_ioctl_release_spawn_lease(from_domain0);
}

// Only the two daemons agnocastlib forks are leased. The discovery agent claims its own slot
// before exec, and an application never spawns itself.
void test_case_spawn_lease_rejects_unleased_role(struct kunit * test)
{
  struct spawn_lease * lease = NULL;
  int ret = agnocast_ioctl_acquire_spawn_lease(
    current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &lease);

  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
  KUNIT_EXPECT_NULL(test, lease);
  KUNIT_EXPECT_EQ(test, agnocast_get_spawn_lease_num(), 0);
}

// AGNOCAST_DOMAIN_ID_NONE is the key the unlink daemon's lease resolves to, so a bridge manager
// asking for it would contend for a lease that is not its own.
void test_case_spawn_lease_rejects_reserved_domain_for_bridge(struct kunit * test)
{
  struct spawn_lease * lease = NULL;
  int ret = agnocast_ioctl_acquire_spawn_lease(
    current->nsproxy->ipc_ns, PROCESS_ROLE_BRIDGE_MANAGER, AGNOCAST_DOMAIN_ID_NONE, &lease);

  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
  KUNIT_EXPECT_NULL(test, lease);
  KUNIT_EXPECT_EQ(test, agnocast_get_spawn_lease_num(), 0);
}

// Deliberately leaks the lease: spawn_lease_fops pins the module so this cannot happen in
// production, but agnocast_exit_free_data() must still sweep it, and the assertion that every
// other case here starts from an empty table is what proves it did.
void test_case_spawn_lease_held_lease_is_freed_on_module_exit(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_spawn_lease_num(), 0);

  struct spawn_lease * lease = NULL;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_acquire_spawn_lease(
      current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, AGNOCAST_DOMAIN_ID_NONE, &lease),
    0);
  KUNIT_EXPECT_NOT_NULL(test, lease);
  KUNIT_EXPECT_EQ(test, agnocast_get_spawn_lease_num(), 1);
}
