// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_check_and_request_bridge_shutdown.h"

#include "../agnocast.h"

#include <kunit/test.h>

static pid_t pid_carbs = 9000;

// When only the bridge manager exists, check_and_request_bridge_shutdown
// should return ret_should_shutdown=true and clear is_performance_bridge_manager
void test_case_check_and_request_bridge_shutdown_when_alone(struct kunit * test)
{
  // Register bridge manager only
  pid_t bridge_pid = pid_carbs++;
  union ioctl_add_process_args bridge_args = {};
  int ret = agnocast_ioctl_add_process(bridge_pid, current->nsproxy->ipc_ns, true, &bridge_args);
  KUNIT_EXPECT_EQ(test, ret, 0);

  // Check shutdown - only bridge manager exists (process_num == 1)
  struct ioctl_check_and_request_bridge_shutdown_args shutdown_args = {};
  ret = agnocast_ioctl_check_and_request_bridge_shutdown(
    bridge_pid, current->nsproxy->ipc_ns, &shutdown_args);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_TRUE(test, shutdown_args.ret_should_shutdown);

  // Verify is_performance_bridge_manager was cleared - new process should not see bridge manager
  pid_t normal_pid = pid_carbs++;
  union ioctl_add_process_args normal_args = {};
  ret = agnocast_ioctl_add_process(normal_pid, current->nsproxy->ipc_ns, false, &normal_args);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_FALSE(test, normal_args.ret_performance_bridge_daemon_exist);
}

// When other processes exist, check_and_request_bridge_shutdown
// should return ret_should_shutdown=false and keep is_performance_bridge_manager set
void test_case_check_and_request_bridge_shutdown_when_others_exist(struct kunit * test)
{
  // Register bridge manager
  pid_t bridge_pid = pid_carbs++;
  union ioctl_add_process_args bridge_args = {};
  int ret = agnocast_ioctl_add_process(bridge_pid, current->nsproxy->ipc_ns, true, &bridge_args);
  KUNIT_EXPECT_EQ(test, ret, 0);

  // Register another process
  pid_t other_pid = pid_carbs++;
  union ioctl_add_process_args other_args = {};
  ret = agnocast_ioctl_add_process(other_pid, current->nsproxy->ipc_ns, false, &other_args);
  KUNIT_EXPECT_EQ(test, ret, 0);

  // Check shutdown - other process exists (process_num > 1)
  struct ioctl_check_and_request_bridge_shutdown_args shutdown_args = {};
  ret = agnocast_ioctl_check_and_request_bridge_shutdown(
    bridge_pid, current->nsproxy->ipc_ns, &shutdown_args);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_FALSE(test, shutdown_args.ret_should_shutdown);

  // Verify is_performance_bridge_manager is still set - new process should see bridge manager
  pid_t new_pid = pid_carbs++;
  union ioctl_add_process_args new_args = {};
  ret = agnocast_ioctl_add_process(new_pid, current->nsproxy->ipc_ns, false, &new_args);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_TRUE(test, new_args.ret_performance_bridge_daemon_exist);
}
