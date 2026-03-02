#include "agnocast_kunit_bridge_shutdown.h"

#include "../agnocast.h"

#include <kunit/test.h>

static pid_t pid_bs = 8000;

// Registering with is_bridge_manager=true should succeed and set the flag
// so that subsequent processes see ret_performance_bridge_daemon_exist=true
void test_case_bridge_manager_flag_set_on_registration(struct kunit * test)
{
  // Register bridge manager
  pid_t bridge_pid = pid_bs++;
  union ioctl_add_process_args bridge_args = {};
  int ret = agnocast_ioctl_add_process(bridge_pid, current->nsproxy->ipc_ns, true, &bridge_args);
  KUNIT_EXPECT_EQ(test, ret, 0);

  // Verify the flag was set by checking a new process sees it
  pid_t normal_pid = pid_bs++;
  union ioctl_add_process_args normal_args = {};
  ret = agnocast_ioctl_add_process(normal_pid, current->nsproxy->ipc_ns, false, &normal_args);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_TRUE(test, normal_args.ret_performance_bridge_daemon_exist);
}

// When a bridge manager is already registered, a new process calling add_process
// should receive ret_performance_bridge_daemon_exist=true
void test_case_bridge_manager_detected_by_new_process(struct kunit * test)
{
  // Register bridge manager
  pid_t bridge_pid = pid_bs++;
  union ioctl_add_process_args bridge_args = {};
  int ret = agnocast_ioctl_add_process(bridge_pid, current->nsproxy->ipc_ns, true, &bridge_args);
  KUNIT_EXPECT_EQ(test, ret, 0);

  // Register normal process - should see bridge manager exists
  pid_t normal_pid = pid_bs++;
  union ioctl_add_process_args normal_args = {};
  ret = agnocast_ioctl_add_process(normal_pid, current->nsproxy->ipc_ns, false, &normal_args);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_TRUE(test, normal_args.ret_performance_bridge_daemon_exist);
}

// notify_bridge_shutdown clears is_bridge_manager, so a new process
// should receive ret_performance_bridge_daemon_exist=false
void test_case_notify_bridge_shutdown_clears_flag(struct kunit * test)
{
  // Register bridge manager
  pid_t bridge_pid = pid_bs++;
  union ioctl_add_process_args bridge_args = {};
  int ret = agnocast_ioctl_add_process(bridge_pid, current->nsproxy->ipc_ns, true, &bridge_args);
  KUNIT_EXPECT_EQ(test, ret, 0);

  // Notify shutdown
  agnocast_ioctl_notify_bridge_shutdown(bridge_pid);

  // Register normal process - should see no bridge manager
  pid_t normal_pid = pid_bs++;
  union ioctl_add_process_args normal_args = {};
  ret = agnocast_ioctl_add_process(normal_pid, current->nsproxy->ipc_ns, false, &normal_args);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_FALSE(test, normal_args.ret_performance_bridge_daemon_exist);
}
