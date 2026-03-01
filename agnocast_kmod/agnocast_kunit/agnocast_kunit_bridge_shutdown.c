#include "agnocast_kunit_bridge_shutdown.h"

#include "../agnocast.h"

#include <kunit/test.h>

static pid_t pid_bs = 8000;

// is_bridge_manager=true で登録すると ret_performance_bridge_daemon_exist が反映される
void test_case_bridge_manager_flag_set_on_registration(struct kunit * test)
{
  pid_t bridge_pid = pid_bs++;
  union ioctl_add_process_args args = {};
  int ret = agnocast_ioctl_add_process(bridge_pid, current->nsproxy->ipc_ns, true, &args);

  KUNIT_EXPECT_EQ(test, ret, 0);
}

// bridge_manager が登録済みの状態で新プロセスが add_process すると
// ret_performance_bridge_daemon_exist=true が返る
void test_case_bridge_manager_detected_by_new_process(struct kunit * test)
{
  // Register bridge manager
  pid_t bridge_pid = pid_bs++;
  union ioctl_add_process_args bridge_args = {};
  agnocast_ioctl_add_process(bridge_pid, current->nsproxy->ipc_ns, true, &bridge_args);

  // Register normal process - should see bridge manager exists
  pid_t normal_pid = pid_bs++;
  union ioctl_add_process_args normal_args = {};
  int ret = agnocast_ioctl_add_process(normal_pid, current->nsproxy->ipc_ns, false, &normal_args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_TRUE(test, normal_args.ret_performance_bridge_daemon_exist);
}

// notify_bridge_shutdown で is_bridge_manager がクリアされ、
// 新プロセスが ret_performance_bridge_daemon_exist=false を受け取る
void test_case_notify_bridge_shutdown_clears_flag(struct kunit * test)
{
  // Register bridge manager
  pid_t bridge_pid = pid_bs++;
  union ioctl_add_process_args bridge_args = {};
  agnocast_ioctl_add_process(bridge_pid, current->nsproxy->ipc_ns, true, &bridge_args);

  // Notify shutdown
  agnocast_ioctl_notify_bridge_shutdown(bridge_pid);

  // Register normal process - should see no bridge manager
  pid_t normal_pid = pid_bs++;
  union ioctl_add_process_args normal_args = {};
  agnocast_ioctl_add_process(normal_pid, current->nsproxy->ipc_ns, false, &normal_args);

  KUNIT_EXPECT_FALSE(test, normal_args.ret_performance_bridge_daemon_exist);
}

// bridge_manager のみの状態で check_and_request_bridge_shutdown →
// ret_should_shutdown=true かつ is_bridge_manager がクリアされる
void test_case_check_and_request_bridge_shutdown_when_alone(struct kunit * test)
{
  // Register bridge manager only
  pid_t bridge_pid = pid_bs++;
  union ioctl_add_process_args bridge_args = {};
  agnocast_ioctl_add_process(bridge_pid, current->nsproxy->ipc_ns, true, &bridge_args);

  // Check shutdown - only bridge manager exists (process_num == 1)
  struct ioctl_check_and_request_bridge_shutdown_args shutdown_args = {};
  int ret = agnocast_ioctl_check_and_request_bridge_shutdown(
    bridge_pid, current->nsproxy->ipc_ns, &shutdown_args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_TRUE(test, shutdown_args.ret_should_shutdown);

  // Verify is_bridge_manager was cleared - new process should not see bridge manager
  pid_t normal_pid = pid_bs++;
  union ioctl_add_process_args normal_args = {};
  agnocast_ioctl_add_process(normal_pid, current->nsproxy->ipc_ns, false, &normal_args);

  KUNIT_EXPECT_FALSE(test, normal_args.ret_performance_bridge_daemon_exist);
}

// 他プロセスが存在する状態で check_and_request_bridge_shutdown →
// ret_should_shutdown=false かつ is_bridge_manager が残る
void test_case_check_and_request_bridge_shutdown_when_others_exist(struct kunit * test)
{
  // Register bridge manager
  pid_t bridge_pid = pid_bs++;
  union ioctl_add_process_args bridge_args = {};
  agnocast_ioctl_add_process(bridge_pid, current->nsproxy->ipc_ns, true, &bridge_args);

  // Register another process
  pid_t other_pid = pid_bs++;
  union ioctl_add_process_args other_args = {};
  agnocast_ioctl_add_process(other_pid, current->nsproxy->ipc_ns, false, &other_args);

  // Check shutdown - other process exists (process_num > 1)
  struct ioctl_check_and_request_bridge_shutdown_args shutdown_args = {};
  int ret = agnocast_ioctl_check_and_request_bridge_shutdown(
    bridge_pid, current->nsproxy->ipc_ns, &shutdown_args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_FALSE(test, shutdown_args.ret_should_shutdown);

  // Verify is_bridge_manager is still set - new process should see bridge manager
  pid_t new_pid = pid_bs++;
  union ioctl_add_process_args new_args = {};
  agnocast_ioctl_add_process(new_pid, current->nsproxy->ipc_ns, false, &new_args);

  KUNIT_EXPECT_TRUE(test, new_args.ret_performance_bridge_daemon_exist);
}
