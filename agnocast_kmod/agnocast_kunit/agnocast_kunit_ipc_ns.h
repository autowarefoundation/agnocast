/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_IPC_NS                                               \
  KUNIT_CASE(test_case_ipc_ns_same_topic_name_is_separate),             \
    KUNIT_CASE(test_case_ipc_ns_exit_cleans_up_only_own_namespace),     \
    KUNIT_CASE(test_case_ipc_ns_retired_once_unlink_daemon_leaves),     \
    KUNIT_CASE(test_case_ipc_ns_retired_when_unlink_daemon_dies),       \
    KUNIT_CASE(test_case_ipc_ns_exited_entry_drained_after_retirement), \
    KUNIT_CASE(test_case_ipc_ns_agent_reusing_stale_pid_is_removed)

void test_case_ipc_ns_same_topic_name_is_separate(struct kunit * test);
void test_case_ipc_ns_exit_cleans_up_only_own_namespace(struct kunit * test);
void test_case_ipc_ns_retired_once_unlink_daemon_leaves(struct kunit * test);
void test_case_ipc_ns_retired_when_unlink_daemon_dies(struct kunit * test);
void test_case_ipc_ns_exited_entry_drained_after_retirement(struct kunit * test);
void test_case_ipc_ns_agent_reusing_stale_pid_is_removed(struct kunit * test);
