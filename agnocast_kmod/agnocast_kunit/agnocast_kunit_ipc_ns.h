/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_IPC_NS                                                \
  KUNIT_CASE(test_case_ipc_ns_same_topic_name_is_separate),              \
    KUNIT_CASE(test_case_ipc_ns_exit_cleans_up_only_own_namespace),      \
    KUNIT_CASE(test_case_ipc_ns_write_lock_leaves_other_namespace_free), \
    KUNIT_CASE(test_case_ipc_ns_retired_once_empty)

void test_case_ipc_ns_same_topic_name_is_separate(struct kunit * test);
void test_case_ipc_ns_exit_cleans_up_only_own_namespace(struct kunit * test);
void test_case_ipc_ns_write_lock_leaves_other_namespace_free(struct kunit * test);
void test_case_ipc_ns_retired_once_empty(struct kunit * test);
