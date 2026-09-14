/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_GET_EXIT_PROCESS                                   \
  KUNIT_CASE(test_case_get_exit_process_idle_poll_empty_namespace),   \
    KUNIT_CASE(test_case_get_exit_process_idle_poll_process_remains), \
    KUNIT_CASE(test_case_get_exit_process_commit_last_process)

void test_case_get_exit_process_idle_poll_empty_namespace(struct kunit * test);
void test_case_get_exit_process_idle_poll_process_remains(struct kunit * test);
void test_case_get_exit_process_commit_last_process(struct kunit * test);
