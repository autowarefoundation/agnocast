/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_ADD_PROCESS                                                      \
  KUNIT_CASE(test_case_add_process_normal), KUNIT_CASE(test_case_add_process_many), \
    KUNIT_CASE(test_case_add_process_twice),                                        \
    KUNIT_CASE(test_case_add_process_perf_manager_per_domain),                      \
    KUNIT_CASE(test_case_discovery_agent_register_first_wins),                      \
    KUNIT_CASE(test_case_discovery_agent_register_duplicate_loses),                 \
    KUNIT_CASE(test_case_discovery_agent_exist_reflects_registration),              \
    KUNIT_CASE(test_case_discovery_agent_commit_exit_when_idle),                    \
    KUNIT_CASE(test_case_discovery_agent_commit_exit_vetoed_when_busy),             \
    KUNIT_CASE(test_case_discovery_agent_reaped_on_exit),                           \
    KUNIT_CASE(test_case_discovery_agent_orphan_race),                              \
    KUNIT_CASE(test_case_discovery_agent_exists_ioctl),                             \
    KUNIT_CASE(test_case_discovery_agent_commit_ignores_exited_process),            \
    KUNIT_CASE(test_case_add_process_too_many)

void test_case_add_process_normal(struct kunit * test);
void test_case_add_process_many(struct kunit * test);
void test_case_add_process_twice(struct kunit * test);
void test_case_add_process_perf_manager_per_domain(struct kunit * test);
void test_case_discovery_agent_register_first_wins(struct kunit * test);
void test_case_discovery_agent_register_duplicate_loses(struct kunit * test);
void test_case_discovery_agent_exist_reflects_registration(struct kunit * test);
void test_case_discovery_agent_commit_exit_when_idle(struct kunit * test);
void test_case_discovery_agent_commit_exit_vetoed_when_busy(struct kunit * test);
void test_case_discovery_agent_reaped_on_exit(struct kunit * test);
void test_case_discovery_agent_orphan_race(struct kunit * test);
void test_case_discovery_agent_exists_ioctl(struct kunit * test);
void test_case_discovery_agent_commit_ignores_exited_process(struct kunit * test);
void test_case_add_process_too_many(struct kunit * test);
