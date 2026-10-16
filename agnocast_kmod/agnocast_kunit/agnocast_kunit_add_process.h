/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_ADD_PROCESS                                                                \
  KUNIT_CASE(test_case_add_process_normal), KUNIT_CASE(test_case_add_process_many),           \
    KUNIT_CASE(test_case_add_process_twice),                                                  \
    KUNIT_CASE(test_case_add_process_bridge_manager_per_domain),                              \
    KUNIT_CASE(test_case_add_process_too_many),                                               \
    KUNIT_CASE(test_case_add_process_rejects_unknown_role),                                   \
    KUNIT_CASE(test_case_register_shm_invalid_fd), KUNIT_CASE(test_case_register_shm_normal), \
    KUNIT_CASE(test_case_register_shm_duplicate)

void test_case_add_process_normal(struct kunit * test);
void test_case_add_process_many(struct kunit * test);
void test_case_add_process_twice(struct kunit * test);
void test_case_add_process_bridge_manager_per_domain(struct kunit * test);
void test_case_add_process_too_many(struct kunit * test);
void test_case_add_process_rejects_unknown_role(struct kunit * test);
void test_case_register_shm_invalid_fd(struct kunit * test);
void test_case_register_shm_normal(struct kunit * test);
void test_case_register_shm_duplicate(struct kunit * test);
