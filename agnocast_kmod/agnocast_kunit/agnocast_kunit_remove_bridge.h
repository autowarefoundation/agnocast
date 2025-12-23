#pragma once
#include <kunit/test.h>

#define TEST_CASES_REMOVE_BRIDGE                                                           \
  KUNIT_CASE(test_case_remove_bridge_normal), KUNIT_CASE(test_case_remove_bridge_partial), \
    KUNIT_CASE(test_case_remove_bridge_not_found),                                         \
    KUNIT_CASE(test_case_remove_bridge_pid_mismatch)

void test_case_remove_bridge_normal(struct kunit * test);
void test_case_remove_bridge_partial(struct kunit * test);
void test_case_remove_bridge_not_found(struct kunit * test);
void test_case_remove_bridge_pid_mismatch(struct kunit * test);