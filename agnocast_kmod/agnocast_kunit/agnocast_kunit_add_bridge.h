#pragma once
#include <kunit/test.h>

#define TEST_CASES_ADD_BRIDGE                                                             \
  KUNIT_CASE(test_case_add_bridge_normal), KUNIT_CASE(test_case_add_bridge_update_flags), \
    KUNIT_CASE(test_case_add_bridge_already_exists_diff_pid)

void test_case_add_bridge_normal(struct kunit * test);
void test_case_add_bridge_update_flags(struct kunit * test);
void test_case_add_bridge_already_exists_diff_pid(struct kunit * test);