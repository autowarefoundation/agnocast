#pragma once
#include <kunit/test.h>

#define TEST_CASES_GET_PROCESS_NUM                                                          \
  KUNIT_CASE(test_case_get_process_num_zero), KUNIT_CASE(test_case_get_process_num_single), \
    KUNIT_CASE(test_case_get_process_num_max)

void test_case_get_process_num_zero(struct kunit * test);
void test_case_get_process_num_single(struct kunit * test);
void test_case_get_process_num_max(struct kunit * test);
