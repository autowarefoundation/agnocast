#pragma once
#include <kunit/test.h>

#define TEST_CASES_GET_PUBLISHER_NUM                                                            \
  KUNIT_CASE(test_case_get_publisher_num_normal), KUNIT_CASE(test_case_get_publisher_num_many), \
    KUNIT_CASE(test_case_get_publisher_num_different_topic),                                    \
    KUNIT_CASE(test_case_get_publisher_num_with_exit),                                          \
    KUNIT_CASE(test_case_get_publisher_num_no_publisher),                                       \
    KUNIT_CASE(test_case_get_publisher_num_bridge_exist)

void test_case_get_publisher_num_normal(struct kunit * test);
void test_case_get_publisher_num_many(struct kunit * test);
void test_case_get_publisher_num_different_topic(struct kunit * test);
void test_case_get_publisher_num_with_exit(struct kunit * test);
void test_case_get_publisher_num_no_publisher(struct kunit * test);
void test_case_get_publisher_num_bridge_exist(struct kunit * test);
