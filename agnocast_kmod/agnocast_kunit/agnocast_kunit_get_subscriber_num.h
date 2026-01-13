#pragma once
#include <kunit/test.h>

#define TEST_CASES_GET_SUBSCRIBER_NUM                                                             \
  KUNIT_CASE(test_case_get_subscriber_num_normal), KUNIT_CASE(test_case_get_subscriber_num_many), \
    KUNIT_CASE(test_case_get_subscriber_num_different_topic),                                     \
    KUNIT_CASE(test_case_get_subscriber_num_with_exit),                                           \
    KUNIT_CASE(test_case_get_subscriber_num_no_subscriber),                                       \
    KUNIT_CASE(test_case_get_subscriber_num_include_ros2),                                        \
    KUNIT_CASE(test_case_get_subscriber_num_bridge_exist)

void test_case_get_subscriber_num_normal(struct kunit * test);
void test_case_get_subscriber_num_many(struct kunit * test);
void test_case_get_subscriber_num_different_topic(struct kunit * test);
void test_case_get_subscriber_num_with_exit(struct kunit * test);
void test_case_get_subscriber_num_no_subscriber(struct kunit * test);
void test_case_get_subscriber_num_include_ros2(struct kunit * test);
void test_case_get_subscriber_num_bridge_exist(struct kunit * test);
