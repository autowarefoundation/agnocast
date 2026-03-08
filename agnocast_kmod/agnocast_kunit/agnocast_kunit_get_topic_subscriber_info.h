#pragma once
#include <kunit/test.h>

#define TEST_CASES_GET_TOPIC_SUBSCRIBER_INFO                 \
  KUNIT_CASE(test_case_get_topic_sub_info_one_subscriber),   \
    KUNIT_CASE(test_case_get_topic_sub_info_no_subscribers), \
    KUNIT_CASE(test_case_get_topic_sub_info_topic_not_found)

void test_case_get_topic_sub_info_one_subscriber(struct kunit * test);
void test_case_get_topic_sub_info_no_subscribers(struct kunit * test);
void test_case_get_topic_sub_info_topic_not_found(struct kunit * test);
