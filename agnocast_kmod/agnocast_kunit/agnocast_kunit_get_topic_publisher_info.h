#pragma once
#include <kunit/test.h>

#define TEST_CASES_GET_TOPIC_PUBLISHER_INFO                 \
  KUNIT_CASE(test_case_get_topic_pub_info_one_publisher),   \
    KUNIT_CASE(test_case_get_topic_pub_info_no_publishers), \
    KUNIT_CASE(test_case_get_topic_pub_info_topic_not_found)

void test_case_get_topic_pub_info_one_publisher(struct kunit * test);
void test_case_get_topic_pub_info_no_publishers(struct kunit * test);
void test_case_get_topic_pub_info_topic_not_found(struct kunit * test);
