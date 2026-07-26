/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_GET_TOPIC_PUBLISHER_INFO                   \
  KUNIT_CASE(test_case_get_topic_pub_info_one_publisher),     \
    KUNIT_CASE(test_case_get_topic_pub_info_no_publishers),   \
    KUNIT_CASE(test_case_get_topic_pub_info_topic_not_found), \
    KUNIT_CASE(test_case_get_topic_pub_info_selects_by_domain)

void test_case_get_topic_pub_info_one_publisher(struct kunit * test);
void test_case_get_topic_pub_info_no_publishers(struct kunit * test);
void test_case_get_topic_pub_info_topic_not_found(struct kunit * test);
void test_case_get_topic_pub_info_selects_by_domain(struct kunit * test);
