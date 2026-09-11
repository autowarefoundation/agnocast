/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_GET_TOPIC_LIST                                             \
  KUNIT_CASE(test_case_get_topic_list_no_topic),                              \
    KUNIT_CASE(test_case_get_topic_list_skips_service_response_topic),        \
    KUNIT_CASE(test_case_get_topic_list_lists_service_request_topic),         \
    KUNIT_CASE(test_case_get_topic_list_pairs_each_topic_with_its_domain_id), \
    KUNIT_CASE(test_case_get_topic_list_fails_when_buffer_is_full)

void test_case_get_topic_list_no_topic(struct kunit * test);
void test_case_get_topic_list_skips_service_response_topic(struct kunit * test);
void test_case_get_topic_list_lists_service_request_topic(struct kunit * test);
void test_case_get_topic_list_pairs_each_topic_with_its_domain_id(struct kunit * test);
void test_case_get_topic_list_fails_when_buffer_is_full(struct kunit * test);
