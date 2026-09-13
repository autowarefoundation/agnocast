/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_RECLAIM_MSGS                                         \
  KUNIT_CASE(test_case_reclaim_msgs_nothing_to_release),                \
    KUNIT_CASE(test_case_reclaim_msgs_releases_what_publish_could_not), \
    KUNIT_CASE(test_case_reclaim_msgs_rejects_a_foreign_process),       \
    KUNIT_CASE(test_case_reclaim_msgs_topic_not_found),                 \
    KUNIT_CASE(test_case_reclaim_msgs_publisher_not_found)

void test_case_reclaim_msgs_nothing_to_release(struct kunit * test);
void test_case_reclaim_msgs_releases_what_publish_could_not(struct kunit * test);
void test_case_reclaim_msgs_rejects_a_foreign_process(struct kunit * test);
void test_case_reclaim_msgs_topic_not_found(struct kunit * test);
void test_case_reclaim_msgs_publisher_not_found(struct kunit * test);
