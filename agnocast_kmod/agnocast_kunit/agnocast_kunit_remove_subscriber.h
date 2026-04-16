/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_REMOVE_SUBSCRIBER                                    \
  KUNIT_CASE(test_case_remove_subscriber_basic),                        \
    KUNIT_CASE(test_case_remove_subscriber_keeps_topic_with_publisher), \
    KUNIT_CASE(test_case_remove_subscriber_clears_references),          \
    KUNIT_CASE(test_case_remove_subscriber_triggers_gc),                \
    KUNIT_CASE(test_case_remove_subscriber_shared_ref_gc)

void test_case_remove_subscriber_basic(struct kunit * test);
void test_case_remove_subscriber_keeps_topic_with_publisher(struct kunit * test);
void test_case_remove_subscriber_clears_references(struct kunit * test);
void test_case_remove_subscriber_triggers_gc(struct kunit * test);
void test_case_remove_subscriber_shared_ref_gc(struct kunit * test);
