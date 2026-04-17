/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_REMOVE_PUBLISHER                                      \
  KUNIT_CASE(test_case_remove_publisher_basic),                          \
    KUNIT_CASE(test_case_remove_publisher_keeps_topic_with_subscriber),  \
    KUNIT_CASE(test_case_remove_publisher_cleans_unreferenced_messages), \
    KUNIT_CASE(test_case_remove_publisher_leaves_orphaned_messages)

void test_case_remove_publisher_basic(struct kunit * test);
void test_case_remove_publisher_keeps_topic_with_subscriber(struct kunit * test);
void test_case_remove_publisher_cleans_unreferenced_messages(struct kunit * test);
void test_case_remove_publisher_leaves_orphaned_messages(struct kunit * test);
