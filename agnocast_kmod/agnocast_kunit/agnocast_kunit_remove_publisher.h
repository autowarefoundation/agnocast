/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_REMOVE_PUBLISHER                                      \
  KUNIT_CASE(test_case_remove_publisher_basic),                          \
    KUNIT_CASE(test_case_remove_publisher_keeps_topic_with_subscriber),  \
    KUNIT_CASE(test_case_remove_publisher_cleans_unreferenced_messages), \
    KUNIT_CASE(test_case_remove_publisher_leaves_orphaned_messages),     \
    KUNIT_CASE(test_case_remove_and_add_publisher),                      \
    KUNIT_CASE(test_case_remove_and_add_publisher_does_not_reuse_serial)

void test_case_remove_publisher_basic(struct kunit * test);
void test_case_remove_publisher_keeps_topic_with_subscriber(struct kunit * test);
void test_case_remove_publisher_cleans_unreferenced_messages(struct kunit * test);
void test_case_remove_publisher_leaves_orphaned_messages(struct kunit * test);
void test_case_remove_and_add_publisher(struct kunit * test);
void test_case_remove_and_add_publisher_does_not_reuse_serial(struct kunit * test);
