/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_ADD_SUBSCRIBER                                             \
  KUNIT_CASE(test_case_add_subscriber_normal),                                \
    KUNIT_CASE(test_case_add_subscriber_too_many_subscribers),                \
    KUNIT_CASE(test_case_add_subscriber_domain_isolation),                    \
    KUNIT_CASE(test_case_add_subscriber_same_domain_shared),                  \
    KUNIT_CASE(test_case_add_subscriber_acquires_notify_context),             \
    KUNIT_CASE(test_case_add_subscriber_take_sub_acquires_no_notify_context), \
    KUNIT_CASE(test_case_add_subscriber_invalid_eventfd),                     \
    KUNIT_CASE(test_case_add_subscriber_releases_notify_context_on_failure)

void test_case_add_subscriber_normal(struct kunit * test);
void test_case_add_subscriber_too_many_subscribers(struct kunit * test);
void test_case_add_subscriber_domain_isolation(struct kunit * test);
void test_case_add_subscriber_same_domain_shared(struct kunit * test);
void test_case_add_subscriber_acquires_notify_context(struct kunit * test);
void test_case_add_subscriber_take_sub_acquires_no_notify_context(struct kunit * test);
void test_case_add_subscriber_invalid_eventfd(struct kunit * test);
void test_case_add_subscriber_releases_notify_context_on_failure(struct kunit * test);
