/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_GET_SUBSCRIBER_QOS                                                              \
  KUNIT_CASE(test_case_qos_volatile_best_effort), KUNIT_CASE(test_case_qos_volatile_reliable),     \
    KUNIT_CASE(test_case_qos_transient_best_effort), KUNIT_CASE(test_case_qos_transient_reliable), \
    KUNIT_CASE(test_case_sub_error_topic_not_found),                                               \
    KUNIT_CASE(test_case_error_subscriber_not_found)

void test_case_qos_volatile_best_effort(struct kunit * test);
void test_case_qos_volatile_reliable(struct kunit * test);
void test_case_qos_transient_best_effort(struct kunit * test);
void test_case_qos_transient_reliable(struct kunit * test);

void test_case_sub_error_topic_not_found(struct kunit * test);
void test_case_error_subscriber_not_found(struct kunit * test);
