/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_GET_PUBLISHER_QOS                                       \
  KUNIT_CASE(test_case_qos_volatile), KUNIT_CASE(test_case_qos_transient), \
    KUNIT_CASE(test_case_pub_error_topic_not_found),                       \
    KUNIT_CASE(test_case_error_publisher_not_found)

void test_case_qos_volatile(struct kunit * test);
void test_case_qos_transient(struct kunit * test);

void test_case_pub_error_topic_not_found(struct kunit * test);
void test_case_error_publisher_not_found(struct kunit * test);
