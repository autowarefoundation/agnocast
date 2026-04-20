/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_CHECK_AND_REQUEST_BRIDGE_SHUTDOWN                  \
  KUNIT_CASE(test_case_check_and_request_bridge_shutdown_when_alone), \
    KUNIT_CASE(test_case_check_and_request_bridge_shutdown_when_others_exist)

void test_case_check_and_request_bridge_shutdown_when_alone(struct kunit * test);
void test_case_check_and_request_bridge_shutdown_when_others_exist(struct kunit * test);
