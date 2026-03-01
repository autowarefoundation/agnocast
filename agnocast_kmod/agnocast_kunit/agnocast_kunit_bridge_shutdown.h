#pragma once
#include <kunit/test.h>

#define TEST_CASES_BRIDGE_SHUTDOWN                                      \
  KUNIT_CASE(test_case_bridge_manager_flag_set_on_registration),        \
    KUNIT_CASE(test_case_bridge_manager_detected_by_new_process),       \
    KUNIT_CASE(test_case_notify_bridge_shutdown_clears_flag),           \
    KUNIT_CASE(test_case_check_and_request_bridge_shutdown_when_alone), \
    KUNIT_CASE(test_case_check_and_request_bridge_shutdown_when_others_exist)

void test_case_bridge_manager_flag_set_on_registration(struct kunit * test);
void test_case_bridge_manager_detected_by_new_process(struct kunit * test);
void test_case_notify_bridge_shutdown_clears_flag(struct kunit * test);
void test_case_check_and_request_bridge_shutdown_when_alone(struct kunit * test);
void test_case_check_and_request_bridge_shutdown_when_others_exist(struct kunit * test);
