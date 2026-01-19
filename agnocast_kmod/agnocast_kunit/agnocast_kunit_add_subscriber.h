#pragma once
#include <kunit/test.h>

#define TEST_CASES_ADD_SUBSCRIBER              \
  KUNIT_CASE(test_case_add_subscriber_normal), \
    KUNIT_CASE(test_case_add_subscriber_too_many_subscribers)

void test_case_add_subscriber_normal(struct kunit * test);
void test_case_add_subscriber_too_many_subscribers(struct kunit * test);
