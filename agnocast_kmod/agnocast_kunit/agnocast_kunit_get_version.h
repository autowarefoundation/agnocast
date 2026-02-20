#pragma once
#include <kunit/test.h>

#define TEST_CASES_GET_VERSION KUNIT_CASE(test_case_get_version)

void test_case_get_version(struct kunit * test);
