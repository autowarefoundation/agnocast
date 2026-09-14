/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_EXIT_FREE_DATA KUNIT_CASE(test_case_exit_free_data_releases_notify_context)

void test_case_exit_free_data_releases_notify_context(struct kunit * test);
