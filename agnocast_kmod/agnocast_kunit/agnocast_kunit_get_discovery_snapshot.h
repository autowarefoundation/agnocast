/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_GET_DISCOVERY_SNAPSHOT                      \
  KUNIT_CASE(test_case_get_discovery_snapshot_empty),          \
    KUNIT_CASE(test_case_get_discovery_snapshot_probe_counts), \
    KUNIT_CASE(test_case_get_discovery_snapshot_fill)

void test_case_get_discovery_snapshot_empty(struct kunit * test);
void test_case_get_discovery_snapshot_probe_counts(struct kunit * test);
void test_case_get_discovery_snapshot_fill(struct kunit * test);
