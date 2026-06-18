/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_DOMAIN_BRIDGE                                  \
  KUNIT_CASE(test_case_add_domain_bridge_normal),                 \
    KUNIT_CASE(test_case_add_domain_bridge_same_domain_rejected), \
    KUNIT_CASE(test_case_add_domain_bridge_reverse_direction),    \
    KUNIT_CASE(test_case_add_domain_bridge_third_domain_rejected)

void test_case_add_domain_bridge_normal(struct kunit * test);
void test_case_add_domain_bridge_same_domain_rejected(struct kunit * test);
void test_case_add_domain_bridge_reverse_direction(struct kunit * test);
void test_case_add_domain_bridge_third_domain_rejected(struct kunit * test);
