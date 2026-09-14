/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_ADD_DOMAIN_BRIDGE_PREFIX                                                 \
  KUNIT_CASE(test_case_add_domain_bridge_prefix_normal),                                    \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_same_domain_rejected),                    \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_empty_rejected),                          \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_root_rejected),                           \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_relative_rejected),                       \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_redeclaration_is_idempotent),             \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_reverse_direction),                       \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_accepted_beside_a_disjoint_prefix),       \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_accepted_beside_an_exact_rule),           \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_accepted_with_a_topic_outside_it),        \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_accepted_with_a_covered_topic_elsewhere), \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_nested_over_disjoint_domains),            \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_repointed_pair_rejected),                 \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_nested_rejected),                         \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_nested_rejected_either_order),            \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_late_reverse_direction_rejected),         \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_rejected_when_covered_endpoint_exists),   \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_over_exact_rejected),                     \
    KUNIT_CASE(test_case_add_domain_bridge_prefix_over_exact_rename_rejected)

void test_case_add_domain_bridge_prefix_normal(struct kunit * test);
void test_case_add_domain_bridge_prefix_same_domain_rejected(struct kunit * test);
void test_case_add_domain_bridge_prefix_empty_rejected(struct kunit * test);
void test_case_add_domain_bridge_prefix_root_rejected(struct kunit * test);
void test_case_add_domain_bridge_prefix_relative_rejected(struct kunit * test);
void test_case_add_domain_bridge_prefix_redeclaration_is_idempotent(struct kunit * test);
void test_case_add_domain_bridge_prefix_reverse_direction(struct kunit * test);
void test_case_add_domain_bridge_prefix_accepted_beside_a_disjoint_prefix(struct kunit * test);
void test_case_add_domain_bridge_prefix_accepted_beside_an_exact_rule(struct kunit * test);
void test_case_add_domain_bridge_prefix_accepted_with_a_topic_outside_it(struct kunit * test);
void test_case_add_domain_bridge_prefix_accepted_with_a_covered_topic_elsewhere(
  struct kunit * test);
void test_case_add_domain_bridge_prefix_nested_over_disjoint_domains(struct kunit * test);
void test_case_add_domain_bridge_prefix_repointed_pair_rejected(struct kunit * test);
void test_case_add_domain_bridge_prefix_nested_rejected(struct kunit * test);
void test_case_add_domain_bridge_prefix_nested_rejected_either_order(struct kunit * test);
void test_case_add_domain_bridge_prefix_late_reverse_direction_rejected(struct kunit * test);
void test_case_add_domain_bridge_prefix_rejected_when_covered_endpoint_exists(struct kunit * test);
void test_case_add_domain_bridge_prefix_over_exact_rejected(struct kunit * test);
void test_case_add_domain_bridge_prefix_over_exact_rename_rejected(struct kunit * test);
