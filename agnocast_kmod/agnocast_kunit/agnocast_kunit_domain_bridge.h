/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_DOMAIN_BRIDGE                                              \
  KUNIT_CASE(test_case_add_domain_bridge_normal),                             \
    KUNIT_CASE(test_case_add_domain_bridge_same_domain_rejected),             \
    KUNIT_CASE(test_case_add_domain_bridge_reverse_direction),                \
    KUNIT_CASE(test_case_add_domain_bridge_third_domain_rejected),            \
    KUNIT_CASE(test_case_add_domain_bridge_rejected_when_endpoint_exists),    \
    KUNIT_CASE(test_case_domain_bridge_groups_wrappers),                      \
    KUNIT_CASE(test_case_domain_bridge_cross_domain_enumeration),             \
    KUNIT_CASE(test_case_domain_bridge_direction_respected),                  \
    KUNIT_CASE(test_case_domain_bridge_partial_remove_keeps_struct),          \
    KUNIT_CASE(test_case_domain_bridge_partial_remove_sub_keeps_struct),      \
    KUNIT_CASE(test_case_domain_bridge_exit_frees_shared_struct),             \
    KUNIT_CASE(test_case_domain_bridge_get_subscriber_num_filtered),          \
    KUNIT_CASE(test_case_domain_bridge_get_publisher_num_filtered),           \
    KUNIT_CASE(test_case_domain_bridge_shm_info_skips_undelivered_publisher), \
    KUNIT_CASE(test_case_domain_bridge_rename_groups_wrappers),               \
    KUNIT_CASE(test_case_domain_bridge_rename_cross_domain_delivery),         \
    KUNIT_CASE(test_case_domain_bridge_rename_fanout_rejected),               \
    KUNIT_CASE(test_case_domain_bridge_rename_multi_publisher),               \
    KUNIT_CASE(test_case_domain_bridge_rename_notify_uses_canonical_name),    \
    KUNIT_CASE(test_case_domain_bridge_rename_exit_cleanup_uses_canonical_name)

void test_case_add_domain_bridge_normal(struct kunit * test);
void test_case_add_domain_bridge_same_domain_rejected(struct kunit * test);
void test_case_add_domain_bridge_reverse_direction(struct kunit * test);
void test_case_add_domain_bridge_third_domain_rejected(struct kunit * test);
void test_case_add_domain_bridge_rejected_when_endpoint_exists(struct kunit * test);
void test_case_domain_bridge_groups_wrappers(struct kunit * test);
void test_case_domain_bridge_cross_domain_enumeration(struct kunit * test);
void test_case_domain_bridge_direction_respected(struct kunit * test);
void test_case_domain_bridge_partial_remove_keeps_struct(struct kunit * test);
void test_case_domain_bridge_partial_remove_sub_keeps_struct(struct kunit * test);
void test_case_domain_bridge_exit_frees_shared_struct(struct kunit * test);
void test_case_domain_bridge_get_subscriber_num_filtered(struct kunit * test);
void test_case_domain_bridge_get_publisher_num_filtered(struct kunit * test);
void test_case_domain_bridge_shm_info_skips_undelivered_publisher(struct kunit * test);
void test_case_domain_bridge_rename_groups_wrappers(struct kunit * test);
void test_case_domain_bridge_rename_cross_domain_delivery(struct kunit * test);
void test_case_domain_bridge_rename_fanout_rejected(struct kunit * test);
void test_case_domain_bridge_rename_multi_publisher(struct kunit * test);
void test_case_domain_bridge_rename_notify_uses_canonical_name(struct kunit * test);
void test_case_domain_bridge_rename_exit_cleanup_uses_canonical_name(struct kunit * test);
