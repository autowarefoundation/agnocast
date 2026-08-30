/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_SPAWN_LEASE                                            \
  KUNIT_CASE(test_case_spawn_lease_first_acquire_wins),                   \
    KUNIT_CASE(test_case_spawn_lease_refused_while_held),                 \
    KUNIT_CASE(test_case_spawn_lease_granted_again_after_release),        \
    KUNIT_CASE(test_case_spawn_lease_refused_when_daemon_registered),     \
    KUNIT_CASE(test_case_spawn_lease_does_not_block_daemon_registration), \
    KUNIT_CASE(test_case_spawn_lease_bridge_is_per_domain),               \
    KUNIT_CASE(test_case_spawn_lease_unlink_ignores_domain),              \
    KUNIT_CASE(test_case_spawn_lease_rejects_unleased_role),              \
    KUNIT_CASE(test_case_spawn_lease_rejects_reserved_domain_for_bridge), \
    KUNIT_CASE(test_case_spawn_lease_held_lease_is_freed_on_module_exit)

void test_case_spawn_lease_first_acquire_wins(struct kunit * test);
void test_case_spawn_lease_refused_while_held(struct kunit * test);
void test_case_spawn_lease_granted_again_after_release(struct kunit * test);
void test_case_spawn_lease_refused_when_daemon_registered(struct kunit * test);
void test_case_spawn_lease_does_not_block_daemon_registration(struct kunit * test);
void test_case_spawn_lease_bridge_is_per_domain(struct kunit * test);
void test_case_spawn_lease_unlink_ignores_domain(struct kunit * test);
void test_case_spawn_lease_rejects_unleased_role(struct kunit * test);
void test_case_spawn_lease_rejects_reserved_domain_for_bridge(struct kunit * test);
void test_case_spawn_lease_held_lease_is_freed_on_module_exit(struct kunit * test);
