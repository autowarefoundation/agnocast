/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_ADD_PROCESS                                                             \
  KUNIT_CASE(test_case_add_process_normal), KUNIT_CASE(test_case_add_process_many),        \
    KUNIT_CASE(test_case_add_process_twice),                                               \
    KUNIT_CASE(test_case_add_process_bridge_manager_per_domain),                           \
    KUNIT_CASE(test_case_add_process_too_many),                                            \
    KUNIT_CASE(test_case_add_process_unlink_daemon_registration),                          \
    KUNIT_CASE(test_case_add_process_unlink_daemon_duplicate_refused),                     \
    KUNIT_CASE(test_case_add_process_unlink_daemon_respawns_after_death),                  \
    KUNIT_CASE(test_case_add_process_unlink_daemon_is_not_counted_as_work),                \
    KUNIT_CASE(test_case_add_process_unlink_daemon_removed_on_death),                      \
    KUNIT_CASE(test_case_add_process_unlink_daemon_deregisters_when_told_to_exit),         \
    KUNIT_CASE(test_case_add_process_rejects_unknown_role),                                \
    KUNIT_CASE(test_case_add_process_rejects_the_daemon_domain_id),                        \
    KUNIT_CASE(test_case_add_process_daemon_may_send_the_daemon_domain_id),                \
    KUNIT_CASE(test_case_add_process_unlink_daemon_deregistration_frees_its_mempool_slot), \
    KUNIT_CASE(test_case_add_process_unlink_daemon_stays_registered_while_draining)

void test_case_add_process_normal(struct kunit * test);
void test_case_add_process_many(struct kunit * test);
void test_case_add_process_twice(struct kunit * test);
void test_case_add_process_bridge_manager_per_domain(struct kunit * test);
void test_case_add_process_too_many(struct kunit * test);
void test_case_add_process_unlink_daemon_registration(struct kunit * test);
void test_case_add_process_unlink_daemon_duplicate_refused(struct kunit * test);
void test_case_add_process_unlink_daemon_respawns_after_death(struct kunit * test);
void test_case_add_process_unlink_daemon_is_not_counted_as_work(struct kunit * test);
void test_case_add_process_unlink_daemon_removed_on_death(struct kunit * test);
void test_case_add_process_unlink_daemon_deregisters_when_told_to_exit(struct kunit * test);
void test_case_add_process_rejects_unknown_role(struct kunit * test);
void test_case_add_process_rejects_the_daemon_domain_id(struct kunit * test);
void test_case_add_process_daemon_may_send_the_daemon_domain_id(struct kunit * test);
void test_case_add_process_unlink_daemon_deregistration_frees_its_mempool_slot(struct kunit * test);
void test_case_add_process_unlink_daemon_stays_registered_while_draining(struct kunit * test);
