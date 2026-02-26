#pragma once
#include <kunit/test.h>

#define TEST_CASES_DO_EXIT                                                                     \
  KUNIT_CASE(test_case_is_agnocast_pid), KUNIT_CASE(test_case_do_exit),                        \
    KUNIT_CASE(test_case_do_exit_many), KUNIT_CASE(test_case_do_exit_with_publisher),          \
    KUNIT_CASE(test_case_do_exit_with_subscriber),                                             \
    KUNIT_CASE(test_case_do_exit_with_many_pubsub_in_one_process),                             \
    KUNIT_CASE(test_case_do_exit_with_many_pubsub_in_different_processes_and_publisher_exit),  \
    KUNIT_CASE(test_case_do_exit_with_many_pubsub_in_different_processes_and_subscriber_exit), \
    KUNIT_CASE(test_case_do_exit_with_many_pubsub_in_different_processes_and_all_pubsub_exit), \
    KUNIT_CASE(test_case_do_exit_with_entry),                                                  \
    KUNIT_CASE(test_case_do_exit_with_entry_with_subscriber_reference),                        \
    KUNIT_CASE(test_case_do_exit_with_multi_references_publisher_exit_first),                  \
    KUNIT_CASE(test_case_do_exit_with_multi_references_subscriber_exit_first)

void test_case_is_agnocast_pid(struct kunit * test);
void test_case_do_exit(struct kunit * test);
void test_case_do_exit_many(struct kunit * test);
void test_case_do_exit_with_publisher(struct kunit * test);
void test_case_do_exit_with_subscriber(struct kunit * test);
void test_case_do_exit_with_many_pubsub_in_one_process(struct kunit * test);
void test_case_do_exit_with_many_pubsub_in_different_processes_and_publisher_exit(
  struct kunit * test);
void test_case_do_exit_with_many_pubsub_in_different_processes_and_subscriber_exit(
  struct kunit * test);
void test_case_do_exit_with_many_pubsub_in_different_processes_and_all_pubsub_exit(
  struct kunit * test);
void test_case_do_exit_with_entry(struct kunit * test);
void test_case_do_exit_with_entry_with_subscriber_reference(struct kunit * test);
void test_case_do_exit_with_multi_references_publisher_exit_first(struct kunit * test);
void test_case_do_exit_with_multi_references_subscriber_exit_first(struct kunit * test);
