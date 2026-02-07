#pragma once
#include <kunit/test.h>

#define TEST_CASES_RELEASE_SUB_REF                        \
  KUNIT_CASE(test_case_release_sub_ref_no_topic),         \
    KUNIT_CASE(test_case_release_sub_ref_no_message),     \
    KUNIT_CASE(test_case_release_sub_ref_no_pubsub_id),   \
    KUNIT_CASE(test_case_release_sub_ref_last_reference), \
    KUNIT_CASE(test_case_release_sub_ref_multi_reference)

void test_case_release_sub_ref_no_topic(struct kunit * test);
void test_case_release_sub_ref_no_message(struct kunit * test);
void test_case_release_sub_ref_no_pubsub_id(struct kunit * test);
void test_case_release_sub_ref_last_reference(struct kunit * test);
void test_case_release_sub_ref_multi_reference(struct kunit * test);
