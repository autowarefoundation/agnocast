/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_GPU_REGION                                                         \
  KUNIT_CASE(test_case_gpu_region_round_trip), KUNIT_CASE(test_case_gpu_region_blob), \
    KUNIT_CASE(test_case_gpu_region_reject_geometry_over_mapping),                    \
    KUNIT_CASE(test_case_gpu_region_reject_empty_geometry),                           \
    KUNIT_CASE(test_case_gpu_region_second_registration_gets_a_new_id),               \
    KUNIT_CASE(test_case_gpu_region_get_selects_the_named_region),                    \
    KUNIT_CASE(test_case_gpu_region_get_unknown_id),                                  \
    KUNIT_CASE(test_case_gpu_region_reject_beyond_the_cap),                           \
    KUNIT_CASE(test_case_gpu_region_add_topic_not_found),                             \
    KUNIT_CASE(test_case_gpu_region_add_publisher_not_found),                         \
    KUNIT_CASE(test_case_gpu_region_get_publisher_not_found),                         \
    KUNIT_CASE(test_case_gpu_region_get_without_registration),                        \
    KUNIT_CASE(test_case_gpu_region_get_blob_buffer_too_small)

void test_case_gpu_region_round_trip(struct kunit * test);
void test_case_gpu_region_blob(struct kunit * test);

void test_case_gpu_region_reject_geometry_over_mapping(struct kunit * test);
void test_case_gpu_region_reject_empty_geometry(struct kunit * test);
void test_case_gpu_region_reject_beyond_the_cap(struct kunit * test);

void test_case_gpu_region_second_registration_gets_a_new_id(struct kunit * test);
void test_case_gpu_region_get_selects_the_named_region(struct kunit * test);
void test_case_gpu_region_get_unknown_id(struct kunit * test);

void test_case_gpu_region_add_topic_not_found(struct kunit * test);
void test_case_gpu_region_add_publisher_not_found(struct kunit * test);
void test_case_gpu_region_get_publisher_not_found(struct kunit * test);
void test_case_gpu_region_get_without_registration(struct kunit * test);
void test_case_gpu_region_get_blob_buffer_too_small(struct kunit * test);
