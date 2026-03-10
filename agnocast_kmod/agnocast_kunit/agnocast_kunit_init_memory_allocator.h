#pragma once
#include <kunit/test.h>

#define TEST_CASES_INIT_MEMORY_ALLOCATOR                                  \
  KUNIT_CASE(test_case_init_memory_allocator_non_default),                \
    KUNIT_CASE(test_case_init_memory_allocator_zero_mempool_num),         \
    KUNIT_CASE(test_case_init_memory_allocator_negative_mempool_num),     \
    KUNIT_CASE(test_case_init_memory_allocator_zero_mempool_size_gb),     \
    KUNIT_CASE(test_case_init_memory_allocator_negative_mempool_size_gb), \
    KUNIT_CASE(test_case_init_memory_allocator_overflow_mul),             \
    KUNIT_CASE(test_case_init_memory_allocator_overflow_add)

void test_case_init_memory_allocator_non_default(struct kunit * test);
void test_case_init_memory_allocator_zero_mempool_num(struct kunit * test);
void test_case_init_memory_allocator_negative_mempool_num(struct kunit * test);
void test_case_init_memory_allocator_zero_mempool_size_gb(struct kunit * test);
void test_case_init_memory_allocator_negative_mempool_size_gb(struct kunit * test);
void test_case_init_memory_allocator_overflow_mul(struct kunit * test);
void test_case_init_memory_allocator_overflow_add(struct kunit * test);
