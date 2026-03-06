#include "agnocast_kunit_init_memory_allocator.h"

#include "../agnocast_memory_allocator.h"

#include <kunit/test.h>
#include <linux/limits.h>

// Helper to save and restore module parameters around each test.
// Tests call init_memory_allocator() with modified globals, then
// cleanup_memory_allocator() to free, and finally restore the originals
// so the suite-wide allocator (re-initialized in agnocast_test_exit) is unaffected.

struct saved_params
{
  int mempool_num;
  unsigned long mempool_start_addr;
  int mempool_size_gb;
};

static void save_params(struct saved_params * p)
{
  p->mempool_num = mempool_num;
  p->mempool_start_addr = mempool_start_addr;
  p->mempool_size_gb = mempool_size_gb;
}

static void restore_params(const struct saved_params * p)
{
  mempool_num = p->mempool_num;
  mempool_start_addr = p->mempool_start_addr;
  mempool_size_gb = p->mempool_size_gb;
}

void test_case_init_memory_allocator_non_default(struct kunit * test)
{
  struct saved_params saved;
  save_params(&saved);

  // Use small non-default values
  mempool_num = 2;
  mempool_size_gb = 1;
  mempool_start_addr = 0x80000000000;

  int ret = init_memory_allocator();
  KUNIT_ASSERT_EQ(test, ret, 0);

  // Verify by assigning memory to a process
  struct mempool_entry * entry = assign_memory(9999);
  KUNIT_ASSERT_NOT_NULL(test, entry);
  KUNIT_EXPECT_EQ(test, entry->addr, (uint64_t)0x80000000000);

  free_memory(9999);
  cleanup_memory_allocator();

  // Restore and re-init for subsequent tests
  restore_params(&saved);
  KUNIT_ASSERT_EQ(test, init_memory_allocator(), 0);
}

void test_case_init_memory_allocator_zero_mempool_num(struct kunit * test)
{
  struct saved_params saved;
  save_params(&saved);

  mempool_num = 0;

  int ret = init_memory_allocator();
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);

  restore_params(&saved);
}

void test_case_init_memory_allocator_negative_mempool_num(struct kunit * test)
{
  struct saved_params saved;
  save_params(&saved);

  mempool_num = -1;

  int ret = init_memory_allocator();
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);

  restore_params(&saved);
}

void test_case_init_memory_allocator_zero_mempool_size_gb(struct kunit * test)
{
  struct saved_params saved;
  save_params(&saved);

  mempool_size_gb = 0;

  int ret = init_memory_allocator();
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);

  restore_params(&saved);
}

void test_case_init_memory_allocator_negative_mempool_size_gb(struct kunit * test)
{
  struct saved_params saved;
  save_params(&saved);

  mempool_size_gb = -1;

  int ret = init_memory_allocator();
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);

  restore_params(&saved);
}

void test_case_init_memory_allocator_overflow_mul(struct kunit * test)
{
  struct saved_params saved;
  save_params(&saved);

  // INT_MAX pools * large size_gb should overflow uint64_t multiplication
  mempool_num = INT_MAX;
  mempool_size_gb = INT_MAX;

  int ret = init_memory_allocator();
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);

  restore_params(&saved);
}

void test_case_init_memory_allocator_overflow_add(struct kunit * test)
{
  struct saved_params saved;
  save_params(&saved);

  // Large start address + moderate total size should overflow uint64_t addition
  mempool_start_addr = ULONG_MAX;
  mempool_num = 1;
  mempool_size_gb = 1;

  int ret = init_memory_allocator();
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);

  restore_params(&saved);
}
