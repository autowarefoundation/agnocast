// SPDX-License-Identifier: Apache-2.0
#include "memory_allocator.hpp"

#include <gtest/gtest.h>

#include <climits>
#include <cstdint>
#include <stdexcept>

namespace
{
constexpr uint64_t kStartAddr = 0x40000000000ULL;
constexpr pid_t kPidA = 1000;
constexpr pid_t kPidB = 1001;
}  // namespace

TEST(InitMemoryAllocator, InitMemoryAllocatorNonDefault)
{
  MemoryAllocator alloc(2, kStartAddr, 1);
  EXPECT_EQ(alloc.mempool_num(), 2);
  EXPECT_EQ(alloc.mempool_size_bytes(), 1024ULL * 1024ULL * 1024ULL);

  auto * e1 = alloc.assign_memory(kPidA);
  ASSERT_NE(e1, nullptr);
  auto * e2 = alloc.assign_memory(kPidB);
  ASSERT_NE(e2, nullptr);
  EXPECT_NE(e1->addr, e2->addr);
  EXPECT_EQ(alloc.assign_memory(1002), nullptr);
}

TEST(InitMemoryAllocator, InitMemoryAllocatorZeroMempoolNum)
{
  EXPECT_THROW((void)MemoryAllocator(0, kStartAddr, 1), std::invalid_argument);
}

TEST(InitMemoryAllocator, InitMemoryAllocatorNegativeMempoolNum)
{
  EXPECT_THROW((void)MemoryAllocator(-1, kStartAddr, 1), std::invalid_argument);
}

TEST(InitMemoryAllocator, InitMemoryAllocatorZeroMempoolSizeGb)
{
  EXPECT_THROW((void)MemoryAllocator(1, kStartAddr, 0), std::invalid_argument);
}

TEST(InitMemoryAllocator, InitMemoryAllocatorNegativeMempoolSizeGb)
{
  EXPECT_THROW((void)MemoryAllocator(1, kStartAddr, -1), std::invalid_argument);
}

TEST(InitMemoryAllocator, InitMemoryAllocatorOverflowMul)
{
  EXPECT_THROW((void)MemoryAllocator(INT_MAX, kStartAddr, INT_MAX), std::overflow_error);
}

TEST(InitMemoryAllocator, InitMemoryAllocatorOverflowAdd)
{
  EXPECT_THROW((void)MemoryAllocator(2, UINT64_MAX - 1, 1), std::overflow_error);
}
