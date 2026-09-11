// Publisher-side GPU slot bookkeeping. Everything here runs without a GPU or
// the kernel module: GpuSlotPool::create needs both, but the slot sizing policy
// and the handle lifetime rules do not, and they are what decide whether a
// publisher can keep borrowing.

#include "agnocast/internal/gpu_message.hpp"
#include "agnocast/internal/gpu_slot_pool.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <utility>

namespace
{

using agnocast::internal::gpu_array;
using agnocast::internal::gpu_slot_size_for;

// A region is sized for the payload asked for, but rounded so that a payload
// which grows keeps reusing its region rather than needing a new one per size.
TEST(GpuSlotSizeTest, RoundsUpToAPowerOfTwo)
{
  EXPECT_EQ(gpu_slot_size_for(1), 256u);
  EXPECT_EQ(gpu_slot_size_for(256), 256u);
  EXPECT_EQ(gpu_slot_size_for(257), 512u);
  EXPECT_EQ(gpu_slot_size_for(4096), 4096u);
  EXPECT_EQ(gpu_slot_size_for(4097), 8192u);
  EXPECT_EQ(gpu_slot_size_for(3u << 20), 4u << 20);
}

// The floor is cudaMalloc's alignment, so that every slot boundary
// (slot_index * slot_size) is as aligned as a device pointer the caller would
// have allocated themselves.
TEST(GpuSlotSizeTest, EverySizeIsAMultipleOfTheCudaMallocAlignment)
{
  for (uint64_t capacity = 1; capacity <= agnocast::internal::kMaxGpuPayloadCapacity;
       capacity = capacity * 3 + 1) {
    const uint32_t size = gpu_slot_size_for(capacity);
    EXPECT_EQ(size % 256u, 0u) << "capacity " << capacity;
    EXPECT_GE(size, capacity) << "capacity " << capacity;
  }
}

// The property the region cap depends on: a payload that grows monotonically
// must not need a new region for every new size.
TEST(GpuSlotSizeTest, GrowingPayloadsShareABoundedNumberOfSizes)
{
  uint32_t previous = 0;
  int distinct = 0;
  for (uint64_t capacity = 1; capacity <= std::numeric_limits<uint32_t>::max() / 2;
       capacity += capacity / 8 + 1) {
    const uint32_t size = gpu_slot_size_for(capacity);
    ASSERT_GE(size, capacity) << "capacity " << capacity;
    if (size != previous) {
      distinct++;
      previous = size;
    }
  }
  EXPECT_LE(distinct, 32);
}

// Above 2 GiB the next power of two does not fit in a uint32 slot size. The size
// then tracks the capacity, but it must still be rounded up to the alignment
// floor and must never wrap below the payload: slot k begins at k * slot_size,
// so an unrounded size would misalign every slot after the first -- which a
// typed device access faults on, poisoning the process's CUDA context.
TEST(GpuSlotSizeTest, LargeCapacitiesStayAlignedAndNeverWrap)
{
  EXPECT_EQ(gpu_slot_size_for(1ULL << 31), 1u << 31);

  const uint32_t just_over = gpu_slot_size_for((1ULL << 31) + 1);
  EXPECT_GE(just_over, (1ULL << 31) + 1);
  EXPECT_EQ(just_over % 256u, 0u);

  const uint32_t at_max = gpu_slot_size_for(agnocast::internal::kMaxGpuPayloadCapacity);
  EXPECT_GE(at_max, agnocast::internal::kMaxGpuPayloadCapacity);
  EXPECT_EQ(at_max % 256u, 0u);
}

// A message can outlive the publisher that owns its region, so releasing a slot
// of a region this process does not own must do nothing rather than reach a
// freed pool.
TEST(GpuArrayTest, ReleasingAnUnknownRegionIsANoOp)
{
  agnocast::internal::release_gpu_slot(12345, 0);  // must not crash
  {
    const gpu_array<uint8_t> data(12345, 0, 64, 7);
    EXPECT_TRUE(data.valid());
  }  // destructor releases into a pool table that holds no such region
}

TEST(GpuArrayTest, DefaultConstructedResolvesToNothing)
{
  const gpu_array<uint8_t> data;
  EXPECT_FALSE(data.valid());
  EXPECT_EQ(data.size(), 0u);
  EXPECT_EQ(data.region_id(), 0u);
  EXPECT_EQ(data.get(), nullptr);
}

// A subscriber that has not mapped the region yet must get nullptr, not a wild
// address.
TEST(GpuArrayTest, UnmappedRegionResolvesToNullptr)
{
  const gpu_array<uint8_t> data(9999, 0, 64, 7);
  EXPECT_EQ(data.get(), nullptr);
}

// Move-only, because two handles to one slot would release it twice.
TEST(GpuArrayTest, MoveTransfersTheSlotExactlyOnce)
{
  gpu_array<uint8_t> first(4242, 3, 128, 7);
  const gpu_array<uint8_t> second(std::move(first));

  EXPECT_FALSE(first.valid());  // NOLINT(bugprone-use-after-move)
  EXPECT_TRUE(second.valid());
  EXPECT_EQ(second.region_id(), 4242u);
  EXPECT_EQ(second.slot_index(), 3u);
  EXPECT_EQ(second.size(), 128u);
}

}  // namespace
