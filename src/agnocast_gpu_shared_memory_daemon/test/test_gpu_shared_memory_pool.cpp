// Unit tests for GpuSharedMemoryPool using a mock backend, so all slot bookkeeping
// (best-fit-by-size-class free list, blocking/non-blocking alloc, cleanup) is
// exercised without a GPU.
#include "gpu_shared_memory_pool.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

namespace proto = agnocast::gpu_shared_memory_daemon;

namespace
{

// Records calls and hands out deterministic fake resources. Can be told to fail
// on the Nth create_slot() to exercise the init-failure cleanup path.
class MockSlotBackend : public proto::GpuSlotBackend
{
public:
  proto::BackendType backend_type() const override { return proto::BackendType::kCudaIpc; }

  bool initialize(std::string & gpu_uuid_out) override
  {
    initialize_calls++;
    gpu_uuid_out = "GPU-mock-0001";
    return initialize_result;
  }

  bool create_slot(std::size_t size, proto::AllocatedSlotResources & out) override
  {
    if (
      fail_on_create_index >= 0 && create_calls == static_cast<std::size_t>(fail_on_create_index)) {
      ++create_calls;
      return false;
    }
    const auto tag = static_cast<std::uint8_t>(create_calls + 1);
    out = proto::AllocatedSlotResources{};
    out.device_ptr = reinterpret_cast<void *>(static_cast<std::uintptr_t>(create_calls + 1));
    out.data_ready_event =
      reinterpret_cast<void *>(static_cast<std::uintptr_t>(0x1000 + create_calls));
    out.data_done_event =
      reinterpret_cast<void *>(static_cast<std::uintptr_t>(0x2000 + create_calls));
    out.mem_handle = {tag, static_cast<std::uint8_t>(size & 0xff)};
    out.data_ready_event_handle = {static_cast<std::uint8_t>(0xa0 + tag)};
    out.data_done_event_handle = {static_cast<std::uint8_t>(0xb0 + tag)};
    ++create_calls;
    return true;
  }

  void destroy_slot(proto::AllocatedSlotResources & resources) override
  {
    if (resources.device_ptr != nullptr) {
      ++destroy_calls;
    }
    resources = proto::AllocatedSlotResources{};
  }

  bool initialize_result = true;
  int fail_on_create_index = -1;  // -1 = never fail
  std::size_t initialize_calls = 0;
  std::size_t create_calls = 0;
  std::size_t destroy_calls = 0;
};

// Config: class 0 = 1024 B x 2, class 1 = 4096 B x 3 (5 slots total).
proto::PoolConfig two_class_config()
{
  proto::PoolConfig config;
  config.size_classes = {
    proto::SizeClassConfig{1024u, 2u},
    proto::SizeClassConfig{4096u, 3u},
  };
  return config;
}

}  // namespace

TEST(GpuSharedMemoryPool, InitializeCreatesAllSlots)
{
  MockSlotBackend backend;
  proto::GpuSharedMemoryPool pool(backend);

  ASSERT_TRUE(pool.initialize(two_class_config()));
  EXPECT_EQ(backend.initialize_calls, 1u);
  EXPECT_EQ(backend.create_calls, 5u);
  EXPECT_EQ(pool.total_slots(), 5u);
  EXPECT_EQ(pool.free_slot_count(), 5u);
  EXPECT_EQ(pool.gpu_uuid(), "GPU-mock-0001");
  EXPECT_EQ(pool.backend_type(), proto::BackendType::kCudaIpc);
}

TEST(GpuSharedMemoryPool, ListResponseHasAllDescriptors)
{
  MockSlotBackend backend;
  proto::GpuSharedMemoryPool pool(backend);
  ASSERT_TRUE(pool.initialize(two_class_config()));

  const auto list = pool.make_list_response();
  ASSERT_EQ(list.slots.size(), 5u);

  for (std::uint32_t id = 0; id < list.slots.size(); ++id) {
    const auto & slot = list.slots[id];
    EXPECT_EQ(slot.slot_id, id);
    EXPECT_FALSE(slot.mem_handle.empty());
    EXPECT_FALSE(slot.data_ready_event.empty());
    EXPECT_FALSE(slot.data_done_event.empty());
    if (id < 2) {
      EXPECT_EQ(slot.size_class_index, 0u);
      EXPECT_EQ(slot.slot_size, 1024u);
    } else {
      EXPECT_EQ(slot.size_class_index, 1u);
      EXPECT_EQ(slot.slot_size, 4096u);
    }
  }
}

TEST(GpuSharedMemoryPool, AllocateBestFitBySizeClass)
{
  MockSlotBackend backend;
  proto::GpuSharedMemoryPool pool(backend);
  ASSERT_TRUE(pool.initialize(two_class_config()));

  std::uint32_t small_slot = 0;
  ASSERT_EQ(pool.allocate(1000, small_slot), proto::Status::kOk);
  EXPECT_EQ(pool.make_list_response().slots[small_slot].size_class_index, 0u);

  std::uint32_t big_slot = 0;
  ASSERT_EQ(pool.allocate(2000, big_slot), proto::Status::kOk);
  EXPECT_EQ(pool.make_list_response().slots[big_slot].size_class_index, 1u);
}

TEST(GpuSharedMemoryPool, AllocateFallsThroughToLargerClassWhenExhausted)
{
  MockSlotBackend backend;
  proto::GpuSharedMemoryPool pool(backend);
  ASSERT_TRUE(pool.initialize(two_class_config()));

  // Drain both slots in class 0 with small requests.
  std::uint32_t a = 0;
  std::uint32_t b = 0;
  ASSERT_EQ(pool.allocate(100, a), proto::Status::kOk);
  ASSERT_EQ(pool.allocate(100, b), proto::Status::kOk);

  // A third small request must fall through to class 1.
  std::uint32_t c = 0;
  ASSERT_EQ(pool.allocate(100, c), proto::Status::kOk);
  EXPECT_EQ(pool.make_list_response().slots[c].size_class_index, 1u);
}

TEST(GpuSharedMemoryPool, AllocateSizeTooLarge)
{
  MockSlotBackend backend;
  proto::GpuSharedMemoryPool pool(backend);
  ASSERT_TRUE(pool.initialize(two_class_config()));

  std::uint32_t slot = 42;
  EXPECT_EQ(pool.allocate(4097, slot), proto::Status::kSizeTooLarge);
  EXPECT_EQ(pool.allocate(1u << 30, slot), proto::Status::kSizeTooLarge);
}

TEST(GpuSharedMemoryPool, AllocateReturnsNoFreeSlotWhenExhausted)
{
  MockSlotBackend backend;
  proto::GpuSharedMemoryPool pool(backend);
  ASSERT_TRUE(pool.initialize(two_class_config()));

  std::vector<std::uint32_t> ids;
  for (int i = 0; i < 5; ++i) {
    std::uint32_t slot = 0;
    ASSERT_EQ(pool.allocate(100, slot), proto::Status::kOk);
    ids.push_back(slot);
  }
  EXPECT_EQ(pool.free_slot_count(), 0u);

  std::uint32_t slot = 0;
  EXPECT_EQ(pool.allocate(100, slot), proto::Status::kNoFreeSlot);
}

TEST(GpuSharedMemoryPool, FreeReturnsSlotToPool)
{
  MockSlotBackend backend;
  proto::GpuSharedMemoryPool pool(backend);
  ASSERT_TRUE(pool.initialize(two_class_config()));

  std::uint32_t slot = 0;
  ASSERT_EQ(pool.allocate(100, slot), proto::Status::kOk);
  EXPECT_EQ(pool.free_slot_count(), 4u);

  ASSERT_EQ(pool.free_slot(slot), proto::Status::kOk);
  EXPECT_EQ(pool.free_slot_count(), 5u);
}

TEST(GpuSharedMemoryPool, FreeRejectsInvalidAndDoubleFree)
{
  MockSlotBackend backend;
  proto::GpuSharedMemoryPool pool(backend);
  ASSERT_TRUE(pool.initialize(two_class_config()));

  EXPECT_EQ(pool.free_slot(999), proto::Status::kInvalidSlot);

  std::uint32_t slot = 0;
  ASSERT_EQ(pool.allocate(100, slot), proto::Status::kOk);
  ASSERT_EQ(pool.free_slot(slot), proto::Status::kOk);
  EXPECT_EQ(pool.free_slot(slot), proto::Status::kInvalidSlot);
}

TEST(GpuSharedMemoryPool, ShutdownDestroysEverySlot)
{
  MockSlotBackend backend;
  {
    proto::GpuSharedMemoryPool pool(backend);
    ASSERT_TRUE(pool.initialize(two_class_config()));
    EXPECT_EQ(backend.destroy_calls, 0u);
    pool.shutdown();
    EXPECT_EQ(backend.destroy_calls, 5u);
    EXPECT_EQ(pool.total_slots(), 0u);
    // Idempotent.
    pool.shutdown();
    EXPECT_EQ(backend.destroy_calls, 5u);
  }
}

TEST(GpuSharedMemoryPool, DestructorReleasesSlots)
{
  MockSlotBackend backend;
  {
    proto::GpuSharedMemoryPool pool(backend);
    ASSERT_TRUE(pool.initialize(two_class_config()));
  }
  EXPECT_EQ(backend.destroy_calls, 5u);
}

TEST(GpuSharedMemoryPool, InitializeFailureReleasesPartialAllocations)
{
  MockSlotBackend backend;
  backend.fail_on_create_index = 3;  // fail while creating the 4th slot
  proto::GpuSharedMemoryPool pool(backend);

  EXPECT_FALSE(pool.initialize(two_class_config()));
  // The 3 slots created before the failure must all be destroyed.
  EXPECT_EQ(backend.destroy_calls, 3u);
  EXPECT_EQ(pool.total_slots(), 0u);
}

TEST(GpuSharedMemoryPool, InitializeFailsWhenBackendInitFails)
{
  MockSlotBackend backend;
  backend.initialize_result = false;
  proto::GpuSharedMemoryPool pool(backend);

  EXPECT_FALSE(pool.initialize(two_class_config()));
  EXPECT_EQ(backend.create_calls, 0u);
}

TEST(GpuSharedMemoryPool, ExhaustedAllocateNeverBlocksAndFreedSlotIsReusable)
{
  // The daemon must never block on a client request. Once the pool is exhausted,
  // allocate returns kNoFreeSlot immediately; after a free, allocate succeeds again.
  MockSlotBackend backend;
  proto::GpuSharedMemoryPool pool(backend);
  ASSERT_TRUE(pool.initialize(two_class_config()));

  std::vector<std::uint32_t> ids;
  for (int i = 0; i < 5; ++i) {
    std::uint32_t slot = 0;
    ASSERT_EQ(pool.allocate(100, slot), proto::Status::kOk);
    ids.push_back(slot);
  }

  std::uint32_t denied = 0;
  EXPECT_EQ(pool.allocate(100, denied), proto::Status::kNoFreeSlot);

  ASSERT_EQ(pool.free_slot(ids.front()), proto::Status::kOk);

  std::uint32_t reused = 0;
  EXPECT_EQ(pool.allocate(100, reused), proto::Status::kOk);
  EXPECT_EQ(reused, ids.front());
}
