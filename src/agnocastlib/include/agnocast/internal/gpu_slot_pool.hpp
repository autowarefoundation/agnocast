#pragma once

#include "agnocast/agnocast_ioctl.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace agnocast::internal
{

// The largest payload a slot can be sized for. Above this the rounding below
// would overflow, and a region of that size cannot be allocated anyway.
constexpr uint64_t kMaxGpuPayloadCapacity = 0xFFFFFF00ULL;

// The slot size a region is created with: the next power of two above the
// capacity asked for, floored at the alignment cudaMalloc guarantees and rounded
// to that alignment above 2 GiB. Always >= capacity and always a multiple of the
// alignment, so slot k, at k * slot_size, is aligned too. Requires
// capacity <= kMaxGpuPayloadCapacity.
[[nodiscard]] uint32_t gpu_slot_size_for(uint64_t capacity) noexcept;

// Hands out the slots of one region. Publisher-local: a slot is free exactly
// when the message occupying it has been destroyed, which the kmod already
// decides, so nothing here duplicates that bookkeeping.
//
// A pool is registered under its region id and unregisters on destruction, so
// releasing the slot of a message that outlived its pool is a no-op rather than
// a use-after-free.
class GpuSlotPool
{
public:
  // `capacity` is the payload size to fit. Slots are sized by
  // gpu_slot_size_for(capacity), so slot_size() may report more than was asked
  // for and an acquire for a somewhat larger payload can still succeed.
  [[nodiscard]] static std::unique_ptr<GpuSlotPool> create(
    std::string_view topic_name, topic_local_id_t publisher_id, uint64_t capacity,
    uint32_t slot_count);

  // Releases the region when no slot is still out. A slot still held means a
  // message still refers to the region, so the kmod keeps it alive until the
  // publisher's entries drain, and this process's mapping is left to exit.
  ~GpuSlotPool();

  GpuSlotPool(const GpuSlotPool &) = delete;
  GpuSlotPool & operator=(const GpuSlotPool &) = delete;

  [[nodiscard]] uint32_t region_id() const noexcept { return region_id_; }
  [[nodiscard]] uint32_t slot_size() const noexcept { return slot_size_; }

  // No slot is out, so nothing refers to this region and it can be released,
  // which is what lets a publisher at the region cap trade it for another size.
  [[nodiscard]] bool is_idle() const;

  // Fails rather than waits when every slot is in flight: blocking would stall
  // the publishing thread on subscribers it does not control. Also fails when
  // `capacity` exceeds a slot, so a payload is never written into a region that
  // was sized for a smaller one.
  [[nodiscard]] bool acquire(uint64_t capacity, uint32_t & out_slot_index);
  void release(uint32_t slot_index);

  [[nodiscard]] size_t available() const;

private:
  GpuSlotPool(
    std::string_view topic_name, topic_local_id_t publisher_id, uint32_t region_id,
    uint32_t slot_size, uint32_t slot_count);

  // Owned rather than borrowed: the name is needed to release the region during
  // publisher teardown.
  std::string topic_name_;
  topic_local_id_t publisher_id_;
  uint32_t region_id_;
  uint32_t slot_size_;
  uint32_t slot_count_;
  mutable std::mutex mutex_;
  // Which slots are out. Sized at construction and never resized, so a release
  // between a borrow and its publish cannot allocate from the mempool.
  std::vector<bool> slot_is_out_;
};

}  // namespace agnocast::internal
