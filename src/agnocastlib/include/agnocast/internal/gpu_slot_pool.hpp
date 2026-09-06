#pragma once

#include "agnocast/agnocast_ioctl.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string_view>
#include <vector>

namespace agnocast::internal
{

// Hands out the slots of one region. Publisher-local: a slot is free exactly
// when the message occupying it has been destroyed, which the kmod already
// decides, so nothing here duplicates that bookkeeping.
//
// A pool registers itself under its region id and unregisters on destruction,
// so a message outliving its publisher resolves to nothing rather than to a
// freed pool.
class GpuSlotPool
{
public:
  [[nodiscard]] static std::unique_ptr<GpuSlotPool> create(
    std::string_view topic_name, topic_local_id_t publisher_id, uint32_t slot_size,
    uint32_t slot_count);

  ~GpuSlotPool();

  GpuSlotPool(const GpuSlotPool &) = delete;
  GpuSlotPool & operator=(const GpuSlotPool &) = delete;

  [[nodiscard]] uint32_t region_id() const noexcept { return region_id_; }
  [[nodiscard]] uint32_t slot_size() const noexcept { return slot_size_; }

  // Fails rather than waits when every slot is in flight: blocking would stall
  // the publishing thread on subscribers it does not control. Also fails when
  // `capacity` exceeds a slot, so a payload is never written into a region that
  // was sized for a smaller one.
  [[nodiscard]] bool acquire(uint64_t capacity, uint32_t & out_slot_index);
  void release(uint32_t slot_index);

  [[nodiscard]] size_t available() const;

private:
  GpuSlotPool(uint32_t region_id, uint32_t slot_size, uint32_t slot_count);

  uint32_t region_id_;
  uint32_t slot_size_;
  mutable std::mutex mutex_;
  std::vector<uint32_t> free_slots_;
};

}  // namespace agnocast::internal
