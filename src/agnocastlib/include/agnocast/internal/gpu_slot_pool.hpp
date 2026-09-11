#pragma once

#include "agnocast/agnocast_ioctl.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace agnocast::internal
{

// Runs an action unless it is dismissed, so a slot reserved before a step that
// can throw is returned on the way out. Lives here rather than in a header of
// its own because the borrow path is its only caller: what has to be undone
// there -- a slot addressed by index, a counter that opens the shared-memory
// allocation window -- is not an object with a destructor of its own.
template <typename Action>
class ScopeGuard
{
public:
  explicit ScopeGuard(Action action) : action_(std::move(action)) {}
  ~ScopeGuard()
  {
    if (armed_) action_();
  }

  ScopeGuard(const ScopeGuard &) = delete;
  ScopeGuard & operator=(const ScopeGuard &) = delete;
  ScopeGuard(ScopeGuard &&) = delete;
  ScopeGuard & operator=(ScopeGuard &&) = delete;

  // Called once the step the guard covered has succeeded.
  void dismiss() noexcept { armed_ = false; }

private:
  Action action_;
  bool armed_ = true;
};

template <typename Action>
[[nodiscard]] ScopeGuard<Action> make_scope_guard(Action action)
{
  return ScopeGuard<Action>(std::move(action));
}

// The largest payload a slot can be sized for. Above this a slot size could not
// be rounded up to the alignment below without overflowing, and a region of that
// size cannot be allocated anyway.
constexpr uint64_t kMaxGpuPayloadCapacity = 0xFFFFFF00ULL;

// The slot size a region is created with, rounded up from the payload capacity
// asked for: the next power of two, with a floor of the alignment cudaMalloc
// guarantees, above which it rounds to that alignment instead. The result is
// always >= capacity and always a multiple of the alignment, so slot k, which
// begins at k * slot_size, is aligned too. Requires
// capacity <= kMaxGpuPayloadCapacity. See the sizing policy in
// docs/gpu_memory.md.
[[nodiscard]] uint32_t gpu_slot_size_for(uint64_t capacity) noexcept;

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
  // `capacity` is the payload size the caller needs to fit. Slots are sized by
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
  [[nodiscard]] uint32_t slot_count() const noexcept { return slot_count_; }

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

  // Owned rather than borrowed: the pool outlives nothing, but it is destroyed
  // during publisher teardown and the name is needed to release the region then.
  std::string topic_name_;
  topic_local_id_t publisher_id_;
  uint32_t region_id_;
  uint32_t slot_size_;
  uint32_t slot_count_;
  mutable std::mutex mutex_;
  std::vector<uint32_t> free_slots_;
  // Which slots are currently out, so that releasing one that is not can be
  // refused. Sized at construction and never resized.
  std::vector<bool> slot_is_out_;
};

}  // namespace agnocast::internal
