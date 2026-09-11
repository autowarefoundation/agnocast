#include "agnocast/internal/gpu_slot_pool.hpp"

#include "agnocast/agnocast_utils.hpp"
#include "agnocast/internal/gpu_message.hpp"

#include <limits>
#include <unordered_map>

namespace agnocast::internal
{

namespace
{

// Leaked deliberately: a message destructor can reach this after static
// destruction would have run.
std::mutex & pool_table_mutex()
{
  static auto * mtx = new std::mutex();  // NOLINT(cppcoreguidelines-owning-memory)
  return *mtx;
}

std::unordered_map<uint32_t, GpuSlotPool *> & pool_table()
{
  static auto * pools =
    new std::unordered_map<uint32_t, GpuSlotPool *>();  // NOLINT(cppcoreguidelines-owning-memory)
  return *pools;
}

// The alignment cudaMalloc guarantees, so that a slot address is as aligned as
// a device pointer the user would have allocated themselves.
constexpr uint32_t kMinSlotSize = 256;

}  // namespace

uint32_t gpu_slot_size_for(const uint64_t capacity) noexcept
{
  if (capacity <= kMinSlotSize) return kMinSlotSize;

  // Above 2 GiB the next power of two does not fit in the slot size, so the
  // capacity stands as asked; a region of that size fails in the backend either
  // way, and failing there reports the real reason.
  constexpr uint64_t max_slot_size = std::numeric_limits<uint32_t>::max();
  if (capacity > max_slot_size) return static_cast<uint32_t>(max_slot_size);
  if (capacity > (1ULL << 31)) return static_cast<uint32_t>(capacity);

  uint32_t rounded = kMinSlotSize;
  while (rounded < capacity) rounded *= 2;
  return rounded;
}

GpuSlotPool::GpuSlotPool(
  const std::string_view topic_name, const topic_local_id_t publisher_id, const uint32_t region_id,
  const uint32_t slot_size, const uint32_t slot_count)
: topic_name_(topic_name),
  publisher_id_(publisher_id),
  region_id_(region_id),
  slot_size_(slot_size),
  slot_count_(slot_count)
{
  free_slots_.reserve(slot_count);
  for (uint32_t i = slot_count; i > 0; i--) {
    free_slots_.push_back(i - 1);
  }
}

std::unique_ptr<GpuSlotPool> GpuSlotPool::create(
  const std::string_view topic_name, const topic_local_id_t publisher_id, const uint64_t capacity,
  const uint32_t slot_count)
{
  const uint32_t slot_size = gpu_slot_size_for(capacity);

  const uint32_t region_id =
    GpuRegionRegistry::instance().create(topic_name, publisher_id, slot_size, slot_count);
  if (region_id == 0) return nullptr;

  auto pool = std::unique_ptr<GpuSlotPool>(
    new GpuSlotPool(topic_name, publisher_id, region_id, slot_size, slot_count));

  const std::lock_guard<std::mutex> lock(pool_table_mutex());
  pool_table()[region_id] = pool.get();
  return pool;
}

GpuSlotPool::~GpuSlotPool()
{
  {
    // Unregistered first, and under the same lock release_gpu_slot holds across
    // its call, so an in-flight release finishes before the check below and none
    // starts after it.
    const std::lock_guard<std::mutex> lock(pool_table_mutex());
    pool_table().erase(region_id_);
  }

  // Every slot free means nothing refers to this region: the kmod released each
  // entry, which is what let the publisher delete the message holding the slot.
  // With a slot still out the region has to stay, and the kmod keeps it until
  // the publisher's entries drain.
  if (is_idle()) {
    GpuRegionRegistry::instance().destroy(topic_name_, publisher_id_, region_id_);
  }
}

bool GpuSlotPool::is_idle() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return free_slots_.size() == slot_count_;
}

bool GpuSlotPool::acquire(const uint64_t capacity, uint32_t & out_slot_index)
{
  if (capacity > slot_size_) return false;

  const std::lock_guard<std::mutex> lock(mutex_);
  if (free_slots_.empty()) return false;
  out_slot_index = free_slots_.back();
  free_slots_.pop_back();
  return true;
}

void GpuSlotPool::release(const uint32_t slot_index)
{
  const std::lock_guard<std::mutex> lock(mutex_);

  // Bounded rather than trusted. The index comes from a handle in shared memory,
  // and free_slots_ is reserved for exactly slot_count entries: growing past
  // that would both hand one slot out twice and, because a release can happen
  // between a borrow and its publish, reallocate the vector into the
  // shared-memory mempool.
  if (slot_index >= slot_count_ || free_slots_.size() >= slot_count_) {
    RCLCPP_ERROR(
      logger, "ignoring the release of GPU slot %u of region %u: not an outstanding slot",
      slot_index, region_id_);
    return;
  }
  free_slots_.push_back(slot_index);
}

size_t GpuSlotPool::available() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return free_slots_.size();
}

void release_gpu_slot(const uint32_t region_id, const uint32_t slot_index) noexcept
{
  // The lock is held across the release, not just the lookup: the message being
  // destroyed here may be the last one outliving its publisher, so dropping it
  // first would let ~GpuSlotPool free the pool between the two. Nothing takes
  // the table lock while holding a pool's own lock, so this is the only nesting.
  const std::lock_guard<std::mutex> lock(pool_table_mutex());
  const auto it = pool_table().find(region_id);
  if (it == pool_table().end()) return;  // not ours: a peer's region
  it->second->release(slot_index);
}

}  // namespace agnocast::internal
