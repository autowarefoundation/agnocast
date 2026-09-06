#include "agnocast/internal/gpu_slot_pool.hpp"

#include "agnocast/internal/gpu_message.hpp"

#include <unordered_map>

namespace agnocast::internal
{

namespace
{

// Leaked for the same reason as the region table: a message destructor can reach
// this after static destruction would have run.
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

}  // namespace

GpuSlotPool::GpuSlotPool(
  const uint32_t region_id, const uint32_t slot_size, const uint32_t slot_count)
: region_id_(region_id), slot_size_(slot_size)
{
  free_slots_.reserve(slot_count);
  for (uint32_t i = slot_count; i > 0; i--) {
    free_slots_.push_back(i - 1);
  }
}

std::unique_ptr<GpuSlotPool> GpuSlotPool::create(
  const std::string_view topic_name, const topic_local_id_t publisher_id, const uint32_t slot_size,
  const uint32_t slot_count)
{
  const uint32_t region_id =
    GpuRegionRegistry::instance().create(topic_name, publisher_id, slot_size, slot_count);
  if (region_id == 0) return nullptr;

  auto pool = std::unique_ptr<GpuSlotPool>(new GpuSlotPool(region_id, slot_size, slot_count));

  const std::lock_guard<std::mutex> lock(pool_table_mutex());
  pool_table()[region_id] = pool.get();
  return pool;
}

GpuSlotPool::~GpuSlotPool()
{
  const std::lock_guard<std::mutex> lock(pool_table_mutex());
  pool_table().erase(region_id_);
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
  free_slots_.push_back(slot_index);
}

size_t GpuSlotPool::available() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return free_slots_.size();
}

void release_gpu_slot(const uint32_t region_id, const uint32_t slot_index) noexcept
{
  GpuSlotPool * pool = nullptr;
  {
    const std::lock_guard<std::mutex> lock(pool_table_mutex());
    const auto it = pool_table().find(region_id);
    if (it == pool_table().end()) return;  // not ours: a peer's region
    pool = it->second;
  }
  pool->release(slot_index);
}

}  // namespace agnocast::internal
