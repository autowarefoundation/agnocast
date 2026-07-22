#include "gpu_shared_memory_pool.hpp"

#include <cstdio>
#include <utility>

namespace agnocast::gpu_shared_memory_daemon
{

GpuSharedMemoryPool::GpuSharedMemoryPool(GpuSlotBackend & backend) : backend_(backend)
{
}

GpuSharedMemoryPool::~GpuSharedMemoryPool()
{
  shutdown();
}

bool GpuSharedMemoryPool::initialize(const PoolConfig & config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (initialized_) {
    return true;
  }

  if (!backend_.initialize(gpu_uuid_)) {
    std::fprintf(stderr, "[agnocast_gpu_shared_memory_daemon] backend initialize failed\n");
    return false;
  }

  class_sizes_.clear();
  free_by_class_.assign(config.size_classes.size(), {});
  slots_.clear();

  std::uint32_t next_slot_id = 0;
  for (std::size_t c = 0; c < config.size_classes.size(); ++c) {
    const auto & size_class = config.size_classes[c];
    class_sizes_.push_back(size_class.slot_size_bytes);

    for (std::uint32_t i = 0; i < size_class.slot_count; ++i) {
      Slot slot;
      slot.slot_id = next_slot_id;
      slot.size_class_index = static_cast<std::uint32_t>(c);
      slot.slot_size = size_class.slot_size_bytes;
      slot.allocated = false;

      if (!backend_.create_slot(size_class.slot_size_bytes, slot.resources)) {
        std::fprintf(
          stderr, "[agnocast_gpu_shared_memory_daemon] create_slot failed for size %llu\n",
          static_cast<unsigned long long>(size_class.slot_size_bytes));
        // Release everything created so far, then report failure.
        for (auto & created : slots_) {
          backend_.destroy_slot(created.resources);
        }
        slots_.clear();
        class_sizes_.clear();
        free_by_class_.clear();
        return false;
      }

      slot.descriptor.slot_id = slot.slot_id;
      slot.descriptor.size_class_index = slot.size_class_index;
      slot.descriptor.slot_size = slot.slot_size;
      slot.descriptor.mem_handle = slot.resources.mem_handle;
      slot.descriptor.data_ready_event = slot.resources.data_ready_event_handle;
      slot.descriptor.data_done_event = slot.resources.data_done_event_handle;

      free_by_class_[c].push_back(slot.slot_id);
      slots_.push_back(std::move(slot));
      ++next_slot_id;
    }
  }

  initialized_ = true;
  return true;
}

void GpuSharedMemoryPool::shutdown()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & slot : slots_) {
    backend_.destroy_slot(slot.resources);
  }
  slots_.clear();
  class_sizes_.clear();
  free_by_class_.clear();
  initialized_ = false;
}

ListResponse GpuSharedMemoryPool::make_list_response() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  ListResponse response;
  response.slots.reserve(slots_.size());
  for (const auto & slot : slots_) {
    response.slots.push_back(slot.descriptor);
  }
  return response;
}

std::size_t GpuSharedMemoryPool::first_fitting_class(std::size_t size) const
{
  for (std::size_t c = 0; c < class_sizes_.size(); ++c) {
    if (class_sizes_[c] >= size) {
      return c;
    }
  }
  return class_sizes_.size();
}

Status GpuSharedMemoryPool::allocate(std::size_t size, std::uint32_t & slot_id_out)
{
  std::lock_guard<std::mutex> lock(mutex_);

  const std::size_t first_class = first_fitting_class(size);
  if (first_class >= class_sizes_.size()) {
    // No size class is large enough: unsatisfiable.
    return Status::kSizeTooLarge;
  }

  // Best fit: try the smallest fitting class, falling through to larger classes.
  // Never blocks; if nothing is free the caller is told and retries on its side.
  for (std::size_t c = first_class; c < free_by_class_.size(); ++c) {
    if (!free_by_class_[c].empty()) {
      const std::uint32_t slot_id = free_by_class_[c].back();
      free_by_class_[c].pop_back();
      slots_[slot_id].allocated = true;
      slot_id_out = slot_id;
      return Status::kOk;
    }
  }
  return Status::kNoFreeSlot;
}

Status GpuSharedMemoryPool::free_slot(std::uint32_t slot_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (slot_id >= slots_.size() || !slots_[slot_id].allocated) {
    return Status::kInvalidSlot;
  }
  Slot & slot = slots_[slot_id];
  slot.allocated = false;
  free_by_class_[slot.size_class_index].push_back(slot_id);
  return Status::kOk;
}

std::size_t GpuSharedMemoryPool::total_slots() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return slots_.size();
}

std::size_t GpuSharedMemoryPool::free_slot_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::size_t count = 0;
  for (const auto & free_list : free_by_class_) {
    count += free_list.size();
  }
  return count;
}

}  // namespace agnocast::gpu_shared_memory_daemon
