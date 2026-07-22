// Manages the pool of shareable GPU memory slots for one GPU.
//
// Owns slot bookkeeping (a best-fit-by-size-class free list) and delegates all
// GPU work (allocation, event creation, handle export/teardown) to a GpuSlotBackend.
// Because the backend is injected, every path here is exercised in unit tests with
// a mock backend, no GPU required. The Unix-domain-socket server (later step) is a
// separate layer that calls into this class.
#pragma once

#include "agnocast_gpu_shared_memory_daemon/pool_config.hpp"
#include "agnocast_gpu_shared_memory_daemon/protocol.hpp"
#include "gpu_slot_backend.hpp"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace agnocast::gpu_shared_memory_daemon
{

class GpuSharedMemoryPool
{
public:
  // The backend must outlive the pool.
  explicit GpuSharedMemoryPool(GpuSlotBackend & backend);
  ~GpuSharedMemoryPool();

  GpuSharedMemoryPool(const GpuSharedMemoryPool &) = delete;
  GpuSharedMemoryPool & operator=(const GpuSharedMemoryPool &) = delete;

  // Discovers the GPU UUID and pre-allocates every slot described by `config`.
  // On any failure, releases whatever was created and returns false. Call once.
  bool initialize(const PoolConfig & config);

  // Releases all slots. Idempotent; also called by the destructor.
  void shutdown();

  const std::string & gpu_uuid() const { return gpu_uuid_; }
  BackendType backend_type() const { return backend_.backend_type(); }

  // Snapshot of every slot's descriptor (exported handles) for a ListResponse.
  ListResponse make_list_response() const;

  // Reserves the smallest-size-class free slot whose capacity >= `size`, falling
  // through to larger classes if a class is exhausted. Writes the slot id on kOk.
  //   kSizeTooLarge : no size class is large enough (never blocks; unsatisfiable).
  //   kNoFreeSlot   : non_blocking and no fitting slot is free right now.
  // When non_blocking is false and a fitting class exists, blocks until a slot frees.
  Status allocate(std::size_t size, bool non_blocking, std::uint32_t & slot_id_out);

  // Returns a previously allocated slot to the pool. kInvalidSlot if the id is
  // unknown or the slot is not currently allocated.
  Status free_slot(std::uint32_t slot_id);

  // Introspection for tests.
  std::size_t total_slots() const;
  std::size_t free_slot_count() const;

private:
  struct Slot
  {
    std::uint32_t slot_id = 0;
    std::uint32_t size_class_index = 0;
    std::uint64_t slot_size = 0;
    bool allocated = false;
    AllocatedSlotResources resources;
    SlotDescriptor descriptor;  // cached export blobs (constant after init)
  };

  // Index of the smallest size class whose capacity >= size, or size_classes size
  // if none fits. Caller holds mutex_.
  std::size_t first_fitting_class(std::size_t size) const;

  GpuSlotBackend & backend_;
  std::string gpu_uuid_;

  mutable std::mutex mutex_;
  std::condition_variable slot_freed_;

  std::vector<Slot> slots_;                                // indexed by slot_id
  std::vector<std::uint64_t> class_sizes_;                 // capacity per size class, ascending
  std::vector<std::vector<std::uint32_t>> free_by_class_;  // free slot ids per size class
  bool initialized_ = false;
};

}  // namespace agnocast::gpu_shared_memory_daemon
