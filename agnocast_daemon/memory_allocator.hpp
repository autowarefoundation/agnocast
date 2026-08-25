// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <sys/types.h>

#include <cstdint>
#include <mutex>
#include <vector>

// Userspace port of agnocast_kmod/agnocast_memory_allocator.c.
//
// Manages a fixed array of mempool slots.  Each slot is a contiguous virtual
// address region of mempool_size_bytes that a client process maps with
// MAP_FIXED_NOREPLACE.  The allocator tracks which PIDs are mapped to each
// slot; it does NOT call mmap() itself — that is the client's responsibility.
//
// Lock ordering: this mutex must be acquired AFTER MetadataStore::global_mutex_
// and TopicWrapper::topic_rwsem (see metadata_store.hpp).
class MemoryAllocator
{
public:
  static constexpr int kDefaultMempoolNum = 4096;
  static constexpr uint64_t kDefaultStartAddr = 0x40000000000ULL;
  static constexpr int kDefaultSizeGb = 16;

  // Equivalent of agnocast_kmod::mempool_entry.
  struct MempoolEntry
  {
    uint64_t addr = 0;
    uint32_t mapped_num = 0;
    std::vector<pid_t> mapped_pids;
  };

  explicit MemoryAllocator(
    int mempool_num = kDefaultMempoolNum, uint64_t start_addr = kDefaultStartAddr,
    int size_gb = kDefaultSizeGb);

  // Assign a free slot to pid.  Returns nullptr if all slots are occupied.
  // Equivalent of agnocast_kmod::assign_memory().
  MempoolEntry * assign_memory(pid_t pid);

  // Register pid as an additional user of an existing slot.
  // Returns 0 on success, -EEXIST if pid is already mapped, -EINVAL if entry is null.
  // Equivalent of agnocast_kmod::reference_memory().
  int reference_memory(MempoolEntry * entry, pid_t pid);

  // Release the slot mapping for pid.
  // Equivalent of agnocast_kmod::free_memory().
  void free_memory(pid_t pid);

  uint64_t mempool_size_bytes() const noexcept { return mempool_size_bytes_; }
  int mempool_num() const noexcept { return static_cast<int>(entries_.size()); }

private:
  std::mutex mutex_;
  std::vector<MempoolEntry> entries_;
  uint64_t mempool_size_bytes_ = 0;
};
