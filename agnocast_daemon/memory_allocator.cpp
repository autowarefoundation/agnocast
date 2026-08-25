// SPDX-License-Identifier: Apache-2.0
#include "memory_allocator.hpp"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <stdexcept>

MemoryAllocator::MemoryAllocator(int mempool_num, uint64_t start_addr, int size_gb)
{
  if (mempool_num <= 0) {
    throw std::invalid_argument("mempool_num must be positive");
  }
  if (size_gb <= 0) {
    throw std::invalid_argument("mempool_size_gb must be positive");
  }

  mempool_size_bytes_ = static_cast<uint64_t>(size_gb) * 1024ULL * 1024ULL * 1024ULL;

  // Overflow check: mempool_num * mempool_size_bytes
  if (
    mempool_size_bytes_ != 0 &&
    static_cast<uint64_t>(mempool_num) > UINT64_MAX / mempool_size_bytes_) {
    throw std::overflow_error("overflow computing total memory size");
  }

  // Overflow check: start_addr + total_size
  const uint64_t total_size = static_cast<uint64_t>(mempool_num) * mempool_size_bytes_;
  if (start_addr > UINT64_MAX - total_size) {
    throw std::overflow_error("overflow computing end address");
  }

  entries_.resize(mempool_num);
  uint64_t addr = start_addr;
  for (auto & e : entries_) {
    e.addr = addr;
    addr += mempool_size_bytes_;
  }
}

MemoryAllocator::MempoolEntry * MemoryAllocator::assign_memory(pid_t pid)
{
  std::lock_guard lock(mutex_);
  for (auto & e : entries_) {
    if (e.mapped_num == 0) {
      e.mapped_pids.push_back(pid);
      e.mapped_num = 1;
      return &e;
    }
  }
  return nullptr;
}

int MemoryAllocator::reference_memory(MempoolEntry * entry, pid_t pid)
{
  if (!entry) return -EINVAL;

  std::lock_guard lock(mutex_);
  for (pid_t p : entry->mapped_pids) {
    if (p == pid) return -EEXIST;
  }
  entry->mapped_pids.push_back(pid);
  entry->mapped_num++;
  return 0;
}

void MemoryAllocator::free_memory(pid_t pid)
{
  std::lock_guard lock(mutex_);
  for (auto & e : entries_) {
    auto & pids = e.mapped_pids;
    auto it = std::find(pids.begin(), pids.end(), pid);
    if (it != pids.end()) {
      pids.erase(it);
      e.mapped_num--;
      return;
    }
  }
}
