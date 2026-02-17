#pragma once

#include <linux/types.h>

// Maximum number of processes that can be mapped to a memory pool
#define MAX_PROCESS_NUM_PER_MEMPOOL 32

#define MEMPOOL_NUM 1024

// Default is 8GB, can be overridden by insmod parameter mempool_size_gb
extern int mempool_size_gb;
// Mempool size in bytes (calculated from mempool_size_gb)
extern uint64_t mempool_size_bytes;

struct mempool_entry
{
  uint64_t addr;
  uint32_t mapped_num;
  pid_t mapped_pids[MAX_PROCESS_NUM_PER_MEMPOOL];
};

void init_memory_allocator(void);
struct mempool_entry * assign_memory(const pid_t pid);
int reference_memory(struct mempool_entry * mempool_entry, const pid_t pid);
void free_memory(const pid_t pid);

#ifdef KUNIT_BUILD
void exit_memory_allocator(void);
#endif
