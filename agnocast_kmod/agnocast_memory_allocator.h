#pragma once

#include <linux/list.h>
#include <linux/types.h>

#define MEMPOOL_NUM 1024

// Default is 8GB, can be overridden by insmod parameter mempool_size_gb
extern int mempool_size_gb;
// Mempool size in bytes (calculated from mempool_size_gb)
extern uint64_t mempool_size_bytes;

struct mapped_pid_entry
{
  pid_t pid;
  struct list_head list;
};

struct mempool_entry
{
  uint64_t addr;
  uint32_t mapped_num;
  struct list_head mapped_pid_head;
};

void init_memory_allocator(void);
struct mempool_entry * assign_memory(const pid_t pid);
int reference_memory(struct mempool_entry * mempool_entry, const pid_t pid);
void free_memory(const pid_t pid);

#ifdef KUNIT_BUILD
void exit_memory_allocator(void);
#endif
