#pragma once

#include <linux/list.h>
#include <linux/types.h>

// Default is 4096, can be overridden by insmod parameter mempool_num
extern int mempool_num;
// Default is 0x40000000000, can be overridden by insmod parameter mempool_start_addr
extern unsigned long mempool_start_addr;
// Default is 16GB, can be overridden by insmod parameter mempool_size_gb
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

int init_memory_allocator(void);
void cleanup_memory_allocator(void);
struct mempool_entry * assign_memory(const pid_t pid);
int reference_memory(struct mempool_entry * mempool_entry, const pid_t pid);
void free_memory(const pid_t pid);
void exit_memory_allocator(void);
