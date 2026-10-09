/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once

#include <linux/list.h>
#include <linux/types.h>

// TODO(bdm-k): complete this doc.
//
// Lifecycle of mempool entries:
//   During kernel module initialization, init_memory_allocator() initializes
//   all the entries. The `addr` field is set to the entry's memory range start,
//   which never changes.
//
//   When a process joins, an empty entry is allocated to it: its PID is added
//   to the `mapped_pid_head` list, and `mapped_num` is set to 1. The process
//   then registers its memfd to set `memf`.

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
  struct file * memf;
};

int init_memory_allocator(void);
void cleanup_memory_allocator(void);
struct mempool_entry * assign_memory(const pid_t pid);
// This function takes ownership of `memf`; the caller must not call fput() on
// it afterward.
int register_memory_file(struct mempool_entry * mempool_entry, struct file * memf);
// On success, returns a borrowed reference to the entry's memory file.
int reference_memory(struct mempool_entry * mempool_entry, const pid_t pid, struct file ** memf);
void free_memory(const pid_t pid);
void exit_memory_allocator(void);
