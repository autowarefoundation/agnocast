#include "agnocast_memory_allocator.h"

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/spinlock.h>

MODULE_LICENSE("Dual BSD/GPL");

static struct mempool_entry mempool_entries[MEMPOOL_NUM];
static DEFINE_SPINLOCK(mempool_lock);

// Module parameter: mempool size in GB (default: 8GB)
int mempool_size_gb = 8;
module_param(mempool_size_gb, int, 0444);
MODULE_PARM_DESC(mempool_size_gb, "Default mempool size in GB (default: 8)");

uint64_t mempool_size_bytes = 0;

void init_memory_allocator(void)
{
  uint64_t addr = 0x40000000000;

  mempool_size_bytes = (uint64_t)mempool_size_gb * 1024ULL * 1024ULL * 1024ULL;

  pr_info(
    "Agnocast: Initializing memory allocator with pool size: %llu bytes (%d GB)\n",
    mempool_size_bytes, mempool_size_gb);

  for (int i = 0; i < MEMPOOL_NUM; i++) {
    mempool_entries[i].addr = addr;
    mempool_entries[i].mapped_num = 0;
    for (int j = 0; j < MAX_PROCESS_NUM_PER_MEMPOOL; j++) {
      mempool_entries[i].mapped_pids[j] = -1;
    }
    addr += mempool_size_bytes;
  }
}

struct mempool_entry * assign_memory(const pid_t pid)
{
  struct mempool_entry * result = NULL;
  unsigned long flags;

  spin_lock_irqsave(&mempool_lock, flags);
  for (int i = 0; i < MEMPOOL_NUM; i++) {
    if (mempool_entries[i].mapped_num == 0) {
      mempool_entries[i].mapped_num = 1;
      mempool_entries[i].mapped_pids[0] = pid;
      result = &mempool_entries[i];
      break;
    }
  }
  spin_unlock_irqrestore(&mempool_lock, flags);

  return result;
}

int reference_memory(struct mempool_entry * mempool_entry, const pid_t pid)
{
  int ret = 0;
  unsigned long flags;

  spin_lock_irqsave(&mempool_lock, flags);

  if (mempool_entry->mapped_num == MAX_PROCESS_NUM_PER_MEMPOOL) {
    ret = -ENOBUFS;
    goto unlock;
  }

  for (int i = 0; i < mempool_entry->mapped_num; i++) {
    if (mempool_entry->mapped_pids[i] == pid) {
      ret = -EEXIST;
      goto unlock;
    }
  }

  mempool_entry->mapped_pids[mempool_entry->mapped_num] = pid;
  mempool_entry->mapped_num++;

unlock:
  spin_unlock_irqrestore(&mempool_lock, flags);
  return ret;
}

void free_memory(const pid_t pid)
{
  unsigned long flags;

  spin_lock_irqsave(&mempool_lock, flags);
  for (int i = 0; i < MEMPOOL_NUM; i++) {
    for (int j = 0; j < mempool_entries[i].mapped_num; j++) {
      if (mempool_entries[i].mapped_pids[j] == pid) {
        for (int k = j; k < MAX_PROCESS_NUM_PER_MEMPOOL - 1; k++) {
          mempool_entries[i].mapped_pids[k] = mempool_entries[i].mapped_pids[k + 1];
        }
        mempool_entries[i].mapped_pids[MAX_PROCESS_NUM_PER_MEMPOOL - 1] = -1;
        mempool_entries[i].mapped_num--;
        break;
      }
    }
  }
  spin_unlock_irqrestore(&mempool_lock, flags);
}

#ifdef KUNIT_BUILD
void exit_memory_allocator(void)
{
  for (int i = 0; i < MEMPOOL_NUM; i++) {
    mempool_entries[i].mapped_num = 0;
    for (int j = 0; j < MAX_PROCESS_NUM_PER_MEMPOOL; j++) {
      mempool_entries[i].mapped_pids[j] = -1;
    }
  }
}
#endif
