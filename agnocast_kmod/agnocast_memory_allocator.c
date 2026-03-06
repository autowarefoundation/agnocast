#include "agnocast_memory_allocator.h"

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

MODULE_LICENSE("Dual BSD/GPL");

static struct mempool_entry * mempool_entries;
static DEFINE_SPINLOCK(mempool_lock);

#define DEFAULT_MEMPOOL_NUM 1024
#define DEFAULT_MEMPOOL_START_ADDR 0x40000000000ULL
#define DEFAULT_MEMPOOL_SIZE_GB 8

// Module parameter: number of memory pools
int mempool_num = DEFAULT_MEMPOOL_NUM;
module_param(mempool_num, int, 0444);
MODULE_PARM_DESC(mempool_num, "Number of memory pools (default: 1024)");

// Module parameter: start address of the allocator
unsigned long mempool_start_addr = DEFAULT_MEMPOOL_START_ADDR;
module_param(mempool_start_addr, ulong, 0444);
MODULE_PARM_DESC(
  mempool_start_addr, "Start address of the memory allocator (default: 0x40000000000)");

// Module parameter: mempool size in GB
int mempool_size_gb = DEFAULT_MEMPOOL_SIZE_GB;
module_param(mempool_size_gb, int, 0444);
MODULE_PARM_DESC(mempool_size_gb, "Default mempool size in GB (default: 8)");

uint64_t mempool_size_bytes = 0;

int init_memory_allocator(void)
{
  uint64_t addr = (uint64_t)mempool_start_addr;

  mempool_size_bytes = (uint64_t)mempool_size_gb * 1024ULL * 1024ULL * 1024ULL;

  mempool_entries = kmalloc_array(mempool_num, sizeof(*mempool_entries), GFP_KERNEL);
  if (!mempool_entries) {
    pr_err("Agnocast: Failed to allocate mempool_entries array\n");
    return -ENOMEM;
  }

  pr_info(
    "Agnocast: Initializing memory allocator with %d pools, pool size: %llu bytes (%d GB), "
    "start address: 0x%llx\n",
    mempool_num, mempool_size_bytes, mempool_size_gb, (uint64_t)mempool_start_addr);

  if (mempool_num != DEFAULT_MEMPOOL_NUM) {
    pr_warn(
      "Agnocast: mempool_num=%d is not the default value (%d). "
      "This parameter is experimental and may be removed or changed in future versions.\n",
      mempool_num, DEFAULT_MEMPOOL_NUM);
  }

  if ((uint64_t)mempool_start_addr != DEFAULT_MEMPOOL_START_ADDR) {
    pr_warn(
      "Agnocast: mempool_start_addr=0x%lx is not the default value (0x%llx). "
      "This parameter is experimental and may be removed or changed in future versions.\n",
      mempool_start_addr, DEFAULT_MEMPOOL_START_ADDR);
  }

  if (mempool_size_gb != DEFAULT_MEMPOOL_SIZE_GB) {
    pr_warn(
      "Agnocast: mempool_size_gb=%d is not the default value (%d). "
      "This parameter is experimental and may be removed or changed in future versions.\n",
      mempool_size_gb, DEFAULT_MEMPOOL_SIZE_GB);
  }

  {
    uint64_t end_addr = addr + (uint64_t)mempool_num * mempool_size_bytes;
    uint64_t limit_85tb = 85ULL * 1024ULL * 1024ULL * 1024ULL * 1024ULL;
    if (end_addr > limit_85tb) {
      pr_warn(
        "Agnocast: memory region end address 0x%llx exceeds 85TB (0x%llx). "
        "This may cause address space conflicts.\n",
        end_addr, limit_85tb);
    }
  }

  for (int i = 0; i < mempool_num; i++) {
    mempool_entries[i].addr = addr;
    mempool_entries[i].mapped_num = 0;
    INIT_LIST_HEAD(&mempool_entries[i].mapped_pid_head);
    addr += mempool_size_bytes;
  }

  return 0;
}

struct mempool_entry * assign_memory(const pid_t pid)
{
  struct mempool_entry * result = NULL;
  struct mapped_pid_entry * new_entry;
  unsigned long flags;

  // Allocate outside spinlock because kmalloc with GFP_KERNEL may sleep
  new_entry = kmalloc(sizeof(*new_entry), GFP_KERNEL);
  if (!new_entry) {
    return NULL;
  }
  new_entry->pid = pid;

  spin_lock_irqsave(&mempool_lock, flags);
  for (int i = 0; i < mempool_num; i++) {
    if (mempool_entries[i].mapped_num == 0) {
      list_add(&new_entry->list, &mempool_entries[i].mapped_pid_head);
      mempool_entries[i].mapped_num = 1;
      result = &mempool_entries[i];
      break;
    }
  }
  spin_unlock_irqrestore(&mempool_lock, flags);

  if (!result) {
    kfree(new_entry);
  }

  return result;
}

int reference_memory(struct mempool_entry * mempool_entry, const pid_t pid)
{
  struct mapped_pid_entry * new_entry;
  struct mapped_pid_entry * entry;
  unsigned long flags;

  // Allocate outside spinlock because kmalloc with GFP_KERNEL may sleep
  new_entry = kmalloc(sizeof(*new_entry), GFP_KERNEL);
  if (!new_entry) {
    return -ENOMEM;
  }
  new_entry->pid = pid;

  spin_lock_irqsave(&mempool_lock, flags);
  list_for_each_entry(entry, &mempool_entry->mapped_pid_head, list)
  {
    if (entry->pid == pid) {
      spin_unlock_irqrestore(&mempool_lock, flags);
      kfree(new_entry);
      return -EEXIST;
    }
  }
  list_add(&new_entry->list, &mempool_entry->mapped_pid_head);
  mempool_entry->mapped_num++;
  spin_unlock_irqrestore(&mempool_lock, flags);

  return 0;
}

void free_memory(const pid_t pid)
{
  struct mapped_pid_entry * entry;
  struct mapped_pid_entry * tmp;
  unsigned long flags;

  spin_lock_irqsave(&mempool_lock, flags);
  for (int i = 0; i < mempool_num; i++) {
    list_for_each_entry_safe(entry, tmp, &mempool_entries[i].mapped_pid_head, list)
    {
      if (entry->pid == pid) {
        list_del(&entry->list);
        kfree(entry);
        mempool_entries[i].mapped_num--;
        break;
      }
    }
  }
  spin_unlock_irqrestore(&mempool_lock, flags);
}

void exit_memory_allocator(void)
{
  struct mapped_pid_entry * entry;
  struct mapped_pid_entry * tmp;

  for (int i = 0; i < mempool_num; i++) {
    list_for_each_entry_safe(entry, tmp, &mempool_entries[i].mapped_pid_head, list)
    {
      list_del(&entry->list);
      kfree(entry);
    }
    mempool_entries[i].mapped_num = 0;
  }
}

void cleanup_memory_allocator(void)
{
  if (!mempool_entries) {
    return;
  }

  exit_memory_allocator();
  kfree(mempool_entries);
  mempool_entries = NULL;
}
