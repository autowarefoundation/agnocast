#include "agnocast_memory_allocator.h"

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
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
    INIT_LIST_HEAD(&mempool_entries[i].mapped_pid_head);
    addr += mempool_size_bytes;
  }
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
  for (int i = 0; i < MEMPOOL_NUM; i++) {
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
  for (int i = 0; i < MEMPOOL_NUM; i++) {
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

#ifdef KUNIT_BUILD
void exit_memory_allocator(void)
{
  struct mapped_pid_entry * entry;
  struct mapped_pid_entry * tmp;

  for (int i = 0; i < MEMPOOL_NUM; i++) {
    list_for_each_entry_safe(entry, tmp, &mempool_entries[i].mapped_pid_head, list)
    {
      list_del(&entry->list);
      kfree(entry);
    }
    mempool_entries[i].mapped_num = 0;
  }
}
#endif
