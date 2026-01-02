#pragma once

#include <linux/sched.h>
#include <sched.h>
#include <sys/syscall.h>
#include <sys/types.h>

struct sched_attr
{
  uint32_t size;

  uint32_t sched_policy;
  uint64_t sched_flags;

  /* SCHED_NORMAL, SCHED_BATCH */
  int32_t sched_nice;

  /* SCHED_FIFO, SCHED_RR */
  uint32_t sched_priority;

  /* SCHED_DEADLINE (nsec) */
  uint64_t sched_runtime;
  uint64_t sched_deadline;
  uint64_t sched_period;
};

int sched_setattr(pid_t pid, const struct sched_attr * attr, unsigned int flags)
{
  return syscall(__NR_sched_setattr, pid, attr, flags);
}
