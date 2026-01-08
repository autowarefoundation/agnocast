#include "agnocast_kunit_get_process_num.h"

#include "../agnocast.h"
#include "../agnocast_memory_allocator.h"

#include <kunit/test.h>

static pid_t pid_act = 2000;
static const int max_process_num = MEMPOOL_NUM;

void test_case_get_process_num_zero(struct kunit * test)
{
  int count = get_process_num(current->nsproxy->ipc_ns);

  KUNIT_EXPECT_EQ(test, count, 0);
}

void test_case_get_process_num_single(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, get_process_num(current->nsproxy->ipc_ns), 0);

  // Act
  uint64_t local_pid = pid_act++;
  union ioctl_add_process_args args;
  int ret = add_process(local_pid, current->nsproxy->ipc_ns, &args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, get_process_num(current->nsproxy->ipc_ns), 1);
}

void test_case_get_process_num_max(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, get_process_num(current->nsproxy->ipc_ns), 0);

  // Act
  for (int i = 0; i < max_process_num; i++) {
    uint64_t local_pid = pid_act++;
    union ioctl_add_process_args args;
    add_process(local_pid, current->nsproxy->ipc_ns, &args);
  }

  // Assert
  KUNIT_EXPECT_EQ(test, get_process_num(current->nsproxy->ipc_ns), max_process_num);
}
