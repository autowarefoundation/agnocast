// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_internal.h"

#ifndef VERSION
#define VERSION "unknown"
#endif

MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(VERSION);

#ifndef KUNIT_BUILD
static int agnocast_init(void)
{
  int ret;

  ret = agnocast_init_device();
  if (ret < 0) return ret;

  ret = agnocast_init_kthread();
  if (ret < 0) {
    agnocast_exit_device();
    return ret;
  }

  ret = agnocast_init_exit_hook();
  if (ret < 0) {
    agnocast_exit_kthread();
    agnocast_exit_device();
    return ret;
  }

  ret = init_memory_allocator();
  if (ret < 0) {
    agnocast_exit_exit_hook();
    agnocast_exit_kthread();
    agnocast_exit_device();
    return ret;
  }

  dev_info(agnocast_device, "Agnocast installed! v%s\n", VERSION);
  return 0;
}

static void agnocast_exit(void)
{
  agnocast_exit_kthread();
  agnocast_exit_exit_hook();

  agnocast_exit_free_data();
  cleanup_memory_allocator();
  dev_info(agnocast_device, "Agnocast removed!\n");
  agnocast_exit_device();
}

module_init(agnocast_init) module_exit(agnocast_exit)
#endif
