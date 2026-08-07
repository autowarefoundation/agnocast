/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once

#include <linux/types.h>

// Fake eventfd contexts backing the KUNIT_BUILD half of the agnocast_eventfd_* wrappers. KUnit
// runs in-kernel with no fd table to install an eventfd into, and the kernel exports no way to
// create one from a module, so real contexts are unobtainable; these hand out opaque per-fd tokens
// and count what the module does with them. That is enough to assert what the real API would hide:
// which subscribers a publish signals, and whether every destruction path releases its context.

// Test fds are plain indices into [0, MAX).
#define AGNOCAST_KUNIT_EVENTFD_MAX_FD 256

// Rejected by agnocast_eventfd_get(), as the real eventfd_ctx_fdget() rejects a non-eventfd fd.
#define AGNOCAST_KUNIT_EVENTFD_BAD_FD AGNOCAST_KUNIT_EVENTFD_MAX_FD

struct agnocast_kunit_eventfd_slot
{
  uint32_t get_count;
  uint32_t signal_count;
  uint32_t put_count;
};

// Call at the start of each test: the suite's exit hook tears down the previous test's state, and
// those releases land on these counters too.
void agnocast_kunit_eventfd_reset(void);

// Counters for one fake fd, or NULL if fd is out of range.
const struct agnocast_kunit_eventfd_slot * agnocast_kunit_eventfd_slot_of(int fd);

// Total gets minus total puts; zero means nothing leaked.
int64_t agnocast_kunit_eventfd_outstanding(void);
