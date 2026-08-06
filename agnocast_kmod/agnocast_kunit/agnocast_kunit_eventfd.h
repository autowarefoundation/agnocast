/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once

#include <linux/types.h>

// Fake eventfd contexts for the KUnit build.
//
// The module under test acquires an eventfd_ctx from a user-supplied fd and later signals and
// releases it. None of that is reachable from KUnit: the suite runs in-kernel with no fd table to
// install an eventfd into, and the kernel exports no way to create one from a module. So the
// KUNIT_BUILD half of the agnocast_eventfd_* wrappers (see agnocast_internal.h) lands here instead,
// handing out opaque per-fd tokens and counting what the module does with them.
//
// That is enough to assert the two things the real API would otherwise hide: which subscribers a
// publish signals, and whether every destruction path releases the context it took.

// Number of fake descriptors available to tests. Test fds are plain indices into [0, MAX).
#define AGNOCAST_KUNIT_EVENTFD_MAX_FD 256

// An fd outside the table. agnocast_eventfd_get() rejects it, the way the real
// eventfd_ctx_fdget() rejects a descriptor that is not an eventfd.
#define AGNOCAST_KUNIT_EVENTFD_BAD_FD AGNOCAST_KUNIT_EVENTFD_MAX_FD

struct agnocast_kunit_eventfd_slot
{
  uint32_t get_count;
  uint32_t signal_count;
  uint32_t put_count;
};

// Clears every counter. Call at the start of each test: the suite's exit hook tears down leftover
// state from the previous test, and those releases land on these counters too.
void agnocast_kunit_eventfd_reset(void);

// Counters for one fake fd, or NULL if fd is out of range.
const struct agnocast_kunit_eventfd_slot * agnocast_kunit_eventfd_slot_of(int fd);

// Total gets minus total puts. Zero means every context handed out has been released, which is the
// leak check for the subscriber destruction paths.
int64_t agnocast_kunit_eventfd_outstanding(void);
