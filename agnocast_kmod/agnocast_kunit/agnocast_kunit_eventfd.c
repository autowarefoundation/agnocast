// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_eventfd.h"

#include "../agnocast_internal.h"

// One slot per fake fd. A slot's address doubles as the opaque eventfd_ctx pointer handed to the
// module under test: distinct per fd, non-NULL, and never dereferenced as a real context.
static struct agnocast_kunit_eventfd_slot slots[AGNOCAST_KUNIT_EVENTFD_MAX_FD];

void agnocast_kunit_eventfd_reset(void)
{
  memset(slots, 0, sizeof(slots));
}

const struct agnocast_kunit_eventfd_slot * agnocast_kunit_eventfd_slot_of(int fd)
{
  if (fd < 0 || fd >= AGNOCAST_KUNIT_EVENTFD_MAX_FD) return NULL;
  return &slots[fd];
}

int64_t agnocast_kunit_eventfd_outstanding(void)
{
  int64_t outstanding = 0;
  for (int i = 0; i < AGNOCAST_KUNIT_EVENTFD_MAX_FD; i++) {
    outstanding += (int64_t)slots[i].get_count - (int64_t)slots[i].put_count;
  }
  return outstanding;
}

static struct agnocast_kunit_eventfd_slot * slot_from_ctx(struct eventfd_ctx * ctx)
{
  struct agnocast_kunit_eventfd_slot * slot = (struct agnocast_kunit_eventfd_slot *)ctx;

  // A pointer from outside the table means the module fabricated or corrupted a context, which the
  // real build would turn into a wild eventfd access. Fail loudly rather than count it.
  if (WARN_ON_ONCE(slot < slots || slot >= slots + AGNOCAST_KUNIT_EVENTFD_MAX_FD)) return NULL;

  return slot;
}

struct eventfd_ctx * agnocast_eventfd_get(int fd)
{
  if (fd < 0 || fd >= AGNOCAST_KUNIT_EVENTFD_MAX_FD) return ERR_PTR(-EINVAL);

  slots[fd].get_count++;
  return (struct eventfd_ctx *)&slots[fd];
}

void agnocast_eventfd_signal(struct eventfd_ctx * ctx)
{
  struct agnocast_kunit_eventfd_slot * slot = slot_from_ctx(ctx);

  if (slot) slot->signal_count++;
}

void agnocast_eventfd_put(struct eventfd_ctx * ctx)
{
  struct agnocast_kunit_eventfd_slot * slot = slot_from_ctx(ctx);

  if (slot) slot->put_count++;
}
