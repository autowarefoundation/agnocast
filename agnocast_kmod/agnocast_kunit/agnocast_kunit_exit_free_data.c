// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_exit_free_data.h"

#include "../agnocast.h"
#include "agnocast_kunit_eventfd.h"
#include "agnocast_kunit_helpers.h"

#include <kunit/test.h>

static const pid_t PID_BASE = 1000;
static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * NODE_NAME = "/kunit_test_node";
static const uint32_t QOS_DEPTH = 1;
#define QOS_IS_TRANSIENT_LOCAL false
#define QOS_IS_RELIABLE true
#define IS_TAKE_SUB false
#define IGNORE_LOCAL_PUBLICATIONS false
#define IS_BRIDGE false

static void setup_one_subscriber(struct kunit * test, const pid_t pid, const int eventfd)
{
  agnocast_kunit_setup_subscriber(
    test, TOPIC_NAME, NODE_NAME, pid, QOS_DEPTH, QOS_IS_TRANSIENT_LOCAL, QOS_IS_RELIABLE,
    IS_TAKE_SUB, IGNORE_LOCAL_PUBLICATIONS, IS_BRIDGE, eventfd);
}

// Module unload tears topics down with their subscribers still registered, so this is the only
// path on which agnocast_release_topic_wrapper() sees live subscriber_info entries.
void test_case_exit_free_data_releases_notify_context(struct kunit * test)
{
  // Arrange
  agnocast_kunit_eventfd_reset();
  const int subscriber_num = 3;
  agnocast_kunit_setup_process(test, PID_BASE, 0);
  for (int eventfd = 0; eventfd < subscriber_num; eventfd++) {
    setup_one_subscriber(test, PID_BASE, eventfd);
  }
  KUNIT_ASSERT_EQ(test, agnocast_kunit_eventfd_outstanding(), (int64_t)subscriber_num);
  KUNIT_ASSERT_TRUE(test, agnocast_is_in_topic_htable(TOPIC_NAME, current->nsproxy->ipc_ns));

  // Act
  agnocast_exit_free_data();

  // Assert
  KUNIT_EXPECT_EQ(test, agnocast_get_topic_num(current->nsproxy->ipc_ns), 0);
  for (int eventfd = 0; eventfd < subscriber_num; eventfd++) {
    const struct agnocast_kunit_eventfd_slot * slot = agnocast_kunit_eventfd_slot_of(eventfd);
    KUNIT_ASSERT_NOT_NULL(test, slot);
    KUNIT_EXPECT_EQ(test, slot->put_count, 1);
  }
  KUNIT_EXPECT_EQ(test, agnocast_kunit_eventfd_outstanding(), (int64_t)0);
}
