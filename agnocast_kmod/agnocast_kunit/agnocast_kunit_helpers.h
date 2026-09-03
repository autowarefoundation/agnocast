/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once

#include "../agnocast.h"

#include <kunit/test.h>

// Register an application process. Returns the mempool base address.
uint64_t agnocast_kunit_setup_process(
  struct kunit * test, const pid_t pid, const uint32_t domain_id);

// Add a publisher; the process must already be registered. Returns the publisher id.
topic_local_id_t agnocast_kunit_setup_publisher(
  struct kunit * test, const char * topic_name, const char * node_name, const pid_t pid,
  const uint32_t qos_depth, const bool qos_is_transient_local, const bool is_bridge);

// Add a subscriber; the process must already be registered. Returns the subscriber id.
topic_local_id_t agnocast_kunit_setup_subscriber(
  struct kunit * test, const char * topic_name, const char * node_name, const pid_t pid,
  const uint32_t qos_depth, const bool qos_is_transient_local, const bool qos_is_reliable,
  const bool is_take_sub, const bool ignore_local_publications, const bool is_bridge,
  const int eventfd);

// Register a process and add a publisher. Writes the mempool base to *ret_addr when non-NULL.
topic_local_id_t agnocast_kunit_setup_one_publisher(
  struct kunit * test, const char * topic_name, const char * node_name, const pid_t pid,
  const uint32_t qos_depth, const bool qos_is_transient_local, const bool is_bridge,
  const uint32_t domain_id, uint64_t * ret_addr);

// Register a process and add a subscriber.
topic_local_id_t agnocast_kunit_setup_one_subscriber(
  struct kunit * test, const char * topic_name, const char * node_name, const pid_t pid,
  const uint32_t qos_depth, const bool qos_is_transient_local, const bool qos_is_reliable,
  const bool is_take_sub, const bool ignore_local_publications, const bool is_bridge,
  const int eventfd, const uint32_t domain_id);
