// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "protocol.h"

#include <bitset>
#include <cstdint>

// Equivalent of agnocast_kmod::entry_node.
// The red-black tree (struct rb_root) is replaced by std::map<int64_t, EntryNode>
// in TopicStruct, keyed by entry_id.
struct EntryNode
{
  int64_t entry_id = 0;
  int32_t publisher_id = 0;  // topic_local_id_t
  uint64_t msg_virtual_address = 0;

  // Equivalent of DECLARE_BITMAP(referencing_subscribers, MAX_TOPIC_LOCAL_ID).
  // Bit i is set while subscriber i holds a reference to this entry.
  std::bitset<AGNOCAST_PROTO_MAX_TOPIC_LOCAL_ID> referencing_subscribers;

  bool is_referenced() const noexcept { return referencing_subscribers.any(); }
};
