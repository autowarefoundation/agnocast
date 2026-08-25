// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "memory_allocator.hpp"
#include "message_entry.hpp"

#include <sys/types.h>

#include <cstdint>
#include <map>
#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

// ============================================================
// Data structures — userspace equivalents of agnocast_internal.h
// ============================================================

struct TopicKey
{
  std::string name;
  uint32_t domain_id = 0;

  bool operator==(const TopicKey & other) const
  {
    return domain_id == other.domain_id && name == other.name;
  }
};

struct TopicKeyHash
{
  std::size_t operator()(const TopicKey & key) const
  {
    return std::hash<std::string>{}(key.name) ^ (static_cast<std::size_t>(key.domain_id) << 1);
  }
};

struct ExitSubscriptionEntry
{
  std::string topic_name;
  int32_t subscriber_id = 0;  // topic_local_id_t
};

// Equivalent of agnocast_kmod::process_info.
// ipc_namespace is omitted: container isolation is a future concern.
struct ProcessInfo
{
  bool exited = false;
  bool is_performance_bridge_manager = false;
  pid_t pid = 0;
  // The process's ROS_DOMAIN_ID (0 if unset), fixed for the process's lifetime.
  uint32_t domain_id = 0;
  MemoryAllocator::MempoolEntry * mempool_entry = nullptr;  // owned by MemoryAllocator
  std::vector<ExitSubscriptionEntry> exit_subscriptions;
};

// Equivalent of agnocast_kmod::publisher_info.
struct PublisherInfo
{
  int32_t id = 0;  // topic_local_id_t
  uint32_t domain_id = 0;
  pid_t pid = 0;
  std::string node_name;
  uint32_t qos_depth = 0;
  bool qos_is_transient_local = false;
  uint32_t entries_num = 0;
  bool is_bridge = false;
};

// Equivalent of agnocast_kmod::subscriber_info.
struct SubscriberInfo
{
  int32_t id = 0;  // topic_local_id_t
  uint32_t domain_id = 0;
  pid_t pid = 0;
  uint32_t qos_depth = 0;
  bool qos_is_transient_local = false;
  bool qos_is_reliable = false;
  int64_t latest_received_entry_id = -1;
  std::string node_name;
  bool is_take_sub = false;
  bool ignore_local_publications = false;
  bool need_mmap_update = false;
  bool is_bridge = false;
};

// Equivalent of agnocast_kmod::topic_struct.
// The red-black tree (struct rb_root) is replaced by std::map<int64_t, EntryNode>.
struct TopicStruct
{
  std::map<int64_t, EntryNode> entries;  // keyed by entry_id
  std::unordered_map<int32_t, PublisherInfo> pub_info_map;
  std::unordered_map<int32_t, SubscriberInfo> sub_info_map;
  int32_t current_pubsub_id = 0;
  int64_t current_entry_id = 0;
  uint32_t ros2_subscriber_num = 0;
  uint32_t ros2_publisher_num = 0;
};

// Equivalent of agnocast_kmod::topic_wrapper.
// Heap-allocated (via unique_ptr in MetadataStore::topic_map_) because
// std::shared_mutex is neither copyable nor movable.
struct TopicWrapper
{
  std::string key;
  uint32_t domain_id = 0;

  // Lock ordering: global_mutex_ (MetadataStore) → topic_rwsem → mutex_ (MemoryAllocator).
  // shared_lock  for read-only operations within the topic.
  // unique_lock  for publish/subscribe/modify operations.
  mutable std::shared_mutex topic_rwsem;

  TopicStruct topic;

  TopicWrapper() = default;
  TopicWrapper(const TopicWrapper &) = delete;
  TopicWrapper & operator=(const TopicWrapper &) = delete;
  TopicWrapper(TopicWrapper &&) = delete;
  TopicWrapper & operator=(TopicWrapper &&) = delete;
};

// Equivalent of agnocast_kmod::bridge_info.
struct BridgeInfo
{
  std::string topic_name;
  pid_t pid = 0;
  bool has_r2a = false;  // ROS2 → Agnocast
  bool has_a2r = false;  // Agnocast → ROS2
};

// ============================================================
// MetadataStore — owns all runtime state
// ============================================================

// Equivalent of the three global hashtables in agnocast_kmod/agnocast_internal.c
// (topic_hashtable, proc_info_htable, bridge_htable) plus global_htables_rwsem.
//
// Lock ordering (to prevent deadlocks, always acquire in this order):
//   1. global_mutex_  (this class)
//   2. topic_rwsem    (per-topic, in TopicWrapper)
//   3. mutex_         (MemoryAllocator)
class MetadataStore
{
public:
  // shared_lock  when searching / reading without modifying the maps.
  // unique_lock  when inserting or erasing entries from the maps.
  mutable std::shared_mutex global_mutex_;

  std::unordered_map<TopicKey, std::unique_ptr<TopicWrapper>, TopicKeyHash> topic_map_;
  std::unordered_map<pid_t, ProcessInfo> proc_info_map_;
  std::unordered_map<std::string, BridgeInfo> bridge_map_;

  // Caller must hold global_mutex_ (unique_lock) before calling.
  TopicWrapper * find_or_create_topic(const TopicKey & key);

  // Caller must hold global_mutex_ (at least shared_lock) before calling.
  TopicWrapper * find_topic(const TopicKey & key) const;
  TopicWrapper * find_or_create_topic_for_process(pid_t pid, const std::string & topic_name);
  TopicWrapper * find_topic_for_process(pid_t pid, const std::string & topic_name) const;
  uint32_t get_process_domain_id(pid_t pid) const;
  ProcessInfo * find_process(pid_t pid);
};
