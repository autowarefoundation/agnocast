// SPDX-License-Identifier: Apache-2.0
#include "metadata_store.hpp"

TopicWrapper * MetadataStore::find_or_create_topic(const TopicKey & key)
{
  auto it = topic_map_.find(key);
  if (it != topic_map_.end()) {
    return it->second.get();
  }
  auto wrapper = std::make_unique<TopicWrapper>();
  wrapper->key = key.name;
  wrapper->domain_id = key.domain_id;
  auto * ptr = wrapper.get();
  topic_map_.emplace(key, std::move(wrapper));
  return ptr;
}

TopicWrapper * MetadataStore::find_topic(const TopicKey & key) const
{
  auto it = topic_map_.find(key);
  return (it != topic_map_.end()) ? it->second.get() : nullptr;
}

TopicWrapper * MetadataStore::find_or_create_topic_for_process(
  pid_t pid, const std::string & topic_name)
{
  return find_or_create_topic(TopicKey{topic_name, get_process_domain_id(pid)});
}

TopicWrapper * MetadataStore::find_topic_for_process(
  pid_t pid, const std::string & topic_name) const
{
  return find_topic(TopicKey{topic_name, get_process_domain_id(pid)});
}

uint32_t MetadataStore::get_process_domain_id(pid_t pid) const
{
  auto it = proc_info_map_.find(pid);
  return (it != proc_info_map_.end()) ? it->second.domain_id : 0;
}

ProcessInfo * MetadataStore::find_process(pid_t pid)
{
  auto it = proc_info_map_.find(pid);
  return (it != proc_info_map_.end()) ? &it->second : nullptr;
}
