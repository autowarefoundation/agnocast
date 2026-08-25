// SPDX-License-Identifier: Apache-2.0
#include "command_handlers.hpp"

#include <fcntl.h>
#include <mqueue.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <shared_mutex>
#include <string>
#include <vector>

#ifndef AGNOCAST_VERSION
#define AGNOCAST_VERSION "2.3.5"
#endif

namespace
{

SubscriberInfo * find_subscriber_info(TopicWrapper * wrapper, int32_t subscriber_id)
{
  auto it = wrapper->topic.sub_info_map.find(subscriber_id);
  return (it != wrapper->topic.sub_info_map.end()) ? &it->second : nullptr;
}

PublisherInfo * find_publisher_info(TopicWrapper * wrapper, int32_t publisher_id)
{
  auto it = wrapper->topic.pub_info_map.find(publisher_id);
  return (it != wrapper->topic.pub_info_map.end()) ? &it->second : nullptr;
}

// Mirrors agnocastlib create_shm_name / create_mq_name_for_agnocast_publish.
std::string make_shm_name(pid_t pid)
{
  return "/agnocast@" + std::to_string(pid);
}

std::string make_mq_name_for_publish(const std::string & topic_name, int32_t subscriber_id)
{
  if (topic_name.empty() || topic_name[0] != '/') {
    return {};
  }
  std::string mq_name = topic_name;
  mq_name[0] = '@';
  mq_name = "/agnocast" + mq_name + "@" + std::to_string(subscriber_id);
  for (size_t i = 1; i < mq_name.size(); ++i) {
    if (mq_name[i] == '/') {
      mq_name[i] = '_';
    }
  }
  return mq_name;
}

void unlink_posix_resources_for_exit(
  pid_t pid, const std::vector<ExitSubscriptionEntry> & exit_subscriptions)
{
  const std::string shm_name = make_shm_name(pid);
  if (shm_unlink(shm_name.c_str()) < 0 && errno != ENOENT) {
    fprintf(
      stderr, "agnocast_daemon: shm_unlink(%s) failed (pid=%d): %s\n", shm_name.c_str(), pid,
      strerror(errno));
  }

  for (const auto & entry : exit_subscriptions) {
    const std::string mq_name = make_mq_name_for_publish(entry.topic_name, entry.subscriber_id);
    if (mq_name.empty()) continue;
    if (mq_unlink(mq_name.c_str()) < 0 && errno != ENOENT) {
      fprintf(
        stderr, "agnocast_daemon: mq_unlink(%s) failed (pid=%d): %s\n", mq_name.c_str(), pid,
        strerror(errno));
    }
  }
}

EntryNode * find_message_entry(TopicStruct & topic, int64_t entry_id)
{
  auto it = topic.entries.find(entry_id);
  return (it != topic.entries.end()) ? &it->second : nullptr;
}

bool is_referenced(const EntryNode & en)
{
  return en.referencing_subscribers.any();
}

int add_subscriber_reference(EntryNode & en, int32_t id)
{
  if (id < 0 || id >= static_cast<int32_t>(AGNOCAST_PROTO_MAX_TOPIC_LOCAL_ID)) {
    return -EINVAL;
  }
  if (en.referencing_subscribers.test(static_cast<size_t>(id))) {
    return -EALREADY;
  }
  en.referencing_subscribers.set(static_cast<size_t>(id));
  return 0;
}

int insert_subscriber_info(
  TopicWrapper * wrapper, const char * node_name, pid_t subscriber_pid, uint32_t qos_depth,
  bool qos_is_transient_local, bool qos_is_reliable, bool is_take_sub,
  bool ignore_local_publications, bool is_bridge, SubscriberInfo ** out)
{
  if (wrapper->topic.sub_info_map.size() >= AGNOCAST_PROTO_MAX_SUBSCRIBER_NUM) {
    return -ENOBUFS;
  }
  if (wrapper->topic.current_pubsub_id >= static_cast<int32_t>(AGNOCAST_PROTO_MAX_TOPIC_LOCAL_ID)) {
    return -ENOSPC;
  }

  const int32_t new_id = wrapper->topic.current_pubsub_id++;
  SubscriberInfo info;
  info.id = new_id;
  info.domain_id = wrapper->domain_id;
  info.pid = subscriber_pid;
  info.qos_depth = qos_depth;
  info.qos_is_transient_local = qos_is_transient_local;
  info.qos_is_reliable = qos_is_reliable;
  info.latest_received_entry_id =
    qos_is_transient_local ? -1 : static_cast<int64_t>(wrapper->topic.current_entry_id++);
  info.node_name = node_name;
  info.is_take_sub = is_take_sub;
  info.ignore_local_publications = ignore_local_publications;
  info.need_mmap_update = true;
  info.is_bridge = is_bridge;

  auto [it, inserted] = wrapper->topic.sub_info_map.emplace(new_id, std::move(info));
  if (!inserted) return -ENOMEM;
  *out = &it->second;
  return 0;
}

int insert_publisher_info(
  TopicWrapper * wrapper, const char * node_name, pid_t publisher_pid, uint32_t qos_depth,
  bool qos_is_transient_local, bool is_bridge, PublisherInfo ** out)
{
  if (wrapper->topic.pub_info_map.size() >= AGNOCAST_PROTO_MAX_PUBLISHER_NUM) {
    return -ENOBUFS;
  }
  if (wrapper->topic.current_pubsub_id >= static_cast<int32_t>(AGNOCAST_PROTO_MAX_TOPIC_LOCAL_ID)) {
    return -ENOSPC;
  }

  const int32_t new_id = wrapper->topic.current_pubsub_id++;
  PublisherInfo info;
  info.id = new_id;
  info.domain_id = wrapper->domain_id;
  info.pid = publisher_pid;
  info.node_name = node_name;
  info.qos_depth = qos_depth;
  info.qos_is_transient_local = qos_is_transient_local;
  info.entries_num = 0;
  info.is_bridge = is_bridge;

  auto [it, inserted] = wrapper->topic.pub_info_map.emplace(new_id, std::move(info));
  if (!inserted) return -ENOMEM;
  *out = &it->second;
  return 0;
}

int set_publisher_shm_info(
  const TopicWrapper * wrapper, pid_t subscriber_pid, MemoryAllocator & allocator,
  AgnocastPublisherShmInfo * pub_shm_infos, uint32_t pub_shm_infos_size, uint32_t * ret_pub_shm_num,
  MetadataStore & store)
{
  uint32_t publisher_num = 0;
  for (const auto & [id, pub_info] : wrapper->topic.pub_info_map) {
    (void)id;
    if (subscriber_pid == pub_info.pid) continue;

    ProcessInfo * proc_info = store.find_process(pub_info.pid);
    if (!proc_info || proc_info->exited) continue;

    const int ret = allocator.reference_memory(proc_info->mempool_entry, subscriber_pid);
    if (ret < 0) {
      if (ret == -EEXIST) continue;
      return ret;
    }

    if (publisher_num == pub_shm_infos_size) return -ENOBUFS;

    pub_shm_infos[publisher_num].pid = pub_info.pid;
    pub_shm_infos[publisher_num].shm_addr = proc_info->mempool_entry->addr;
    pub_shm_infos[publisher_num].shm_size = allocator.mempool_size_bytes();
    publisher_num++;
  }
  *ret_pub_shm_num = publisher_num;
  return 0;
}

int release_msgs_to_meet_depth(
  TopicWrapper * wrapper, PublisherInfo * pub_info, PublishMsgResponse * resp)
{
  resp->released_num = 0;
  if (pub_info->entries_num <= pub_info->qos_depth) return 0;

  uint32_t num_search_entries = pub_info->entries_num - pub_info->qos_depth;
  auto it = wrapper->topic.entries.begin();

  while (num_search_entries > 0 && resp->released_num < AGNOCAST_PROTO_MAX_RELEASE_NUM) {
    if (it == wrapper->topic.entries.end()) return -ENODATA;

    EntryNode & en = it->second;
    const int64_t entry_id = en.entry_id;
    ++it;

    if (en.publisher_id != pub_info->id) continue;
    num_search_entries--;

    if (is_referenced(en)) continue;

    resp->released_addrs[resp->released_num++] = en.msg_virtual_address;
    wrapper->topic.entries.erase(entry_id);
    pub_info->entries_num--;
  }
  return 0;
}

int receive_msg_core(
  TopicWrapper * wrapper, SubscriberInfo * sub_info, int32_t subscriber_id,
  ReceiveMsgResponse * resp, MetadataStore & store)
{
  resp->entry_num = 0;
  resp->call_again = false;

  if (wrapper->topic.entries.empty()) return 0;

  const int64_t newest_entry_id = wrapper->topic.entries.rbegin()->first;
  const int64_t latest_received_entry_id = sub_info->latest_received_entry_id;
  const int64_t qos_start = newest_entry_id - static_cast<int64_t>(sub_info->qos_depth) + 1;
  const int64_t start_entry_id =
    (qos_start > latest_received_entry_id) ? qos_start : (latest_received_entry_id + 1);

  auto it = wrapper->topic.entries.lower_bound(start_entry_id);
  for (; it != wrapper->topic.entries.end(); ++it) {
    EntryNode & en = it->second;

    if (resp->entry_num == AGNOCAST_PROTO_MAX_RECEIVE_NUM) {
      resp->call_again = true;
      break;
    }

    PublisherInfo * pub_info = find_publisher_info(wrapper, en.publisher_id);
    if (!pub_info) return -ENODATA;

    ProcessInfo * proc_info = store.find_process(pub_info->pid);
    if (!proc_info || proc_info->exited) continue;

    if (sub_info->ignore_local_publications && (sub_info->pid == pub_info->pid)) {
      continue;
    }

    const int ret = add_subscriber_reference(en, subscriber_id);
    if (ret < 0) return ret;

    resp->entry_ids[resp->entry_num] = en.entry_id;
    resp->entry_addrs[resp->entry_num] = en.msg_virtual_address;
    resp->entry_num++;
  }

  if (resp->entry_num > 0) {
    sub_info->latest_received_entry_id = resp->entry_ids[resp->entry_num - 1];
  }
  return 0;
}

void cleanup_unreferenced_entries_from_exited_publishers(
  TopicWrapper * wrapper, int32_t subscriber_id, MetadataStore & store)
{
  std::vector<int64_t> to_remove;
  for (auto & [entry_id, en] : wrapper->topic.entries) {
    (void)entry_id;
    en.referencing_subscribers.reset(static_cast<size_t>(subscriber_id));

    if (is_referenced(en)) continue;

    PublisherInfo * pub_info = find_publisher_info(wrapper, en.publisher_id);
    if (!pub_info) continue;

    ProcessInfo * proc_info = store.find_process(pub_info->pid);
    if (proc_info && !proc_info->exited) continue;

    to_remove.push_back(en.entry_id);
    pub_info->entries_num--;
    if (pub_info->entries_num == 0) {
      wrapper->topic.pub_info_map.erase(pub_info->id);
    }
  }
  for (int64_t eid : to_remove) {
    wrapper->topic.entries.erase(eid);
  }
}

void try_remove_empty_topic(MetadataStore & store, const TopicKey & key)
{
  auto it = store.topic_map_.find(key);
  if (it == store.topic_map_.end()) return;
  TopicWrapper * wrapper = it->second.get();
  if (!wrapper->topic.pub_info_map.empty() || !wrapper->topic.sub_info_map.empty()) return;
  wrapper->topic.entries.clear();
  store.topic_map_.erase(it);
}

const char * notify_mq_topic_name(const TopicWrapper * wrapper)
{
  // Without a domain-bridge rename rule, the canonical MQ topic name equals the wrapper key.
  return wrapper->key.c_str();
}

int get_process_num(const MetadataStore & store)
{
  return static_cast<int>(store.proc_info_map_.size());
}

int get_process_num_in_domain(const MetadataStore & store, uint32_t domain_id)
{
  int count = 0;
  for (const auto & [pid, proc] : store.proc_info_map_) {
    (void)pid;
    if (!proc.exited && proc.domain_id == domain_id) {
      count++;
    }
  }
  return count;
}

bool has_alive_performance_bridge_manager(const MetadataStore & store, uint32_t domain_id)
{
  for (const auto & [pid, proc] : store.proc_info_map_) {
    (void)pid;
    if (proc.domain_id == domain_id && proc.is_performance_bridge_manager && !proc.exited) {
      return true;
    }
  }
  return false;
}

BridgeInfo * find_bridge_info(MetadataStore & store, const std::string & topic_name)
{
  auto it = store.bridge_map_.find(topic_name);
  return (it != store.bridge_map_.end()) ? &it->second : nullptr;
}

}  // namespace

// ============================================================
// CommandHandlers
// ============================================================

CommandHandlers::CommandHandlers(MetadataStore & store, MemoryAllocator & allocator)
: store_(store), allocator_(allocator)
{
}

void CommandHandlers::send_response(int fd, int32_t error_code)
{
  ResponseHeader hdr{error_code, 0};
  send(fd, &hdr, sizeof(hdr), MSG_NOSIGNAL);
}

void CommandHandlers::send_response(
  int fd, int32_t error_code, const void * payload, uint32_t payload_size)
{
  ResponseHeader hdr{error_code, payload_size};
  iovec iov[2];
  iov[0] = {const_cast<ResponseHeader *>(&hdr), sizeof(hdr)};
  iov[1] = {const_cast<void *>(payload), payload_size};
  msghdr msg{};
  msg.msg_iov = iov;
  msg.msg_iovlen = 2;
  sendmsg(fd, &msg, MSG_NOSIGNAL);
}

void CommandHandlers::dispatch(
  int client_fd, pid_t client_pid, const RequestHeader & hdr, const void * payload)
{
  switch (static_cast<CommandType>(hdr.command)) {
    case AGNOCAST_CMD_GET_VERSION:
      handle_get_version(client_fd);
      break;
    case AGNOCAST_CMD_ADD_PROCESS:
      handle_add_process(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_ADD_SUBSCRIBER:
      handle_add_subscriber(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_ADD_PUBLISHER:
      handle_add_publisher(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_RELEASE_SUB_REF:
      handle_release_sub_ref(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_PUBLISH_MSG:
      handle_publish_msg(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_RECEIVE_MSG:
      handle_receive_msg(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_TAKE_MSG:
      handle_take_msg(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_GET_SUBSCRIBER_NUM:
      handle_get_subscriber_num(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_GET_EXIT_PROCESS:
      handle_get_exit_process(client_fd);
      break;
    case AGNOCAST_CMD_GET_SUBSCRIBER_QOS:
      handle_get_subscriber_qos(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_GET_PUBLISHER_QOS:
      handle_get_publisher_qos(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_ADD_BRIDGE:
      handle_add_bridge(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_REMOVE_BRIDGE:
      handle_remove_bridge(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_GET_PUBLISHER_NUM:
      handle_get_publisher_num(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_REMOVE_SUBSCRIBER:
      handle_remove_subscriber(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_REMOVE_PUBLISHER:
      handle_remove_publisher(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_CHECK_AND_REQUEST_BRIDGE_SHUTDOWN:
      handle_check_and_request_bridge_shutdown(client_fd, client_pid);
      break;
    case AGNOCAST_CMD_GET_TOPIC_LIST:
      handle_get_topic_list(client_fd);
      break;
    case AGNOCAST_CMD_GET_TOPIC_SUBSCRIBER_INFO:
      handle_get_topic_subscriber_info(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_GET_TOPIC_PUBLISHER_INFO:
      handle_get_topic_publisher_info(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_GET_NODE_SUBSCRIBER_TOPICS:
      handle_get_node_subscriber_topics(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_GET_NODE_PUBLISHER_TOPICS:
      handle_get_node_publisher_topics(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_SET_ROS2_SUBSCRIBER_NUM:
      handle_set_ros2_subscriber_num(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_SET_ROS2_PUBLISHER_NUM:
      handle_set_ros2_publisher_num(client_fd, client_pid, payload);
      break;
    case AGNOCAST_CMD_NOTIFY_BRIDGE_SHUTDOWN:
      handle_notify_bridge_shutdown(client_fd, client_pid);
      break;
    default:
      send_response(client_fd, EINVAL);
      break;
  }
}

void CommandHandlers::handle_get_version(int fd)
{
  GetVersionResponse resp{};
  strncpy(resp.version, AGNOCAST_VERSION, sizeof(resp.version) - 1);
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_add_process(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const AddProcessRequest *>(payload);

  std::unique_lock glock(store_.global_mutex_);

  if (store_.find_process(pid)) {
    send_response(fd, EINVAL);
    return;
  }

  AddProcessResponse resp{};
  resp.unlink_daemon_exist = (get_process_num(store_) > 0);
  resp.performance_bridge_daemon_exist =
    has_alive_performance_bridge_manager(store_, req->domain_id);
  resp.discovery_agent_exist = false;

  if (req->is_performance_bridge_manager && resp.performance_bridge_daemon_exist) {
    send_response(fd, 0, &resp, sizeof(resp));
    return;
  }

  ProcessInfo proc;
  proc.exited = false;
  proc.is_performance_bridge_manager = req->is_performance_bridge_manager;
  proc.domain_id = req->domain_id;
  proc.pid = pid;
  proc.mempool_entry = allocator_.assign_memory(pid);
  if (!proc.mempool_entry) {
    send_response(fd, ENOMEM);
    return;
  }

  resp.shm_addr = proc.mempool_entry->addr;
  resp.shm_size = allocator_.mempool_size_bytes();

  store_.proc_info_map_.emplace(pid, std::move(proc));
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_add_subscriber(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const AddSubscriberRequest *>(payload);

  std::unique_lock glock(store_.global_mutex_);
  TopicWrapper * wrapper = store_.find_or_create_topic_for_process(pid, req->topic_name);

  SubscriberInfo * sub_info = nullptr;
  const int ret = insert_subscriber_info(
    wrapper, req->node_name, pid, req->qos_depth, req->qos_is_transient_local, req->qos_is_reliable,
    req->is_take_sub, req->ignore_local_publications, req->is_bridge, &sub_info);
  if (ret < 0) {
    send_response(fd, -ret);
    return;
  }

  AddSubscriberResponse resp{};
  resp.subscriber_id = sub_info->id;
  strncpy(
    resp.mq_topic_name, notify_mq_topic_name(wrapper), AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE - 1);
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_add_publisher(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const AddPublisherRequest *>(payload);

  std::unique_lock glock(store_.global_mutex_);
  TopicWrapper * wrapper = store_.find_or_create_topic_for_process(pid, req->topic_name);

  PublisherInfo * pub_info = nullptr;
  const int ret = insert_publisher_info(
    wrapper, req->node_name, pid, req->qos_depth, req->qos_is_transient_local, req->is_bridge,
    &pub_info);
  if (ret < 0) {
    send_response(fd, -ret);
    return;
  }

  for (auto & [id, sub] : wrapper->topic.sub_info_map) {
    (void)id;
    sub.need_mmap_update = true;
  }

  AddPublisherResponse resp{};
  resp.publisher_id = pub_info->id;
  strncpy(
    resp.mq_topic_name, notify_mq_topic_name(wrapper), AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE - 1);
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_release_sub_ref(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const ReleaseSubRefRequest *>(payload);

  std::shared_lock glock(store_.global_mutex_);
  TopicWrapper * wrapper = store_.find_topic_for_process(pid, req->topic_name);
  if (!wrapper) {
    send_response(fd, EINVAL);
    return;
  }

  std::shared_lock tlock(wrapper->topic_rwsem);
  EntryNode * en = find_message_entry(wrapper->topic, req->entry_id);
  if (!en) {
    send_response(fd, EINVAL);
    return;
  }

  if (
    req->pubsub_id < 0 ||
    req->pubsub_id >= static_cast<int32_t>(AGNOCAST_PROTO_MAX_TOPIC_LOCAL_ID)) {
    send_response(fd, EINVAL);
    return;
  }

  if (!en->referencing_subscribers.test(static_cast<size_t>(req->pubsub_id))) {
    send_response(fd, EINVAL);
    return;
  }

  en->referencing_subscribers.reset(static_cast<size_t>(req->pubsub_id));
  send_response(fd, 0);
}

void CommandHandlers::handle_publish_msg(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const PublishMsgRequest *>(payload);

  std::shared_lock glock(store_.global_mutex_);
  TopicWrapper * wrapper = store_.find_topic_for_process(pid, req->topic_name);
  if (!wrapper) {
    send_response(fd, EINVAL);
    return;
  }

  std::unique_lock tlock(wrapper->topic_rwsem);

  PublisherInfo * pub_info = find_publisher_info(wrapper, req->publisher_id);
  if (!pub_info) {
    send_response(fd, EINVAL);
    return;
  }

  ProcessInfo * proc_info = store_.find_process(pub_info->pid);
  if (!proc_info) {
    send_response(fd, EINVAL);
    return;
  }

  const uint64_t mempool_start = proc_info->mempool_entry->addr;
  const uint64_t mempool_end = mempool_start + allocator_.mempool_size_bytes();
  if (req->msg_virtual_address < mempool_start || req->msg_virtual_address >= mempool_end) {
    send_response(fd, EINVAL);
    return;
  }

  PublishMsgResponse resp{};
  EntryNode node;
  node.entry_id = wrapper->topic.current_entry_id++;
  node.publisher_id = pub_info->id;
  node.msg_virtual_address = req->msg_virtual_address;
  resp.entry_id = node.entry_id;
  wrapper->topic.entries.emplace(node.entry_id, std::move(node));
  pub_info->entries_num++;

  int ret = release_msgs_to_meet_depth(wrapper, pub_info, &resp);
  if (ret < 0) {
    send_response(fd, -ret);
    return;
  }

  uint32_t subscriber_num = 0;
  for (const auto & [id, sub] : wrapper->topic.sub_info_map) {
    (void)id;
    if (sub.is_take_sub) continue;
    if (sub.ignore_local_publications && (sub.pid == pub_info->pid)) continue;
    resp.subscriber_ids[subscriber_num++] = sub.id;
  }
  resp.subscriber_num = subscriber_num;

  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_receive_msg(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const ReceiveMsgRequest *>(payload);

  std::shared_lock glock(store_.global_mutex_);
  TopicWrapper * wrapper = store_.find_topic_for_process(pid, req->topic_name);
  if (!wrapper) {
    send_response(fd, EINVAL);
    return;
  }

  std::unique_lock tlock(wrapper->topic_rwsem);

  SubscriberInfo * sub_info = find_subscriber_info(wrapper, req->subscriber_id);
  if (!sub_info) {
    send_response(fd, EINVAL);
    return;
  }

  ReceiveMsgResponse resp{};
  int ret = receive_msg_core(wrapper, sub_info, req->subscriber_id, &resp, store_);
  if (ret < 0) {
    send_response(fd, -ret);
    return;
  }

  if (!sub_info->need_mmap_update) {
    resp.pub_shm_num = 0;
    send_response(fd, 0, &resp, sizeof(resp));
    return;
  }

  ret = set_publisher_shm_info(
    wrapper, sub_info->pid, allocator_, resp.pub_shm_infos, AGNOCAST_PROTO_MAX_PUBLISHER_NUM,
    &resp.pub_shm_num, store_);
  if (ret < 0) {
    send_response(fd, -ret);
    return;
  }
  sub_info->need_mmap_update = false;
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_take_msg(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const TakeMsgRequest *>(payload);

  std::shared_lock glock(store_.global_mutex_);
  TopicWrapper * wrapper = store_.find_topic_for_process(pid, req->topic_name);
  if (!wrapper) {
    send_response(fd, EINVAL);
    return;
  }

  std::unique_lock tlock(wrapper->topic_rwsem);
  SubscriberInfo * sub_info = find_subscriber_info(wrapper, req->subscriber_id);
  if (!sub_info) {
    send_response(fd, EINVAL);
    return;
  }

  TakeMsgResponse resp{};
  resp.addr = 0;
  resp.entry_id = -1;

  uint32_t searched_count = 0;
  EntryNode * candidate_en = nullptr;

  for (auto rit = wrapper->topic.entries.rbegin(); rit != wrapper->topic.entries.rend(); ++rit) {
    EntryNode & en = rit->second;
    if (searched_count >= sub_info->qos_depth) break;

    if (!req->allow_same_message && en.entry_id == sub_info->latest_received_entry_id) {
      break;
    }
    if (en.entry_id < sub_info->latest_received_entry_id) break;

    PublisherInfo * pub_info = find_publisher_info(wrapper, en.publisher_id);
    if (!pub_info) {
      send_response(fd, ENODATA);
      return;
    }

    ProcessInfo * proc_info = store_.find_process(pub_info->pid);
    if (!proc_info || proc_info->exited) continue;

    if (sub_info->ignore_local_publications && (sub_info->pid == pub_info->pid)) continue;

    candidate_en = &en;
    searched_count++;
  }

  if (candidate_en) {
    bool already_referenced = false;
    if (req->allow_same_message) {
      already_referenced =
        candidate_en->referencing_subscribers.test(static_cast<size_t>(req->subscriber_id));
    }
    if (!already_referenced) {
      const int ret = add_subscriber_reference(*candidate_en, req->subscriber_id);
      if (ret < 0) {
        send_response(fd, -ret);
        return;
      }
    }
    resp.addr = candidate_en->msg_virtual_address;
    resp.entry_id = candidate_en->entry_id;
    sub_info->latest_received_entry_id = resp.entry_id;
  }

  if (!sub_info->need_mmap_update) {
    resp.pub_shm_num = 0;
    send_response(fd, 0, &resp, sizeof(resp));
    return;
  }

  const int ret = set_publisher_shm_info(
    wrapper, sub_info->pid, allocator_, resp.pub_shm_infos, AGNOCAST_PROTO_MAX_PUBLISHER_NUM,
    &resp.pub_shm_num, store_);
  if (ret < 0) {
    send_response(fd, -ret);
    return;
  }
  sub_info->need_mmap_update = false;
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_get_subscriber_num(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const GetSubscriberNumRequest *>(payload);

  GetSubscriberNumResponse resp{};

  std::shared_lock glock(store_.global_mutex_);
  TopicWrapper * wrapper = store_.find_topic_for_process(pid, req->topic_name);
  if (!wrapper) {
    send_response(fd, 0, &resp, sizeof(resp));
    return;
  }

  std::shared_lock tlock(wrapper->topic_rwsem);
  for (const auto & [id, sub] : wrapper->topic.sub_info_map) {
    (void)id;
    if (sub.is_bridge) resp.a2r_bridge_exist = true;
    if (sub.pid == pid) {
      resp.same_process_subscriber_num++;
    } else {
      resp.other_process_subscriber_num++;
    }
  }
  for (const auto & [id, pub] : wrapper->topic.pub_info_map) {
    (void)id;
    if (pub.is_bridge) {
      resp.r2a_bridge_exist = true;
      break;
    }
  }
  resp.ros2_subscriber_num = wrapper->topic.ros2_subscriber_num;
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_get_exit_process(int fd)
{
  pid_t global_pid = -1;
  uint32_t committed_count = 0;

  GetExitProcessResponse resp{};
  resp.pid = -1;
  resp.daemon_should_exit = false;

  {
    std::unique_lock glock(store_.global_mutex_);
    for (auto & [pid, proc] : store_.proc_info_map_) {
      if (!proc.exited) continue;

      global_pid = pid;
      resp.pid = pid;

      committed_count = 0;
      for (const auto & entry : proc.exit_subscriptions) {
        if (committed_count >= AGNOCAST_PROTO_MAX_SUBSCRIPTION_NUM_PER_PROCESS) break;
        auto & out = resp.subscription_mq_infos[committed_count++];
        strncpy(
          out.topic_name, entry.topic_name.c_str(), AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE - 1);
        out.subscriber_id = entry.subscriber_id;
      }
      resp.subscription_mq_info_num = committed_count;
      break;
    }
  }

  // Commit phase (mirrors agnocast_commit_exit_process).
  {
    std::unique_lock glock(store_.global_mutex_);
    if (global_pid >= 0) {
      auto it = store_.proc_info_map_.find(global_pid);
      if (it != store_.proc_info_map_.end()) {
        auto & proc = it->second;
        if (committed_count > 0) {
          proc.exit_subscriptions.erase(
            proc.exit_subscriptions.begin(),
            proc.exit_subscriptions.begin() +
              std::min(committed_count, static_cast<uint32_t>(proc.exit_subscriptions.size())));
        }
        if (proc.exit_subscriptions.empty()) {
          store_.proc_info_map_.erase(it);
        }
      }
    }
    resp.daemon_should_exit = store_.proc_info_map_.empty();
  }

  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_get_subscriber_qos(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const GetSubscriberQosRequest *>(payload);

  std::shared_lock glock(store_.global_mutex_);
  TopicWrapper * wrapper = store_.find_topic_for_process(pid, req->topic_name);
  if (!wrapper) {
    send_response(fd, EINVAL);
    return;
  }

  std::shared_lock tlock(wrapper->topic_rwsem);
  SubscriberInfo * sub = find_subscriber_info(wrapper, req->subscriber_id);
  if (!sub) {
    send_response(fd, EINVAL);
    return;
  }

  GetSubscriberQosResponse resp{};
  resp.depth = sub->qos_depth;
  resp.is_transient_local = sub->qos_is_transient_local;
  resp.is_reliable = sub->qos_is_reliable;
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_get_publisher_qos(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const GetPublisherQosRequest *>(payload);

  std::shared_lock glock(store_.global_mutex_);
  TopicWrapper * wrapper = store_.find_topic_for_process(pid, req->topic_name);
  if (!wrapper) {
    send_response(fd, EINVAL);
    return;
  }

  std::shared_lock tlock(wrapper->topic_rwsem);
  PublisherInfo * pub = find_publisher_info(wrapper, req->publisher_id);
  if (!pub) {
    send_response(fd, EINVAL);
    return;
  }

  GetPublisherQosResponse resp{};
  resp.depth = pub->qos_depth;
  resp.is_transient_local = pub->qos_is_transient_local;
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_add_bridge(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const AddBridgeRequest *>(payload);

  std::unique_lock glock(store_.global_mutex_);
  BridgeInfo * existing = find_bridge_info(store_, req->topic_name);

  AddBridgeResponse resp{};
  if (existing) {
    if (existing->pid != pid) {
      resp.pid = existing->pid;
      resp.has_r2a = existing->has_r2a;
      resp.has_a2r = existing->has_a2r;
      send_response(fd, EEXIST, &resp, sizeof(resp));
      return;
    }
    if (req->is_r2a) {
      existing->has_r2a = true;
    } else {
      existing->has_a2r = true;
    }
    resp.pid = existing->pid;
    resp.has_r2a = existing->has_r2a;
    resp.has_a2r = existing->has_a2r;
    send_response(fd, 0, &resp, sizeof(resp));
    return;
  }

  BridgeInfo br;
  br.topic_name = req->topic_name;
  br.pid = pid;
  br.has_r2a = req->is_r2a;
  br.has_a2r = !req->is_r2a;
  resp.pid = pid;
  resp.has_r2a = br.has_r2a;
  resp.has_a2r = br.has_a2r;
  store_.bridge_map_.emplace(req->topic_name, std::move(br));
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_remove_bridge(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const RemoveBridgeRequest *>(payload);

  std::unique_lock glock(store_.global_mutex_);
  BridgeInfo * br = find_bridge_info(store_, req->topic_name);
  if (!br) {
    send_response(fd, ENOENT);
    return;
  }
  if (br->pid != pid) {
    send_response(fd, EPERM);
    return;
  }

  if (req->is_r2a) {
    br->has_r2a = false;
  } else {
    br->has_a2r = false;
  }

  if (!br->has_r2a && !br->has_a2r) {
    store_.bridge_map_.erase(req->topic_name);
  }
  send_response(fd, 0);
}

void CommandHandlers::handle_get_publisher_num(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const GetPublisherNumRequest *>(payload);

  GetPublisherNumResponse resp{};

  std::shared_lock glock(store_.global_mutex_);
  TopicWrapper * wrapper = store_.find_topic_for_process(pid, req->topic_name);
  if (!wrapper) {
    send_response(fd, 0, &resp, sizeof(resp));
    return;
  }

  std::shared_lock tlock(wrapper->topic_rwsem);
  resp.publisher_num = static_cast<uint32_t>(wrapper->topic.pub_info_map.size());
  resp.ros2_publisher_num = wrapper->topic.ros2_publisher_num;
  for (const auto & [id, pub] : wrapper->topic.pub_info_map) {
    (void)id;
    if (pub.is_bridge) {
      resp.r2a_bridge_exist = true;
      break;
    }
  }
  for (const auto & [id, sub] : wrapper->topic.sub_info_map) {
    (void)id;
    if (sub.is_bridge) {
      resp.a2r_bridge_exist = true;
      break;
    }
  }
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_remove_subscriber(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const RemoveSubscriberRequest *>(payload);

  std::unique_lock glock(store_.global_mutex_);
  TopicWrapper * wrapper = store_.find_topic_for_process(pid, req->topic_name);
  if (!wrapper) {
    send_response(fd, EINVAL);
    return;
  }

  std::unique_lock tlock(wrapper->topic_rwsem);
  if (!find_subscriber_info(wrapper, req->subscriber_id)) {
    send_response(fd, ENODATA);
    return;
  }

  wrapper->topic.sub_info_map.erase(req->subscriber_id);
  cleanup_unreferenced_entries_from_exited_publishers(wrapper, req->subscriber_id, store_);
  try_remove_empty_topic(store_, TopicKey{req->topic_name, store_.get_process_domain_id(pid)});
  send_response(fd, 0);
}

void CommandHandlers::handle_remove_publisher(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const RemovePublisherRequest *>(payload);

  std::unique_lock glock(store_.global_mutex_);
  TopicWrapper * wrapper = store_.find_topic_for_process(pid, req->topic_name);
  if (!wrapper) {
    send_response(fd, EINVAL);
    return;
  }

  std::unique_lock tlock(wrapper->topic_rwsem);
  PublisherInfo * pub_info = find_publisher_info(wrapper, req->publisher_id);
  if (!pub_info) {
    send_response(fd, ENODATA);
    return;
  }

  std::vector<int64_t> to_remove;
  for (auto & [entry_id, en] : wrapper->topic.entries) {
    if (en.publisher_id != req->publisher_id) continue;
    if (!is_referenced(en)) {
      to_remove.push_back(entry_id);
    }
  }
  for (int64_t eid : to_remove) {
    wrapper->topic.entries.erase(eid);
    pub_info->entries_num--;
  }

  if (pub_info->entries_num == 0) {
    wrapper->topic.pub_info_map.erase(req->publisher_id);
  }
  try_remove_empty_topic(store_, TopicKey{req->topic_name, store_.get_process_domain_id(pid)});
  send_response(fd, 0);
}

void CommandHandlers::handle_check_and_request_bridge_shutdown(int fd, pid_t pid)
{
  CheckAndRequestBridgeShutdownResponse resp{};

  std::unique_lock glock(store_.global_mutex_);
  ProcessInfo * proc = store_.find_process(pid);
  const uint32_t domain_id = proc ? proc->domain_id : 0;
  if (get_process_num_in_domain(store_, domain_id) <= 1) {
    if (proc) proc->is_performance_bridge_manager = false;
    resp.should_shutdown = true;
  } else {
    resp.should_shutdown = false;
  }
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_get_topic_list(int fd)
{
  GetTopicListResponse resp{};
  resp.topic_num = 0;

  std::shared_lock glock(store_.global_mutex_);
  for (const auto & [key, wrapper] : store_.topic_map_) {
    (void)wrapper;
    if (resp.topic_num >= AGNOCAST_PROTO_MAX_TOPIC_NUM) {
      send_response(fd, ENOBUFS);
      return;
    }
    strncpy(
      resp.topic_names[resp.topic_num], key.name.c_str(),
      AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE - 1);
    resp.topic_num++;
  }
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_get_topic_subscriber_info(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const GetTopicSubscriberInfoRequest *>(payload);

  GetTopicSubscriberInfoResponse resp{};
  resp.entry_num = 0;

  std::shared_lock glock(store_.global_mutex_);
  TopicWrapper * wrapper = store_.find_topic_for_process(pid, req->topic_name);
  if (!wrapper) {
    send_response(fd, 0, &resp, sizeof(resp));
    return;
  }

  std::shared_lock tlock(wrapper->topic_rwsem);
  if (wrapper->topic.sub_info_map.size() > AGNOCAST_PROTO_MAX_SUBSCRIBER_NUM) {
    send_response(fd, ENOBUFS);
    return;
  }

  uint32_t idx = 0;
  for (const auto & [id, sub] : wrapper->topic.sub_info_map) {
    (void)id;
    auto & e = resp.entries[idx++];
    strncpy(e.node_name, sub.node_name.c_str(), AGNOCAST_PROTO_NODE_NAME_BUFFER_SIZE - 1);
    e.qos_depth = sub.qos_depth;
    e.qos_is_transient_local = sub.qos_is_transient_local;
    e.qos_is_reliable = sub.qos_is_reliable;
    e.is_bridge = sub.is_bridge;
  }
  resp.entry_num = idx;
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_get_topic_publisher_info(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const GetTopicPublisherInfoRequest *>(payload);

  GetTopicPublisherInfoResponse resp{};
  resp.entry_num = 0;

  std::shared_lock glock(store_.global_mutex_);
  TopicWrapper * wrapper = store_.find_topic_for_process(pid, req->topic_name);
  if (!wrapper) {
    send_response(fd, 0, &resp, sizeof(resp));
    return;
  }

  std::shared_lock tlock(wrapper->topic_rwsem);
  if (wrapper->topic.pub_info_map.size() > AGNOCAST_PROTO_MAX_PUBLISHER_NUM) {
    send_response(fd, ENOBUFS);
    return;
  }

  uint32_t idx = 0;
  for (const auto & [id, pub] : wrapper->topic.pub_info_map) {
    (void)id;
    auto & e = resp.entries[idx++];
    strncpy(e.node_name, pub.node_name.c_str(), AGNOCAST_PROTO_NODE_NAME_BUFFER_SIZE - 1);
    e.qos_depth = pub.qos_depth;
    e.qos_is_transient_local = pub.qos_is_transient_local;
    e.qos_is_reliable = false;
    e.is_bridge = pub.is_bridge;
  }
  resp.entry_num = idx;
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_get_node_subscriber_topics(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const GetNodeSubscriberTopicsRequest *>(payload);

  GetNodeTopicsResponse resp{};
  resp.topic_num = 0;

  std::shared_lock glock(store_.global_mutex_);
  for (const auto & [key, wrapper_ptr] : store_.topic_map_) {
    TopicWrapper * wrapper = wrapper_ptr.get();
    std::shared_lock tlock(wrapper->topic_rwsem);
    bool found = false;
    for (const auto & [id, sub] : wrapper->topic.sub_info_map) {
      (void)id;
      if (sub.node_name == req->node_name) {
        found = true;
        break;
      }
    }
    tlock.unlock();

    if (!found) continue;
    if (resp.topic_num >= AGNOCAST_PROTO_MAX_TOPIC_NUM) {
      send_response(fd, ENOBUFS);
      return;
    }
    strncpy(
      resp.topic_names[resp.topic_num], key.name.c_str(),
      AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE - 1);
    resp.topic_num++;
  }
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_get_node_publisher_topics(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const GetNodePublisherTopicsRequest *>(payload);

  GetNodeTopicsResponse resp{};
  resp.topic_num = 0;

  std::shared_lock glock(store_.global_mutex_);
  for (const auto & [key, wrapper_ptr] : store_.topic_map_) {
    TopicWrapper * wrapper = wrapper_ptr.get();
    std::shared_lock tlock(wrapper->topic_rwsem);
    bool found = false;
    for (const auto & [id, pub] : wrapper->topic.pub_info_map) {
      (void)id;
      if (pub.node_name == req->node_name) {
        found = true;
        break;
      }
    }
    tlock.unlock();

    if (!found) continue;
    if (resp.topic_num >= AGNOCAST_PROTO_MAX_TOPIC_NUM) {
      send_response(fd, ENOBUFS);
      return;
    }
    strncpy(
      resp.topic_names[resp.topic_num], key.name.c_str(),
      AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE - 1);
    resp.topic_num++;
  }
  send_response(fd, 0, &resp, sizeof(resp));
}

void CommandHandlers::handle_set_ros2_subscriber_num(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const SetRos2SubscriberNumRequest *>(payload);

  std::shared_lock glock(store_.global_mutex_);
  TopicWrapper * wrapper = store_.find_topic_for_process(pid, req->topic_name);
  if (!wrapper) {
    send_response(fd, ENOENT);
    return;
  }

  std::unique_lock tlock(wrapper->topic_rwsem);
  wrapper->topic.ros2_subscriber_num = req->ros2_subscriber_num;
  send_response(fd, 0);
}

void CommandHandlers::handle_set_ros2_publisher_num(int fd, pid_t pid, const void * payload)
{
  if (!payload) {
    send_response(fd, EINVAL);
    return;
  }
  const auto * req = static_cast<const SetRos2PublisherNumRequest *>(payload);

  std::shared_lock glock(store_.global_mutex_);
  TopicWrapper * wrapper = store_.find_topic_for_process(pid, req->topic_name);
  if (!wrapper) {
    send_response(fd, ENOENT);
    return;
  }

  std::unique_lock tlock(wrapper->topic_rwsem);
  wrapper->topic.ros2_publisher_num = req->ros2_publisher_num;
  send_response(fd, 0);
}

void CommandHandlers::handle_notify_bridge_shutdown(int fd, pid_t pid)
{
  std::unique_lock glock(store_.global_mutex_);
  ProcessInfo * proc = store_.find_process(pid);
  if (proc) proc->is_performance_bridge_manager = false;
  send_response(fd, 0);
}

void CommandHandlers::process_exit_cleanup(pid_t pid)
{
  std::vector<ExitSubscriptionEntry> unlink_subscriptions;

  {
    std::unique_lock glock(store_.global_mutex_);

    ProcessInfo * proc_info = store_.find_process(pid);
    if (!proc_info) return;

    proc_info->exited = true;
    allocator_.free_memory(pid);

    std::vector<TopicKey> topics_to_remove;

    for (auto & [key, wrapper_ptr] : store_.topic_map_) {
      TopicWrapper * wrapper = wrapper_ptr.get();
      std::unique_lock tlock(wrapper->topic_rwsem);

      // Publisher exit cleanup
      std::vector<int32_t> pubs_to_remove;
      for (auto & [pub_id, pub_info] : wrapper->topic.pub_info_map) {
        if (pub_info.pid != pid) continue;

        std::vector<int64_t> entries_to_remove;
        for (auto & [entry_id, en] : wrapper->topic.entries) {
          if (en.publisher_id != pub_id) continue;
          if (!is_referenced(en)) entries_to_remove.push_back(entry_id);
        }
        for (int64_t eid : entries_to_remove) {
          wrapper->topic.entries.erase(eid);
          pub_info.entries_num--;
        }
        if (pub_info.entries_num == 0) pubs_to_remove.push_back(pub_id);
      }
      for (int32_t pub_id : pubs_to_remove) {
        wrapper->topic.pub_info_map.erase(pub_id);
      }

      // Subscriber exit cleanup
      std::vector<int32_t> subs_to_remove;
      for (auto & [sub_id, sub_info] : wrapper->topic.sub_info_map) {
        if (sub_info.pid != pid) continue;

        if (
          proc_info->exit_subscriptions.size() < AGNOCAST_PROTO_MAX_SUBSCRIPTION_NUM_PER_PROCESS) {
          ExitSubscriptionEntry entry;
          entry.topic_name = key.name;
          entry.subscriber_id = sub_id;
          proc_info->exit_subscriptions.push_back(std::move(entry));
        }

        subs_to_remove.push_back(sub_id);
      }
      for (int32_t sub_id : subs_to_remove) {
        wrapper->topic.sub_info_map.erase(sub_id);
        cleanup_unreferenced_entries_from_exited_publishers(wrapper, sub_id, store_);
      }

      if (wrapper->topic.pub_info_map.empty() && wrapper->topic.sub_info_map.empty()) {
        wrapper->topic.entries.clear();
        topics_to_remove.push_back(key);
      }
    }

    for (const auto & key : topics_to_remove) {
      store_.topic_map_.erase(key);
    }

    std::vector<std::string> bridges_to_remove;
    for (auto & [name, br] : store_.bridge_map_) {
      if (br.pid == pid) bridges_to_remove.push_back(name);
    }
    for (const auto & name : bridges_to_remove) {
      store_.bridge_map_.erase(name);
    }

    // Snapshot for POSIX unlink after releasing metadata locks.
    unlink_subscriptions = proc_info->exit_subscriptions;
  }

  unlink_posix_resources_for_exit(pid, unlink_subscriptions);
}

void CommandHandlers::on_client_disconnect(pid_t client_pid)
{
  process_exit_cleanup(client_pid);
}
