// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "command_handlers.hpp"
#include "memory_allocator.hpp"
#include "metadata_store.hpp"
#include "protocol.h"

#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace agnocast_test
{

inline constexpr size_t kResponseBufferSize = AGNOCAST_DAEMON_SOCKET_BUF_SIZE;

class DaemonHarness
{
public:
  explicit DaemonHarness(
    int mempool_num = MemoryAllocator::kDefaultMempoolNum,
    uint64_t start_addr = MemoryAllocator::kDefaultStartAddr,
    int size_gb = MemoryAllocator::kDefaultSizeGb)
  : allocator_(mempool_num, start_addr, size_gb), handlers_(store_, allocator_)
  {
    if (socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv_) != 0) {
      throw std::runtime_error("socketpair failed");
    }
    int buf = static_cast<int>(kResponseBufferSize);
    setsockopt(sv_[0], SOL_SOCKET, SO_SNDBUF, &buf, sizeof(buf));
    setsockopt(sv_[1], SOL_SOCKET, SO_RCVBUF, &buf, sizeof(buf));
    recv_buf_.reset(new uint8_t[kResponseBufferSize]);
  }

  ~DaemonHarness()
  {
    close(sv_[0]);
    close(sv_[1]);
  }

  DaemonHarness(const DaemonHarness &) = delete;
  DaemonHarness & operator=(const DaemonHarness &) = delete;

  int call(
    uint32_t cmd, pid_t pid, const void * req, uint32_t req_size, void * resp_out = nullptr,
    uint32_t resp_capacity = 0, uint32_t * payload_size = nullptr)
  {
    RequestHeader hdr{cmd, req_size};
    ResponseHeader rhdr{};
    ssize_t recv_n = -1;
    std::thread reader([&]() { recv_n = recv(sv_[1], recv_buf_.get(), kResponseBufferSize, 0); });

    handlers_.dispatch(sv_[0], pid, hdr, req);
    reader.join();

    if (recv_n < static_cast<ssize_t>(sizeof(ResponseHeader))) {
      return -EPROTO;
    }
    std::memcpy(&rhdr, recv_buf_.get(), sizeof(ResponseHeader));
    if (payload_size) *payload_size = rhdr.payload_size;
    if (resp_out && rhdr.payload_size > 0) {
      uint32_t copy_size = std::min(rhdr.payload_size, resp_capacity);
      std::memcpy(resp_out, recv_buf_.get() + sizeof(ResponseHeader), copy_size);
    }
    return rhdr.error_code == 0 ? 0 : -static_cast<int>(rhdr.error_code);
  }

  void disconnect(pid_t pid) { handlers_.on_client_disconnect(pid); }

  static void set_topic(char (&dst)[AGNOCAST_PROTO_TOPIC_NAME_BUFFER_SIZE], const char * src)
  {
    std::strncpy(dst, src, sizeof(dst) - 1);
  }

  static void set_node(char (&dst)[AGNOCAST_PROTO_NODE_NAME_BUFFER_SIZE], const char * src)
  {
    std::strncpy(dst, src, sizeof(dst) - 1);
  }

  static TopicKey topic_key(const char * topic) { return TopicKey{topic, 0}; }

  int add_process(
    pid_t pid, uint64_t * shm_addr = nullptr, uint64_t * shm_size = nullptr,
    bool is_performance_bridge_manager = false)
  {
    AddProcessRequest req{};
    req.is_performance_bridge_manager = is_performance_bridge_manager;
    AddProcessResponse resp{};
    int ret = call(AGNOCAST_CMD_ADD_PROCESS, pid, &req, sizeof(req), &resp, sizeof(resp));
    if (ret == 0) {
      if (shm_addr) *shm_addr = resp.shm_addr;
      if (shm_size) *shm_size = resp.shm_size;
    }
    return ret;
  }

  int add_publisher(
    const char * topic, const char * node, pid_t pid, uint32_t qos_depth, int32_t * publisher_id,
    bool transient_local = false, bool is_bridge = false)
  {
    AddPublisherRequest req{};
    set_topic(req.topic_name, topic);
    set_node(req.node_name, node);
    req.qos_depth = qos_depth;
    req.qos_is_transient_local = transient_local;
    req.is_bridge = is_bridge;
    AddPublisherResponse resp{};
    int ret = call(AGNOCAST_CMD_ADD_PUBLISHER, pid, &req, sizeof(req), &resp, sizeof(resp));
    if (ret == 0 && publisher_id) *publisher_id = resp.publisher_id;
    return ret;
  }

  int add_subscriber(
    const char * topic, const char * node, pid_t pid, uint32_t qos_depth, int32_t * subscriber_id,
    bool transient_local = false, bool reliable = true, bool is_take_sub = false,
    bool ignore_local_publications = false, bool is_bridge = false)
  {
    AddSubscriberRequest req{};
    set_topic(req.topic_name, topic);
    set_node(req.node_name, node);
    req.qos_depth = qos_depth;
    req.qos_is_transient_local = transient_local;
    req.qos_is_reliable = reliable;
    req.is_take_sub = is_take_sub;
    req.ignore_local_publications = ignore_local_publications;
    req.is_bridge = is_bridge;
    AddSubscriberResponse resp{};
    int ret = call(AGNOCAST_CMD_ADD_SUBSCRIBER, pid, &req, sizeof(req), &resp, sizeof(resp));
    if (ret == 0 && subscriber_id) *subscriber_id = resp.subscriber_id;
    return ret;
  }

  int publish(
    const char * topic, int32_t publisher_id, uint64_t addr, PublishMsgResponse * out,
    pid_t caller_pid = 0)
  {
    PublishMsgRequest req{};
    set_topic(req.topic_name, topic);
    req.publisher_id = publisher_id;
    req.msg_virtual_address = addr;
    return call(AGNOCAST_CMD_PUBLISH_MSG, caller_pid, &req, sizeof(req), out, sizeof(*out));
  }

  int receive(const char * topic, int32_t subscriber_id, ReceiveMsgResponse * out, pid_t pid = 0)
  {
    ReceiveMsgRequest req{};
    set_topic(req.topic_name, topic);
    req.subscriber_id = subscriber_id;
    return call(AGNOCAST_CMD_RECEIVE_MSG, pid, &req, sizeof(req), out, sizeof(*out));
  }

  int take(
    const char * topic, int32_t subscriber_id, bool allow_same_message, TakeMsgResponse * out,
    pid_t pid = 0)
  {
    TakeMsgRequest req{};
    set_topic(req.topic_name, topic);
    req.subscriber_id = subscriber_id;
    req.allow_same_message = allow_same_message;
    return call(AGNOCAST_CMD_TAKE_MSG, pid, &req, sizeof(req), out, sizeof(*out));
  }

  int release_sub_ref(const char * topic, int32_t pubsub_id, int64_t entry_id)
  {
    ReleaseSubRefRequest req{};
    set_topic(req.topic_name, topic);
    req.pubsub_id = pubsub_id;
    req.entry_id = entry_id;
    return call(AGNOCAST_CMD_RELEASE_SUB_REF, 0, &req, sizeof(req));
  }

  int topic_entries_num(const char * topic) const
  {
    auto it = store_.topic_map_.find(topic_key(topic));
    if (it == store_.topic_map_.end()) return 0;
    return static_cast<int>(it->second->topic.entries.size());
  }

  bool topic_exists(const char * topic) const
  {
    return store_.topic_map_.count(topic_key(topic)) > 0;
  }

  bool publisher_exists(const char * topic, int32_t id) const
  {
    auto it = store_.topic_map_.find(topic_key(topic));
    if (it == store_.topic_map_.end()) return false;
    return it->second->topic.pub_info_map.count(id) > 0;
  }

  bool subscriber_exists(const char * topic, int32_t id) const
  {
    auto it = store_.topic_map_.find(topic_key(topic));
    if (it == store_.topic_map_.end()) return false;
    return it->second->topic.sub_info_map.count(id) > 0;
  }

  int64_t latest_received_entry_id(const char * topic, int32_t sub_id) const
  {
    auto it = store_.topic_map_.find(topic_key(topic));
    if (it == store_.topic_map_.end()) return -2;
    auto sit = it->second->topic.sub_info_map.find(sub_id);
    if (sit == it->second->topic.sub_info_map.end()) return -2;
    return sit->second.latest_received_entry_id;
  }

  int entry_rc(const char * topic, int64_t entry_id, int32_t sub_id) const
  {
    auto it = store_.topic_map_.find(topic_key(topic));
    if (it == store_.topic_map_.end()) return -1;
    auto eit = it->second->topic.entries.find(entry_id);
    if (eit == it->second->topic.entries.end()) return -1;
    return eit->second.referencing_subscribers.test(static_cast<size_t>(sub_id)) ? 1 : 0;
  }

  int alive_proc_num() const
  {
    int count = 0;
    for (const auto & kv : store_.proc_info_map_) {
      if (!kv.second.exited) count++;
    }
    return count;
  }

  bool process_exists(pid_t pid) const { return store_.proc_info_map_.count(pid) > 0; }

  bool process_exited(pid_t pid) const
  {
    auto it = store_.proc_info_map_.find(pid);
    return it != store_.proc_info_map_.end() && it->second.exited;
  }

  MetadataStore store_;
  MemoryAllocator allocator_;

private:
  CommandHandlers handlers_;
  int sv_[2] = {-1, -1};
  std::unique_ptr<uint8_t[]> recv_buf_;
};

}  // namespace agnocast_test
