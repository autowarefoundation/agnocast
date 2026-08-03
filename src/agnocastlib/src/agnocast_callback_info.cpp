#include "agnocast/agnocast_callback_info.hpp"

#include "agnocast/agnocast_epoll_event.hpp"
#include "agnocast/agnocast_executor.hpp"
#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_tracepoint_wrapper.h"

#include <sys/epoll.h>
#include <unistd.h>

#include <array>
#include <stdexcept>
#include <utility>

namespace agnocast
{

std::mutex id2_callback_info_mtx;
const int callback_map_bkt_cnt = 100;  // arbitrary size to prevent rehash
std::unordered_map<uint32_t, CallbackInfo> id2_callback_info(callback_map_bkt_cnt);
std::atomic<uint32_t> next_callback_info_id;

uint32_t allocate_callback_info_id()
{
  const uint32_t callback_info_id = next_callback_info_id.fetch_add(1);
  if (callback_info_id >= MAX_CALLBACK_INFO_ID) {
    throw std::runtime_error("Callback info ID overflow: too many callbacks registered");
  }
  return callback_info_id;
}

void receive_and_execute_message(
  const uint32_t callback_info_id, const pid_t my_pid, const CallbackInfo & callback_info,
  std::mutex & ready_agnocast_executables_mutex,
  std::vector<AgnocastExecutable> & ready_agnocast_executables)
{
  std::vector<std::pair<int64_t, uint64_t>> entries;  // entry_id, entry_addr

  std::array<publisher_shm_info, MAX_PUBLISHER_NUM> pub_shm_infos{};

  union ioctl_receive_msg_args receive_args = {};
  receive_args.topic_name = {callback_info.topic_name.c_str(), callback_info.topic_name.size()};
  receive_args.subscriber_id = callback_info.subscriber_id;
  receive_args.pub_shm_info_addr = reinterpret_cast<uint64_t>(pub_shm_infos.data());
  receive_args.pub_shm_info_size = MAX_PUBLISHER_NUM;

  {
    std::lock_guard<std::mutex> lock(mmap_mtx);

    if (ioctl(agnocast_fd, AGNOCAST_RECEIVE_MSG_CMD, &receive_args) < 0) {
      RCLCPP_ERROR(logger, "AGNOCAST_RECEIVE_MSG_CMD failed: %s", strerror(errno));
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }

    // Map the shared memory region with read permissions whenever a new publisher is discovered.
    for (uint32_t i = 0; i < receive_args.ret_pub_shm_num; i++) {
      const pid_t pid = pub_shm_infos[i].pid;
      const uint64_t addr = pub_shm_infos[i].shm_addr;
      const uint64_t size = pub_shm_infos[i].shm_size;
      map_read_only_area(pid, addr, size);
    }
  }

  // Collect entries (oldest first order from ioctl)
  for (uint16_t i = 0; i < receive_args.ret_entry_num; i++) {
    entries.emplace_back(receive_args.ret_entry_ids[i], receive_args.ret_entry_addrs[i]);
  }

  // Process entries from oldest to newest (ioctl returns oldest first)
  for (const auto & entry : entries) {
    const auto & [entry_id, entry_addr] = entry;
    const void * callback_addr = &entry;  // For CARET

    {
      constexpr uint8_t PID_SHIFT_BITS = 32;
      uint64_t pid_callback_info_id =
        (static_cast<uint64_t>(my_pid) << PID_SHIFT_BITS) | callback_info_id;
      // NOTE: The agnocast_create_callable tracepoint was previously used to associate
      // pid_callback_info_id with callable_addr, as well as to associate callable with entry_addr
      // and entry_id. In the current implementation, however, this information can be obtained when
      // the callback starts and ends, rendering this tracepoint unnecessary. Nevertheless, since
      // the current implementation may change in the future, we are retaining this tracepoint to
      // ensure that CARET can be used without modifying its implementation.
      TRACEPOINT(agnocast_create_callable, callback_addr, entry_id, pid_callback_info_id);
    }

    auto typed_msg = callback_info.message_creator(
      reinterpret_cast<void *>(entry_addr), callback_info.topic_name, callback_info.subscriber_id,
      entry_id);
    // NOTE: agnocast_callable_start should be renamed to agnocast_callback_start. As mentioned
    // earlier, we will not change the name in order to avoid requiring changes to be made to the
    // implementation of CARET.
    TRACEPOINT(agnocast_callable_start, callback_addr);
    callback_info.callback(std::move(*typed_msg));
    // NOTE: agnocast_callable_end should be renamed to agnocast_callback_end. As mentioned earlier,
    // we will not change the name in order to avoid requiring changes to be made to the
    // implementation of CARET.
    TRACEPOINT(agnocast_callable_end, callback_addr);
  }

  // If more entries remain, re-enqueue to allow other callbacks to be scheduled in between.
  if (receive_args.ret_call_again) {
    enqueue_receive_and_execute(
      callback_info_id, my_pid, callback_info, ready_agnocast_executables_mutex,
      ready_agnocast_executables);
  }
}

void enqueue_receive_and_execute(
  const uint32_t callback_info_id, const pid_t my_pid, const CallbackInfo & callback_info,
  std::mutex & ready_agnocast_executables_mutex,
  std::vector<AgnocastExecutable> & ready_agnocast_executables)
{
  auto deferred_callable = std::make_shared<std::function<void()>>(
    [callback_info_id, my_pid, callback_info, &ready_agnocast_executables_mutex,
     &ready_agnocast_executables]() {
      receive_and_execute_message(
        callback_info_id, my_pid, callback_info, ready_agnocast_executables_mutex,
        ready_agnocast_executables);
    });

  std::lock_guard<std::mutex> ready_lock{ready_agnocast_executables_mutex};
  ready_agnocast_executables.emplace_back(
    AgnocastExecutable{deferred_callable, callback_info.callback_group});
}

std::vector<std::string> get_agnocast_topics_by_group(
  const rclcpp::CallbackGroup::SharedPtr & group)
{
  std::vector<std::string> topic_names;

  {
    std::lock_guard<std::mutex> lock(id2_callback_info_mtx);
    for (const auto & [id, callback_info] : id2_callback_info) {
      if (callback_info.callback_group == group) {
        topic_names.push_back(callback_info.topic_name);
      }
    }
  }

  std::sort(topic_names.begin(), topic_names.end());

  return topic_names;
}

void SubscriptionEventHandler::prepare_epoll(
  int epoll_fd, const CallbackGroupValidator & validate_callback_group)
{
  std::lock_guard<std::mutex> lock(id2_callback_info_mtx);

  for (auto & it : id2_callback_info) {
    const uint32_t callback_info_id = it.first;
    CallbackInfo & callback_info = it.second;

    if (!callback_info.need_epoll_update) {
      continue;
    }

    if (!validate_callback_group(callback_info.callback_group)) {
      continue;
    }

    struct epoll_event ev = {};
    ev.events = EPOLLIN;
    ev.data.u64 = pack_epoll_data(EpollEventType::Subscription, callback_info_id);
    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, callback_info.mqdes, &ev) == -1) {
      RCLCPP_ERROR(logger, "epoll_ctl failed: %s", strerror(errno));
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }

    if (callback_info.is_transient_local) {
      agnocast::enqueue_receive_and_execute(
        callback_info_id, my_pid_, callback_info, *ready_agnocast_executables_mutex_,
        *ready_agnocast_executables_);
    }

    callback_info.need_epoll_update = false;
  }
}

void SubscriptionEventHandler::handle(EpollEventLocalID event_local_id)
{
  // Subscription callback event
  const uint32_t callback_info_id = event_local_id;
  CallbackInfo callback_info;

  {
    std::lock_guard<std::mutex> lock(id2_callback_info_mtx);

    const auto it = id2_callback_info.find(callback_info_id);
    if (it == id2_callback_info.end()) {
      // Callback was unregistered (subscription destroyed) - this is normal
      return;
    }

    callback_info = it->second;
  }

  MqMsgAgnocast mq_msg = {};

  // non-blocking
  auto ret =
    mq_receive(callback_info.mqdes, reinterpret_cast<char *>(&mq_msg), sizeof(mq_msg), nullptr);
  if (ret < 0) {
    if (errno != EAGAIN) {
      RCLCPP_ERROR_STREAM(
        logger, "mq_receive failed for topic '"
                  << callback_info.topic_name << "' (subscriber_id=" << callback_info.subscriber_id
                  << "): " << strerror(errno));
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }
    return;
  }

  agnocast::enqueue_receive_and_execute(
    callback_info_id, my_pid_, callback_info, *ready_agnocast_executables_mutex_,
    *ready_agnocast_executables_);
}

}  // namespace agnocast
