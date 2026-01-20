#include "agnocast/agnocast_callback_info.hpp"

#include "agnocast/agnocast.hpp"

namespace agnocast
{

std::mutex id2_callback_info_mtx;
const int callback_map_bkt_cnt = 100;  // arbitrary size to prevent rehash
std::unordered_map<uint32_t, CallbackInfo> id2_callback_info(callback_map_bkt_cnt);
std::atomic<uint32_t> next_callback_info_id;

void receive_message(
  [[maybe_unused]] const uint32_t callback_info_id,  // for CARET
  [[maybe_unused]] const pid_t my_pid,               // for CARET
  const CallbackInfo & callback_info, std::mutex & ready_agnocast_executables_mutex,
  std::vector<AgnocastExecutable> & ready_agnocast_executables)
{
  std::vector<std::pair<int64_t, uint64_t>> entries;  // entry_id, entry_addr

  bool call_again = true;
  while (call_again) {
    union ioctl_receive_msg_args receive_args = {};
    receive_args.topic_name = {callback_info.topic_name.c_str(), callback_info.topic_name.size()};
    receive_args.subscriber_id = callback_info.subscriber_id;

    {
      std::lock_guard<std::mutex> lock(mmap_mtx);

      if (ioctl(agnocast_fd, AGNOCAST_RECEIVE_MSG_CMD, &receive_args) < 0) {
        RCLCPP_ERROR(logger, "AGNOCAST_RECEIVE_MSG_CMD failed: %s", strerror(errno));
        close(agnocast_fd);
        exit(EXIT_FAILURE);
      }

      // Map the shared memory region with read permissions whenever a new publisher is discovered.
      for (uint32_t i = 0; i < receive_args.ret_pub_shm_info.publisher_num; i++) {
        const pid_t pid = receive_args.ret_pub_shm_info.publisher_pids[i];
        const uint64_t addr = receive_args.ret_pub_shm_info.shm_addrs[i];
        const uint64_t size = receive_args.ret_pub_shm_info.shm_sizes[i];
        map_read_only_area(pid, addr, size);
      }
    }

    // Collect entries (oldest first order from ioctl)
    for (uint16_t i = 0; i < receive_args.ret_entry_num; i++) {
      entries.emplace_back(receive_args.ret_entry_ids[i], receive_args.ret_entry_addrs[i]);
    }

    call_again = receive_args.ret_call_again;
  }

  // Process entries from oldest to newest (ioctl returns oldest first)
  for (const auto & entry : entries) {
    const int64_t entry_id = entry.first;
    const uint64_t entry_addr = entry.second;

    const std::shared_ptr<std::function<void()>> callable =
      std::make_shared<std::function<void()>>([callback_info, entry_addr, entry_id]() {
        auto typed_msg = callback_info.message_creator(
          reinterpret_cast<void *>(entry_addr), callback_info.topic_name,
          callback_info.subscriber_id, entry_id);
        callback_info.callback(std::move(*typed_msg));
      });

    {
      constexpr uint8_t PID_SHIFT_BITS = 32;
      uint64_t pid_callback_info_id =
        (static_cast<uint64_t>(my_pid) << PID_SHIFT_BITS) | callback_info_id;
      TRACEPOINT(
        agnocast_create_callable, static_cast<const void *>(callable.get()), entry_id,
        pid_callback_info_id);
    }

    {
      std::lock_guard<std::mutex> ready_lock{ready_agnocast_executables_mutex};
      ready_agnocast_executables.emplace_back(
        AgnocastExecutable{callable, callback_info.callback_group});
    }
  }
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

}  // namespace agnocast
