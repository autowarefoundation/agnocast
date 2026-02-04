#pragma once

#include "agnocast/agnocast_smart_pointer.hpp"

#include <mutex>
#include <type_traits>

namespace agnocast
{

struct AgnocastExecutable;

// Base class for a type-erased object
class AnyObject
{
public:
  virtual ~AnyObject() = default;
  virtual const std::type_info & type() const = 0;
};

// Class for a specific message type
template <typename T>
class TypedMessagePtr : public AnyObject
{
  agnocast::ipc_shared_ptr<T> ptr_;

public:
  explicit TypedMessagePtr(agnocast::ipc_shared_ptr<T> p) : ptr_(std::move(p)) {}

  const std::type_info & type() const override { return typeid(T); }

  agnocast::ipc_shared_ptr<T> && get() && { return std::move(ptr_); }
};

// Type for type-erased callback function
using TypeErasedCallback = std::function<void(AnyObject &&)>;

struct CallbackInfo
{
  std::string topic_name;
  topic_local_id_t subscriber_id;
  bool is_transient_local;
  mqd_t mqdes;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  TypeErasedCallback callback;
  std::function<std::unique_ptr<AnyObject>(
    const void *, const std::string &, const topic_local_id_t, const uint64_t)>
    message_creator;
  bool need_epoll_update = true;
};

std::vector<std::string> get_agnocast_topics_by_group(
  const rclcpp::CallbackGroup::SharedPtr & group);

extern std::mutex id2_callback_info_mtx;
extern std::unordered_map<uint32_t, CallbackInfo> id2_callback_info;
extern std::atomic<uint32_t> next_callback_info_id;
extern std::atomic<bool> need_epoll_updates;

template <typename T, typename Func>
TypeErasedCallback get_erased_callback(Func && callback)
{
  return [callback = std::forward<Func>(callback)](AnyObject && arg) {
    if (typeid(T) == arg.type()) {
      auto && typed_arg = static_cast<TypedMessagePtr<T> &&>(arg);
      callback(std::move(typed_arg).get());
    } else {
      RCLCPP_ERROR(
        logger, "Agnocast internal implementation error: bad allocation when callback is called");
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }
  };
}

template <typename MessageT, typename Func>
uint32_t register_callback(
  Func && callback, const std::string & topic_name, const topic_local_id_t subscriber_id,
  const bool is_transient_local, mqd_t mqdes, const rclcpp::CallbackGroup::SharedPtr callback_group)
{
  // NOTE: ipc_shared_ptr<MessageT> and ipc_shared_ptr<MessageT>&& make no difference in the
  // assertion expression below, but we go with ipc_shared_ptr<MessageT>&&.
  static_assert(
    std::is_invocable_v<std::decay_t<Func>, agnocast::ipc_shared_ptr<MessageT> &&> ||
      std::is_invocable_v<std::decay_t<Func>, agnocast::ipc_shared_ptr<const MessageT> &&>,
    "Callback must be callable with ipc_shared_ptr<T> or ipc_shared_ptr<const T> (const&, &&, or "
    "by-value)");

  TypeErasedCallback erased_callback = get_erased_callback<MessageT>(std::forward<Func>(callback));

  auto message_creator = [](
                           const void * ptr, const std::string & topic_name,
                           const topic_local_id_t subscriber_id, const int64_t entry_id) {
    return std::make_unique<TypedMessagePtr<MessageT>>(agnocast::ipc_shared_ptr<MessageT>(
      const_cast<MessageT *>(static_cast<const MessageT *>(ptr)), topic_name, subscriber_id,
      entry_id));
  };

  uint32_t callback_info_id = next_callback_info_id.fetch_add(1);

  {
    std::lock_guard<std::mutex> lock(id2_callback_info_mtx);
    id2_callback_info[callback_info_id] =
      CallbackInfo{topic_name,     subscriber_id,   is_transient_local, mqdes,
                   callback_group, erased_callback, message_creator};
  }

  need_epoll_updates.store(true);

  return callback_info_id;
}

void receive_message(
  [[maybe_unused]] const uint32_t callback_info_id,  // for CARET
  [[maybe_unused]] const pid_t my_pid,               // for CARET
  const CallbackInfo & callback_info, std::mutex & ready_agnocast_executables_mutex,
  std::vector<AgnocastExecutable> & ready_agnocast_executables);

}  // namespace agnocast
