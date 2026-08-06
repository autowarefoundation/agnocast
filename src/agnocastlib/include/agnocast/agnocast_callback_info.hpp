#pragma once

#include "agnocast/agnocast_epoll.hpp"
#include "agnocast/agnocast_epoll_update_dispatcher.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/cuda_deferred_release.hpp"
#include "agnocast/cuda_message_tag.hpp"
#include "agnocast/cuda_pool_api.hpp"
#include "agnocast/cuda_stream.hpp"
#include "agnocast/gpu_metadata.hpp"

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <type_traits>

namespace agnocast
{

// Capped slightly below UINT32_MAX to provide a safe margin against
// atomic wrap-around (overflow back to 0) during concurrent fetch_add calls.
constexpr uint32_t MAX_CALLBACK_INFO_ID_SAFETY_MARGIN = 1000;
constexpr uint32_t MAX_CALLBACK_INFO_ID = UINT32_MAX - MAX_CALLBACK_INFO_ID_SAFETY_MARGIN;

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

// Type for message creator function that constructs a type-erased message
// envelope (AnyObject) from a raw shared-memory pointer and its metadata.
using MessageCreator = std::function<std::unique_ptr<AnyObject>(
  void *, const std::string &, const topic_local_id_t, const int64_t)>;

struct CallbackInfo
{
  std::string topic_name;
  topic_local_id_t subscriber_id;
  bool is_transient_local;
  mqd_t mqdes;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  TypeErasedCallback callback;
  MessageCreator message_creator;
  bool need_epoll_update = true;
};

std::vector<std::string> get_agnocast_topics_by_group(
  const rclcpp::CallbackGroup::SharedPtr & group);

// Lock ordering: when acquiring both id2_callback_info_mtx and id2_timer_info_mtx,
// always lock id2_callback_info_mtx first to avoid deadlocks.
extern std::mutex id2_callback_info_mtx;
extern std::unordered_map<uint32_t, CallbackInfo> id2_callback_info;
extern std::atomic<uint32_t> next_callback_info_id;

uint32_t allocate_callback_info_id();

// Creates an ipc_shared_ptr for a subscriber-received message.
// For CUDA messages: resolves the subscriber-local GPU pointer for the message's pool
// slot into control_block->gpu_data_ptr (accessed via ipc_shared_ptr::gpu_data()),
// makes `cuda_stream` wait for the publisher's write, and records the stream so that
// the kernel-side reference release is deferred until this reader's GPU reads finish.
// For non-CUDA messages: simply wraps the pointer.
template <typename MessageT>
agnocast::ipc_shared_ptr<MessageT> create_subscriber_ipc_ptr(
  MessageT * msg, const std::string & topic_name, const topic_local_id_t subscriber_id,
  const int64_t entry_id, const CudaStream & cuda_stream)
{
  if constexpr (is_cuda_message_v<MessageT>) {
    auto * meta = static_cast<GpuMetadata *>(msg->gpu_metadata_);
    if (!meta) {
      RCLCPP_ERROR(
        logger,
        "CUDA message on topic '%s' has null gpu_metadata_. "
        "The publisher may have failed to set GpuMetadata during publish().",
        topic_name.c_str());
      std::abort();
    }

    // Resolve this process's local device pointer for the pool slot. The proxy
    // imported every slot once at startup, so there is no per-message import.
    void * gpu_data_ptr = nullptr;
    if (!agnocast_cuda_ptr_from_slot_id(meta->slot_id, &gpu_data_ptr)) {
      RCLCPP_ERROR(
        logger,
        "CUDA message on topic '%s' references unknown pool slot %u. "
        "Is the GPU pool proxy initialized (pool daemon running for this GPU)?",
        topic_name.c_str(), meta->slot_id);
      std::abort();
    }
    // Install the done-edge primitives. Done here, inside the CUDA branch, so that
    // agnocastlib's own translation units never reference the CUDA C ABI and a
    // non-CUDA executable need not link libagnocast_cuda. set_gpu_done_ops() is
    // idempotent, so doing it per message type is harmless.
    set_gpu_done_ops(GpuDoneOps{
      &agnocast_cuda_record_read_done, &agnocast_cuda_query_read_done,
      &agnocast_cuda_wait_read_done});

    const int stream_kind = cuda_stream.kind_value();
    if (agnocast_cuda_stream_ordering_unsafe(stream_kind, cuda_stream.handle) == 1) {
      RCLCPP_ERROR(
        logger,
        "Subscription on topic '%s' reads CUDA messages on a default CUDA stream, but this "
        "process created a stream with cudaStreamNonBlocking. A default stream does not order "
        "against a non-blocking stream, so the wait for the publisher's write would be a no-op "
        "and reads could observe partially written data. Fix: set "
        "SubscriptionOptions::cuda_stream to the stream the callback reads on.",
        topic_name.c_str());
      std::abort();
    }

    // Order this subscriber's reads on its declared stream after the publisher's
    // write. Because the stream is an explicit handle, this wait means the same thing
    // from any thread, so it can stay on the message-take path even under a
    // multi-threaded executor.
    agnocast_cuda_wait_data_ready(meta->slot_id, stream_kind, cuda_stream.handle);

    auto ipc_ptr = agnocast::ipc_shared_ptr<MessageT>(msg, topic_name, subscriber_id, entry_id);
    ipc_ptr.set_gpu_data_ptr(gpu_data_ptr);
    ipc_ptr.set_gpu_done_stream(stream_kind, cuda_stream.handle);
    return ipc_ptr;
  } else {
    (void)cuda_stream;  // CUDA streams are meaningless for a host-memory message
    return agnocast::ipc_shared_ptr<MessageT>(msg, topic_name, subscriber_id, entry_id);
  }
}

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
  const bool is_transient_local, mqd_t mqdes, rclcpp::CallbackGroup::SharedPtr callback_group,
  const CudaStream & cuda_stream = CudaStream{})
{
  // NOTE: ipc_shared_ptr<MessageT> and ipc_shared_ptr<MessageT>&& make no difference in the
  // assertion expression below, but we go with ipc_shared_ptr<MessageT>&&.
  static_assert(
    std::is_invocable_v<std::decay_t<Func>, agnocast::ipc_shared_ptr<MessageT> &&> ||
      std::is_invocable_v<std::decay_t<Func>, agnocast::ipc_shared_ptr<const MessageT> &&>,
    "Callback must be callable with ipc_shared_ptr<T> or ipc_shared_ptr<const T> (const&, &&, or "
    "by-value)");

  TypeErasedCallback erased_callback = get_erased_callback<MessageT>(std::forward<Func>(callback));

  auto message_creator = [cuda_stream](
                           void * ptr, const std::string & topic_name,
                           const topic_local_id_t subscriber_id, const int64_t entry_id) {
    auto * msg = static_cast<MessageT *>(ptr);
    return std::make_unique<TypedMessagePtr<MessageT>>(
      create_subscriber_ipc_ptr(msg, topic_name, subscriber_id, entry_id, cuda_stream));
  };

  uint32_t callback_info_id = allocate_callback_info_id();

  {
    std::lock_guard<std::mutex> lock(id2_callback_info_mtx);
    id2_callback_info[callback_info_id] = CallbackInfo{
      topic_name,
      subscriber_id,
      is_transient_local,
      mqdes,
      std::move(callback_group),
      std::move(erased_callback),
      std::move(message_creator)};
  }

  EpollUpdateDispatcher::get_instance().request_update_all();

  return callback_info_id;
}

void receive_and_execute_message(
  uint32_t callback_info_id, pid_t my_pid, const CallbackInfo & callback_info,
  std::mutex & ready_agnocast_executables_mutex,
  std::vector<AgnocastExecutable> & ready_agnocast_executables);

void enqueue_receive_and_execute(
  uint32_t callback_info_id, pid_t my_pid, const CallbackInfo & callback_info,
  std::mutex & ready_agnocast_executables_mutex,
  std::vector<AgnocastExecutable> & ready_agnocast_executables);

class SubscriptionEventHandler : public EpollEventHandler
{
  pid_t my_pid_;
  std::mutex * ready_agnocast_executables_mutex_;
  std::vector<AgnocastExecutable> * ready_agnocast_executables_;

public:
  SubscriptionEventHandler(
    const pid_t my_pid, std::mutex * ready_agnocast_executables_mutex,
    std::vector<AgnocastExecutable> * ready_agnocast_executables)
  : my_pid_(my_pid),
    ready_agnocast_executables_mutex_(ready_agnocast_executables_mutex),
    ready_agnocast_executables_(ready_agnocast_executables)
  {
  }

  [[nodiscard]] EpollEventType get_type() const override { return EpollEventType::Subscription; }

  void prepare_epoll(int epoll_fd, const CallbackGroupValidator & validate_callback_group) override;

  void handle(EpollEventLocalID event_local_id) override;
};

}  // namespace agnocast
