#pragma once

#include "agnocast/agnocast_callback_info.hpp"
#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_public_api.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_tracepoint_wrapper.h"
#include "agnocast/agnocast_utils.hpp"
#include "rclcpp/detail/qos_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

#include <mqueue.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace agnocast
{
class Node;

extern std::mutex mmap_mtx;

void map_read_only_area(const pid_t pid, const uint64_t shm_addr, const uint64_t shm_size);

// Get the default callback group from an agnocast::Node for tracepoint use.
// Defined in .cpp to avoid circular inclusion between agnocast_subscription.hpp and
// agnocast_node.hpp.
rclcpp::CallbackGroup::SharedPtr get_default_callback_group_for_tracepoint(agnocast::Node * node);
const void * get_node_base_address(Node * node);

/**
 * @brief Options for configuring an Agnocast subscription.
 */
AGNOCAST_PUBLIC
struct SubscriptionOptions
{
  /// Callback group for the subscription (nullptr = default group).
  rclcpp::CallbackGroup::SharedPtr callback_group{nullptr};
  /// If true, messages from publishers in the same process are ignored.
  bool ignore_local_publications{false};
  /// QoS parameter override options (same semantics as rclcpp).
  rclcpp::QosOverridingOptions qos_overriding_options{};
};

/**
 * @brief Role of a subscription with respect to the ROS<->Agnocast bridge.
 *
 * Encodes two properties of a subscription:
 *   - whether it is used by the bridge implementation itself
 *   - whether it should issue an R2A bridge request on construction
 *
 *   | Role            | kmod `is_bridge` | bridge request issued |
 *   |-----------------|------------------|-----------------------|
 *   | Default         | false            | yes (R2A)             |
 *   | AgnocastOnly    | false            | no                    |
 *   | BridgeInternal  | true             | no                    |
 */
enum class SubscriptionRole : uint8_t {
  /// User-created subscription; issues an R2A bridge request.
  Default,
  /// Used internally by the service/client implementation; no bridge request.
  /// Not intended for direct use by application code.
  AgnocastOnly,
  /// Used by the bridge implementation itself; marked as bridge in kmod and
  /// issues no bridge request.
  /// Not intended for direct use by application code.
  BridgeInternal,
};

// These are cut out of the class for information hiding.
mqd_t open_mq_for_subscription(
  const std::string & topic_name, const topic_local_id_t subscriber_id,
  std::pair<mqd_t, std::string> & mq_subscription);
void remove_mq(const std::pair<mqd_t, std::string> & mq_subscription);
uint32_t get_publisher_count_core(const std::string & topic_name);

template <typename NodeT>
rclcpp::CallbackGroup::SharedPtr get_valid_callback_group(
  NodeT * node, const SubscriptionOptions & options)
{
  rclcpp::CallbackGroup::SharedPtr callback_group = options.callback_group;

  if (callback_group) {
    if (!node->get_node_base_interface()->callback_group_in_node(callback_group)) {
      RCLCPP_ERROR(logger, "Cannot create agnocast subscription, callback group not in node.");
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }
  } else {
    callback_group = node->get_node_base_interface()->get_default_callback_group();
  }

  return callback_group;
}

class SubscriptionBase
{
protected:
  topic_local_id_t id_{-1};
  const std::string topic_name_;
  void initialize(
    const rclcpp::QoS & qos, const bool is_take_sub, const bool ignore_local_publications,
    SubscriptionRole role, const std::string & node_name, const std::string & type_name);

  template <typename NodeT>
  rclcpp::QoS init_base(
    NodeT * node, const rclcpp::QoS & qos, const std::string & type_name, bool is_take_sub,
    const SubscriptionOptions & options, SubscriptionRole role);

public:
  SubscriptionBase(rclcpp::Node * node, const std::string & topic_name);
  SubscriptionBase(agnocast::Node * node, const std::string & topic_name);

  uint32_t get_publisher_count() const { return get_publisher_count_core(topic_name_); }

  virtual ~SubscriptionBase()
  {
    if (id_ >= 0) {
      // NOTE: Unmapping memory when a subscriber is destroyed is not implemented. Multiple
      // subscribers
      // may share the same mmap region, requiring reference counting in kmod. Since leaving the
      // memory mapped should not cause any functional issues, this is left as future work.
      struct ioctl_remove_subscriber_args remove_subscriber_args
      {
      };
      remove_subscriber_args.topic_name = {topic_name_.c_str(), topic_name_.size()};
      remove_subscriber_args.subscriber_id = id_;
      if (ioctl(agnocast_fd, AGNOCAST_REMOVE_SUBSCRIBER_CMD, &remove_subscriber_args) < 0) {
        RCLCPP_WARN(logger, "Failed to remove subscriber (id=%d) from kernel.", id_);
      }
    }
  }
};

/**
 * @brief Agnocast subscription for a compile-time known message type.
 *
 * Delivers messages via a callback that is invoked each time a publisher
 * writes to the topic. Allocate instances with
 * `agnocast::create_subscription<MessageT>()` or construct directly.
 *
 * @tparam MessageT  ROS message type.
 */
AGNOCAST_PUBLIC
template <typename MessageT>
class Subscription : public SubscriptionBase
{
  std::pair<mqd_t, std::string> mq_subscription_;
  uint32_t callback_info_id_;

  template <typename NodeT, typename Func>
  rclcpp::QoS constructor_impl(
    NodeT * node, const rclcpp::QoS & qos, Func && callback,
    rclcpp::CallbackGroup::SharedPtr callback_group, agnocast::SubscriptionOptions options,
    SubscriptionRole role)
  {
    // Gated to message types — service types pulled in by
    // BasicService<ServiceT> have no rosidl message name. The empty string
    // signals "skip registry" to initialize().
    std::string type_name;
    if constexpr (rosidl_generator_traits::is_message<MessageT>::value) {
      type_name = rosidl_generator_traits::name<MessageT>();
    }

    const rclcpp::QoS actual_qos = init_base(node, qos, type_name, false, options, role);

    mqd_t mq = open_mq_for_subscription(topic_name_, id_, mq_subscription_);

    const bool is_transient_local =
      actual_qos.durability() == rclcpp::DurabilityPolicy::TransientLocal;
    callback_info_id_ = agnocast::register_callback<MessageT>(
      std::forward<Func>(callback), topic_name_, id_, is_transient_local, mq, callback_group);

    return actual_qos;
  }

public:
  using SharedPtr = std::shared_ptr<Subscription<MessageT>>;

  template <typename Func>
  Subscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
    agnocast::SubscriptionOptions options, SubscriptionRole role = SubscriptionRole::Default)
  : SubscriptionBase(node, topic_name)
  {
    rclcpp::CallbackGroup::SharedPtr callback_group = get_valid_callback_group(node, options);

    const void * callback_addr = static_cast<const void *>(&callback);
    const char * callback_symbol = tracetools::get_symbol(callback);

    const rclcpp::QoS actual_qos =
      constructor_impl(node, qos, std::forward<Func>(callback), callback_group, options, role);

    {
      uint64_t pid_callback_info_id = (static_cast<uint64_t>(getpid()) << 32) | callback_info_id_;
      TRACEPOINT(
        agnocast_subscription_init, static_cast<const void *>(this),
        static_cast<const void *>(
          node->get_node_base_interface()->get_shared_rcl_node_handle().get()),
        callback_addr, static_cast<const void *>(callback_group.get()), callback_symbol,
        topic_name_.c_str(), actual_qos.depth(), pid_callback_info_id);
    }
  }

  template <typename Func>
  Subscription(
    agnocast::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    Func && callback, agnocast::SubscriptionOptions options,
    SubscriptionRole role = SubscriptionRole::Default)
  : SubscriptionBase(node, topic_name)
  {
    rclcpp::CallbackGroup::SharedPtr callback_group = get_valid_callback_group(node, options);

    const void * callback_addr = static_cast<const void *>(&callback);
    const char * callback_symbol = tracetools::get_symbol(callback);

    const rclcpp::QoS actual_qos =
      constructor_impl(node, qos, std::forward<Func>(callback), callback_group, options, role);

    {
      uint64_t pid_callback_info_id = (static_cast<uint64_t>(getpid()) << 32) | callback_info_id_;
      TRACEPOINT(
        agnocast_subscription_init, static_cast<const void *>(this),
        static_cast<const void *>(get_node_base_address(node)), callback_addr,
        static_cast<const void *>(callback_group.get()), callback_symbol, topic_name_.c_str(),
        actual_qos.depth(), pid_callback_info_id);
    }
  }

  ~Subscription()
  {
    // Remove from callback info map to prevent stale references on re-subscription and to avoid
    // fd reuse conflicts. When mq_close() is called in remove_mq(), the OS may later reuse the
    // same fd number for a new subscription. If the old entry remains in id2_callback_info,
    // adding the new fd to epoll (EPOLL_CTL_ADD) can fail with EEXIST because epoll still
    // associates that fd number with the stale entry.
    {
      std::lock_guard<std::mutex> lock(id2_callback_info_mtx);
      id2_callback_info.erase(callback_info_id_);
    }
    remove_mq(mq_subscription_);
  }
};

/**
 * @brief Agnocast polling take-subscription for a compile-time known message type.
 *
 * Does not use a callback; the caller retrieves the latest message by calling
 * take(). Use PollingSubscriber<MessageT> for a higher-level wrapper.
 *
 * @tparam MessageT  ROS message type.
 */
AGNOCAST_PUBLIC
template <typename MessageT>
class TakeSubscription : public SubscriptionBase
{
private:
  // Cached pointer from the most recent take(allow_same_message=true) call.
  // When the same entry is returned again, a copy sharing the same control_block is returned
  // so that the kernel-side reference is not released until all userspace copies are destroyed.
  agnocast::ipc_shared_ptr<const MessageT> last_taken_ptr_;
  std::mutex last_taken_ptr_mtx_;

  template <typename NodeT>
  rclcpp::QoS constructor_impl(
    NodeT * node, const rclcpp::QoS & qos, agnocast::SubscriptionOptions options,
    SubscriptionRole role)
  {
    // Gated to message types — service types pulled in by
    // BasicService<ServiceT> have no rosidl message name. The empty string
    // signals "skip registry" to initialize().
    std::string type_name;
    if constexpr (rosidl_generator_traits::is_message<MessageT>::value) {
      type_name = rosidl_generator_traits::name<MessageT>();
    }
    return init_base(node, qos, type_name, true, options, role);
  }

public:
  using SharedPtr = std::shared_ptr<TakeSubscription<MessageT>>;

  TakeSubscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions(),
    SubscriptionRole role = SubscriptionRole::Default)
  : SubscriptionBase(node, topic_name)
  {
    const rclcpp::QoS actual_qos = constructor_impl(node, qos, options, role);

    {
      auto default_cbg = node->get_node_base_interface()->get_default_callback_group();
      auto dummy_cb = []() {};
      std::string dummy_cb_symbols = "dummy_take" + topic_name_;
      TRACEPOINT(
        agnocast_subscription_init, static_cast<const void *>(this),
        static_cast<const void *>(
          node->get_node_base_interface()->get_shared_rcl_node_handle().get()),
        static_cast<const void *>(&dummy_cb), static_cast<const void *>(default_cbg.get()),
        dummy_cb_symbols.c_str(), topic_name_.c_str(), actual_qos.depth(), 0);
    }
  }

  TakeSubscription(
    agnocast::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions(),
    SubscriptionRole role = SubscriptionRole::Default)
  : SubscriptionBase(node, topic_name)
  {
    const rclcpp::QoS actual_qos = constructor_impl(node, qos, options, role);

    {
      auto default_cbg = get_default_callback_group_for_tracepoint(node);
      auto dummy_cb = []() {};
      std::string dummy_cb_symbols = "dummy_take" + topic_name_;
      TRACEPOINT(
        agnocast_subscription_init, static_cast<const void *>(this),
        static_cast<const void *>(get_node_base_address(node)),
        static_cast<const void *>(&dummy_cb), static_cast<const void *>(default_cbg.get()),
        dummy_cb_symbols.c_str(), topic_name_.c_str(), actual_qos.depth(), 0);
    }
  }

  /**
   * @brief Retrieve the latest message from the topic.
   * @param allow_same_message  If true, may return the same message as the previous call
   *                            (useful for always having the latest value). If false, returns
   *                            only new messages since the last take.
   * @return Shared pointer to the message, or empty if unavailable.
   */
  AGNOCAST_PUBLIC
  agnocast::ipc_shared_ptr<const MessageT> take(bool allow_same_message = false)
  {
    publisher_shm_info pub_shm_infos[MAX_PUBLISHER_NUM]{};

    union ioctl_take_msg_args take_args;
    take_args.topic_name = {topic_name_.c_str(), topic_name_.size()};
    take_args.subscriber_id = id_;
    take_args.allow_same_message = allow_same_message;
    take_args.pub_shm_info_addr = reinterpret_cast<uint64_t>(pub_shm_infos);
    take_args.pub_shm_info_size = MAX_PUBLISHER_NUM;

    {
      std::lock_guard<std::mutex> lock(mmap_mtx);

      if (ioctl(agnocast_fd, AGNOCAST_TAKE_MSG_CMD, &take_args) < 0) {
        RCLCPP_ERROR(logger, "AGNOCAST_TAKE_MSG_CMD failed: %s", strerror(errno));
        close(agnocast_fd);
        exit(EXIT_FAILURE);
      }

      for (uint32_t i = 0; i < take_args.ret_pub_shm_num; i++) {
        const pid_t pid = pub_shm_infos[i].pid;
        const uint64_t addr = pub_shm_infos[i].shm_addr;
        const uint64_t size = pub_shm_infos[i].shm_size;
        map_read_only_area(pid, addr, size);
      }
    }

    if (take_args.ret_addr == 0) {
      TRACEPOINT(agnocast_take, static_cast<void *>(this), 0, 0);
      return agnocast::ipc_shared_ptr<const MessageT>();
    }

    TRACEPOINT(
      agnocast_take, static_cast<void *>(this), reinterpret_cast<void *>(take_args.ret_addr),
      take_args.ret_entry_id);

    if (allow_same_message) {
      // Declared outside the lock scope so that its destructor (which may call ioctl to release
      // the kernel reference) runs after the mutex is released, avoiding unnecessary contention.
      agnocast::ipc_shared_ptr<const MessageT> old_ptr;
      {
        std::lock_guard<std::mutex> lock(last_taken_ptr_mtx_);

        // When the kernel returned the same entry as last time, return a copy of the cached
        // pointer (sharing the same control_block) instead of creating a new one.
        // This keeps the kernel-side reference alive until all copies are destroyed.
        if (last_taken_ptr_ && last_taken_ptr_.get_entry_id() == take_args.ret_entry_id) {
          return last_taken_ptr_;
        }

        MessageT * ptr = reinterpret_cast<MessageT *>(take_args.ret_addr);
        auto result =
          agnocast::ipc_shared_ptr<const MessageT>(ptr, topic_name_, id_, take_args.ret_entry_id);
        old_ptr = std::move(last_taken_ptr_);
        last_taken_ptr_ = result;
        return result;
      }
    }

    MessageT * ptr = reinterpret_cast<MessageT *>(take_args.ret_addr);
    return agnocast::ipc_shared_ptr<const MessageT>(ptr, topic_name_, id_, take_args.ret_entry_id);
  }
};

/**
 * @brief Agnocast polling subscriber for a compile-time known message type.
 *
 * Wraps TakeSubscription<MessageT> and exposes a simple take_data() API
 * that always returns the most recent message (or an empty pointer if nothing
 * has been published yet).
 *
 * @tparam MessageT  ROS message type.
 */
AGNOCAST_PUBLIC
template <typename MessageT>
class PollingSubscriber
{
  typename TakeSubscription<MessageT>::SharedPtr subscriber_;

public:
  using SharedPtr = std::shared_ptr<PollingSubscriber<MessageT>>;

  explicit PollingSubscriber(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1},
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions(),
    SubscriptionRole role = SubscriptionRole::Default)
  {
    subscriber_ =
      std::make_shared<TakeSubscription<MessageT>>(node, topic_name, qos, options, role);
  };

  explicit PollingSubscriber(
    agnocast::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1},
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions(),
    SubscriptionRole role = SubscriptionRole::Default)
  {
    subscriber_ =
      std::make_shared<TakeSubscription<MessageT>>(node, topic_name, qos, options, role);
  };

  /// @deprecated Use take_data() instead.
  const agnocast::ipc_shared_ptr<const MessageT> takeData() { return subscriber_->take(true); };
  /// @brief Retrieve the latest message. Always returns the most recent message even if already
  /// retrieved. Returns an empty pointer if no message has been published yet.
  /// @return Shared pointer to the latest message.
  AGNOCAST_PUBLIC
  const agnocast::ipc_shared_ptr<const MessageT> take_data() { return subscriber_->take(true); };
};

/// @brief Mirrors `rclcpp::GenericSubscription` semantics: the topic type is supplied
/// as a runtime string (e.g. "std_msgs/msg/String") rather than a compile-time
/// template argument. The typesupport library is loaded eagerly in the
/// constructor and held for the subscription's lifetime.
///
/// Messages are delivered to the callback as serialized data, outside of Agnocast shared memory.
///
/// The supported callback signatures are:
///   - `void(std::shared_ptr<rclcpp::SerializedMessage>)` (and `const` / `const`-T variants)
///   - `void(std::unique_ptr<rclcpp::SerializedMessage>)` (and `const` / `const`-T variants)
///   - `void(rclcpp::SerializedMessage &)` (and `const` variants)
AGNOCAST_PUBLIC
class GenericSubscription : public SubscriptionBase
{
  std::pair<mqd_t, std::string> mq_subscription_;
  uint32_t callback_info_id_;
  /// Keeps the dynamically loaded typesupport .so and its handle together for our lifetime.
  TypeSupportBundle type_support_;

  static TypeSupportBundle load_typesupport_impl(const std::string & topic_type);

  template <typename NodeT>
  rclcpp::QoS constructor_impl(
    NodeT * node, const std::string & topic_type, const rclcpp::QoS & qos,
    TypeErasedCallback callback, rclcpp::CallbackGroup::SharedPtr callback_group,
    const agnocast::SubscriptionOptions & options, SubscriptionRole role);

public:
  using SharedPtr = std::shared_ptr<GenericSubscription>;

  template <typename Func>
  GenericSubscription(
    rclcpp::Node * node, const std::string & topic_name, const std::string & topic_type,
    const rclcpp::QoS & qos, Func && callback,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions(),
    SubscriptionRole role = SubscriptionRole::Default)
  : SubscriptionBase(node, topic_name)
  {
    rclcpp::CallbackGroup::SharedPtr callback_group = get_valid_callback_group(node, options);

    const void * callback_addr = static_cast<const void *>(&callback);
    const char * callback_symbol = tracetools::get_symbol(callback);

    type_support_ = load_typesupport_impl(topic_type);
    TypeErasedCallback erased =
      get_erased_generic_callback(std::forward<Func>(callback), type_support_);

    const rclcpp::QoS actual_qos =
      constructor_impl(node, topic_type, qos, std::move(erased), callback_group, options, role);

    {
      uint64_t pid_callback_info_id = (static_cast<uint64_t>(getpid()) << 32) | callback_info_id_;
      TRACEPOINT(
        agnocast_subscription_init, static_cast<const void *>(this),
        static_cast<const void *>(
          node->get_node_base_interface()->get_shared_rcl_node_handle().get()),
        callback_addr, static_cast<const void *>(callback_group.get()), callback_symbol,
        topic_name_.c_str(), actual_qos.depth(), pid_callback_info_id);
    }
  }

  template <typename Func>
  GenericSubscription(
    agnocast::Node * node, const std::string & topic_name, const std::string & topic_type,
    const rclcpp::QoS & qos, Func && callback,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions(),
    SubscriptionRole role = SubscriptionRole::Default)
  : SubscriptionBase(node, topic_name)
  {
    rclcpp::CallbackGroup::SharedPtr callback_group = get_valid_callback_group(node, options);

    const void * callback_addr = static_cast<const void *>(&callback);
    const char * callback_symbol = tracetools::get_symbol(callback);

    type_support_ = load_typesupport_impl(topic_type);
    TypeErasedCallback erased =
      get_erased_generic_callback(std::forward<Func>(callback), type_support_);

    const rclcpp::QoS actual_qos =
      constructor_impl(node, topic_type, qos, std::move(erased), callback_group, options, role);

    {
      uint64_t pid_callback_info_id = (static_cast<uint64_t>(getpid()) << 32) | callback_info_id_;
      TRACEPOINT(
        agnocast_subscription_init, static_cast<const void *>(this),
        static_cast<const void *>(get_node_base_address(node)), callback_addr,
        static_cast<const void *>(callback_group.get()), callback_symbol, topic_name_.c_str(),
        actual_qos.depth(), pid_callback_info_id);
    }
  }

  // Destructor defined in .cpp so that ~shared_ptr<rcpputils::SharedLibrary>
  // (held inside TypeSupportBundle) sees the complete SharedLibrary type
  // (forward-declared in this header via agnocast_callback_info.hpp).
  ~GenericSubscription();
};

}  // namespace agnocast
