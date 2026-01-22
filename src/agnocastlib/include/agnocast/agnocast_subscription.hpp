#pragma once

#include "agnocast/agnocast_callback_info.hpp"
#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_tracepoint_wrapper.h"
#include "agnocast/agnocast_utils.hpp"
#include "rclcpp/detail/qos_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fcntl.h>
#include <mqueue.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace agnocast
{
class Node;

extern std::mutex mmap_mtx;

void map_read_only_area(const pid_t pid, const uint64_t shm_addr, const uint64_t shm_size);

struct SubscriptionOptions
{
  rclcpp::CallbackGroup::SharedPtr callback_group{nullptr};
  bool ignore_local_publications{false};
  rclcpp::QosOverridingOptions qos_overriding_options;
};

// These are cut out of the class for information hiding.
mqd_t open_mq_for_subscription(
  const std::string & topic_name, const topic_local_id_t subscriber_id,
  std::pair<mqd_t, std::string> & mq_subscription);
void remove_mq(const std::pair<mqd_t, std::string> & mq_subscription);

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
  topic_local_id_t id_;
  const std::string topic_name_;
  union ioctl_add_subscriber_args initialize(
    const rclcpp::QoS & qos, const bool is_take_sub, const bool ignore_local_publications,
    const bool is_bridge, const std::string & node_name);

public:
  SubscriptionBase(rclcpp::Node * node, const std::string & topic_name);
  SubscriptionBase(agnocast::Node * node, const std::string & topic_name);

  virtual ~SubscriptionBase()
  {
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
};

template <typename MessageT, typename BridgeRequestPolicy>
class BasicSubscription : public SubscriptionBase
{
  std::pair<mqd_t, std::string> mq_subscription_;

  template <typename NodeT, typename Func>
  uint32_t constructor_impl(
    NodeT * node, const rclcpp::QoS & qos, Func && callback,
    rclcpp::CallbackGroup::SharedPtr callback_group, agnocast::SubscriptionOptions options,
    const bool is_bridge)
  {
    auto node_parameters = node->get_node_parameters_interface();
    const rclcpp::QoS actual_qos =
      options.qos_overriding_options.get_policy_kinds().size()
        ? rclcpp::detail::declare_qos_parameters(
            options.qos_overriding_options, node_parameters, topic_name_, qos,
            rclcpp::detail::SubscriptionQosParametersTraits{})
        : qos;

    validate_qos(actual_qos);

    union ioctl_add_subscriber_args add_subscriber_args = initialize(
      actual_qos, false, options.ignore_local_publications, is_bridge,
      node->get_fully_qualified_name());

    id_ = add_subscriber_args.ret_id;
    BridgeRequestPolicy::template request_bridge<MessageT>(topic_name_, id_);

    mqd_t mq = open_mq_for_subscription(topic_name_, id_, mq_subscription_);

    const bool is_transient_local =
      actual_qos.durability() == rclcpp::DurabilityPolicy::TransientLocal;
    uint32_t callback_info_id = agnocast::register_callback<MessageT>(
      std::forward<Func>(callback), topic_name_, id_, is_transient_local, mq, callback_group);

    return callback_info_id;
  }

public:
  using SharedPtr = std::shared_ptr<BasicSubscription<MessageT, BridgeRequestPolicy>>;

  template <typename Func>
  BasicSubscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
    agnocast::SubscriptionOptions options, const bool is_bridge = false)
  : SubscriptionBase(node, topic_name)
  {
    rclcpp::CallbackGroup::SharedPtr callback_group = get_valid_callback_group(node, options);

    [[maybe_unused]] uint32_t callback_info_id =
      constructor_impl(node, qos, std::forward<Func>(callback), callback_group, options, is_bridge);

    {
      uint64_t pid_callback_info_id = (static_cast<uint64_t>(getpid()) << 32) | callback_info_id;
      TRACEPOINT(
        agnocast_subscription_init, static_cast<const void *>(this),
        static_cast<const void *>(
          node->get_node_base_interface()->get_shared_rcl_node_handle().get()),
        static_cast<const void *>(&callback), static_cast<const void *>(callback_group.get()),
        tracetools::get_symbol(callback), topic_name_.c_str(), qos.depth(), pid_callback_info_id);
    }
  }

  template <typename Func>
  BasicSubscription(
    agnocast::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    Func && callback, agnocast::SubscriptionOptions options)
  : SubscriptionBase(node, topic_name)
  {
    rclcpp::CallbackGroup::SharedPtr callback_group = get_valid_callback_group(node, options);

    [[maybe_unused]] uint32_t callback_info_id =
      constructor_impl(node, qos, std::forward<Func>(callback), callback_group, options, false);

    // TODO(atsushi421): CARET tracepoint for agnocast::Node
  }

  ~BasicSubscription() { remove_mq(mq_subscription_); }
};

template <typename MessageT, typename BridgeRequestPolicy>
class BasicTakeSubscription : public SubscriptionBase
{
public:
  using SharedPtr = std::shared_ptr<BasicTakeSubscription<MessageT, BridgeRequestPolicy>>;

  BasicTakeSubscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions())
  : SubscriptionBase(node, topic_name)
  {
    auto node_parameters = node->get_node_parameters_interface();
    const rclcpp::QoS actual_qos =
      options.qos_overriding_options.get_policy_kinds().size()
        ? rclcpp::detail::declare_qos_parameters(
            options.qos_overriding_options, node_parameters, topic_name_, qos,
            rclcpp::detail::SubscriptionQosParametersTraits{})
        : qos;

    validate_qos(actual_qos);

    {
      auto dummy_cbg = node->get_node_base_interface()->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
      auto dummy_cb = []() {};
      std::string dummy_cb_symbols = "dummy_take" + topic_name;
      TRACEPOINT(
        agnocast_subscription_init, static_cast<const void *>(this),
        static_cast<const void *>(
          node->get_node_base_interface()->get_shared_rcl_node_handle().get()),
        static_cast<const void *>(&dummy_cb), static_cast<const void *>(dummy_cbg.get()),
        dummy_cb_symbols.c_str(), topic_name_.c_str(), actual_qos.depth(), 0);
    }

    union ioctl_add_subscriber_args add_subscriber_args = initialize(
      actual_qos, true, options.ignore_local_publications, false, node->get_fully_qualified_name());

    id_ = add_subscriber_args.ret_id;

    BridgeRequestPolicy::template request_bridge<MessageT>(topic_name_, id_);
  }

  agnocast::ipc_shared_ptr<const MessageT> take(bool allow_same_message = false)
  {
    union ioctl_take_msg_args take_args;
    take_args.topic_name = {topic_name_.c_str(), topic_name_.size()};
    take_args.subscriber_id = id_;
    take_args.allow_same_message = allow_same_message;

    {
      std::lock_guard<std::mutex> lock(mmap_mtx);

      if (ioctl(agnocast_fd, AGNOCAST_TAKE_MSG_CMD, &take_args) < 0) {
        RCLCPP_ERROR(logger, "AGNOCAST_TAKE_MSG_CMD failed: %s", strerror(errno));
        close(agnocast_fd);
        exit(EXIT_FAILURE);
      }

      for (uint32_t i = 0; i < take_args.ret_pub_shm_info.publisher_num; i++) {
        const pid_t pid = take_args.ret_pub_shm_info.publisher_pids[i];
        const uint64_t addr = take_args.ret_pub_shm_info.shm_addrs[i];
        const uint64_t size = take_args.ret_pub_shm_info.shm_sizes[i];
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

    MessageT * ptr = reinterpret_cast<MessageT *>(take_args.ret_addr);
    return agnocast::ipc_shared_ptr<const MessageT>(ptr, topic_name_, id_, take_args.ret_entry_id);
  }
};

// Wrapper of BasicTakeSubscription for Autoware
template <typename MessageT, typename BridgeRequestPolicy>
class BasicPollingSubscriber
{
  typename BasicTakeSubscription<MessageT, BridgeRequestPolicy>::SharedPtr subscriber_;

public:
  using SharedPtr = std::shared_ptr<BasicPollingSubscriber<MessageT, BridgeRequestPolicy>>;

  explicit BasicPollingSubscriber(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1},
    agnocast::SubscriptionOptions options = agnocast::SubscriptionOptions())
  {
    subscriber_ = std::make_shared<BasicTakeSubscription<MessageT, BridgeRequestPolicy>>(
      node, topic_name, qos, options);
  };

  // `takeData()` is remaining for backward compatibility.
  const agnocast::ipc_shared_ptr<const MessageT> takeData() { return subscriber_->take(true); };
  const agnocast::ipc_shared_ptr<const MessageT> take_data() { return subscriber_->take(true); };
};

struct RosToAgnocastRequestPolicy;

template <typename MessageT>
using Subscription = agnocast::BasicSubscription<MessageT, agnocast::RosToAgnocastRequestPolicy>;

template <typename MessageT>
using TakeSubscription =
  agnocast::BasicTakeSubscription<MessageT, agnocast::RosToAgnocastRequestPolicy>;

template <typename MessageT>
using PollingSubscriber =
  agnocast::BasicPollingSubscriber<MessageT, agnocast::RosToAgnocastRequestPolicy>;

}  // namespace agnocast
