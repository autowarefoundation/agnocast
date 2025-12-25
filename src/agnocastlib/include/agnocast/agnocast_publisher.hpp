#pragma once

#include "agnocast/agnocast_ioctl.hpp"
#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/agnocast_tracepoint_wrapper.h"
#include "agnocast/agnocast_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fcntl.h>
#include <mqueue.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <mutex>
#include <queue>
#include <thread>

namespace agnocast
{
class Node;

// These are cut out of the class for information hiding.
topic_local_id_t initialize_publisher(
  const std::string & topic_name, const std::string & node_name, const rclcpp::QoS & qos);
union ioctl_publish_msg_args publish_core(
  [[maybe_unused]] const void * publisher_handle, /* for CARET */ const std::string & topic_name,
  const topic_local_id_t publisher_id, const uint64_t msg_virtual_address,
  std::unordered_map<topic_local_id_t, std::tuple<mqd_t, bool>> & opened_mqs);
uint32_t get_subscription_count_core(const std::string & topic_name);
void increment_borrowed_publisher_num();
void decrement_borrowed_publisher_num();

extern int agnocast_fd;
extern "C" uint32_t agnocast_get_borrowed_publisher_num();

struct PublisherOptions
{
  // Currently empty, reserved for future use.
};

template <typename MessageT, typename BridgeRequestPolicy>
class BasicPublisher
{
  topic_local_id_t id_ = -1;
  std::string topic_name_;
  std::unordered_map<topic_local_id_t, std::tuple<mqd_t, bool>> opened_mqs_;
  PublisherOptions options_;

  template <typename NodeT>
  void constructor_impl(NodeT * node, const std::string & topic_name, const rclcpp::QoS & qos)
  {
    topic_name_ = node->get_node_topics_interface()->resolve_topic_name(topic_name);
    id_ = initialize_publisher(topic_name_, node->get_fully_qualified_name(), qos);
    BridgeRequestPolicy::template request_bridge<MessageT>(topic_name_, id_);
  }

public:
  using SharedPtr = std::shared_ptr<BasicPublisher<MessageT, BridgeRequestPolicy>>;

  BasicPublisher(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const PublisherOptions & /*options*/)
  {
    constructor_impl(node, topic_name, qos);

    TRACEPOINT(
      agnocast_publisher_init, static_cast<const void *>(this),
      static_cast<const void *>(
        node->get_node_base_interface()->get_shared_rcl_node_handle().get()),
      topic_name_.c_str(), qos.depth());
  }

  BasicPublisher(agnocast::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
  {
    constructor_impl(node, topic_name, qos);

    // TODO: CARET tracepoint for agnocast::Node
  }

  ~BasicPublisher()
  {
    for (auto & [_, t] : opened_mqs_) {
      mqd_t mq = std::get<0>(t);
      if (mq_close(mq) == -1) {
        RCLCPP_ERROR(logger, "mq_close failed: %s", strerror(errno));
      }
    }

    // NOTE: When a publisher is destroyed, subscribers should unmap its memory, but this is not yet
    // implemented. Since multiple publishers in the same process share a mempool, process-level
    // reference counting in kmod is needed. Leaving memory mapped causes no functional issues, so
    // this is left as future work.
    struct ioctl_remove_publisher_args remove_publisher_args
    {
    };
    remove_publisher_args.topic_name = {topic_name_.c_str(), topic_name_.size()};
    remove_publisher_args.publisher_id = id_;
    if (ioctl(agnocast_fd, AGNOCAST_REMOVE_PUBLISHER_CMD, &remove_publisher_args) < 0) {
      RCLCPP_WARN(logger, "Failed to remove publisher (id=%d) from kernel.", id_);
    }
  }

  ipc_shared_ptr<MessageT> borrow_loaned_message()
  {
    increment_borrowed_publisher_num();
    MessageT * ptr = new MessageT();
    return ipc_shared_ptr<MessageT>(ptr, topic_name_.c_str(), id_);
  }

  void publish(ipc_shared_ptr<MessageT> && message)
  {
    if (!message || topic_name_ != message.get_topic_name()) {
      RCLCPP_ERROR(logger, "Invalid message to publish.");
      close(agnocast_fd);
      exit(EXIT_FAILURE);
    }

    decrement_borrowed_publisher_num();

    const union ioctl_publish_msg_args publish_msg_args =
      publish_core(this, topic_name_, id_, reinterpret_cast<uint64_t>(message.get()), opened_mqs_);

    message.set_entry_id(publish_msg_args.ret_entry_id);

    for (uint32_t i = 0; i < publish_msg_args.ret_released_num; i++) {
      MessageT * release_ptr = reinterpret_cast<MessageT *>(publish_msg_args.ret_released_addrs[i]);
      delete release_ptr;
    }

    message.reset();
  }

  // Note: Currently returns only agnocast core subscribers.
  // We also want to include the ros2 subscriber's number in the future.
  uint32_t get_subscription_count() const { return get_subscription_count_core(topic_name_); }
};

struct AgnocastToRosRequestPolicy;

template <typename MessageT>
using Publisher = agnocast::BasicPublisher<MessageT, agnocast::AgnocastToRosRequestPolicy>;

}  // namespace agnocast
