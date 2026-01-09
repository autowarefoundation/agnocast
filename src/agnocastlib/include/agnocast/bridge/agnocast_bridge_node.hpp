#pragma once

#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include <dlfcn.h>
#include <fcntl.h>
#include <mqueue.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace agnocast

{

static constexpr size_t DEFAULT_QOS_DEPTH = 10;

template <typename MessageT>
void send_bridge_request(
  const std::string & topic_name, topic_local_id_t id, BridgeDirection direction);
template <typename MessageT>
void send_performance_bridge_request(
  const std::string & topic_name, topic_local_id_t id, BridgeDirection direction);

template <typename MessageT>
void request_bridge_core(
  const std::string & topic_name, topic_local_id_t id, BridgeDirection direction)
{
  auto bridge_mode = get_bridge_mode();
  if (bridge_mode == BridgeMode::Standard) {
    send_bridge_request<MessageT>(topic_name, id, direction);
  } else if (bridge_mode == BridgeMode::Performance) {
    send_performance_bridge_request<MessageT>(topic_name, id, direction);
  }
}

// Policy for agnocast::Subscription.
// Requests a bridge that forwards messages from ROS 2 to Agnocast (R2A).
struct RosToAgnocastRequestPolicy
{
  template <typename MessageT>
  static void request_bridge(const std::string & topic_name, topic_local_id_t id)
  {
    request_bridge_core<MessageT>(topic_name, id, BridgeDirection::ROS2_TO_AGNOCAST);
  }
};

// Policy for agnocast::Publisher.
// Requests a bridge that forwards messages from Agnocast to ROS 2 (A2R).
struct AgnocastToRosRequestPolicy
{
  template <typename MessageT>
  static void request_bridge(const std::string & topic_name, topic_local_id_t id)
  {
    request_bridge_core<MessageT>(topic_name, id, BridgeDirection::AGNOCAST_TO_ROS2);
  }
};

// Dummy policy to avoid circular header dependencies.
// Used internally by BridgeNode, Service, and Client where bridge requests
// are not needed and would cause include cycles.
struct NoBridgeRequestPolicy
{
  template <typename MessageT>
  static void request_bridge(const std::string & /*unused*/, topic_local_id_t /*unused*/)
  {
    // Do nothing
  }
};

template <typename MessageT>
class RosToAgnocastBridge
{
  using AgnoPub = agnocast::BasicPublisher<MessageT, NoBridgeRequestPolicy>;
  typename AgnoPub::SharedPtr agnocast_pub_;
  typename rclcpp::Subscription<MessageT>::SharedPtr ros_sub_;
  rclcpp::CallbackGroup::SharedPtr ros_cb_group_;

public:
  explicit RosToAgnocastBridge(
    const rclcpp::Node::SharedPtr & parent_node, const std::string & topic_name,
    const rclcpp::QoS & sub_qos)
  {
    // Agnocast relies on shared memory, so network reliability concepts do not apply.
    // TransientLocal is hardcoded here as a catch-all configuration that supports
    // any subscriber requirement (volatile or durable) by preserving data.
    agnocast_pub_ = std::make_shared<AgnoPub>(
      parent_node.get(), topic_name, rclcpp::QoS(DEFAULT_QOS_DEPTH).transient_local(),
      agnocast::PublisherOptions{});
    ros_cb_group_ =
      parent_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions ros_opts;
    ros_opts.ignore_local_publications = true;
    ros_opts.callback_group = ros_cb_group_;
    auto pub_ptr = agnocast_pub_;

    // The ROS subscription acts as a proxy for the requesting Agnocast subscriber.
    // sub_qos applies the Agnocast subscriber's settings (e.g. history depth)
    // to the ROS side to ensure the bridge satisfies the downstream requirements.
    ros_sub_ = parent_node->create_subscription<MessageT>(
      topic_name, sub_qos,
      [this](const typename MessageT::ConstSharedPtr msg) {
        auto loaned_msg = this->agnocast_pub_->borrow_loaned_message();
        *loaned_msg = *msg;
        this->agnocast_pub_->publish(std::move(loaned_msg));
      },
      ros_opts);
  }
};

// We should document that things don't work well when Agnocast publishers have a mix of transient
// local and volatile durability settings. If we ever face a requirement to support topics with
// such mixed durability settings, we could achieve this by creating Agnocast subscribers with
// transient local, and making an exception so that only Agnocast subscribers used for the bridge
// feature can also receive from volatile Agnocast publishers. (This isn't very clean, so we'd
// prefer to avoid it if possible.)
template <typename MessageT>
class AgnocastToRosBridge
{
  using AgnoSub = agnocast::BasicSubscription<MessageT, NoBridgeRequestPolicy>;
  typename rclcpp::Publisher<MessageT>::SharedPtr ros_pub_;
  typename AgnoSub::SharedPtr agnocast_sub_;
  rclcpp::CallbackGroup::SharedPtr agno_cb_group_;

public:
  explicit AgnocastToRosBridge(
    const rclcpp::Node::SharedPtr & parent_node, const std::string & topic_name,
    const rclcpp::QoS & sub_qos)
  {
    // ROS Publisher configuration acts as a source for downstream ROS nodes.
    // We use Reliable and TransientLocal as a "catch-all" configuration.
    // This ensures that this bridge can serve both Volatile and Durable (TransientLocal)
    // ROS subscribers without connectivity issues.
    ros_pub_ = parent_node->create_publisher<MessageT>(
      topic_name, rclcpp::QoS(DEFAULT_QOS_DEPTH).reliable().transient_local());
    agno_cb_group_ =
      parent_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    agnocast::SubscriptionOptions agno_opts;
    agno_opts.ignore_local_publications = true;
    agno_opts.callback_group = agno_cb_group_;

    // Subscribe to Agnocast (shared memory).
    // The QoS settings are now passed via argument to inherit the settings
    // from the corresponding Agnocast publisher (e.g. Reliable or BestEffort).
    agnocast_sub_ = std::make_shared<AgnoSub>(
      parent_node.get(), topic_name, sub_qos,
      [this](const agnocast::ipc_shared_ptr<MessageT> msg) {
        auto loaned_msg = this->ros_pub_->borrow_loaned_message();
        if (loaned_msg.is_valid()) {
          loaned_msg.get() = *msg;
          this->ros_pub_->publish(std::move(loaned_msg));
        } else {
          this->ros_pub_->publish(*msg);
        }
      },
      agno_opts);
  }
};

template <typename MessageT>
std::shared_ptr<void> start_ros_to_agno_node(
  rclcpp::Node::SharedPtr node, const BridgeTargetInfo & info)
{
  std::string topic_name(static_cast<const char *>(info.topic_name));
  return std::make_shared<RosToAgnocastBridge<MessageT>>(
    node, topic_name, get_subscriber_qos(topic_name, info.target_id));
}

template <typename MessageT>
std::shared_ptr<void> start_agno_to_ros_node(
  rclcpp::Node::SharedPtr node, const BridgeTargetInfo & info)
{
  std::string topic_name(static_cast<const char *>(info.topic_name));
  return std::make_shared<AgnocastToRosBridge<MessageT>>(
    node, topic_name, get_publisher_qos(topic_name, info.target_id));
}

template <typename MsgStruct>
void send_mq_message(
  const std::string & mq_name, const MsgStruct & msg, long msg_size_limit,
  const rclcpp::Logger & logger)
{
  struct mq_attr attr = {};
  attr.mq_maxmsg = BRIDGE_MQ_MAX_MESSAGES;
  attr.mq_msgsize = msg_size_limit;

  mqd_t mq =
    mq_open(mq_name.c_str(), O_CREAT | O_WRONLY | O_NONBLOCK | O_CLOEXEC, BRIDGE_MQ_PERMS, &attr);

  if (mq == (mqd_t)-1) {
    RCLCPP_ERROR(
      logger, "mq_open failed for name '%s': %s (errno: %d)", mq_name.c_str(), strerror(errno),
      errno);
    return;
  }

  if (mq_send(mq, reinterpret_cast<const char *>(&msg), sizeof(msg), 0) < 0) {
    RCLCPP_ERROR(
      logger, "mq_send failed for name '%s': %s (errno: %d)", mq_name.c_str(), strerror(errno),
      errno);
  }

  mq_close(mq);
}

template <typename MessageT>
void send_bridge_request(
  const std::string & topic_name, topic_local_id_t id, BridgeDirection direction)
{
  static const auto logger = rclcpp::get_logger("agnocast_bridge_requester");
  // We capture 'fn_reverse' because bridge_manager is responsible for managing both directions
  // independently. Storing the reverse factory allows us to instantiate the return path on-demand
  // within the same process.
  auto fn_current = (direction == BridgeDirection::ROS2_TO_AGNOCAST)
                      ? &start_ros_to_agno_node<MessageT>
                      : &start_agno_to_ros_node<MessageT>;
  auto fn_reverse = (direction == BridgeDirection::ROS2_TO_AGNOCAST)
                      ? &start_agno_to_ros_node<MessageT>
                      : &start_ros_to_agno_node<MessageT>;

  Dl_info info = {};
  if (dladdr(reinterpret_cast<void *>(fn_current), &info) == 0 || !info.dli_fname) {
    RCLCPP_ERROR(logger, "dladdr failed or filename NULL.");
    return;
  }

  MqMsgBridge msg = {};
  msg.direction = direction;
  msg.target.target_id = id;
  snprintf(
    static_cast<char *>(msg.target.topic_name), TOPIC_NAME_BUFFER_SIZE, "%s", topic_name.c_str());
  snprintf(
    static_cast<char *>(msg.factory.shared_lib_path), SHARED_LIB_PATH_BUFFER_SIZE, "%s",
    info.dli_fname);
  snprintf(
    static_cast<char *>(msg.factory.symbol_name), SYMBOL_NAME_BUFFER_SIZE, "%s",
    info.dli_sname ? info.dli_sname : MAIN_EXECUTABLE_SYMBOL);
  auto base_addr = reinterpret_cast<uintptr_t>(info.dli_fbase);
  msg.factory.fn_offset = reinterpret_cast<uintptr_t>(fn_current) - base_addr;
  msg.factory.fn_offset_reverse = reinterpret_cast<uintptr_t>(fn_reverse) - base_addr;

  std::string mq_name = create_mq_name_for_bridge(bridge_manager_pid);
  send_mq_message(mq_name, msg, BRIDGE_MQ_MESSAGE_SIZE, logger);
}

template <typename MessageT>
void send_performance_bridge_request(
  const std::string & topic_name, topic_local_id_t id, BridgeDirection direction)
{
  static const auto logger = rclcpp::get_logger("agnocast_performance_bridge_requester");

  const std::string message_type_name = rosidl_generator_traits::name<MessageT>();

  MqMsgPerformanceBridge msg = {};
  snprintf(msg.message_type, MESSAGE_TYPE_BUFFER_SIZE, "%s", message_type_name.c_str());
  snprintf(msg.target.topic_name, TOPIC_NAME_BUFFER_SIZE, "%s", topic_name.c_str());
  msg.target.target_id = id;
  msg.direction = direction;

  std::string mq_name = create_mq_name_for_bridge(PERFORMANCE_BRIDGE_VIRTUAL_PID);
  send_mq_message(mq_name, msg, PERFORMANCE_BRIDGE_MQ_MESSAGE_SIZE, logger);
}

}  // namespace agnocast
