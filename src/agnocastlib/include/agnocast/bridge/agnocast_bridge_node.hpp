#pragma once

#include "agnocast/agnocast_client.hpp"
#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/agnocast_subscription.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/version.h"

#include <fcntl.h>
#include <mqueue.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

namespace agnocast

{

static constexpr size_t DEFAULT_QOS_DEPTH = 10;

template <typename MessageT>
void send_standard_pubsub_bridge_registration(
  const std::string & topic_name, topic_local_id_t id, BridgeDirection direction);
template <typename ServiceT>
void send_standard_service_bridge_registration(
  const std::string & service_name, BridgeDirection direction,
  const std::optional<std::pair<std::string, std::string>> & shadow_node_identity);
template <typename MessageT>
void send_performance_pubsub_bridge_registration(
  const std::string & topic_name, topic_local_id_t id, BridgeDirection direction);
inline void send_performance_pubsub_bridge_registration_by_type_name(
  const std::string & topic_name, topic_local_id_t id, const std::string & message_type_name,
  BridgeDirection direction);
template <typename ServiceT>
void send_performance_service_bridge_registration(
  const std::string & service_name, BridgeDirection direction,
  const std::optional<std::pair<std::string, std::string>> & shadow_node_identity);

template <typename MessageT>
void register_pubsub_bridge_core(
  const std::string & topic_name, topic_local_id_t id, BridgeDirection direction)
{
  auto bridge_mode = get_bridge_mode();
  if (bridge_mode == BridgeMode::Standard) {
    send_standard_pubsub_bridge_registration<MessageT>(topic_name, id, direction);
  } else if (bridge_mode == BridgeMode::Performance) {
    send_performance_pubsub_bridge_registration<MessageT>(topic_name, id, direction);
  }
}

inline void register_pubsub_bridge_by_type_name(
  const std::string & topic_name, topic_local_id_t id, const std::string & message_type,
  BridgeDirection direction)
{
  auto bridge_mode = get_bridge_mode();
  if (bridge_mode == BridgeMode::Standard) {
    static const auto logger = rclcpp::get_logger("agnocast_bridge_registrar");
    RCLCPP_WARN_ONCE(
      logger,
      "GenericSubscription does not support standard-mode bridge. "
      "Set AGNOCAST_BRIDGE_MODE=performance to enable bridging.");
  } else if (bridge_mode == BridgeMode::Performance) {
    send_performance_pubsub_bridge_registration_by_type_name(
      topic_name, id, message_type, direction);
  }
}

template <typename ServiceT>
void register_service_bridge_core(
  const std::string & service_name, BridgeDirection direction,
  const std::optional<std::pair<std::string, std::string>> & shadow_node_identity)
{
  auto bridge_mode = get_bridge_mode();
  if (bridge_mode == BridgeMode::Standard) {
    send_standard_service_bridge_registration<ServiceT>(
      service_name, direction, shadow_node_identity);
  } else if (bridge_mode == BridgeMode::Performance) {
    send_performance_service_bridge_registration<ServiceT>(
      service_name, direction, shadow_node_identity);
  }
}

// Policy for agnocast::Subscription.
// Registers a bridge that forwards messages from ROS 2 to Agnocast (R2A).
struct RosToAgnocastPubsubRegistrationPolicy
{
  template <typename MessageT>
  static void register_bridge(const std::string & topic_name, topic_local_id_t id)
  {
    register_pubsub_bridge_core<MessageT>(topic_name, id, BridgeDirection::ROS2_TO_AGNOCAST);
  }
};

// Policy for agnocast::Publisher.
// Registers a bridge that forwards messages from Agnocast to ROS 2 (A2R).
struct AgnocastToRosPubsubRegistrationPolicy
{
  template <typename MessageT>
  static void register_bridge(const std::string & topic_name, topic_local_id_t id)
  {
    register_pubsub_bridge_core<MessageT>(topic_name, id, BridgeDirection::AGNOCAST_TO_ROS2);
  }
};

// Policy for agnocast::Service.
// Registers a bridge that forwards requests from ROS 2 to Agnocast (R2A).
struct RosToAgnocastServiceRegistrationPolicy
{
  template <typename NodeT, typename ServiceT>
  static void register_bridge(NodeT * node, const std::string & service_name)
  {
    std::optional<std::pair<std::string, std::string>> shadow_node_identity{std::nullopt};
    if constexpr (std::is_same_v<std::remove_cv_t<NodeT>, agnocast::Node>) {
      shadow_node_identity =
        std::make_pair(std::string(node->get_namespace()), std::string(node->get_name()));
    }
    register_service_bridge_core<ServiceT>(
      service_name, BridgeDirection::ROS2_TO_AGNOCAST, shadow_node_identity);
  }
};

// Dummy policy to avoid circular header dependencies.
// Used internally by BridgeNode, Service, and Client where bridge registrations
// are not needed and would cause include cycles.
struct NoBridgeRegistrationPolicy
{
  template <typename T, typename... Args>
  static void register_bridge(Args &&... args)
  {
    register_bridge_impl(std::forward<Args>(args)...);
  }

private:
  static void register_bridge_impl(const std::string &, topic_local_id_t) {}
  template <typename NodeT>
  static void register_bridge_impl(NodeT *, const std::string &)
  {
  }
};

template <typename MessageT>
class RosToAgnocastPubsubBridge : public PubsubBridgeBase
{
  using AgnoPub = agnocast::BasicPublisher<MessageT, NoBridgeRegistrationPolicy>;
  typename AgnoPub::SharedPtr agnocast_pub_;
  typename rclcpp::Subscription<MessageT>::SharedPtr ros_sub_;
  rclcpp::CallbackGroup::SharedPtr ros_cb_group_;

public:
  explicit RosToAgnocastPubsubBridge(
    const rclcpp::Node::SharedPtr & parent_node, const std::string & topic_name,
    const rclcpp::QoS & sub_qos)
  {
    // Agnocast relies on shared memory, so network reliability concepts do not apply.
    // TransientLocal is hardcoded here as a catch-all configuration that supports
    // any subscriber requirement (volatile or durable) by preserving data.
    agnocast::PublisherOptions agno_opts;

    agnocast_pub_ = std::make_shared<AgnoPub>(
      parent_node.get(), topic_name, rclcpp::QoS(DEFAULT_QOS_DEPTH).transient_local(), agno_opts,
      true);
    ros_cb_group_ =
      parent_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions ros_opts;
    ros_opts.ignore_local_publications = true;
    ros_opts.callback_group = ros_cb_group_;

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

  rclcpp::CallbackGroup::SharedPtr get_callback_group() const override { return ros_cb_group_; }
};

// We should document that things don't work well when Agnocast publishers have a mix of transient
// local and volatile durability settings. If we ever face a requirement to support topics with
// such mixed durability settings, we could achieve this by creating Agnocast subscribers with
// transient local, and making an exception so that only Agnocast subscribers used for the bridge
// feature can also receive from volatile Agnocast publishers. (This isn't very clean, so we'd
// prefer to avoid it if possible.)
template <typename MessageT>
class AgnocastToRosPubsubBridge : public PubsubBridgeBase
{
  using AgnoSub = agnocast::BasicSubscription<MessageT, NoBridgeRegistrationPolicy>;
  typename rclcpp::Publisher<MessageT>::SharedPtr ros_pub_;
  typename AgnoSub::SharedPtr agnocast_sub_;
  rclcpp::CallbackGroup::SharedPtr agno_cb_group_;

public:
  explicit AgnocastToRosPubsubBridge(
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
      agno_opts, true);
  }

  rclcpp::CallbackGroup::SharedPtr get_callback_group() const override { return agno_cb_group_; }
};

template <typename MessageT>
std::shared_ptr<PubsubBridgeBase> start_r2a_pubsub_node(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const rclcpp::QoS & qos)
{
  return std::make_shared<RosToAgnocastPubsubBridge<MessageT>>(node, topic_name, qos);
}

template <typename MessageT>
std::shared_ptr<PubsubBridgeBase> start_a2r_pubsub_node(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const rclcpp::QoS & qos)
{
  return std::make_shared<AgnocastToRosPubsubBridge<MessageT>>(node, topic_name, qos);
}

template <typename ServiceT>
class RosToAgnocastServiceBridge : public ServiceBridgeBase
{
  typename rclcpp::Service<ServiceT>::SharedPtr ros_srv_;
  typename agnocast::Client<ServiceT>::SharedPtr agno_client_;
  rclcpp::CallbackGroup::SharedPtr ros_srv_cb_group_;
  rclcpp::CallbackGroup::SharedPtr agno_client_cb_group_;

public:
  explicit RosToAgnocastServiceBridge(
    const rclcpp::Node::SharedPtr & parent_node, const std::string & service_name,
    const rclcpp::QoS & qos)
  {
    ros_srv_cb_group_ = parent_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    agno_client_cb_group_ =
      parent_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    agno_client_ = std::make_shared<agnocast::Client<ServiceT>>(
      parent_node.get(), service_name, qos, agno_client_cb_group_);

    ros_srv_ = parent_node->create_service<ServiceT>(
      service_name,
      [this](
        typename rclcpp::Service<ServiceT>::SharedPtr service_handle,
        std::shared_ptr<rmw_request_id_t> request_header,
        typename ServiceT::Request::SharedPtr ros_req) {
        auto agno_req = this->agno_client_->borrow_loaned_request();
        *agno_req = *ros_req;

        this->agno_client_->async_send_request(
          std::move(agno_req), [service_handle, request_header](
                                 typename agnocast::Client<ServiceT>::SharedFuture future) {
            auto agno_res = future.get();
            typename ServiceT::Response ros_res;
            ros_res = *agno_res;
            service_handle->send_response(*request_header, ros_res);
          });
      },
#if RCLCPP_VERSION_MAJOR >= 28
      qos, ros_srv_cb_group_);
#else
      qos.get_rmw_qos_profile(), ros_srv_cb_group_);
#endif
  }

  std::pair<rclcpp::CallbackGroup::SharedPtr, rclcpp::CallbackGroup::SharedPtr>
  get_callback_groups() const override
  {
    return {ros_srv_cb_group_, agno_client_cb_group_};
  }
};

template <typename ServiceT>
std::shared_ptr<ServiceBridgeBase> start_r2a_service_node(
  rclcpp::Node::SharedPtr node, const std::string & service_name, const rclcpp::QoS & qos)
{
  return std::make_shared<RosToAgnocastServiceBridge<ServiceT>>(node, service_name, qos);
}

template <typename MsgStruct>
void send_mq_message(
  const std::string & mq_name, const MsgStruct & msg, long msg_size_limit,
  const rclcpp::Logger & logger)
{
  struct mq_attr attr = {};
  int64_t max_messages = BRIDGE_MQ_MAX_MESSAGES;
  if (get_bridge_mode() == BridgeMode::Performance) {
    max_messages = PERFORMANCE_BRIDGE_MQ_MAX_MESSAGES;
  }
  attr.mq_maxmsg = max_messages;
  attr.mq_msgsize = msg_size_limit;

  mqd_t mq =
    mq_open(mq_name.c_str(), O_CREAT | O_WRONLY | O_NONBLOCK | O_CLOEXEC, BRIDGE_MQ_PERMS, &attr);

  if (mq == (mqd_t)-1) {
    RCLCPP_ERROR(
      logger, "mq_open failed for name '%s': %s (errno: %d)", mq_name.c_str(), strerror(errno),
      errno);
    return;
  }

  constexpr int BRIDGE_MQ_SEND_MAX_RETRIES = 100;
  constexpr useconds_t BRIDGE_MQ_SEND_RETRY_INTERVAL_US = 100000;  // 100ms

  int send_result = -1;
  int last_errno = 0;
  for (int retry = 0; retry <= BRIDGE_MQ_SEND_MAX_RETRIES; ++retry) {
    send_result = mq_send(mq, reinterpret_cast<const char *>(&msg), sizeof(msg), 0);
    if (send_result == 0) break;
    last_errno = errno;
    if (last_errno != EAGAIN) break;
    if (retry < BRIDGE_MQ_SEND_MAX_RETRIES) {
      usleep(BRIDGE_MQ_SEND_RETRY_INTERVAL_US);
    }
  }
  if (send_result < 0) {
    RCLCPP_ERROR(
      logger, "mq_send failed for name '%s': %s (errno: %d)", mq_name.c_str(), strerror(last_errno),
      last_errno);
  }

  mq_close(mq);
}

template <typename MessageT>
void send_standard_pubsub_bridge_registration(
  const std::string & topic_name, topic_local_id_t id, BridgeDirection direction)
{
  static const auto logger = rclcpp::get_logger("agnocast_bridge_registrar");

  auto fn_r2a = reinterpret_cast<uintptr_t>(&start_r2a_pubsub_node<MessageT>);
  auto fn_a2r = reinterpret_cast<uintptr_t>(&start_a2r_pubsub_node<MessageT>);

  auto [msg, reason] =
    BridgeRegistrationMsgBuilder(BridgeRegistrationMsgBuilder::Mode::Standard, logger)
      .set_direction(direction)
      .set_is_service(false)
      .set_pubsub_target_id(id)
      .set_topic_name(topic_name.c_str())
      .set_factory(fn_r2a, fn_a2r)
      .build_standard_message();
  if (!reason.empty()) {
    RCLCPP_ERROR(logger, "Failed to build standard pubsub bridge registration: %s", reason.c_str());
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  std::string mq_name = create_mq_name_for_bridge(standard_bridge_manager_pid);
  send_mq_message(mq_name, msg, BRIDGE_MQ_MESSAGE_SIZE, logger);
}

template <typename ServiceT>
void send_standard_service_bridge_registration(
  const std::string & service_name, BridgeDirection direction,
  const std::optional<std::pair<std::string, std::string>> & shadow_node_identity)
{
  static const auto logger = rclcpp::get_logger("agnocast_service_bridge_registrar");

  auto fn_r2a = reinterpret_cast<uintptr_t>(&start_r2a_service_node<ServiceT>);
  // TODO(bdm-k): Specify `start_a2r_service_node` once it's implemented.
  // Service bridges currently support only the ROS2 -> Agnocast direction.
  auto fn_a2r = fn_r2a;  // dummy value

  auto [msg, reason] =
    BridgeRegistrationMsgBuilder(BridgeRegistrationMsgBuilder::Mode::Standard, logger)
      .set_direction(direction)
      .set_is_service(true)
      .set_service_name(service_name.c_str())
      .set_shadow_node_identity(shadow_node_identity)
      .set_factory(fn_r2a, fn_a2r)
      .build_standard_message();
  if (!reason.empty()) {
    RCLCPP_ERROR(
      logger, "Failed to build standard service bridge registration: %s", reason.c_str());
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  std::string mq_name = create_mq_name_for_bridge(standard_bridge_manager_pid);
  send_mq_message(mq_name, msg, BRIDGE_MQ_MESSAGE_SIZE, logger);
}

template <typename MessageT>
void send_performance_pubsub_bridge_registration(
  const std::string & topic_name, topic_local_id_t id, BridgeDirection direction)
{
  const std::string message_type_name = rosidl_generator_traits::name<MessageT>();
  send_performance_pubsub_bridge_registration_by_type_name(
    topic_name, id, message_type_name, direction);
}

inline void send_performance_pubsub_bridge_registration_by_type_name(
  const std::string & topic_name, topic_local_id_t id, const std::string & message_type_name,
  BridgeDirection direction)
{
  static const auto logger = rclcpp::get_logger("agnocast_performance_bridge_registrar");

  auto [msg, reason] =
    BridgeRegistrationMsgBuilder(BridgeRegistrationMsgBuilder::Mode::Performance, logger)
      .set_direction(direction)
      .set_is_service(false)
      .set_message_type(message_type_name.c_str())
      .set_topic_name(topic_name.c_str())
      .set_pubsub_target_id(id)
      .build_performance_message();
  if (!reason.empty()) {
    RCLCPP_ERROR(
      logger, "Failed to build performance pubsub bridge registration: %s", reason.c_str());
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  std::string mq_name = create_mq_name_for_bridge(PERFORMANCE_BRIDGE_VIRTUAL_PID);
  send_mq_message(mq_name, msg, PERFORMANCE_BRIDGE_MQ_MESSAGE_SIZE, logger);
}

template <typename ServiceT>
void send_performance_service_bridge_registration(
  const std::string & service_name, BridgeDirection direction,
  const std::optional<std::pair<std::string, std::string>> & shadow_node_identity)
{
  static const auto logger = rclcpp::get_logger("agnocast_performance_service_bridge_registrar");

  const std::string service_type_name = rosidl_generator_traits::name<ServiceT>();

  auto [msg, reason] =
    BridgeRegistrationMsgBuilder(BridgeRegistrationMsgBuilder::Mode::Performance, logger)
      .set_direction(direction)
      .set_is_service(true)
      .set_service_type(service_type_name.c_str())
      .set_service_name(service_name.c_str())
      .set_shadow_node_identity(shadow_node_identity)
      .build_performance_message();
  if (!reason.empty()) {
    RCLCPP_ERROR(
      logger, "Failed to build performance service bridge registration: %s", reason.c_str());
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  std::string mq_name = create_mq_name_for_bridge(PERFORMANCE_BRIDGE_VIRTUAL_PID);
  send_mq_message(mq_name, msg, PERFORMANCE_BRIDGE_MQ_MESSAGE_SIZE, logger);
}

}  // namespace agnocast
