#include "agnocast/agnocast.hpp"
#include "agnocast/internal/type_registry_writer.hpp"
#include "agnocast/node/agnocast_node.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rclcpp/version.h"
#include "rcpputils/shared_library.hpp"

namespace agnocast
{

SubscriptionBase::SubscriptionBase(rclcpp::Node * node, const std::string & topic_name)
: topic_name_(node->get_node_topics_interface()->resolve_topic_name(topic_name))
{
  validate_ld_preload();
}

SubscriptionBase::SubscriptionBase(
  agnocast::Node * node, const std::string & topic_name)  // NOLINT(modernize-pass-by-value)
: topic_name_(node->get_node_topics_interface()->resolve_topic_name(topic_name))
{
  validate_ld_preload();
}

union ioctl_add_subscriber_args SubscriptionBase::initialize(
  const rclcpp::QoS & qos, const bool is_take_sub, const bool ignore_local_publications,
  const bool is_bridge, const std::string & node_name, const std::string & type_name)
{
  // Announce to the per-IPC-namespace discovery agent before the kmod call so
  // the registry line is in place whenever a later snapshot sees the
  // ioctl-side endpoint. Empty `type_name` (e.g. service types) skips this.
  if (!type_name.empty()) {
    internal::TypeRegistryWriter::instance().register_type(
      topic_name_, type_name, "sub", node_name);
  }

  union ioctl_add_subscriber_args add_subscriber_args = {};
  add_subscriber_args.topic_name = {topic_name_.c_str(), topic_name_.size()};
  add_subscriber_args.node_name = {node_name.c_str(), node_name.size()};
  add_subscriber_args.qos_depth = static_cast<uint32_t>(qos.depth());
  add_subscriber_args.qos_is_transient_local =
    qos.durability() == rclcpp::DurabilityPolicy::TransientLocal;
  add_subscriber_args.qos_is_reliable = qos.reliability() == rclcpp::ReliabilityPolicy::Reliable;
  add_subscriber_args.is_take_sub = is_take_sub;
  add_subscriber_args.ignore_local_publications = ignore_local_publications;
  add_subscriber_args.is_bridge = is_bridge;
  if (ioctl(agnocast_fd, AGNOCAST_ADD_SUBSCRIBER_CMD, &add_subscriber_args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_ADD_SUBSCRIBER_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  return add_subscriber_args;
}

uint32_t get_publisher_count_core(const std::string & topic_name)
{
  union ioctl_get_publisher_num_args args = {};
  args.topic_name = {topic_name.c_str(), topic_name.size()};
  if (ioctl(agnocast_fd, AGNOCAST_GET_PUBLISHER_NUM_CMD, &args) < 0) {
    RCLCPP_ERROR(logger, "AGNOCAST_GET_PUBLISHER_NUM_CMD failed: %s", strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }

  uint32_t count = args.ret_publisher_num;
  // If an R2A bridge exists, exclude the agnocast publisher created by the bridge
  if (args.ret_r2a_bridge_exist && count > 0) {
    count--;
  }

  uint32_t ros2_count = args.ret_ros2_publisher_num;
  // If an A2R bridge exists, exclude the ROS 2 publisher created by the bridge
  if (args.ret_a2r_bridge_exist && ros2_count > 0) {
    ros2_count--;
  }

  return count + ros2_count;
}

mqd_t open_mq_for_subscription(
  const std::string & topic_name, const topic_local_id_t subscriber_id,
  std::pair<mqd_t, std::string> & mq_subscription)
{
  std::string mq_name = create_mq_name_for_agnocast_publish(topic_name, subscriber_id);
  struct mq_attr attr = {};
  attr.mq_flags = 0;                        // Blocking queue
  attr.mq_msgsize = sizeof(MqMsgAgnocast);  // Maximum message size
  attr.mq_curmsgs = 0;  // Number of messages currently in the queue (not set by mq_open)
  attr.mq_maxmsg = 1;

  const int mq_mode = 0666;
  mqd_t mq = mq_open(mq_name.c_str(), O_CREAT | O_RDONLY | O_NONBLOCK, mq_mode, &attr);
  if (mq == -1) {
    RCLCPP_ERROR_STREAM(
      logger, "mq_open failed for topic '" << topic_name << "' (subscriber_id=" << subscriber_id
                                           << ", mq_name='" << mq_name
                                           << "'): " << strerror(errno));
    close(agnocast_fd);
    exit(EXIT_FAILURE);
  }
  mq_subscription = std::make_pair(mq, mq_name);

  return mq;
}

void remove_mq(const std::pair<mqd_t, std::string> & mq_subscription)
{
  /* The message queue is destroyed after all the publisher processes close it. */
  if (mq_close(mq_subscription.first) == -1) {
    RCLCPP_ERROR_STREAM(
      logger,
      "mq_close failed for mq_name='" << mq_subscription.second << "': " << strerror(errno));
  }
  if (mq_unlink(mq_subscription.second.c_str()) == -1) {
    RCLCPP_ERROR_STREAM(
      logger,
      "mq_unlink failed for mq_name='" << mq_subscription.second << "': " << strerror(errno));
  }
}

rclcpp::CallbackGroup::SharedPtr get_default_callback_group_for_tracepoint(agnocast::Node * node)
{
  return node->get_node_base_interface()->get_default_callback_group();
}

TypeSupportBundle GenericSubscription::load_typesupport_impl(const std::string & topic_type)
{
  TypeSupportBundle result;
  result.library = rclcpp::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");
  // rclcpp::get_typesupport_handle() was deprecated in Jazzy (rclcpp 28) in favor of
  // rclcpp::get_message_typesupport_handle() to distinguish message handles from service handles.
#if RCLCPP_VERSION_MAJOR >= 28
  result.handle =
    rclcpp::get_message_typesupport_handle(topic_type, "rosidl_typesupport_cpp", *result.library);
#else
  result.handle =
    rclcpp::get_typesupport_handle(topic_type, "rosidl_typesupport_cpp", *result.library);
#endif
  return result;
}

template <typename NodeT>
rclcpp::QoS GenericSubscription::constructor_impl(
  NodeT * node, const std::string & topic_type, const rclcpp::QoS & qos,
  TypeErasedCallback callback, rclcpp::CallbackGroup::SharedPtr callback_group,
  const agnocast::SubscriptionOptions & options, SubscriptionRole role)
{
  const bool override_qos = !options.qos_overriding_options.get_policy_kinds().empty();
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters =
    override_qos ? node->get_node_parameters_interface() : nullptr;
  const rclcpp::QoS actual_qos = override_qos
                                   ? rclcpp::detail::declare_qos_parameters(
                                       options.qos_overriding_options, node_parameters, topic_name_,
                                       qos, rclcpp::detail::SubscriptionQosParametersTraits{})
                                   : qos;

  validate_subscription_qos(actual_qos);

  const std::string node_name = node->get_fully_qualified_name();

  const bool is_bridge = (role == SubscriptionRole::BridgeInternal);
  union ioctl_add_subscriber_args add_subscriber_args = initialize(
    actual_qos, false, options.ignore_local_publications, is_bridge, node_name, topic_type);

  id_ = add_subscriber_args.ret_id;

  if (role == SubscriptionRole::Default) {
    register_pubsub_bridge_by_type_name(
      topic_name_, id_, topic_type, BridgeDirection::ROS2_TO_AGNOCAST);
  }

  mqd_t mq = open_mq_for_subscription(topic_name_, id_, mq_subscription_);

  const bool is_transient_local =
    actual_qos.durability() == rclcpp::DurabilityPolicy::TransientLocal;
  callback_info_id_ = agnocast::register_generic_callback(
    std::move(callback), topic_name_, id_, is_transient_local, mq, std::move(callback_group));

  return actual_qos;
}

GenericSubscription::~GenericSubscription()
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

template rclcpp::QoS GenericSubscription::constructor_impl<rclcpp::Node>(
  rclcpp::Node *, const std::string &, const rclcpp::QoS &, TypeErasedCallback,
  rclcpp::CallbackGroup::SharedPtr, const agnocast::SubscriptionOptions &, SubscriptionRole);

template rclcpp::QoS GenericSubscription::constructor_impl<agnocast::Node>(
  agnocast::Node *, const std::string &, const rclcpp::QoS &, TypeErasedCallback,
  rclcpp::CallbackGroup::SharedPtr, const agnocast::SubscriptionOptions &, SubscriptionRole);

}  // namespace agnocast
