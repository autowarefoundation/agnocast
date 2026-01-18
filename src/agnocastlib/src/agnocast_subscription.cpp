#include "agnocast/agnocast.hpp"

namespace agnocast
{

SubscriptionBase::SubscriptionBase(rclcpp::Node * node, const std::string & topic_name)
: id_(0), topic_name_(node->get_node_topics_interface()->resolve_topic_name(topic_name))
{
  validate_ld_preload();
}

SubscriptionBase::SubscriptionBase(
  agnocast::Node * node, const std::string & topic_name)  // NOLINT(modernize-pass-by-value)
: id_(0), topic_name_(node->get_node_topics_interface()->resolve_topic_name(topic_name))
{
  validate_ld_preload();
}

union ioctl_add_subscriber_args SubscriptionBase::initialize(
  const rclcpp::QoS & qos, const bool is_take_sub, const bool ignore_local_publications,
  const bool is_bridge, const std::string & node_name)
{
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
    RCLCPP_ERROR(logger, "mq_open failed: %s", strerror(errno));
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
    RCLCPP_ERROR(logger, "mq_close failed: %s", strerror(errno));
  }
  if (mq_unlink(mq_subscription.second.c_str()) == -1) {
    RCLCPP_ERROR(logger, "mq_unlink failed: %s", strerror(errno));
  }
}

}  // namespace agnocast
