#pragma once

#include "agnocast/agnocast.hpp"
#include "agnocast/message_filters/simple_filter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace agnocast
{
namespace message_filters
{

namespace detail
{
inline rclcpp::QoS to_rclcpp_qos(const rmw_qos_profile_t & rmw_qos)
{
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos));
  qos.get_rmw_qos_profile() = rmw_qos;
  return qos;
}
}  // namespace detail

/**
 * \brief Base class for Subscriber, allows subscription without knowing the message type.
 */
template <class NodeType = rclcpp::Node>
class SubscriberBase
{
public:
  using NodePtr = std::shared_ptr<NodeType>;

  virtual ~SubscriberBase() = default;

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param node The rclcpp::Node::SharedPtr to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos (optional) The rmw qos profile to use to subscribe.
   */
  virtual void subscribe(
    NodePtr node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default) = 0;

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param node The rclcpp::Node to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos (optional) The rmw qos profile to use to subscribe.
   */
  virtual void subscribe(
    NodeType * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default) = 0;

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   * This override allows SubscriptionOptions to be passed into the class without changing API.
   *
   * \param node The rclcpp::Node::SharedPtr to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos The rmw qos profile to use to subscribe.
   * \param options The subscription options to use to subscribe.
   */
  virtual void subscribe(
    NodePtr node, const std::string & topic, const rmw_qos_profile_t qos,
    agnocast::SubscriptionOptions options)
  {
    this->subscribe(node.get(), topic, qos, options);
  }

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param node The rclcpp::Node to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos The rmw qos profile to use to subscribe.
   * \param options The subscription options to use to subscribe.
   */
  virtual void subscribe(
    NodeType * node, const std::string & topic, const rmw_qos_profile_t qos,
    agnocast::SubscriptionOptions options)
  {
    (void)options;
    subscribe(node, topic, qos);
  }

  /**
   * \brief Re-subscribe to a topic.  Only works if this subscriber has previously been subscribed
   * to a topic.
   */
  virtual void subscribe() = 0;

  /**
   * \brief Force immediate unsubscription of this subscriber from its topic
   */
  virtual void unsubscribe() = 0;
};

template <typename T>
using SubscriberBasePtr = std::shared_ptr<SubscriberBase<T>>;

/**
 * \brief Agnocast subscription filter.
 *
 * This class acts as a highest-level filter, simply passing messages from an agnocast subscription
 * through to the filters which have connected to it.
 *
 * When this object is destroyed it will unsubscribe from the agnocast subscription.
 *
 * The Subscriber object is templated on the type of message being subscribed to.
 *
 * \section connections CONNECTIONS
 *
 * Subscriber has no input connection.
 *
 * The output connection for the Subscriber object is the same signature as for agnocast
 * subscription callbacks, ie.
\verbatim
void callback(const agnocast::ipc_shared_ptr<M const>&);
\endverbatim
 */
template <class M, class NodeType = rclcpp::Node>
class Subscriber : public SubscriberBase<NodeType>, public SimpleFilter<M>
{
public:
  using NodePtr = std::shared_ptr<NodeType>;
  using MConstPtr = ipc_shared_ptr<M const>;
  using EventType = MessageEvent<M const>;

  /**
   * \brief Constructor
   *
   * See the agnocast::create_subscription() variants for more information on the parameters
   *
   * \param node The rclcpp::Node::SharedPtr to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos (optional) The rmw qos profile to use to subscribe.
   */
  Subscriber(
    // cppcheck-suppress passedByValue  // shared_ptr by-value is intentional (shared ownership)
    NodePtr node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default)  // NOLINT
  {
    subscribe(node, topic, qos);
  }

  Subscriber(
    NodeType * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default)  // NOLINT
  {
    subscribe(node, topic, qos);
  }

  /**
   * \brief Constructor
   *
   * See the agnocast::create_subscription() variants for more information on the parameters
   *
   * \param node The rclcpp::Node::SharedPtr to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos The rmw qos profile to use to subscribe.
   * \param options The subscription options to use to subscribe.
   */
  Subscriber(
    // cppcheck-suppress passedByValue  // shared_ptr by-value is intentional (shared ownership)
    NodePtr node, const std::string & topic, const rmw_qos_profile_t qos,
    agnocast::SubscriptionOptions options)
  {
    subscribe(node, topic, qos, options);
  }

  Subscriber(
    NodeType * node, const std::string & topic, const rmw_qos_profile_t qos,
    agnocast::SubscriptionOptions options)
  {
    subscribe(node, topic, qos, options);
  }

  /**
   * \brief Empty constructor, use subscribe() to subscribe to a topic
   */
  Subscriber() = default;

  ~Subscriber() override { unsubscribe(); }

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param node The rclcpp::Node::SharedPtr to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos (optional) The rmw qos profile to use to subscribe.
   */
  // cppcheck-suppress virtualCallInConstructor  // Subscriber is not intended to be derived
  void subscribe(
    NodePtr node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default) override
  {
    subscribe(node.get(), topic, qos, agnocast::SubscriptionOptions{});
    node_raw_ = nullptr;
    node_shared_ = node;
  }

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param node The rclcpp::Node to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos (optional) The rmw qos profile to use to subscribe.
   */
  // cppcheck-suppress virtualCallInConstructor  // Subscriber is not intended to be derived
  void subscribe(
    NodeType * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default) override
  {
    subscribe(node, topic, qos, agnocast::SubscriptionOptions{});
  }

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param node The rclcpp::Node::SharedPtr to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos The rmw qos profile to use to subscribe.
   * \param options The subscription options to use to subscribe.
   */
  // cppcheck-suppress virtualCallInConstructor  // Subscriber is not intended to be derived
  void subscribe(
    NodePtr node, const std::string & topic, const rmw_qos_profile_t qos,
    agnocast::SubscriptionOptions options) override
  {
    subscribe(node.get(), topic, qos, options);
    node_raw_ = nullptr;
    node_shared_ = node;
  }

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param node The rclcpp::Node to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos The rmw qos profile to use to subscribe.
   * \param options The subscription options to use to subscribe.
   */
  // cppcheck-suppress virtualCallInConstructor  // Subscriber is not intended to be derived
  void subscribe(
    NodeType * node, const std::string & topic, const rmw_qos_profile_t qos,
    agnocast::SubscriptionOptions options) override
  {
    unsubscribe();

    if (!topic.empty()) {
      topic_ = topic;
      qos_ = qos;
      options_ = options;
      sub_ = agnocast::create_subscription<M>(
        node, topic, detail::to_rclcpp_qos(qos),
        [this](ipc_shared_ptr<M> msg) { this->cb(std::move(msg)); }, options);
      node_raw_ = node;
    }
  }

  /**
   * \brief Re-subscribe to a topic.  Only works if this subscriber has previously been subscribed
   * to a topic.
   */
  void subscribe() override
  {
    if (!topic_.empty()) {
      if (node_raw_ != nullptr) {
        subscribe(node_raw_, topic_, qos_, options_);
      } else if (node_shared_ != nullptr) {
        subscribe(node_shared_, topic_, qos_, options_);
      }
    }
  }

  /**
   * \brief Force immediate unsubscription of this subscriber from its topic
   */
  // cppcheck-suppress virtualCallInConstructor  // Subscriber is not intended to be derived
  void unsubscribe() override { sub_.reset(); }

  std::string getTopic() const { return topic_; }

  /**
   * \brief Returns the internal agnocast::Subscription<M>::SharedPtr object
   */
  const typename agnocast::Subscription<M>::SharedPtr getSubscriber() const { return sub_; }

  /**
   * \brief Does nothing.  Provided so that Subscriber may be used in a message_filters::Chain
   */
  template <typename F>
  void connectInput(F & f)
  {
    (void)f;
  }

  /**
   * \brief Does nothing.  Provided so that Subscriber may be used in a message_filters::Chain
   */
  void add(const EventType & e) { (void)e; }

private:
  void cb(ipc_shared_ptr<M> msg) { this->signalMessage(EventType(std::move(msg))); }

  typename agnocast::Subscription<M>::SharedPtr sub_;

  NodePtr node_shared_;
  NodeType * node_raw_{nullptr};

  std::string topic_;
  rmw_qos_profile_t qos_{rmw_qos_profile_default};
  agnocast::SubscriptionOptions options_;
};

}  // namespace message_filters
}  // namespace agnocast
