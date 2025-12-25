#pragma once

#include "agnocast/node/node_interfaces/node_base.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"

#include <memory>
#include <string>

namespace agnocast::node_interfaces
{

class NodeTopics : public rclcpp::node_interfaces::NodeTopicsInterface
{
public:
  using SharedPtr = std::shared_ptr<NodeTopics>;
  using WeakPtr = std::weak_ptr<NodeTopics>;

  explicit NodeTopics(NodeBase::SharedPtr node_base);

  virtual ~NodeTopics() = default;

  std::string resolve_topic_name(const std::string & name, bool only_expand = false) const override;
  rclcpp::node_interfaces::NodeBaseInterface * get_node_base_interface() const override;

  // ===== Not supported methods (throw runtime_error) =====
  rclcpp::PublisherBase::SharedPtr create_publisher(
    const std::string & topic_name, const rclcpp::PublisherFactory & publisher_factory,
    const rclcpp::QoS & qos) override;
  void add_publisher(
    rclcpp::PublisherBase::SharedPtr publisher,
    rclcpp::CallbackGroup::SharedPtr callback_group) override;
  rclcpp::SubscriptionBase::SharedPtr create_subscription(
    const std::string & topic_name, const rclcpp::SubscriptionFactory & subscription_factory,
    const rclcpp::QoS & qos) override;
  void add_subscription(
    rclcpp::SubscriptionBase::SharedPtr subscription,
    rclcpp::CallbackGroup::SharedPtr callback_group) override;
  rclcpp::node_interfaces::NodeTimersInterface * get_node_timers_interface() const override;

private:
  NodeBase::SharedPtr node_base_;
};
}  // namespace agnocast::node_interfaces
