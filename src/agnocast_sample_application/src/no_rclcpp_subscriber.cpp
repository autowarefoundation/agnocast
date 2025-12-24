#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/dynamic_size_array.hpp"

#include <iostream>

using std::placeholders::_1;

class NoRclcppSubscriber : public agnocast::Node
{
  agnocast::Subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr sub_dynamic_;

  std::string topic_name_;
  int64_t queue_size_;

  void callback(
    const agnocast::ipc_shared_ptr<agnocast_sample_interfaces::msg::DynamicSizeArray> & message)
  {
    RCLCPP_INFO(
      get_logger(), "I heard dynamic size array message with size: %zu", message->data.size());
  }

public:
  explicit NoRclcppSubscriber() : agnocast::Node("no_rclcpp_subscriber")
  {
    declare_parameter("topic_name", rclcpp::ParameterValue(std::string("my_topic")));
    declare_parameter("queue_size", rclcpp::ParameterValue(int64_t(1)));

    get_parameter("topic_name", topic_name_);
    get_parameter("queue_size", queue_size_);

    std::string resolved_topic = get_node_topics_interface()->resolve_topic_name(topic_name_);

    RCLCPP_INFO(get_logger(), "=== NoRclcppSubscriber Node Info ===");
    RCLCPP_INFO(get_logger(), "Fully qualified name: %s", get_fully_qualified_name().c_str());
    RCLCPP_INFO(get_logger(), "Topic name (input): %s", topic_name_.c_str());
    RCLCPP_INFO(get_logger(), "Topic name (resolved): %s", resolved_topic.c_str());
    RCLCPP_INFO(get_logger(), "Queue size: %ld", queue_size_);
    RCLCPP_INFO(get_logger(), "====================================");

    auto group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    agnocast::SubscriptionOptions agnocast_options;
    agnocast_options.callback_group = group;

    sub_dynamic_ = this->create_subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>(
      resolved_topic, static_cast<size_t>(queue_size_),
      std::bind(&NoRclcppSubscriber::callback, this, _1), agnocast_options);
  }
};

int main(int argc, char ** argv)
{
  agnocast::init(argc, argv);
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  auto node = std::make_shared<NoRclcppSubscriber>();
  executor.add_node(node);
  executor.spin();
  return 0;
}
