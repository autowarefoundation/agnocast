// To test this node, change the subscription topic of no_rclcpp_subscriber to
// /my_topic_republished.

#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/dynamic_size_array.hpp"

#include <iostream>

using std::placeholders::_1;

class NoRclcppPubSub : public agnocast::Node
{
  agnocast::Subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr sub_;
  agnocast::Publisher<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr pub_;

  void callback(
    const agnocast::ipc_shared_ptr<agnocast_sample_interfaces::msg::DynamicSizeArray> & message)
  {
    RCLCPP_INFO(
      get_logger(), "Received message with size: %zu, republishing...", message->data.size());

    auto pub_message = pub_->borrow_loaned_message();
    pub_message->data = message->data;
    pub_->publish(std::move(pub_message));
  }

public:
  explicit NoRclcppPubSub() : Node("no_rclcpp_pubsub")
  {
    RCLCPP_INFO(get_logger(), "NoRclcppPubSub node (name=%s) started.", get_name().c_str());

    sub_ = this->create_subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>(
      "/my_topic", 1, std::bind(&NoRclcppPubSub::callback, this, _1));

    pub_ = this->create_publisher<agnocast_sample_interfaces::msg::DynamicSizeArray>(
      "/my_topic_republished", 1);
  }
};

int main(int argc, char ** argv)
{
  agnocast::init(argc, argv);
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  auto node = std::make_shared<NoRclcppPubSub>();
  executor.add_node(node);
  executor.spin();
  return 0;
}
