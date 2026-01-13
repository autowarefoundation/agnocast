#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <std_msgs/msg/string.hpp>

#include <memory>

namespace agnocastlib_test
{

class TestSubscriptionComponent : public rclcpp::Node
{
public:
  explicit TestSubscriptionComponent(const rclcpp::NodeOptions & options)
  : Node("test_subscription_component", options), received_count_(0)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "test_topic", 10, [this](std_msgs::msg::String::SharedPtr msg) {
        received_count_++;
        RCLCPP_INFO(
          this->get_logger(), "Received: '%s' (count: %d)", msg->data.c_str(), received_count_);
      });

    RCLCPP_INFO(this->get_logger(), "TestSubscriptionComponent initialized");
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  int received_count_;
};

}  // namespace agnocastlib_test

RCLCPP_COMPONENTS_REGISTER_NODE(agnocastlib_test::TestSubscriptionComponent)
