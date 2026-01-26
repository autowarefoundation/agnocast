#include <cie_thread_configurator/cie_thread_configurator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>
#include <thread>

namespace agnocastlib_test
{

class TestPublisherComponent : public rclcpp::Node
{
public:
  explicit TestPublisherComponent(const rclcpp::NodeOptions & options)
  : Node("test_publisher_component", options), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("test_topic", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
      auto message = std_msgs::msg::String();
      message.data = "Hello from test component: " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    });

    // Create a callback group that is NOT automatically added to executor
    no_exec_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false /* automatically_add_to_executor */);
    no_exec_timer_ = this->create_wall_timer(
      std::chrono::seconds(10),
      []() {
        // This callback should never be called since it's not added to executor
      },
      no_exec_callback_group_);

    // Test spawn_non_ros2_thread
    non_ros_thread_ = cie_thread_configurator::spawn_non_ros2_thread(
      "test_non_ros_worker", &TestPublisherComponent::non_ros_thread_func, this);

    RCLCPP_INFO(this->get_logger(), "TestPublisherComponent initialized");
  }

  ~TestPublisherComponent()
  {
    if (non_ros_thread_.joinable()) {
      non_ros_thread_.join();
    }
  }

private:
  void non_ros_thread_func() {}

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr no_exec_callback_group_;
  rclcpp::TimerBase::SharedPtr no_exec_timer_;
  int count_;

  std::thread non_ros_thread_;
};

}  // namespace agnocastlib_test

RCLCPP_COMPONENTS_REGISTER_NODE(agnocastlib_test::TestPublisherComponent)
