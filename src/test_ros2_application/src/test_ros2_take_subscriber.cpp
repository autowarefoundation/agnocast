#include "agnocast_sample_interfaces/msg/dynamic_size_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>

using namespace std::chrono_literals;

class Ros2TakeSubscriber : public rclcpp::Node
{
  rclcpp::Subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Store the latest message received via the subscription callback
  agnocast_sample_interfaces::msg::DynamicSizeArray::SharedPtr latest_message_;
  std::mutex mutex_;

public:
  // Callback groups added for coverage testing
  rclcpp::CallbackGroup::SharedPtr manual_group_;
  rclcpp::CallbackGroup::SharedPtr late_auto_group_;

  explicit Ros2TakeSubscriber() : rclcpp::Node("test_ros2_take_subscriber")
  {
    // Since standard rclcpp doesn't have a PollingSubscriber, we emulate it
    // by using a regular subscription that stores the latest message,
    // and a timer that processes it periodically.
    sub_ = this->create_subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>(
      "/my_topic", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&Ros2TakeSubscriber::subscription_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(1s, std::bind(&Ros2TakeSubscriber::timer_callback, this));

    // [Test Prep 1] Create a group with the auto-add flag set to false
    manual_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

    RCLCPP_INFO(get_logger(), "Ros2TakeSubscriber started");
  }

  void create_late_group()
  {
    // [Test Prep 2] Create this after the node has been added to the Executor
    late_auto_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }

private:
  void subscription_callback(
    const agnocast_sample_interfaces::msg::DynamicSizeArray::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_message_ = message;
  }

  void timer_callback()
  {
    agnocast_sample_interfaces::msg::DynamicSizeArray::SharedPtr message;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      message = latest_message_;
      latest_message_.reset();  // consume the message (take semantics)
    }

    if (message) {
      RCLCPP_INFO(
        get_logger(), "I heard dynamic size array message with id: %ld, size: %zu", message->id,
        message->data.size());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<Ros2TakeSubscriber>();

  // Existing path 3: Automatic addition via add_node
  executor.add_node(node);

  // Uncovered path 1: Direct call to add_callback_group
  executor.add_callback_group(node->manual_group_, node->get_node_base_interface());

  // Uncovered path 2: Create late group (will be picked up by executor automatically)
  node->create_late_group();

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
