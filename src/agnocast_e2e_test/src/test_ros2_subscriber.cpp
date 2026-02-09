#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int64.hpp"

using std::placeholders::_1;

class TestROS2Subscriber : public rclcpp::Node
{
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_;
  bool forever_;
  int64_t target_end_id_;
  int target_end_count_;
  int received_end_count_ = 0;

  void callback(const std_msgs::msg::Int64 & message)
  {
    RCLCPP_INFO(this->get_logger(), "Receiving %ld.", message.data);

    if (message.data == target_end_id_) {
      received_end_count_++;

      if (received_end_count_ >= target_end_count_) {
        RCLCPP_INFO(this->get_logger(), "All messages received. Shutting down.");
        std::cout << std::flush;
        sleep(3);

        if (!forever_) {
          rclcpp::shutdown();
        }
      }
    }
  }

public:
  explicit TestROS2Subscriber(const rclcpp::NodeOptions & options)
  : Node("test_ros2_subscription", options)
  {
    this->declare_parameter<int64_t>("qos_depth", 10);
    this->declare_parameter<bool>("transient_local", true);
    this->declare_parameter<bool>("forever", false);
    this->declare_parameter<int64_t>("target_end_id", 0);
    this->declare_parameter<int>("target_end_count", 1);
    int64_t qos_depth = this->get_parameter("qos_depth").as_int();
    bool transient_local = this->get_parameter("transient_local").as_bool();
    forever_ = this->get_parameter("forever").as_bool();
    target_end_id_ = this->get_parameter("target_end_id").as_int();
    target_end_count_ = this->get_parameter("target_end_count").as_int();

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));
    if (transient_local) {
      qos.transient_local();
    }

    sub_ = this->create_subscription<std_msgs::msg::Int64>(
      "/test_topic", qos, std::bind(&TestROS2Subscriber::callback, this, _1));
  }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TestROS2Subscriber)
