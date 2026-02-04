#include "agnocast/agnocast.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int64.hpp"

#include <chrono>
using namespace std::chrono_literals;

class TestTakeSubscriber : public rclcpp::Node
{
  rclcpp::TimerBase::SharedPtr timer_;
  agnocast::TakeSubscription<std_msgs::msg::Int64>::SharedPtr sub_;
  bool forever_;
  int64_t target_end_id_;

  void timer_callback()
  {
    auto message = sub_->take();

    if (!message) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Receiving %ld.", message->data);

    if (message->data == target_end_id_) {
      RCLCPP_INFO(this->get_logger(), "All messages received. Shutting down.");
      std::cout << std::flush;

      if (!forever_) {
        rclcpp::shutdown();
      }
    }
  }

public:
  explicit TestTakeSubscriber(const rclcpp::NodeOptions & options)
  : Node("test_take_subscription", options)
  {
    this->declare_parameter<int64_t>("qos_depth", 10);
    this->declare_parameter<bool>("transient_local", true);
    this->declare_parameter<bool>("forever", false);
    this->declare_parameter<int64_t>("target_end_id", 0);
    forever_ = this->get_parameter("forever").as_bool();
    target_end_id_ = this->get_parameter("target_end_id").as_int();

    int64_t qos_depth = this->get_parameter("qos_depth").as_int();
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));
    if (this->get_parameter("transient_local").as_bool()) {
      qos.transient_local();
    }

    sub_ =
      std::make_shared<agnocast::TakeSubscription<std_msgs::msg::Int64>>(this, "/test_topic", qos);
    timer_ = this->create_wall_timer(10ms, std::bind(&TestTakeSubscriber::timer_callback, this));
  }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TestTakeSubscriber)
