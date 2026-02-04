#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int64.hpp"

#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class TestROS2Publisher : public rclcpp::Node
{
  enum class State {
    WaitingForConnection,      // Waiting for subscriber connections
    WaitingForTransientLocal,  // Waiting for transient local messages (5 seconds)
    Publishing                 // Normal publishing
  };

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
  int64_t count_;
  int64_t target_pub_num_;

  size_t planned_sub_count_;
  size_t planned_pub_count_;

  State state_ = State::WaitingForConnection;
  rclcpp::Time connection_detected_time_;

  bool forever_;

  bool check_connection_counts()
  {
    if (
      publisher_->get_subscription_count() < planned_sub_count_ ||
      this->count_publishers("/test_topic") < planned_pub_count_) {
      return false;
    }
    return true;
  }

  void timer_callback()
  {
    switch (state_) {
      case State::WaitingForConnection:
        if (check_connection_counts()) {
          connection_detected_time_ = this->now();
          state_ = State::WaitingForTransientLocal;
        }
        break;

      case State::WaitingForTransientLocal:
        // HACK: wait subscribing transient local messages
        if ((this->now() - connection_detected_time_) >= rclcpp::Duration::from_seconds(5.0)) {
          state_ = State::Publishing;
          publish_message();
        }
        break;

      case State::Publishing:
        publish_message();
        break;
    }
  }

  void publish_message()
  {
    auto message = std::make_unique<std_msgs::msg::Int64>();
    message->data = count_;
    publisher_->publish(std::move(message));

    RCLCPP_INFO(this->get_logger(), "Publishing %ld.", count_);
    count_++;

    if (count_ == target_pub_num_) {
      RCLCPP_INFO(this->get_logger(), "All messages published. Shutting down.");
      std::cout << std::flush;
      sleep(3);  // HACK: wait for other nodes in the same container

      if (!forever_) {
        rclcpp::shutdown();
      } else {
        timer_->cancel();
      }
    }
  }

public:
  explicit TestROS2Publisher(const rclcpp::NodeOptions & options)
  : Node("test_ros2_publisher", options)
  {
    this->declare_parameter<int64_t>("qos_depth", 10);
    this->declare_parameter<bool>("transient_local", true);
    this->declare_parameter<int64_t>("init_pub_num", 10);
    this->declare_parameter<int64_t>("pub_num", 10);
    this->declare_parameter<int64_t>("planned_sub_count", 1);
    this->declare_parameter<int64_t>("planned_pub_count", 1);
    this->declare_parameter<bool>("forever", false);

    int64_t qos_depth = this->get_parameter("qos_depth").as_int();
    bool transient_local = this->get_parameter("transient_local").as_bool();
    int64_t init_pub_num = this->get_parameter("init_pub_num").as_int();
    int64_t pub_num = this->get_parameter("pub_num").as_int();

    planned_sub_count_ = this->get_parameter("planned_sub_count").as_int();
    planned_pub_count_ = this->get_parameter("planned_pub_count").as_int();
    forever_ = this->get_parameter("forever").as_bool();

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));
    if (transient_local) {
      qos.transient_local();
      qos.reliable();
    }

    publisher_ = this->create_publisher<std_msgs::msg::Int64>("/test_topic", qos);

    count_ = 0;
    target_pub_num_ = init_pub_num + pub_num;

    // Initial publish
    for (int i = 0; i < init_pub_num; i++) {
      auto message = std::make_unique<std_msgs::msg::Int64>();
      message->data = count_;
      publisher_->publish(std::move(message));
      RCLCPP_INFO(this->get_logger(), "Publishing %ld.", count_);
      count_++;
    }

    timer_ = this->create_wall_timer(100ms, std::bind(&TestROS2Publisher::timer_callback, this));
  }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TestROS2Publisher)
