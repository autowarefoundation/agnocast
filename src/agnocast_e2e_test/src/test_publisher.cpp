#include "agnocast/agnocast.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int64.hpp"

#include <chrono>

using namespace std::chrono_literals;

class TestPublisher : public rclcpp::Node
{
  rclcpp::TimerBase::SharedPtr timer_;
  agnocast::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
  int64_t count_;
  int64_t target_pub_num_;
  int64_t planned_sub_count_;
  size_t planned_pub_count_;
  bool is_ready_ = false;
  bool forever_;

  bool is_ready()
  {
    if (is_ready_) {
      return true;
    }

    const auto total_sub_count =
      publisher_->get_subscription_count() + publisher_->get_intra_subscription_count();
    if (
      total_sub_count < planned_sub_count_ ||
      this->count_publishers("/test_topic") < planned_pub_count_) {
      return false;
    }

    sleep(5);  // HACK: wait subscribing transient local messages
    is_ready_ = true;
    return true;
  }

  void timer_callback()
  {
    if (!is_ready()) {
      return;
    }

    agnocast::ipc_shared_ptr<std_msgs::msg::Int64> message = publisher_->borrow_loaned_message();
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
      }
    }
  }

public:
  explicit TestPublisher(const rclcpp::NodeOptions & options) : Node("test_publisher", options)
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
    planned_pub_count_ = static_cast<size_t>(this->get_parameter("planned_pub_count").as_int());
    forever_ = this->get_parameter("forever").as_bool();

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));
    if (transient_local) {
      qos.transient_local();
    }
    publisher_ = agnocast::create_publisher<std_msgs::msg::Int64>(this, "/test_topic", qos);
    count_ = 0;
    target_pub_num_ = init_pub_num + pub_num;

    // Initial publish
    for (int i = 0; i < init_pub_num; i++) {
      agnocast::ipc_shared_ptr<std_msgs::msg::Int64> message = publisher_->borrow_loaned_message();
      message->data = count_;
      publisher_->publish(std::move(message));
      RCLCPP_INFO(this->get_logger(), "Publishing %ld.", count_);
      count_++;
    }

    timer_ = this->create_wall_timer(10ms, std::bind(&TestPublisher::timer_callback, this));
  }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TestPublisher)
