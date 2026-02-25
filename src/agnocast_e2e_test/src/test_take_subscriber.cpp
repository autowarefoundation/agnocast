#ifdef USE_AGNOCAST_NODE
#include "agnocast/node/agnocast_node.hpp"
#else
#include "agnocast/agnocast.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int64.hpp"

#include <chrono>
using namespace std::chrono_literals;

#ifdef USE_AGNOCAST_NODE
using BaseNode = agnocast::Node;
using TimerSharedPtr = agnocast::TimerBase::SharedPtr;
#define NODE_NAME "test_agnocast_node_take_subscription"
#define CLASS_NAME TestAgnocastNodeTakeSubscriber
#else
using BaseNode = rclcpp::Node;
using TimerSharedPtr = rclcpp::TimerBase::SharedPtr;
#define NODE_NAME "test_take_subscription"
#define CLASS_NAME TestTakeSubscriber
#endif

class CLASS_NAME : public BaseNode
{
  TimerSharedPtr timer_;
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
  explicit CLASS_NAME(const rclcpp::NodeOptions & options) : BaseNode(NODE_NAME, options)
  {
    this->declare_parameter<std::string>("topic_name", "/test_topic");
    this->declare_parameter<int64_t>("qos_depth", 10);
    this->declare_parameter<bool>("transient_local", true);
    this->declare_parameter<bool>("forever", false);
    this->declare_parameter<int64_t>("target_end_id", 0);
    std::string topic_name = this->get_parameter("topic_name").as_string();
    forever_ = this->get_parameter("forever").as_bool();
    target_end_id_ = this->get_parameter("target_end_id").as_int();

    int64_t qos_depth = this->get_parameter("qos_depth").as_int();
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));
    if (this->get_parameter("transient_local").as_bool()) {
      qos.transient_local();
    }

    sub_ =
      std::make_shared<agnocast::TakeSubscription<std_msgs::msg::Int64>>(this, topic_name, qos);
    timer_ = this->create_wall_timer(10ms, std::bind(&CLASS_NAME::timer_callback, this));
  }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CLASS_NAME)
