#ifdef USE_AGNOCAST_NODE
#include "agnocast/node/agnocast_node.hpp"
#else
#include "agnocast/agnocast.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int64.hpp"

using std::placeholders::_1;

#ifdef USE_AGNOCAST_NODE
using BaseNode = agnocast::Node;
#define NODE_NAME "test_agnocast_node_subscription"
#define CLASS_NAME TestAgnocastNodeSubscriber
#else
using BaseNode = rclcpp::Node;
#define NODE_NAME "test_subscription"
#define CLASS_NAME TestSubscriber
#endif

template <typename MessageT, typename NodeT, typename CallbackT>
auto create_agnocast_subscription(
  NodeT * node, const std::string & topic_name, const rclcpp::QoS & qos, CallbackT && callback,
  const agnocast::SubscriptionOptions & options)
{
  if constexpr (std::is_base_of_v<agnocast::Node, NodeT>) {
    return node->template create_subscription<MessageT>(
      topic_name, qos, std::forward<CallbackT>(callback), options);
  } else {
    return agnocast::create_subscription<MessageT>(
      node, topic_name, qos, std::forward<CallbackT>(callback), options);
  }
}

class CLASS_NAME : public BaseNode
{
  agnocast::Subscription<std_msgs::msg::Int64>::SharedPtr sub_;
  bool forever_;
  int64_t target_end_id_;
  int target_end_count_;
  int received_end_count_ = 0;

  void callback(const agnocast::ipc_shared_ptr<std_msgs::msg::Int64> & message)
  {
    RCLCPP_INFO(this->get_logger(), "Receiving %ld.", message->data);

    if (message->data == target_end_id_) {
      received_end_count_++;

      if (received_end_count_ >= target_end_count_) {
        RCLCPP_INFO(this->get_logger(), "All messages received. Shutting down.");
        std::cout << std::flush;
        sleep(3);  // HACK: wait for other nodes in the same container

        if (!forever_) {
          rclcpp::shutdown();
        }
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
    this->declare_parameter<int>("target_end_count", 1);
    std::string topic_name = this->get_parameter("topic_name").as_string();
    forever_ = this->get_parameter("forever").as_bool();
    target_end_id_ = this->get_parameter("target_end_id").as_int();
    target_end_count_ = this->get_parameter("target_end_count").as_int();

    int64_t qos_depth = this->get_parameter("qos_depth").as_int();
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));
    if (this->get_parameter("transient_local").as_bool()) {
      qos.transient_local();
    }

    auto cbg = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    agnocast::SubscriptionOptions sub_options;
    sub_options.callback_group = cbg;
    sub_ = create_agnocast_subscription<std_msgs::msg::Int64>(
      this, topic_name, qos, std::bind(&CLASS_NAME::callback, this, _1), sub_options);
  }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CLASS_NAME)
