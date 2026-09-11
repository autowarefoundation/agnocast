#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/dynamic_size_array.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class CieSubscriber : public rclcpp::Node
{
  rclcpp::CallbackGroup::SharedPtr group1_;
  rclcpp::CallbackGroup::SharedPtr group2_;

  agnocast::Subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr sub_dynamic_;
  agnocast::Subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr
    sub_dynamic2_;

  rclcpp::Subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr sub_ros_;
  rclcpp::Subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr sub_ros2_;

public:
  explicit CieSubscriber(const rclcpp::NodeOptions & options) : Node("minimal_subscriber", options)
  {
    group1_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    group2_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    agnocast::SubscriptionOptions agnocast_options;
    agnocast_options.callback_group = group1_;
    sub_dynamic_ = agnocast::create_subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>(
      this, "/my_topic", 1,
      [](const agnocast::ipc_shared_ptr<agnocast_sample_interfaces::msg::DynamicSizeArray> &
           message) {
        RCLCPP_INFO(
          rclcpp::get_logger("agnocast_sample_application"),
          "receive message id=%ld in group1 from /my_topic", message->id);
      },
      agnocast_options);

    rclcpp::SubscriptionOptions ros_options;
    ros_options.callback_group = group1_;
    sub_ros_ = this->create_subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>(
      "/ros_topic", 1,
      [](const agnocast_sample_interfaces::msg::DynamicSizeArray::SharedPtr message) {
        RCLCPP_INFO(
          rclcpp::get_logger("agnocast_sample_application"),
          "receive message id=%ld in group1 from /ros_topic", message->id);
      },
      ros_options);

    rclcpp::SubscriptionOptions ros_options2;
    ros_options2.callback_group = group2_;
    sub_ros2_ = this->create_subscription<agnocast_sample_interfaces::msg::DynamicSizeArray>(
      "/ros_other_topic", 1,
      [](const agnocast_sample_interfaces::msg::DynamicSizeArray::SharedPtr message) {
        RCLCPP_INFO(
          rclcpp::get_logger("agnocast_sample_application"),
          "receive message id=%ld in group2 from /ros_other_topic", message->id);
      },
      ros_options2);
  }
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CieSubscriber)
