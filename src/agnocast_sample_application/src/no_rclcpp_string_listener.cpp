#include "agnocast/agnocast.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class NoRclcppStringListener : public agnocast::Node
{
  agnocast::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  void callback(const agnocast::ipc_shared_ptr<std_msgs::msg::String> & message)
  {
    RCLCPP_INFO(get_logger(), "I heard: '%s'", message->data.c_str());
  }

public:
  explicit NoRclcppStringListener(const rclcpp::NodeOptions & options)
  : agnocast::Node("no_rclcpp_string_listener", options)
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/string_chatter", 1, std::bind(&NoRclcppStringListener::callback, this, _1));
  }
};

RCLCPP_COMPONENTS_REGISTER_NODE(NoRclcppStringListener)
