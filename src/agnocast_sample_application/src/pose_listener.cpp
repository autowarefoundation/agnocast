#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

namespace pose_sample
{

class PoseListener : public rclcpp::Node
{
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;

  void callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(
      get_logger(), "Received: frame_id=%s, stamp=%d.%09d, x=%.3f, y=%.3f, z=%.3f, qw=%.3f",
      msg->header.frame_id.c_str(), msg->header.stamp.sec, msg->header.stamp.nanosec,
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.w);
  }

public:
  explicit PoseListener(const rclcpp::NodeOptions & options)
  : rclcpp::Node("pose_listener", options)
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pose_chatter", 1, std::bind(&PoseListener::callback, this, _1));
  }
};

}  // namespace pose_sample

RCLCPP_COMPONENTS_REGISTER_NODE(pose_sample::PoseListener)
