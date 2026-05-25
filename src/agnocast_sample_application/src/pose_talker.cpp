#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class PoseTalker : public rclcpp::Node
{
  int64_t count_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback()
  {
    auto message = std::make_unique<geometry_msgs::msg::PoseStamped>();

    message->header.stamp = now();
    message->header.frame_id = "map";
    message->pose.position.x = static_cast<double>(count_) * 0.1;
    message->pose.position.y = static_cast<double>(count_) * 0.05;
    message->pose.position.z = 0.0;
    message->pose.orientation.x = 0.0;
    message->pose.orientation.y = 0.0;
    message->pose.orientation.z = 0.0;
    message->pose.orientation.w = 1.0;

    RCLCPP_INFO(
      get_logger(), "Publishing [%ld]: frame_id=%s, x=%.2f, y=%.2f", count_,
      message->header.frame_id.c_str(), message->pose.position.x, message->pose.position.y);

    pub_->publish(std::move(message));
    ++count_;
  }

public:
  explicit PoseTalker() : Node("pose_talker"), count_(0)
  {
    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_chatter", 1);

    timer_ = this->create_wall_timer(500ms, std::bind(&PoseTalker::timer_callback, this));
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseTalker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
