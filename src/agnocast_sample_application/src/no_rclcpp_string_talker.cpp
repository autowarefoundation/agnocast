#include "agnocast/agnocast.hpp"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class NoRclcppStringTalker : public agnocast::Node
{
  int64_t count_;
  agnocast::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  agnocast::TimerBase::SharedPtr timer_;

  void timer_callback()
  {
    auto message = pub_->borrow_loaned_message();
    message->data = "Hello, Agnocast! count=" + std::to_string(count_);

    RCLCPP_INFO(get_logger(), "Publishing: '%s'", message->data.c_str());

    pub_->publish(std::move(message));
    ++count_;
  }

public:
  explicit NoRclcppStringTalker() : Node("no_rclcpp_string_talker"), count_(0)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("/string_chatter", 1);

    timer_ = agnocast::create_timer(
      this, std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME), rclcpp::Duration(500ms),
      std::bind(&NoRclcppStringTalker::timer_callback, this));
  }
};

int main(int argc, char ** argv)
{
  agnocast::init(argc, argv);
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  auto node = std::make_shared<NoRclcppStringTalker>();
  executor.add_node(node);
  executor.spin();
  return 0;
}
