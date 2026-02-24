#include "agnocast_sample_interfaces/msg/dynamic_size_array.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
const long long MESSAGE_SIZE = 1000ll * 1024;

class Ros2Publisher : public rclcpp::Node
{
  int64_t count_;
  rclcpp::Publisher<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback()
  {
    auto message = agnocast_sample_interfaces::msg::DynamicSizeArray();

    message.id = count_;
    message.data.reserve(MESSAGE_SIZE / sizeof(uint64_t));
    for (size_t i = 0; i < MESSAGE_SIZE / sizeof(uint64_t); i++) {
      message.data.push_back(i + count_);
    }

    pub_->publish(std::move(message));
    RCLCPP_INFO(get_logger(), "publish message: id=%ld", count_++);
  }

public:
  explicit Ros2Publisher() : Node("test_ros2_publisher")
  {
    count_ = 0;

    pub_ =
      this->create_publisher<agnocast_sample_interfaces::msg::DynamicSizeArray>("/my_topic", 1);

    // Use rclcpp::create_timer with explicit Clock (STEADY_TIME) to match
    // the original Agnocast version's create_timer path.
    // This exercises the full rclcpp timer initialization tracepoints:
    //   - rcl_timer_init
    //   - rclcpp_timer_callback_added
    //   - rclcpp_timer_link_node
    //   - rclcpp_callback_register
    timer_ = rclcpp::create_timer(
      this, std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME), 100ms,
      std::bind(&Ros2Publisher::timer_callback, this));
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<Ros2Publisher>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
