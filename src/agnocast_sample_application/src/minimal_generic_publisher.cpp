#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/dynamic_size_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

using namespace std::chrono_literals;
const long long MESSAGE_SIZE = 1000ll * 1024;

class MinimalGenericPublisher : public rclcpp::Node
{
  int64_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  agnocast::GenericPublisher::SharedPtr publisher_;

  void timer_callback()
  {
    agnocast_sample_interfaces::msg::DynamicSizeArray message;
    message.id = count_;
    message.data.reserve(MESSAGE_SIZE / sizeof(uint64_t));
    for (size_t i = 0; i < MESSAGE_SIZE / sizeof(uint64_t); i++) {
      message.data.push_back(i + count_);
    }

    rclcpp::Serialization<agnocast_sample_interfaces::msg::DynamicSizeArray> serializer;
    rclcpp::SerializedMessage serialized;
    serializer.serialize_message(&message, &serialized);

    publisher_->publish(serialized);
    RCLCPP_INFO(this->get_logger(), "publish message: id=%ld", count_++);
  }

public:
  MinimalGenericPublisher() : Node("minimal_generic_publisher")
  {
    count_ = 0;

    publisher_ = agnocast::create_generic_publisher(
      this, "/my_topic", "agnocast_sample_interfaces/msg/DynamicSizeArray", 1);

    timer_ =
      this->create_wall_timer(100ms, std::bind(&MinimalGenericPublisher::timer_callback, this));
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  agnocast::SingleThreadedAgnocastExecutor executor;
  auto node = std::make_shared<MinimalGenericPublisher>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
