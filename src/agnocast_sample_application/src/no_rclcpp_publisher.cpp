#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/msg/dynamic_size_array.hpp"

using namespace std::chrono_literals;
const long long MESSAGE_SIZE = 1000ll * 1024;

class NoRclcppPublisher : public agnocast::Node
{
  int64_t count_;
  agnocast::Publisher<agnocast_sample_interfaces::msg::DynamicSizeArray>::SharedPtr pub_;

  void timer_callback()
  {
    auto message = pub_->borrow_loaned_message();

    message->id = count_;
    message->data.reserve(MESSAGE_SIZE / sizeof(uint64_t));
    for (size_t i = 0; i < MESSAGE_SIZE / sizeof(uint64_t); i++) {
      message->data.push_back(i + count_);
    }

    pub_->publish(std::move(message));
    RCLCPP_INFO(get_logger(), "publish message: id=%ld", count_++);
  }

public:
  explicit NoRclcppPublisher() : Node("no_rclcpp_publisher")
  {
    count_ = 0;

    pub_ =
      this->create_publisher<agnocast_sample_interfaces::msg::DynamicSizeArray>("/my_topic", 1);

    this->create_wall_timer(100ms, std::bind(&NoRclcppPublisher::timer_callback, this));
  }
};

int main(int argc, char ** argv)
{
  agnocast::init(argc, argv);
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  auto node = std::make_shared<NoRclcppPublisher>();
  executor.add_node(node);
  executor.spin();
  return 0;
}
