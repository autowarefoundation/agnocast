#include "agnocast_sample_interfaces/srv/sum_int_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include <thread>

using namespace std::chrono_literals;

constexpr size_t ARRAY_SIZE = 100;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ros_client");
  std::thread spin_thread([node]() mutable {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  });

  auto client = node->create_client<agnocast_sample_interfaces::srv::SumIntArray>("sum_int_array");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  auto request1 =
    std::make_shared<typename agnocast_sample_interfaces::srv::SumIntArray::Request>();
  for (size_t i = 1; i <= ARRAY_SIZE; ++i) {
    request1->data.push_back(i);
  }
  client->async_send_request(
    std::move(request1),
    [node = node.get()](
      rclcpp::Client<agnocast_sample_interfaces::srv::SumIntArray>::SharedFuture future) {
      RCLCPP_INFO(node->get_logger(), "Result1: %ld", future.get()->sum);
    });

  auto request2 =
    std::make_shared<typename agnocast_sample_interfaces::srv::SumIntArray::Request>();
  for (size_t i = 0; i < ARRAY_SIZE; ++i) {
    request2->data.push_back(i);
  }
  auto future = client->async_send_request(std::move(request2));
  RCLCPP_INFO(node->get_logger(), "Result2: %ld", future.get()->sum);

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
