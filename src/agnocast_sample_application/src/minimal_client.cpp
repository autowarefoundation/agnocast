#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/srv/sum_int_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include <thread>

using namespace std::chrono_literals;

constexpr size_t ARRAY_SIZE = 100;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("minimal_client");
  std::thread spin_thread([node]() mutable {
    agnocast::CallbackIsolatedAgnocastExecutor executor;
    executor.add_node(node);
    executor.spin();
  });

  auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto client = agnocast::create_client<agnocast_sample_interfaces::srv::SumIntArray>(
    node.get(), "sum_int_array", rclcpp::ServicesQoS(), callback_group);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  auto request1 = client->borrow_loaned_request();
  for (size_t i = 1; i <= ARRAY_SIZE; ++i) {
    request1->data.push_back(i);
  }
  client->async_send_request(
    std::move(request1),
    [node = node.get()](
      agnocast::Client<agnocast_sample_interfaces::srv::SumIntArray>::SharedFuture future) {
      RCLCPP_INFO(node->get_logger(), "Result1: %ld", future.get()->sum);
    });

  auto request2 = client->borrow_loaned_request();
  for (size_t i = 0; i < ARRAY_SIZE; ++i) {
    request2->data.push_back(i);
  }
  auto future = client->async_send_request(std::move(request2));
  RCLCPP_INFO(node->get_logger(), "Result2: %ld", future.get()->sum);

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
