#include "agnocast/agnocast.hpp"
#include "agnocast_sample_interfaces/srv/sum_int_array.hpp"

#include <thread>

using namespace std::chrono_literals;

constexpr size_t ARRAY_SIZE = 100;

int main(int argc, char * argv[])
{
  agnocast::init(argc, argv);

  auto node = std::make_shared<agnocast::Node>("no_rclcpp_client");
  std::thread spin_thread([node]() {
    agnocast::AgnocastOnlySingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  });

  using ServiceT = agnocast_sample_interfaces::srv::SumIntArray;
  auto client = node->create_client<ServiceT>("sum_int_array");

  // TODO(Koichi98): Add agnocast::ok() check here
  while (!client->wait_for_service(1s)) {
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  auto request1 = client->borrow_loaned_request();
  for (size_t i = 1; i <= ARRAY_SIZE; ++i) {
    request1->data.push_back(i);
  }
  client->async_send_request(
    std::move(request1), [node](agnocast::Client<ServiceT>::SharedFuture future) {
      RCLCPP_INFO(node->get_logger(), "Result1: %ld", future.get()->sum);
    });

  auto request2 = client->borrow_loaned_request();
  for (size_t i = 0; i < ARRAY_SIZE; ++i) {
    request2->data.push_back(i);
  }
  auto future = client->async_send_request(std::move(request2));
  RCLCPP_INFO(node->get_logger(), "Result2: %ld", future.get()->sum);

  spin_thread.join();
  // TODO(Koichi98): Add agnocast::shutdown() here
  return 0;
}
