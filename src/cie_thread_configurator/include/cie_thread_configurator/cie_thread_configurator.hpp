#pragma once

#include "rclcpp/rclcpp.hpp"

#include "cie_config_msgs/msg/callback_group_info.hpp"
#include "cie_config_msgs/msg/non_ros_thread_info.hpp"

#include <sys/syscall.h>
#include <unistd.h>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace cie_thread_configurator
{

std::string create_callback_group_id(
  rclcpp::CallbackGroup::SharedPtr group, rclcpp::Node::SharedPtr node,
  const std::vector<std::string> & agnocast_topics);

std::string create_callback_group_id(
  rclcpp::CallbackGroup::SharedPtr group,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
  const std::vector<std::string> & agnocast_topics);

// Caution: Do not call in parallel
// Caution: Must be called after rclcpp::init() called
rclcpp::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr create_client_publisher();

// `publisher` is assumed to be the return value of create_client_publisher()
void publish_callback_group_info(
  const rclcpp::Publisher<cie_config_msgs::msg::CallbackGroupInfo>::SharedPtr & publisher,
  int64_t tid, const std::string & callback_group_id);

// Get hardware information from lscpu command
std::map<std::string, std::string> get_hardware_info();

// Get default domain ID from ROS_DOMAIN_ID environment variable
size_t get_default_domain_id();

// Create a node for a different domain
rclcpp::Node::SharedPtr create_node_for_domain(size_t domain_id);

/// Spawn a thread whose scheduling policy can be managed through
/// cie_thread_configurator.
/// Caution: the `thread_name` must be unique among threads managed by
/// cie_thread_configurator.
template <class F, class... Args>
std::thread spawn_non_ros2_thread(const char * thread_name, F && f, Args &&... args)
{
  std::thread t([thread_name = std::string(thread_name), func = std::forward<F>(f),
                 captured_args = std::make_tuple(std::forward<Args>(args)...)]() mutable {
    // Create isolated rclcpp context for publishing thread info
    rclcpp::InitOptions init_options;
    init_options.shutdown_on_signal = false;
    init_options.auto_initialize_logging(false);
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr, init_options);

    rclcpp::NodeOptions options;
    options.context(context);
    auto node =
      std::make_shared<rclcpp::Node>("cie_thread_client", "/cie_thread_configurator", options);

    auto publisher = node->create_publisher<cie_config_msgs::msg::NonRosThreadInfo>(
      "/cie_thread_configurator/non_ros_thread_info", rclcpp::QoS(1000).reliable());
    auto tid = static_cast<pid_t>(syscall(SYS_gettid));

    // Wait for subscriber to connect before publishing (timeout: 1 second)
    constexpr int max_subscriber_wait_iterations = 100;  // 100 * 10ms = 1 second
    int wait_count = 0;
    while (publisher->get_subscription_count() == 0 &&
           wait_count < max_subscriber_wait_iterations) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      ++wait_count;
    }

    if (publisher->get_subscription_count() > 0) {
      auto message = std::make_shared<cie_config_msgs::msg::NonRosThreadInfo>();
      message->thread_id = tid;
      message->thread_name = thread_name;
      publisher->publish(*message);
      const bool all_acked = publisher->wait_for_all_acked(std::chrono::milliseconds(500));
      if (!all_acked) {
        RCLCPP_WARN(
          node->get_logger(),
          "Timed out waiting for NonRosThreadInfo acknowledgment (thread '%s').",
          thread_name.c_str());
      }
    } else {
      RCLCPP_WARN(
        node->get_logger(),
        "No subscriber for NonRosThreadInfo (thread '%s'). "
        "Please run thread_configurator_node if you want to configure thread scheduling.",
        thread_name.c_str());
    }

    context->shutdown("cie_thread_client finished.");

    std::apply(std::move(func), std::move(captured_args));
  });
  return t;
}

}  // namespace cie_thread_configurator
