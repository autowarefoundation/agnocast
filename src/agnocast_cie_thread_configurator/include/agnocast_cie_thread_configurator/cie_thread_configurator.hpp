#pragma once

#include "agnocast_cie_thread_configurator/non_ros_thread_ipc.hpp"
#include "rclcpp/rclcpp.hpp"

#include <sys/syscall.h>
#include <unistd.h>

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <utility>

namespace agnocast_cie_thread_configurator
{

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
    NonRosThreadInfo info;
    info.tid = static_cast<int64_t>(syscall(SYS_gettid));
    info.name = std::move(thread_name);
    send_non_ros_thread_info(info);
    std::apply(std::move(func), std::move(captured_args));
  });
  return t;
}

}  // namespace agnocast_cie_thread_configurator
