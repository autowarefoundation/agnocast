#pragma once

#include <rclcpp/context.hpp>

namespace agnocast
{

// Bring the Agnocast context up in a process that called rclcpp::init() instead of
// agnocast::init(), such as a component container. Without it there is no signal handler and
// agnocast::ok() is false, stranding the Agnocast-only executors an agnocast::Node spawns
// internally.
//
// Unlike agnocast::init(), it parses no command line and configures no rcl logging, so it never
// takes over global state that rclcpp owns. Pass the rclcpp context the caller belongs to.
void ensure_initialized(const rclcpp::Context::SharedPtr & rclcpp_context = nullptr);

}  // namespace agnocast
