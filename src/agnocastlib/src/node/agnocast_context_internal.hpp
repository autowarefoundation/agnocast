#pragma once

#include <rclcpp/context.hpp>

namespace agnocast
{

// Lazily bring up the Agnocast global state when agnocast::init() was not called.
//
// agnocast::init() is only reachable from main(), so it never runs in a process whose
// main() belongs to someone else. The motivating case is a component container: its
// main() calls rclcpp::init() and then dlopens agnocast::Node components. Without this
// helper such a process has no signal handler installed and reports agnocast::ok() ==
// false, which makes the Agnocast-only executors that agnocast::Node spawns internally
// (the use_sim_time clock thread and the tf2 TransformListener thread) unusable.
//
// Idempotent, and safe to call after rclcpp::init(): SignalHandler::install() chains to
// the handler rclcpp already installed instead of replacing it.
//
// Unlike agnocast::init(), this does not parse command-line arguments and does not
// configure rcl logging, so it never takes over global state that rclcpp owns.
//
// Pass the rclcpp context the caller belongs to, when it has one; see
// Context::adopt_rclcpp_context().
void ensure_initialized(const rclcpp::Context::SharedPtr & rclcpp_context = nullptr);

}  // namespace agnocast
