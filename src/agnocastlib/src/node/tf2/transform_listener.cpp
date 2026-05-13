/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Adapted from tf2_ros::TransformListener for Agnocast zero-copy transport.

#include "agnocast/node/tf2/transform_listener.hpp"

#include "rclcpp/logging.hpp"

#include <cstdio>
#include <string>

namespace agnocast
{

TransformListener::TransformListener(tf2::BufferCore & buffer, bool spin_thread) : buffer_(buffer)
{
  rclcpp::NodeOptions options;
  // create a unique name for the node
  // but specify its name in .arguments to override any __node passed on the command line.
  // avoiding sstream because it's behavior can be overridden by external libraries.
  // See this issue: https://github.com/ros2/geometry2/issues/540
  // NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays, hicpp-avoid-c-arrays, modernize-avoid-c-arrays)
  // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers, cert-err33-c)
  // NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay, hicpp-no-array-decay)
  char node_name[42];
  snprintf(
    node_name, sizeof(node_name), "transform_listener_impl_%zx", reinterpret_cast<size_t>(this));
  options.arguments({"--ros-args", "-r", "__node:=" + std::string(node_name)});
  // NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay, hicpp-no-array-decay)
  // NOLINTEND(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers, cert-err33-c)
  // NOLINTEND(cppcoreguidelines-avoid-c-arrays, hicpp-avoid-c-arrays, modernize-avoid-c-arrays)
  options.start_parameter_event_publisher(false);
  options.start_parameter_services(false);
  optional_default_node_ = std::make_shared<agnocast::Node>("_", options);
  init(
    *optional_default_node_, spin_thread, tf2_ros::DynamicListenerQoS(),
    tf2_ros::StaticListenerQoS(), detail::get_default_transform_listener_sub_options(),
    detail::get_default_transform_listener_static_sub_options());
}

TransformListener::TransformListener(
  tf2::BufferCore & buffer, agnocast::Node & node, bool spin_thread, const rclcpp::QoS & qos,
  const rclcpp::QoS & static_qos, const agnocast::SubscriptionOptions & options,
  const agnocast::SubscriptionOptions & static_options)
: buffer_(buffer)
{
  init(node, spin_thread, qos, static_qos, options, static_options);
}

void TransformListener::init(
  agnocast::Node & node, bool spin_thread, const rclcpp::QoS & qos, const rclcpp::QoS & static_qos,
  const agnocast::SubscriptionOptions & options,
  const agnocast::SubscriptionOptions & static_options)
{
  spin_thread_ = spin_thread;
  node_logging_interface_ = node.get_node_logging_interface();
  node_base_interface_ = node.get_node_base_interface();

  using callback_t = std::function<void(agnocast::ipc_shared_ptr<tf2_msgs::msg::TFMessage> &&)>;
  // NOLINTBEGIN(modernize-avoid-bind)
  callback_t cb =
    std::bind(&TransformListener::subscription_callback, this, std::placeholders::_1, false);
  callback_t static_cb =
    std::bind(&TransformListener::subscription_callback, this, std::placeholders::_1, true);
  // NOLINTEND(modernize-avoid-bind)

  if (spin_thread_) {
    // Create new callback group for message_subscription of tf and tf_static
    callback_group_ =
      node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

    // Duplicate to modify option of subscription
    agnocast::SubscriptionOptions tf_options = options;
    agnocast::SubscriptionOptions tf_static_options = static_options;
    tf_options.callback_group = callback_group_;
    tf_static_options.callback_group = callback_group_;

    message_subscription_tf_ =
      node.create_subscription<tf2_msgs::msg::TFMessage>("/tf", qos, std::move(cb), tf_options);
    message_subscription_tf_static_ = node.create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", static_qos, std::move(static_cb), tf_static_options);

    // Create executor with dedicated thread to spin.
    executor_ = std::make_shared<agnocast::AgnocastOnlySingleThreadedExecutor>();
    executor_->add_callback_group(callback_group_, node_base_interface_);
    dedicated_listener_thread_ =
      std::make_unique<std::thread>([executor = executor_]() { executor->spin(); });
    // Tell the buffer we have a dedicated thread to enable timeouts
    buffer_.setUsingDedicatedThread(true);
  } else {
    message_subscription_tf_ =
      node.create_subscription<tf2_msgs::msg::TFMessage>("/tf", qos, std::move(cb), options);
    message_subscription_tf_static_ = node.create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", static_qos, std::move(static_cb), static_options);
  }
}

TransformListener::~TransformListener()
{
  // Diverges from upstream tf2_ros::TransformListener: defensive null checks guard
  // against future changes that could leave these members null while spin_thread_ is true.
  if (spin_thread_ && executor_) {
    executor_->cancel();
    if (dedicated_listener_thread_) {
      dedicated_listener_thread_->join();
    }
  }
}

void TransformListener::subscription_callback(
  agnocast::ipc_shared_ptr<tf2_msgs::msg::TFMessage> && msg, bool is_static)
{
  const tf2_msgs::msg::TFMessage & msg_in = *msg;
  std::string authority = "Authority undetectable";
  // NOLINTBEGIN(modernize-loop-convert, readability-uppercase-literal-suffix)
  // NOLINTBEGIN(hicpp-uppercase-literal-suffix)
  for (size_t i = 0u; i < msg_in.transforms.size(); i++) {
    try {
      buffer_.setTransform(msg_in.transforms[i], authority, is_static);
    } catch (const tf2::TransformException & ex) {
      std::string temp = ex.what();
      RCLCPP_ERROR(
        node_logging_interface_->get_logger(),
        "Failure to set received transform from %s to %s with error: %s\n",
        msg_in.transforms[i].child_frame_id.c_str(), msg_in.transforms[i].header.frame_id.c_str(),
        temp.c_str());
    }
  }
  // NOLINTEND(hicpp-uppercase-literal-suffix)
  // NOLINTEND(modernize-loop-convert, readability-uppercase-literal-suffix)
}

}  // namespace agnocast
