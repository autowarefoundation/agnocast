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

#ifndef AGNOCAST__NODE__TF2__TRANSFORM_LISTENER_HPP_
#define AGNOCAST__NODE__TF2__TRANSFORM_LISTENER_HPP_

#include "agnocast/agnocast.hpp"
#include "agnocast/node/agnocast_only_single_threaded_executor.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/qos_overriding_options.hpp"
#include "tf2/buffer_core.hpp"
#include "tf2_ros/qos.hpp"

#include "tf2_msgs/msg/tf_message.hpp"

#include <memory>
#include <string>
#include <thread>

namespace agnocast
{

namespace detail
{
/// \brief Default subscription options for the dynamic /tf listener subscription.
inline agnocast::SubscriptionOptions get_default_transform_listener_sub_options()
{
  agnocast::SubscriptionOptions options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{
    rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability, rclcpp::QosPolicyKind::History,
    rclcpp::QosPolicyKind::Reliability};
  return options;
}

/// \brief Default subscription options for the /tf_static listener subscription.
inline agnocast::SubscriptionOptions get_default_transform_listener_static_sub_options()
{
  agnocast::SubscriptionOptions options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{
    rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::History,
    rclcpp::QosPolicyKind::Reliability};
  return options;
}
}  // namespace detail

/// \brief Listens for transforms via Agnocast zero-copy IPC.
///
/// This class subscribes to /tf and /tf_static topics using Agnocast's
/// zero-copy shared memory transport, and populates a tf2::BufferCore
/// with the received transforms.
class TransformListener
{
public:
  /// \brief Simplified constructor for transform listener.
  ///
  /// This constructor will create a new agnocast::Node under the hood.
  /// If you already have access to an agnocast::Node and you want to associate the
  /// TransformListener to it, then it's recommended to use the other constructor.
  explicit TransformListener(tf2::BufferCore & buffer, bool spin_thread = true);

  /// \brief Node constructor
  TransformListener(
    tf2::BufferCore & buffer, agnocast::Node & node, bool spin_thread = true,
    const rclcpp::QoS & qos = tf2_ros::DynamicListenerQoS(),
    const rclcpp::QoS & static_qos = tf2_ros::StaticListenerQoS(),
    const agnocast::SubscriptionOptions & options =
      detail::get_default_transform_listener_sub_options(),
    const agnocast::SubscriptionOptions & static_options =
      detail::get_default_transform_listener_static_sub_options());

  virtual ~TransformListener();

  /// \brief Callback for /tf and /tf_static messages.
  virtual void subscription_callback(
    agnocast::ipc_shared_ptr<tf2_msgs::msg::TFMessage> && msg, bool is_static);

private:
  void init(
    agnocast::Node & node, bool spin_thread, const rclcpp::QoS & qos,
    const rclcpp::QoS & static_qos, const agnocast::SubscriptionOptions & options,
    const agnocast::SubscriptionOptions & static_options);

  bool spin_thread_{false};
  std::unique_ptr<std::thread> dedicated_listener_thread_{nullptr};
  std::shared_ptr<agnocast::AgnocastOnlySingleThreadedExecutor> executor_{nullptr};

  std::shared_ptr<agnocast::Node> optional_default_node_{nullptr};
  agnocast::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_{nullptr};
  agnocast::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_static_{
    nullptr};
  tf2::BufferCore & buffer_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_{nullptr};
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_{nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
};

}  // namespace agnocast

#endif  // AGNOCAST__NODE__TF2__TRANSFORM_LISTENER_HPP_
