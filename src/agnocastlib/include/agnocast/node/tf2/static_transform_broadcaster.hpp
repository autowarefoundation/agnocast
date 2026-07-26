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

// Adapted from tf2_ros::StaticTransformBroadcaster for Agnocast zero-copy transport.

#ifndef AGNOCAST__NODE__TF2__STATIC_TRANSFORM_BROADCASTER_HPP_
#define AGNOCAST__NODE__TF2__STATIC_TRANSFORM_BROADCASTER_HPP_

#include "agnocast/agnocast.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_overriding_options.hpp"
#include "tf2_ros/qos.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <vector>

namespace agnocast
{

/// \brief Broadcasts static transforms via Agnocast zero-copy IPC.
///
/// Static transforms are transforms that do not change over time (e.g., sensor mounts).
/// This class accumulates all static transforms and republishes them together, using
/// transient_local QoS so late-joining subscribers receive all transforms.
class StaticTransformBroadcaster
{
public:
  /// \brief Constructor
  /// \param node Reference to an agnocast::Node
  /// \param qos QoS settings for the publisher (default: depth=1, transient_local)
  /// \param options Publisher options (default: enables qos_overriding for
  /// Depth/History/Reliability)
  explicit StaticTransformBroadcaster(
    agnocast::Node & node, const rclcpp::QoS & qos = tf2_ros::StaticBroadcasterQoS(),
    const agnocast::PublisherOptions & options =
      []() {
        agnocast::PublisherOptions options;
        options.qos_overriding_options = rclcpp::QosOverridingOptions{
          rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::History,
          rclcpp::QosPolicyKind::Reliability};
        return options;
      }())
  : publisher_(node.create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", qos, options))
  {
  }

  /** \brief Send a TransformStamped message
   * The stamped data structure includes frame_id, and time, and parent_id already.  */
  void sendTransform(const geometry_msgs::msg::TransformStamped & transform);

  /** \brief Send a vector of TransformStamped messages
   * The stamped data structure includes frame_id, and time, and parent_id already.  */
  void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped> & transforms);

private:
  agnocast::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
  tf2_msgs::msg::TFMessage net_message_;
};

}  // namespace agnocast

#endif  // AGNOCAST__NODE__TF2__STATIC_TRANSFORM_BROADCASTER_HPP_
