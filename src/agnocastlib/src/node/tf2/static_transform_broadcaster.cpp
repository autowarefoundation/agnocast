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

#include "agnocast/node/tf2/static_transform_broadcaster.hpp"

#include <algorithm>
#include <vector>

namespace agnocast
{

namespace
{
void upsert_transform(
  std::vector<geometry_msgs::msg::TransformStamped> & accumulator,
  const geometry_msgs::msg::TransformStamped & transform)
{
  auto it = std::find_if(
    accumulator.begin(), accumulator.end(),
    [&transform](const geometry_msgs::msg::TransformStamped & existing) {
      return transform.child_frame_id == existing.child_frame_id;
    });
  if (it != accumulator.end()) {
    *it = transform;
  } else {
    accumulator.push_back(transform);
  }
}
}  // namespace

void StaticTransformBroadcaster::sendTransform(
  const geometry_msgs::msg::TransformStamped & transform)
{
  upsert_transform(net_message_.transforms, transform);

  // net_message_ accumulates all static transforms for transient_local semantics, so the
  // entire vector must be re-published each time. The loaned SHM payload is freshly allocated
  // per publish, so we copy net_message_.transforms into it; persisting the loaned slot would
  // avoid this copy but requires Agnocast core support.
  auto msg = publisher_->borrow_loaned_message();
  msg->transforms = net_message_.transforms;
  publisher_->publish(std::move(msg));
}

void StaticTransformBroadcaster::sendTransform(
  const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
{
  for (const auto & transform : transforms) {
    upsert_transform(net_message_.transforms, transform);
  }

  // net_message_ accumulates all static transforms for transient_local semantics, so the
  // entire vector must be re-published each time. The loaned SHM payload is freshly allocated
  // per publish, so we copy net_message_.transforms into it; persisting the loaned slot would
  // avoid this copy but requires Agnocast core support.
  auto msg = publisher_->borrow_loaned_message();
  msg->transforms = net_message_.transforms;
  publisher_->publish(std::move(msg));
}

}  // namespace agnocast
