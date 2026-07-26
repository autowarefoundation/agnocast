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

// Adapted from tf2_ros::Buffer for Agnocast zero-copy transport.

#include "agnocast/node/tf2/buffer.hpp"

#include "agnocast/node/agnocast_context.hpp"
#include "rclcpp/version.h"

#include <sstream>
#include <string>
#include <thread>

namespace agnocast
{

geometry_msgs::msg::TransformStamped Buffer::lookupTransform(
  const std::string & target_frame, const std::string & source_frame,
  const tf2::TimePoint & lookup_time, const tf2::Duration timeout) const
{
  // Pass error string to suppress console spam
  std::string error;
  canTransform(target_frame, source_frame, lookup_time, timeout, &error);
  return BufferCore::lookupTransform(target_frame, source_frame, lookup_time);
}

void Buffer::onTimeJump(const rcl_time_jump_t & jump)
{
  if (
    RCL_ROS_TIME_ACTIVATED == jump.clock_change || RCL_ROS_TIME_DEACTIVATED == jump.clock_change) {
    RCLCPP_WARN(getLogger(), "Detected time source change. Clearing TF buffer.");
    clear();
  } else if (jump.delta.nanoseconds < 0) {
    RCLCPP_WARN(getLogger(), "Detected jump back in time. Clearing TF buffer.");
    clear();
  }
}

geometry_msgs::msg::TransformStamped Buffer::lookupTransform(
  const std::string & target_frame, const tf2::TimePoint & target_time,
  const std::string & source_frame, const tf2::TimePoint & source_time,
  const std::string & fixed_frame, const tf2::Duration timeout) const
{
  // Pass error string to suppress console spam
  std::string error;
  canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout, &error);
  return BufferCore::lookupTransform(
    target_frame, target_time, source_frame, source_time, fixed_frame);
}

void conditionally_append_timeout_info(
  std::string * errstr, const rclcpp::Time & start_time, const rclcpp::Time & current_time,
  const rclcpp::Duration & timeout)
{
  if (errstr != nullptr) {
    std::stringstream ss;
    ss << ". canTransform returned after "
       << tf2::durationToSec(tf2_ros::fromRclcpp(current_time - start_time)) << " timeout was "
       << tf2::durationToSec(tf2_ros::fromRclcpp(timeout)) << ".";
    (*errstr) += ss.str();
  }
}

// Mirrors upstream tf2_ros::Buffer, which calls itself with zero timeout in the poll loop.
// NOLINTNEXTLINE(misc-no-recursion)
bool Buffer::canTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration timeout, std::string * errstr) const
{
  // Humble (rclcpp 16.x) gates this unconditionally; Jazzy (rclcpp 28+) only when timeout > 0.
#if RCLCPP_VERSION_MAJOR >= 28
  if (timeout != tf2::durationFromSec(0.0) && !checkAndErrorDedicatedThreadPresent(errstr)) {
    return false;
  }
#else
  if (!checkAndErrorDedicatedThreadPresent(errstr)) {
    return false;
  }
#endif

  rclcpp::Duration rclcpp_timeout(tf2_ros::toRclcpp(timeout));

  // poll for transform if timeout is set
  rclcpp::Time start_time = clock_->now();
  while (
    clock_->now() < start_time + rclcpp_timeout &&
    !canTransform(target_frame, source_frame, time, std::chrono::nanoseconds::zero(), errstr) &&
    (clock_->now() + rclcpp::Duration(3, 0) >= start_time) &&  // don't wait bag loop detected
    agnocast::ok()) {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  bool retval = canTransform(target_frame, source_frame, time, errstr);
  rclcpp::Time current_time = clock_->now();
  conditionally_append_timeout_info(errstr, start_time, current_time, rclcpp_timeout);
  return retval;
}

// Mirrors upstream tf2_ros::Buffer, which calls itself with zero timeout in the poll loop.
// NOLINTNEXTLINE(misc-no-recursion)
bool Buffer::canTransform(
  const std::string & target_frame, const tf2::TimePoint & target_time,
  const std::string & source_frame, const tf2::TimePoint & source_time,
  const std::string & fixed_frame, const tf2::Duration timeout, std::string * errstr) const
{
  // Humble (rclcpp 16.x) gates this unconditionally; Jazzy (rclcpp 28+) only when timeout > 0.
#if RCLCPP_VERSION_MAJOR >= 28
  if (timeout != tf2::durationFromSec(0.0) && !checkAndErrorDedicatedThreadPresent(errstr)) {
    return false;
  }
#else
  if (!checkAndErrorDedicatedThreadPresent(errstr)) {
    return false;
  }
#endif

  rclcpp::Duration rclcpp_timeout(tf2_ros::toRclcpp(timeout));

  // poll for transform if timeout is set
  rclcpp::Time start_time = clock_->now();
  while (clock_->now() < start_time + rclcpp_timeout &&
         !canTransform(
           target_frame, target_time, source_frame, source_time, fixed_frame,
           std::chrono::nanoseconds::zero(), errstr) &&
         (clock_->now() + rclcpp::Duration(3, 0) >= start_time) &&  // don't wait bag loop detected
         agnocast::ok()) {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  bool retval =
    canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, errstr);
  rclcpp::Time current_time = clock_->now();
  conditionally_append_timeout_info(errstr, start_time, current_time, rclcpp_timeout);
  return retval;
}

bool Buffer::checkAndErrorDedicatedThreadPresent(std::string * errstr) const
{
  if (isUsingDedicatedThread()) {
    return true;
  }

  if (errstr != nullptr) {
    *errstr = static_cast<const char *>(threading_error);
  }

  RCLCPP_ERROR(getLogger(), "%s", static_cast<const char *>(threading_error));
  return false;
}

rclcpp::Logger Buffer::getLogger() const
{
  return node_logging_interface_ ? node_logging_interface_->get_logger()
                                 : rclcpp::get_logger("tf2_buffer");
}

}  // namespace agnocast
