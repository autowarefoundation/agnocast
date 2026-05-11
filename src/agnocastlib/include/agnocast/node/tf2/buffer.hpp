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

#ifndef AGNOCAST__NODE__TF2__BUFFER_HPP_
#define AGNOCAST__NODE__TF2__BUFFER_HPP_

#include "agnocast/node/agnocast_node.hpp"
#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_interfaces/get_node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/time.hpp"
#include "tf2/buffer_core.h"
#include "tf2/time.h"
#include "tf2_ros/buffer_interface.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <memory>
#include <string>
#include <utility>

namespace agnocast
{

/// \warning This is NOT a drop-in replacement for tf2_ros::Buffer. The following
/// tf2_ros::Buffer features are intentionally omitted:
///   - AsyncBufferInterface (waitForTransform, cancel, setCreateTimerInterface)
///   - The "tf2_frames" debug service (getFrames)
class Buffer : public tf2::BufferCore, public tf2_ros::BufferInterface
{
public:
  using tf2::BufferCore::canTransform;
  using tf2::BufferCore::lookupTransform;
  using SharedPtr = std::shared_ptr<Buffer>;

  /// \brief Constructor for a Buffer object
  /// \param clock A clock to use for time and sleeping
  /// \param cache_time How long to keep a history of transforms
  /// \param node If passed, the buffer will use this node's logger instead of the default
  template <typename NodeT = std::shared_ptr<agnocast::Node>>
  // cppcheck-suppress noExplicitConstructor  // matches tf2_ros::Buffer signature (upstream
  // alignment)
  Buffer(
    rclcpp::Clock::SharedPtr clock,
    tf2::Duration cache_time = tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME),
    NodeT && node = NodeT())
  : BufferCore(cache_time), clock_(clock)
  {
    if (nullptr == clock_) {
      throw std::invalid_argument("clock must be a valid instance");
    }

    auto post_jump_cb = [this](const rcl_time_jump_t & jump_info) { onTimeJump(jump_info); };

    rcl_jump_threshold_t jump_threshold;
    // Disable forward jump callbacks
    jump_threshold.min_forward.nanoseconds = 0;
    // Anything backwards is a jump
    jump_threshold.min_backward.nanoseconds = -1;
    // Callback if the clock changes too
    jump_threshold.on_clock_change = true;

    jump_handler_ = clock_->create_jump_callback(nullptr, post_jump_cb, jump_threshold);

    if (node) {
      node_logging_interface_ = rclcpp::node_interfaces::get_node_logging_interface(node);
    }
  }

  /// \brief Get the transform between two frames by frame ID.
  /// \param target_frame The frame to which data should be transformed
  /// \param source_frame The frame where the data originated
  /// \param time The time at which the value of the transform is desired. (0 will get the latest)
  /// \param timeout How long to block before failing
  /// \return The transform between the frames
  ///
  /// Possible exceptions tf2::LookupException, tf2::ConnectivityException,
  /// tf2::ExtrapolationException, tf2::InvalidArgumentException
  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration timeout) const override;

  /// \brief Get the transform between two frames by frame ID.
  /// \sa lookupTransform(const std::string&, const std::string&, const tf2::TimePoint&,
  ///                     const tf2::Duration)
  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    const rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(0)) const
  {
    return lookupTransform(
      target_frame, source_frame, tf2_ros::fromRclcpp(time), tf2_ros::fromRclcpp(timeout));
  }

  /// \brief Get the transform between two frames by frame ID assuming fixed frame.
  /// \param target_frame The frame to which data should be transformed
  /// \param target_time The time to which the data should be transformed. (0 will get the latest)
  /// \param source_frame The frame where the data originated
  /// \param source_time The time at which the source_frame should be evaluated. (0 will get the
  /// latest)
  /// \param fixed_frame The frame in which to assume the transform is constant in time.
  /// \param timeout How long to block before failing
  /// \return The transform between the frames
  ///
  /// Possible exceptions tf2::LookupException, tf2::ConnectivityException,
  /// tf2::ExtrapolationException, tf2::InvalidArgumentException
  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame, const tf2::TimePoint & target_time,
    const std::string & source_frame, const tf2::TimePoint & source_time,
    const std::string & fixed_frame, const tf2::Duration timeout) const override;

  /// \brief Get the transform between two frames by frame ID assuming fixed frame.
  /// \sa lookupTransform(const std::string&, const tf2::TimePoint&,
  ///                     const std::string&, const tf2::TimePoint&,
  ///                     const std::string&, const tf2::Duration)
  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame, const rclcpp::Time & target_time,
    const std::string & source_frame, const rclcpp::Time & source_time,
    const std::string & fixed_frame,
    const rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(0)) const
  {
    return lookupTransform(
      target_frame, tf2_ros::fromRclcpp(target_time), source_frame,
      tf2_ros::fromRclcpp(source_time), fixed_frame, tf2_ros::fromRclcpp(timeout));
  }

  /// \brief Test if a transform is possible
  /// \param target_frame The frame into which to transform
  /// \param source_frame The frame from which to transform
  /// \param target_time The time at which to transform
  /// \param timeout How long to block before failing
  /// \param errstr A pointer to a string which will be filled with why the transform failed, if not
  /// NULL
  /// \return True if the transform is possible, false otherwise
  bool canTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration timeout, std::string * errstr = NULL) const override;

  /// \brief Test if a transform is possible
  /// \sa canTransform(const std::string&, const std::string&,
  ///                  const tf2::TimePoint&, const tf2::Duration, std::string*)
  bool canTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    const rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(0),
    std::string * errstr = NULL) const
  {
    return canTransform(
      target_frame, source_frame, tf2_ros::fromRclcpp(time), tf2_ros::fromRclcpp(timeout), errstr);
  }

  /// \brief Test if a transform is possible
  /// \param target_frame The frame into which to transform
  /// \param target_time The time into which to transform
  /// \param source_frame The frame from which to transform
  /// \param source_time The time from which to transform
  /// \param fixed_frame The frame in which to treat the transform as constant in time
  /// \param timeout How long to block before failing
  /// \param errstr A pointer to a string which will be filled with why the transform failed, if not
  /// NULL
  /// \return True if the transform is possible, false otherwise
  bool canTransform(
    const std::string & target_frame, const tf2::TimePoint & target_time,
    const std::string & source_frame, const tf2::TimePoint & source_time,
    const std::string & fixed_frame, const tf2::Duration timeout,
    std::string * errstr = NULL) const override;

  /// \brief Test if a transform is possible
  /// \sa canTransform(const std::string&, const tf2::TimePoint&,
  ///                  const std::string&, const tf2::TimePoint&,
  ///                  const std::string&, const tf2::Duration, std::string*)
  bool canTransform(
    const std::string & target_frame, const rclcpp::Time & target_time,
    const std::string & source_frame, const rclcpp::Time & source_time,
    const std::string & fixed_frame,
    const rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(0),
    std::string * errstr = NULL) const
  {
    return canTransform(
      target_frame, tf2_ros::fromRclcpp(target_time), source_frame,
      tf2_ros::fromRclcpp(source_time), fixed_frame, tf2_ros::fromRclcpp(timeout), errstr);
  }

  /// \brief Get the clock used by this buffer
  rclcpp::Clock::SharedPtr getClock() const { return clock_; }

private:
  void onTimeJump(const rcl_time_jump_t & jump);

  // conditionally error if dedicated_thread unset.
  bool checkAndErrorDedicatedThreadPresent(std::string * errstr) const;

  /// Get the logger to use for calls to RCLCPP log macros.
  rclcpp::Logger getLogger() const;

  /// \brief A clock to use for time and sleeping
  rclcpp::Clock::SharedPtr clock_;

  /// \brief A node logging interface to access the buffer node's logger
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;

  /// \brief Reference to a jump handler registered to the clock
  rclcpp::JumpHandler::SharedPtr jump_handler_;
};

static const char threading_error[] =
  "Do not call canTransform or lookupTransform with a timeout "
  "unless you are using another thread for populating data. Without a dedicated thread it will "
  "always timeout.  If you have a separate thread servicing tf messages, call "
  "setUsingDedicatedThread(true) on your Buffer instance.";

}  // namespace agnocast

#endif  // AGNOCAST__NODE__TF2__BUFFER_HPP_
