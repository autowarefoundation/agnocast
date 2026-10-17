// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Adapted from rclcpp/include/rclcpp/client.hpp so agnocastlib does not depend on
// rclcpp::detail.

#pragma once

#include <chrono>
#include <future>
#include <utility>

namespace agnocast::internal
{

// A future plus the sequence number of the service request that produced it.
template <typename FutureT>
struct FutureAndRequestId
{
  FutureT future;
  int64_t request_id;

  FutureAndRequestId(FutureT impl, int64_t req_id) : future(std::move(impl)), request_id(req_id) {}

  /// Allow implicit conversions to the future type by reference.
  operator FutureT &() { return this->future; }

  /// Deprecated, use the `future` member variable instead.
  /**
   * Allow implicit conversions to the future type by value.
   * \deprecated
   */
  [[deprecated(
    "FutureAndRequestId: use .future instead of an implicit conversion")]] operator FutureT()
  {
    return this->future;
  }

  /// See std::future::get().
  auto get() { return this->future.get(); }
  /// See std::future::valid().
  bool valid() const noexcept { return this->future.valid(); }
  /// See std::future::wait().
  void wait() const { return this->future.wait(); }
  /// See std::future::wait_for().
  template <class Rep, class Period>
  std::future_status wait_for(const std::chrono::duration<Rep, Period> & timeout_duration) const
  {
    return this->future.wait_for(timeout_duration);
  }
  /// See std::future::wait_until().
  template <class Clock, class Duration>
  std::future_status wait_until(const std::chrono::time_point<Clock, Duration> & timeout_time) const
  {
    return this->future.wait_until(timeout_time);
  }

  FutureAndRequestId(FutureAndRequestId && other) noexcept = default;
  FutureAndRequestId(const FutureAndRequestId & other) = delete;
  FutureAndRequestId & operator=(FutureAndRequestId && other) noexcept = default;
  FutureAndRequestId & operator=(const FutureAndRequestId & other) = delete;
  ~FutureAndRequestId() = default;
};

}  // namespace agnocast::internal
