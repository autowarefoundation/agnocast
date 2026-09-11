#pragma once

// Message types whose bulk payload lives in GPU device memory.
//
// Each mirrors a sensor_msgs type and shadows its `data` member: the ROS type's
// is a std::vector<uint8_t> of host bytes, this one is a handle to a slot of a
// shared device region. Every other field keeps its ROS meaning and travels
// through host shared memory as usual. `data.get()` yields a device pointer
// valid in the calling process.

#include "agnocast/internal/gpu_message.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>

namespace agnocast::gpu
{

// Assignment between two of these is deleted. The implicit move-assignment would
// hand one message's reserved slot to another and leave the first resolving to
// nothing, and copy-assignment from a plain sensor_msgs value would overwrite
// every ROS field while leaving the payload handle pointing at the old contents.
// The publisher fills `data` in directly, which is unaffected.
struct PointCloud2 : public sensor_msgs::msg::PointCloud2,
                     public agnocast::internal::gpu_message_tag
{
  agnocast::internal::gpu_array<uint8_t> data;

  PointCloud2() = default;
  ~PointCloud2() = default;
  PointCloud2(const PointCloud2 &) = delete;
  PointCloud2(PointCloud2 &&) = delete;
  PointCloud2 & operator=(const PointCloud2 &) = delete;
  PointCloud2 & operator=(PointCloud2 &&) = delete;
};

struct Image : public sensor_msgs::msg::Image, public agnocast::internal::gpu_message_tag
{
  agnocast::internal::gpu_array<uint8_t> data;

  Image() = default;
  ~Image() = default;
  Image(const Image &) = delete;
  Image(Image &&) = delete;
  Image & operator=(const Image &) = delete;
  Image & operator=(Image &&) = delete;
};

// Size of the device payload, derived from the same fields ROS uses for the host
// one, so a producer sizes its slots from the message it is already filling in.
//
// For a message that has been *received*, `msg.data.size()` is the authoritative
// extent: it is what the slot bound is checked against, whereas these fields are
// values a peer wrote into host shared memory and could contradict. So the
// derived size is never allowed to exceed the payload actually reserved. Reading
// past the payload would otherwise be an out-of-bounds device read -- which on
// most drivers poisons the context for the whole process, not just the kernel
// that did it.
namespace detail
{

// 0 rather than a wrapped product, so an overflow cannot look like a small
// payload. A received handle caps the result; an unpopulated one (size 0, a
// message still being built) leaves the derived size alone.
inline size_t bounded_gpu_data_size(const size_t derived, const bool overflowed, uint64_t reserved)
{
  if (overflowed) return 0;
  if (reserved == 0) return derived;
  return (derived > reserved) ? static_cast<size_t>(reserved) : derived;
}

inline bool mul_overflows(const uint64_t a, const uint64_t b)
{
  return a != 0 && b > std::numeric_limits<uint64_t>::max() / a;
}

}  // namespace detail

inline size_t gpu_data_size(const PointCloud2 & msg)
{
  const uint64_t row = static_cast<uint64_t>(msg.width) * static_cast<uint64_t>(msg.point_step);
  const bool overflowed =
    detail::mul_overflows(msg.width, msg.point_step) || detail::mul_overflows(row, msg.height);
  return detail::bounded_gpu_data_size(
    static_cast<size_t>(row * msg.height), overflowed, msg.data.size());
}

inline size_t gpu_data_size(const Image & msg)
{
  const bool overflowed = detail::mul_overflows(msg.height, msg.step);
  return detail::bounded_gpu_data_size(
    static_cast<size_t>(static_cast<uint64_t>(msg.height) * msg.step), overflowed, msg.data.size());
}

}  // namespace agnocast::gpu
