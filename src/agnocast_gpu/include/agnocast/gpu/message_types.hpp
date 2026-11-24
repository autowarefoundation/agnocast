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
#include <initializer_list>
#include <limits>

namespace agnocast::gpu
{

// Copy and assignment are deleted: move-assignment would hand one message's
// reserved slot to another, and assigning a plain sensor_msgs value would
// overwrite every ROS field while leaving the payload handle pointing at the old
// contents. The publisher fills `data` in directly, which is unaffected.
//
// `data` shadows the ROS type's member rather than replacing it: the base's
// vector is still there, empty, and is what generic code handed a `RosMessageT &`
// would see -- which is why a handle must never be converted to the base type.
template <typename RosMessageT>
struct GpuMessage : public RosMessageT, public agnocast::internal::gpu_message_tag
{
  agnocast::internal::gpu_array<uint8_t> data;

  GpuMessage() = default;
  ~GpuMessage() = default;
  GpuMessage(const GpuMessage &) = delete;
  GpuMessage(GpuMessage &&) = delete;
  GpuMessage & operator=(const GpuMessage &) = delete;
  GpuMessage & operator=(GpuMessage &&) = delete;
};

using PointCloud2 = GpuMessage<sensor_msgs::msg::PointCloud2>;
using Image = GpuMessage<sensor_msgs::msg::Image>;

// Size of the device payload, derived from the same fields ROS uses for the host
// one, so a producer sizes its slots from the message it is already filling in.
//
// On a received message `msg.data.size()` is the authoritative extent -- it is
// what the slot bound is checked against, whereas these fields are values a peer
// wrote into host shared memory -- so the derived size is capped by it. Reading
// past the payload would be an out-of-bounds device read, which on most drivers
// poisons the context for the whole process.
namespace detail
{

// 0 rather than a wrapped product, so an overflow cannot look like a small
// payload. An unpopulated handle (size 0, a message still being built) leaves
// the derived size alone.
inline size_t bounded_extent(std::initializer_list<uint64_t> factors, const uint64_t reserved)
{
  uint64_t product = 1;
  for (const uint64_t factor : factors) {
    if (factor != 0 && product > std::numeric_limits<uint64_t>::max() / factor) return 0;
    product *= factor;
  }
  if (reserved != 0 && product > reserved) return static_cast<size_t>(reserved);
  return static_cast<size_t>(product);
}

}  // namespace detail

inline size_t gpu_data_size(const PointCloud2 & msg)
{
  return detail::bounded_extent({msg.width, msg.point_step, msg.height}, msg.data.size());
}

inline size_t gpu_data_size(const Image & msg)
{
  return detail::bounded_extent({msg.height, msg.step}, msg.data.size());
}

}  // namespace agnocast::gpu
