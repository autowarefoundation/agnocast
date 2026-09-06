#pragma once

// Message types whose bulk payload lives in GPU device memory.
//
// Each mirrors a sensor_msgs type and shadows its `data` member. The ROS type's
// `data` is a std::vector<uint8_t> holding host bytes; here it is a handle to a
// slot of a shared device region, so the payload never enters host memory and is
// never copied between processes. Every other field keeps its ROS meaning and
// travels through Agnocast's host shared memory as usual.
//
// `data.get()` yields a device pointer valid in the calling process, which is
// the shape cuda_blackboard users already write against.

#include "agnocast/internal/gpu_message.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstddef>
#include <cstdint>

namespace agnocast::gpu
{

struct PointCloud2 : public sensor_msgs::msg::PointCloud2,
                     public agnocast::internal::gpu_message_tag
{
  agnocast::internal::gpu_array<uint8_t> data;
};

struct Image : public sensor_msgs::msg::Image, public agnocast::internal::gpu_message_tag
{
  agnocast::internal::gpu_array<uint8_t> data;
};

// Size of the device payload, derived from the same fields ROS uses for the host
// one, so a producer sizes its slots from the message it is already filling in.
inline size_t gpu_data_size(const PointCloud2 & msg)
{
  return static_cast<size_t>(msg.height) * static_cast<size_t>(msg.width) *
         static_cast<size_t>(msg.point_step);
}

inline size_t gpu_data_size(const Image & msg)
{
  return static_cast<size_t>(msg.height) * static_cast<size_t>(msg.step);
}

}  // namespace agnocast::gpu
