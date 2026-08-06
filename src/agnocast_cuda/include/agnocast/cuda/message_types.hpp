#pragma once

#include "agnocast/cuda_message_tag.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstddef>
#include <cstdint>

namespace agnocast::cuda
{

struct PointCloud2 : public sensor_msgs::msg::PointCloud2, public agnocast::cuda_message_tag
{
  uint8_t * data = nullptr;  // GPU device pointer (shadows base class std::vector<uint8_t> data)
};

struct Image : public sensor_msgs::msg::Image, public agnocast::cuda_message_tag
{
  uint8_t * data = nullptr;  // GPU device pointer (shadows base class std::vector<uint8_t> data)
};

}  // namespace agnocast::cuda

namespace agnocast
{

template <>
inline size_t get_cuda_gpu_data_size(const agnocast::cuda::PointCloud2 & msg)
{
  return static_cast<size_t>(msg.height) * static_cast<size_t>(msg.width) *
         static_cast<size_t>(msg.point_step);
}

template <>
inline size_t get_cuda_gpu_data_size(const agnocast::cuda::Image & msg)
{
  return static_cast<size_t>(msg.height) * msg.step;
}

}  // namespace agnocast
