#pragma once

#include <cstddef>
#include <type_traits>

namespace agnocast
{

// Base class for CUDA message types. Provides a pointer to GpuMetadata stored in shared memory.
// CUDA message types (e.g., agnocast::cuda::PointCloud2) inherit from both a ROS message type
// and this tag. The publish/receive machinery uses is_cuda_message_v<T> to detect CUDA messages
// at compile time via if constexpr, so no CUDA headers are needed in agnocastlib.
struct cuda_message_tag
{
  void * gpu_metadata_ = nullptr;
};

template <typename T>
inline constexpr bool is_cuda_message_v =
  std::is_base_of_v<cuda_message_tag, std::remove_const_t<T>>;

// Returns the GPU data size for a CUDA message. Must be specialized by each CUDA message type
// in the agnocast_cuda package (e.g., for PointCloud2: height * width * point_step).
template <typename T>
size_t get_cuda_gpu_data_size(const T & msg);

}  // namespace agnocast
