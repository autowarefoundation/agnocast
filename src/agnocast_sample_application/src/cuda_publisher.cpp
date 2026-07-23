#include "agnocast/agnocast.hpp"
#include "agnocast/cuda/message_types.hpp"

#include <cuda_runtime.h>

using namespace std::chrono_literals;

// Simple CUDA kernel: fills GPU buffer with incrementing values.
__global__ void fill_kernel(uint8_t * data, size_t size, uint8_t offset)
{
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < size) {
    data[idx] = static_cast<uint8_t>((idx + offset) % 256);
  }
}

class CudaPublisher : public agnocast::Node
{
  int64_t count_ = 0;
  agnocast::Publisher<agnocast::cuda::PointCloud2>::SharedPtr pub_;
  agnocast::TimerBase::SharedPtr timer_;

  void timer_callback()
  {
    auto msg = pub_->borrow_loaned_message();

    // Set CPU metadata
    msg->header.stamp.sec = static_cast<int32_t>(count_);
    msg->header.frame_id = "lidar";
    msg->height = 1;
    msg->width = 1024;
    msg->point_step = 16;
    msg->row_step = msg->width * msg->point_step;
    msg->is_dense = true;

    // Allocate and fill GPU data
    const size_t gpu_size = static_cast<size_t>(msg->height) * static_cast<size_t>(msg->width) *
                            static_cast<size_t>(msg->point_step);
    const cudaError_t malloc_result = cudaMalloc(&msg->data, gpu_size);
    if (malloc_result != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaMalloc failed: %s", cudaGetErrorString(malloc_result));
      return;
    }

    const int threads = 256;
    const int blocks = (gpu_size + threads - 1) / threads;
    // The kernel runs on the per-thread default stream (this file is compiled with
    // nvcc --default-stream per-thread). We deliberately do NOT cudaStreamSynchronize
    // here: publish() records Agnocast's dataReadyEvent on the same per-thread stream
    // right after this kernel, and each subscriber issues cudaStreamWaitEvent on that
    // event before reading, so the write-before-read ordering is enforced on the GPU
    // without ever blocking the publisher thread.
    // cppcheck-suppress shiftTooManyBits  // false positive: <<< >>> is CUDA kernel launch syntax
    fill_kernel<<<blocks, threads>>>(msg->data, gpu_size, static_cast<uint8_t>(count_));

    // Catch immediate launch-configuration errors (non-blocking; no device sync).
    const cudaError_t launch_result = cudaGetLastError();
    if (launch_result != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "kernel launch failed: %s", cudaGetErrorString(launch_result));
      cudaFree(msg->data);
      msg->data = nullptr;
      return;
    }

    pub_->publish(std::move(msg));
    RCLCPP_INFO(
      get_logger(), "published CUDA PointCloud2: seq=%lld, gpu_size=%zu",
      static_cast<long long>(count_++), gpu_size);
  }

public:
  CudaPublisher() : Node("cuda_publisher")
  {
    pub_ = this->create_publisher<agnocast::cuda::PointCloud2>("/cuda_pointcloud", 1);
    timer_ = agnocast::create_timer(
      this, std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME), rclcpp::Duration(100ms),
      std::bind(&CudaPublisher::timer_callback, this));
  }
};

int main(int argc, char ** argv)
{
  agnocast::init(argc, argv);
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  auto node = std::make_shared<CudaPublisher>();
  executor.add_node(node);
  executor.spin();
  return 0;
}
