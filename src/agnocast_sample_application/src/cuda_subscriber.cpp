#include "agnocast/agnocast.hpp"
#include "agnocast/cuda/message_types.hpp"

#include <cuda_runtime.h>

#include <algorithm>

using std::placeholders::_1;

class CudaSubscriber : public agnocast::Node
{
  agnocast::Subscription<agnocast::cuda::PointCloud2>::SharedPtr sub_;

  void callback(agnocast::ipc_shared_ptr<const agnocast::cuda::PointCloud2> msg)
  {
    // Read CPU metadata from shared memory
    const uint32_t width = msg->width;
    const uint32_t point_step = msg->point_step;
    const size_t gpu_size = static_cast<size_t>(msg->height) * static_cast<size_t>(width) *
                            static_cast<size_t>(point_step);

    // Get subscriber-local GPU pointer (mapped via CUDA IPC)
    auto * gpu_ptr = static_cast<uint8_t *>(msg.gpu_data());
    if (!gpu_ptr) {
      RCLCPP_ERROR(get_logger(), "gpu_data() returned nullptr");
      return;
    }

    // Read first few bytes from GPU to verify data.
    //
    // This file is compiled with nvcc --default-stream per-thread, so this
    // cudaMemcpy runs on the per-thread default stream. Agnocast issues
    // cudaStreamWaitEvent(dataReadyEvent) on that same stream before the callback,
    // so the copy is GPU-ordered after the publisher's write. cudaMemcpy is also
    // host-synchronous, so the read completes before this message reference is
    // dropped (the pool slot is only reclaimed after all subscribers release).
    uint8_t host_buf[16]{};
    const size_t copy_size = std::min(gpu_size, sizeof(host_buf));
    const cudaError_t memcpy_result =
      cudaMemcpy(host_buf, gpu_ptr, copy_size, cudaMemcpyDeviceToHost);
    if (memcpy_result != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaMemcpy failed: %s", cudaGetErrorString(memcpy_result));
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "received CUDA PointCloud2: width=%u, point_step=%u, gpu_size=%zu, "
      "first_bytes=[%u,%u,%u,%u]",
      width, point_step, gpu_size, host_buf[0], host_buf[1], host_buf[2], host_buf[3]);
  }

public:
  CudaSubscriber() : Node("cuda_subscriber")
  {
    sub_ = this->create_subscription<agnocast::cuda::PointCloud2>(
      "/cuda_pointcloud", 1, std::bind(&CudaSubscriber::callback, this, _1));
  }
};

int main(int argc, char ** argv)
{
  agnocast::init(argc, argv);
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  auto node = std::make_shared<CudaSubscriber>();
  executor.add_node(node);
  executor.spin();
  return 0;
}
