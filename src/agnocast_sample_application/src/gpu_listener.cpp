// Reads point clouds whose payload lives in GPU device memory, written by
// another process. Several of these can run at once: the payload is shared, not
// copied, so an extra subscriber costs no bandwidth.

#include "agnocast/agnocast.hpp"
#include "agnocast/gpu/dispatch.hpp"
#include "agnocast/gpu/message_types.hpp"

#include <cuda_runtime.h>

#include <string>

using agnocast::gpu::dispatch;
using agnocast::gpu::downloads;
using agnocast::gpu::reads;
using agnocast::gpu::TransferOptions;

__global__ void inspect_kernel(const uint8_t * points, size_t size, uint32_t * sum, uint32_t * peak)
{
  uint32_t total = 0;
  uint32_t highest = 0;
  for (size_t i = 0; i < size; i++) {
    total += points[i];
    highest = points[i] > highest ? points[i] : highest;
  }
  *sum = total;
  *peak = highest;
}

class GpuListener : public agnocast::Node
{
public:
  explicit GpuListener(const std::string & topic) : Node("gpu_listener")
  {
    cudaMallocHost(&results_, 2 * sizeof(uint32_t));

    subscription_ = create_subscription<agnocast::gpu::PointCloud2>(
      topic, 4, [this](const agnocast::ipc_shared_ptr<const agnocast::gpu::PointCloud2> & cloud) {
        inspect(cloud);
      });
  }

private:
  void inspect(const agnocast::ipc_shared_ptr<const agnocast::gpu::PointCloud2> & cloud)
  {
    if (!dispatch(
          reads(cloud), downloads(results_, sum_, 1, TransferOptions::kAllocateDeviceAsync),
          downloads(results_ + 1, peak_, 1, TransferOptions::kAllocateDeviceAsync),
          [&](cudaStream_t stream) {
            inspect_kernel<<<1, 1, 0, stream>>>(cloud->data.get(), 256, sum_, peak_);
          })) {
      return;
    }

    RCLCPP_INFO(
      get_logger(), "from %s: %zu bytes on device, sum=%u peak=%u", cloud->header.frame_id.c_str(),
      agnocast::gpu::gpu_data_size(*cloud), results_[0], results_[1]);
  }

  agnocast::Subscription<agnocast::gpu::PointCloud2>::SharedPtr subscription_;
  uint32_t * results_ = nullptr;
  uint32_t * sum_ = nullptr;
  uint32_t * peak_ = nullptr;
};

int main(int argc, char ** argv)
{
  agnocast::init(argc, argv);
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  // Follows /gpu_points by default, or whichever topic is named first on the
  // command line, so the same binary can watch either end of the pipeline.
  auto node = std::make_shared<GpuListener>(argc > 1 ? argv[1] : "/gpu_points");
  executor.add_node(node);
  executor.spin();
  return 0;
}
