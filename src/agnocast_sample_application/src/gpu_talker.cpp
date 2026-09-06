// Publishes point clouds whose payload lives in GPU device memory.
//
// Borrowing a message reserves its payload, dispatch() owns the stream the
// kernel runs on, and the payload is shared with every subscriber on the same
// GPU without a copy.

#include "agnocast/agnocast.hpp"
#include "agnocast/gpu/dispatch.hpp"
#include "agnocast/gpu/message_types.hpp"

#include <cuda_runtime.h>
#include <unistd.h>

#include <string>

using namespace std::chrono_literals;
using agnocast::gpu::dispatch;
using agnocast::gpu::TransferOptions;
using agnocast::gpu::uploads;
using agnocast::gpu::writes;

namespace
{
constexpr uint32_t kWidth = 65536;
constexpr uint32_t kPointStep = 16;
constexpr size_t kCapacity = static_cast<size_t>(kWidth) * kPointStep;
}  // namespace

__global__ void fill_kernel(
  uint8_t * points, size_t size, uint8_t seq, const float * transform, const float * calibration)
{
  const size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < size) {
    const size_t offset = static_cast<size_t>(transform[0]) + static_cast<size_t>(calibration[0]);
    points[i] = static_cast<uint8_t>((i + seq + offset) % 251);
  }
}

class GpuTalker : public agnocast::Node
{
public:
  GpuTalker() : Node("gpu_talker")
  {
    publisher_ = create_publisher<agnocast::gpu::PointCloud2>("/gpu_points", 4);

    // Page-locked so the uploads are genuinely asynchronous; dispatch() warns
    // when they are not. One buffer holds both parameter blocks.
    cudaMallocHost(&parameters_, 8 * sizeof(float));
    for (int i = 0; i < 8; i++) parameters_[i] = static_cast<float>(i);

    timer_ = agnocast::create_timer(
      this, std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME), rclcpp::Duration(200ms),
      [this] { publish_cloud(); });
  }

private:
  void publish_cloud()
  {
    auto cloud = publisher_->borrow_loaned_message(kCapacity);

    cloud->header.frame_id = "lidar_" + std::to_string(getpid());
    cloud->height = 1;
    cloud->width = kWidth;
    cloud->point_step = kPointStep;
    cloud->row_step = kWidth * kPointStep;
    cloud->is_dense = true;

    const auto seq = static_cast<uint8_t>(seq_ % 251);

    // The device buffers start null and are allocated on the dispatch stream,
    // so no synchronous allocation happens inside the callback.
    // Dropping the borrow returns its slot, so a failed fill costs one frame
    // rather than publishing whatever the slot held before.
    const bool filled = dispatch(
      uploads(transform_, parameters_, 4, TransferOptions::kAllocateDeviceAsync),
      uploads(calibration_, parameters_ + 4, 4, TransferOptions::kAllocateDeviceAsync),
      writes(cloud), [&](cudaStream_t stream) {
        fill_kernel<<<(kCapacity + 255) / 256, 256, 0, stream>>>(
          cloud->data.get(), kCapacity, seq, transform_, calibration_);
      });
    if (!filled) return;

    publisher_->publish(std::move(cloud));
    RCLCPP_INFO(get_logger(), "published cloud %ld", seq_++);
  }

  agnocast::Publisher<agnocast::gpu::PointCloud2>::SharedPtr publisher_;
  agnocast::TimerBase::SharedPtr timer_;
  float * parameters_ = nullptr;
  float * transform_ = nullptr;
  float * calibration_ = nullptr;
  int64_t seq_ = 0;
};

int main(int argc, char ** argv)
{
  agnocast::init(argc, argv);
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  auto node = std::make_shared<GpuTalker>();
  executor.add_node(node);
  executor.spin();
  return 0;
}
