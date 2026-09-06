// A pipeline stage: reads a cloud from GPU memory, writes a filtered one, and
// publishes it. Nothing returns to host memory in between.
//
// The single dispatch declares both reads() and writes(), which is the case that
// decides where the stream comes from: one kernel touches the input and the
// output, so a per-message stream would force a choice between the input's and
// the output's, and neither is right. dispatch() owns it instead.

#include "agnocast/agnocast.hpp"
#include "agnocast/gpu/dispatch.hpp"
#include "agnocast/gpu/message_types.hpp"

#include <cuda_runtime.h>

#include <string>

using agnocast::gpu::dispatch;
using agnocast::gpu::downloads;
using agnocast::gpu::reads;
using agnocast::gpu::TransferOptions;
using agnocast::gpu::uploads;
using agnocast::gpu::writes;

__global__ void filter_kernel(
  const uint8_t * in, uint8_t * out, size_t size, const float * gain, uint32_t * kept)
{
  const size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < size) {
    const uint32_t scaled = in[i] * static_cast<uint32_t>(gain[0]);
    out[i] = static_cast<uint8_t>(scaled % 251);
    if (i == 0) {
      *kept = static_cast<uint32_t>(size);
    }
  }
}

class GpuListenTalker : public agnocast::Node
{
public:
  GpuListenTalker() : Node("gpu_listen_talker")
  {
    publisher_ = create_publisher<agnocast::gpu::PointCloud2>("/gpu_points_filtered", 4);

    cudaMallocHost(&gain_, sizeof(float));
    *gain_ = 2.0F;
    cudaMallocHost(&kept_, sizeof(uint32_t));

    subscription_ = create_subscription<agnocast::gpu::PointCloud2>(
      "/gpu_points", 4,
      [this](const agnocast::ipc_shared_ptr<const agnocast::gpu::PointCloud2> & in) {
        filter(in);
      });
  }

private:
  void filter(const agnocast::ipc_shared_ptr<const agnocast::gpu::PointCloud2> & in)
  {
    const size_t bytes = agnocast::gpu::gpu_data_size(*in);
    auto out = publisher_->borrow_loaned_message(bytes);

    out->header.frame_id = in->header.frame_id + "_filtered";
    out->height = in->height;
    out->width = in->width;
    out->point_step = in->point_step;
    out->row_step = in->row_step;
    out->is_dense = in->is_dense;

    const bool filtered = dispatch(
      reads(in), writes(out),
      uploads(device_gain_, gain_, 1, TransferOptions::kAllocateDeviceAsync),
      downloads(kept_, device_kept_, 1, TransferOptions::kAllocateDeviceAsync),
      [&](cudaStream_t stream) {
        filter_kernel<<<(bytes + 255) / 256, 256, 0, stream>>>(
          in->data.get(), out->data.get(), bytes, device_gain_, device_kept_);
      });
    if (!filtered) return;

    out->width = *kept_ / out->point_step;

    // Read before publishing: publish() hands the message to the kernel module
    // and leaves this handle empty, so anything needed afterwards must be copied
    // out first.
    const uint32_t published_width = out->width;
    publisher_->publish(std::move(out));

    RCLCPP_INFO(
      get_logger(), "filtered %s -> %u points", in->header.frame_id.c_str(), published_width);
  }

  agnocast::Subscription<agnocast::gpu::PointCloud2>::SharedPtr subscription_;
  agnocast::Publisher<agnocast::gpu::PointCloud2>::SharedPtr publisher_;
  float * gain_ = nullptr;
  float * device_gain_ = nullptr;
  uint32_t * kept_ = nullptr;
  uint32_t * device_kept_ = nullptr;
};

int main(int argc, char ** argv)
{
  agnocast::init(argc, argv);
  agnocast::AgnocastOnlySingleThreadedExecutor executor;
  auto node = std::make_shared<GpuListenTalker>();
  executor.add_node(node);
  executor.spin();
  return 0;
}
