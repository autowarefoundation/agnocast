#include "agnocast/agnocast.hpp"
#include "agnocast/cuda/message_types.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <stdexcept>
#include <string>

using std::placeholders::_1;

namespace
{

// Which executor to spin. The GPU-IPC stream design supports every executor, so the
// integration test exercises both a single-threaded and a multi-threaded one.
// Set AGNOCAST_SAMPLE_EXECUTOR=mt for the multi-threaded executor.
bool use_multi_threaded_executor()
{
  const char * value = std::getenv("AGNOCAST_SAMPLE_EXECUTOR");
  return value != nullptr && std::string(value) == "mt";
}

}  // namespace

class CudaSubscriber : public agnocast::Node
{
  // A read in flight: the D2H copy Agnocast's ordering guarantees, plus an event that
  // tells us when the copy landed in `host_buf`.
  struct PendingRead
  {
    // PINNED host memory, deliberately. A cudaMemcpyAsync D2H into *pageable* memory
    // blocks the calling thread until the copy completes, which would make the callback
    // effectively synchronous and leave the done edge untested.
    uint8_t * host_buf = nullptr;
    cudaEvent_t done = nullptr;
    uint32_t width = 0;
    uint32_t point_step = 0;
    size_t gpu_size = 0;
    bool in_flight = false;
  };

  static constexpr size_t kHostBufBytes = 16;

  // Enough slots that a burst of messages does not force a host synchronization. The
  // slots are what bounds this node's concurrent reads, not the pool.
  static constexpr size_t kMaxConcurrentReads = 4;

  cudaStream_t stream_ = nullptr;
  std::array<PendingRead, kMaxConcurrentReads> reads_{};
  agnocast::Subscription<agnocast::cuda::PointCloud2>::SharedPtr sub_;

  // Reports every read whose D2H copy has completed. cudaEventQuery never blocks.
  void report_completed_reads()
  {
    for (auto & read : reads_) {
      if (!read.in_flight) {
        continue;
      }
      const cudaError_t state = cudaEventQuery(read.done);
      if (state == cudaErrorNotReady) {
        continue;
      }
      if (state != cudaSuccess) {
        RCLCPP_ERROR(get_logger(), "cudaEventQuery failed: %s", cudaGetErrorString(state));
      } else {
        RCLCPP_INFO(
          get_logger(),
          "received CUDA PointCloud2: width=%u, point_step=%u, gpu_size=%zu, "
          "first_bytes=[%u,%u,%u,%u]",
          read.width, read.point_step, read.gpu_size, read.host_buf[0], read.host_buf[1],
          read.host_buf[2], read.host_buf[3]);
      }
      read.in_flight = false;
    }
  }

  PendingRead * acquire_read_slot()
  {
    for (auto & read : reads_) {
      if (!read.in_flight) {
        return &read;
      }
    }
    return nullptr;
  }

  void callback(agnocast::ipc_shared_ptr<const agnocast::cuda::PointCloud2> msg)
  {
    // Retire finished reads first, which also frees up slots for this one.
    report_completed_reads();

    // Read CPU metadata from shared memory
    const uint32_t width = msg->width;
    const uint32_t point_step = msg->point_step;
    const size_t gpu_size = static_cast<size_t>(msg->height) * static_cast<size_t>(width) *
                            static_cast<size_t>(point_step);

    // Get subscriber-local GPU pointer (mapped via CUDA IPC)
    auto * gpu_ptr = static_cast<const uint8_t *>(msg.gpu_data());
    if (!gpu_ptr) {
      RCLCPP_ERROR(get_logger(), "gpu_data() returned nullptr");
      return;
    }

    PendingRead * read = acquire_read_slot();
    if (read == nullptr) {
      // All slots busy: this node is falling behind its own GPU. Drop the message
      // rather than grow unbounded — a real node would size the slots for its rate.
      RCLCPP_WARN(get_logger(), "all read slots busy; dropping a message");
      return;
    }

    // Copy the first bytes back to the host, asynchronously, and return WITHOUT any
    // host synchronization.
    //
    // This is what arbitrary-stream support buys. Agnocast has already issued
    // cudaStreamWaitEvent(dataReadyEvent) on stream_ (the stream declared in
    // SubscriptionOptions), so this copy is GPU-ordered after the publisher's write.
    // And when this callback returns, the last reference to `msg` is dropped: Agnocast
    // records a read-done marker on stream_ and holds the kernel-side reference until
    // that marker completes, so the publisher cannot hand the pool slot to the next
    // writer while this copy is still in flight.
    const size_t copy_size = std::min(gpu_size, kHostBufBytes);
    const cudaError_t memcpy_result =
      cudaMemcpyAsync(read->host_buf, gpu_ptr, copy_size, cudaMemcpyDeviceToHost, stream_);
    if (memcpy_result != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaMemcpyAsync failed: %s", cudaGetErrorString(memcpy_result));
      return;
    }
    const cudaError_t record_result = cudaEventRecord(read->done, stream_);
    if (record_result != cudaSuccess) {
      RCLCPP_ERROR(get_logger(), "cudaEventRecord failed: %s", cudaGetErrorString(record_result));
      return;
    }

    read->width = width;
    read->point_step = point_step;
    read->gpu_size = gpu_size;
    read->in_flight = true;
  }

public:
  CudaSubscriber() : Node("cuda_subscriber")
  {
    // Non-blocking so this node's reads are not serialized against unrelated blocking
    // streams in the same process. Agnocast requires such a stream to be declared.
    const cudaError_t stream_result = cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
    if (stream_result != cudaSuccess) {
      RCLCPP_ERROR(
        get_logger(), "cudaStreamCreateWithFlags failed: %s", cudaGetErrorString(stream_result));
      throw std::runtime_error("could not create the subscriber's CUDA stream");
    }
    for (auto & read : reads_) {
      const cudaError_t event_result = cudaEventCreateWithFlags(&read.done, cudaEventDisableTiming);
      if (event_result != cudaSuccess) {
        RCLCPP_ERROR(
          get_logger(), "cudaEventCreateWithFlags failed: %s", cudaGetErrorString(event_result));
        throw std::runtime_error("could not create the subscriber's completion events");
      }
      const cudaError_t host_alloc_result = cudaHostAlloc(
        reinterpret_cast<void **>(&read.host_buf), kHostBufBytes, cudaHostAllocDefault);
      if (host_alloc_result != cudaSuccess) {
        RCLCPP_ERROR(
          get_logger(), "cudaHostAlloc failed: %s", cudaGetErrorString(host_alloc_result));
        throw std::runtime_error("could not allocate the subscriber's pinned host buffers");
      }
    }

    agnocast::SubscriptionOptions options;
    options.cuda_stream = stream_;  // cudaStream_t converts implicitly to void *
    sub_ = this->create_subscription<agnocast::cuda::PointCloud2>(
      "/cuda_pointcloud", 1, std::bind(&CudaSubscriber::callback, this, _1), options);
  }

  ~CudaSubscriber()
  {
    for (auto & read : reads_) {
      if (read.done != nullptr) {
        cudaEventDestroy(read.done);
      }
      if (read.host_buf != nullptr) {
        cudaFreeHost(read.host_buf);
      }
    }
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
  }
};

int main(int argc, char ** argv)
{
  agnocast::init(argc, argv);
  auto node = std::make_shared<CudaSubscriber>();

  if (use_multi_threaded_executor()) {
    // Supported because the stream is an explicit handle: the wait for the publisher's
    // write means the same thing from any thread, and so does the read-done marker
    // recorded by whichever thread drops the last message reference.
    agnocast::AgnocastOnlyMultiThreadedExecutor executor(2);
    executor.add_node(node);
    executor.spin();
  } else {
    agnocast::AgnocastOnlySingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }
  return 0;
}
