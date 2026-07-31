#include "proxy_c_api.hpp"

#include "cudart_loader.hpp"
#include "gpu_shared_memory_pool_proxy.hpp"

#include <atomic>
#include <cstdint>

namespace
{
// Per-thread flag: is the current thread inside a CUDA publisher's
// borrow..publish window? Read by the heaphook to decide whether a cudaMalloc
// should be served from the pool.
thread_local int g_in_publish_window = 0;

// Set once by the heaphook when this process creates a non-blocking CUDA stream.
// Process-wide (not per-thread): the stream a publisher forgot to declare may have
// been created on any thread.
std::atomic<bool> g_non_blocking_stream_created{false};
}  // namespace

extern "C" {

void agnocast_cuda_set_publish_window(int active)
{
  g_in_publish_window = active;
}

int agnocast_cuda_in_publish_window(void)
{
  return g_in_publish_window;
}

int agnocast_cuda_pool_allocate(size_t size, void ** out_ptr)
{
  if (out_ptr == nullptr) {
    return 0;
  }
  auto * proxy = agnocast::cuda::GpuSharedMemoryPoolProxy::getInstance();
  if (proxy == nullptr) {
    return 0;
  }
  return proxy->allocateMemory(out_ptr, size) ? 1 : 0;
}

int agnocast_cuda_pool_free(void * ptr)
{
  auto * proxy = agnocast::cuda::GpuSharedMemoryPoolProxy::getInstance();
  if (proxy == nullptr) {
    return 0;
  }
  std::uint32_t slot_id = 0;
  if (!proxy->getSlotIdFromDevicePtr(ptr, slot_id)) {
    return 0;  // not a pooled pointer
  }
  proxy->freeMemory(ptr);
  return 1;
}

int agnocast_cuda_slot_id_from_ptr(void * ptr, std::uint32_t * out_slot_id)
{
  if (out_slot_id == nullptr) {
    return 0;
  }
  auto * proxy = agnocast::cuda::GpuSharedMemoryPoolProxy::getInstance();
  if (proxy == nullptr) {
    return 0;
  }
  return proxy->getSlotIdFromDevicePtr(ptr, *out_slot_id) ? 1 : 0;
}

int agnocast_cuda_ptr_from_slot_id(std::uint32_t slot_id, void ** out_ptr)
{
  if (out_ptr == nullptr) {
    return 0;
  }
  auto * proxy = agnocast::cuda::GpuSharedMemoryPoolProxy::getInstance();
  if (proxy == nullptr) {
    return 0;
  }
  return proxy->getDevicePtrFromSlotId(slot_id, *out_ptr) ? 1 : 0;
}

int agnocast_cuda_record_data_ready(void * ptr, int stream_kind, void * stream)
{
  auto * proxy = agnocast::cuda::GpuSharedMemoryPoolProxy::getInstance();
  if (proxy == nullptr) {
    return 0;
  }
  return proxy->recordDataReady(ptr, stream_kind, stream) ? 1 : 0;
}

int agnocast_cuda_wait_data_ready(std::uint32_t slot_id, int stream_kind, void * stream)
{
  auto * proxy = agnocast::cuda::GpuSharedMemoryPoolProxy::getInstance();
  if (proxy == nullptr) {
    return 0;
  }
  return proxy->waitDataReady(slot_id, stream_kind, stream) ? 1 : 0;
}

int agnocast_cuda_record_read_done(int stream_kind, void * stream, void ** out_token)
{
  auto * proxy = agnocast::cuda::GpuSharedMemoryPoolProxy::getInstance();
  if (proxy == nullptr) {
    return 0;
  }
  return proxy->recordReadDone(stream_kind, stream, out_token) ? 1 : 0;
}

int agnocast_cuda_query_read_done(void * token)
{
  auto * proxy = agnocast::cuda::GpuSharedMemoryPoolProxy::getInstance();
  if (proxy == nullptr) {
    // No proxy means no pool and no marker to poll; report complete so the caller
    // releases rather than holding the reference forever.
    return 1;
  }
  return proxy->queryReadDone(token);
}

void agnocast_cuda_wait_read_done(void * token)
{
  auto * proxy = agnocast::cuda::GpuSharedMemoryPoolProxy::getInstance();
  if (proxy != nullptr) {
    proxy->waitReadDone(token);
  }
}

void agnocast_cuda_note_non_blocking_stream(void)
{
  g_non_blocking_stream_created.store(true, std::memory_order_release);
}

int agnocast_cuda_stream_ordering_unsafe(int stream_kind, void * stream)
{
  if (!g_non_blocking_stream_created.load(std::memory_order_acquire)) {
    return 0;
  }
  auto * proxy = agnocast::cuda::GpuSharedMemoryPoolProxy::getInstance();
  if (proxy == nullptr) {
    return 0;  // no pool in play; the caller's other checks report the real problem
  }
  return proxy->isDefaultStream(stream_kind, stream) ? 1 : 0;
}

void agnocast_cuda_reclaim_gpu_buffer(void * ptr)
{
  if (ptr == nullptr) {
    return;
  }
  auto * proxy = agnocast::cuda::GpuSharedMemoryPoolProxy::getInstance();
  if (proxy != nullptr) {
    std::uint32_t slot_id = 0;
    if (proxy->getSlotIdFromDevicePtr(ptr, slot_id)) {
      proxy->freeMemory(ptr);  // pooled: return the slot
      return;
    }
  }
  // Not pooled (fallback real cudaMalloc): free the real device memory. Uses the
  // runtime-loaded (real) cudaFree, not the heaphook.
  agnocast::cuda::CudartLoader::instance().cudaFree(ptr);
}

}  // extern "C"
