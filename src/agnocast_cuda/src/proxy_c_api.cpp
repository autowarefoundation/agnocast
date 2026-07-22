#include "proxy_c_api.hpp"

#include "gpu_shared_memory_pool_proxy.hpp"

#include <cstdint>

namespace
{
// Per-thread flag: is the current thread inside a CUDA publisher's
// borrow..publish window? Read by the heaphook to decide whether a cudaMalloc
// should be served from the pool.
thread_local int g_in_publish_window = 0;
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

}  // extern "C"
