#include "agnocast/internal/gpu_backend.hpp"
#include "vmm_backend.hpp"

namespace agnocast::gpu
{

namespace
{

agnocast::internal::GpuMemoryBackend * select_backend()
{
  // Leaked: ~GpuRegion dispatches through this object, and a region held by a
  // static-duration object outlives library static destruction.
  static auto * vmm = new VmmBackend();  // NOLINT(cppcoreguidelines-owning-memory)
  return vmm->is_supported() ? vmm : nullptr;
}

// Runs when this library is loaded, so it must touch no CUDA: the backends are
// only constructed here, and every driver call is deferred to first use.
// Selection is likewise deferred, because probing support requires the driver.
__attribute__((constructor)) void register_selected_backend()
{
  agnocast::internal::register_gpu_memory_backend_selector(&select_backend);
}

}  // namespace

}  // namespace agnocast::gpu
