#include "agnocast/internal/gpu_backend.hpp"
#include "vmm_backend.hpp"

namespace agnocast::gpu
{

namespace
{

agnocast::internal::GpuMemoryBackend * select_backend()
{
  // Leaked: a region dispatches through this object from its destructor, which
  // can run after library static destruction.
  static auto * vmm = new VmmBackend();  // NOLINT(cppcoreguidelines-owning-memory)
  return vmm->is_supported() ? vmm : nullptr;
}

// Runs when this library is loaded, so it must touch no CUDA: backends are only
// constructed here, and both selection and every driver call are deferred to
// first use.
__attribute__((constructor)) void register_selected_backend()
{
  agnocast::internal::register_gpu_memory_backend_selector(&select_backend);
}

}  // namespace

}  // namespace agnocast::gpu
