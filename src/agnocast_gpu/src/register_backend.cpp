#include "agnocast/internal/gpu_backend.hpp"
#include "agnocast_gpu_version.hpp"
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

// Read by agnocastlib right after it loads this library, which refuses a version
// other than its own: the GpuMemoryBackend interface between the two is internal
// and carries no ABI guarantee, so a mismatched pair would not fail cleanly.
extern "C" const char * agnocast_gpu_get_version()
{
  return agnocast::gpu::VERSION;
}
