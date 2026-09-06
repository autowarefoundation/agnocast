#include "agnocast/internal/gpu_backend.hpp"

#include "agnocast/agnocast_utils.hpp"

#include <dlfcn.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <mutex>

namespace agnocast::internal
{

namespace
{

// The registries below are leaked deliberately. A user's static-duration object
// holding a GPU publisher is destroyed in an order this library does not
// control, so teardown can reach them after namespace-scope statics are gone.

std::mutex & backend_mutex()
{
  static auto * mtx = new std::mutex();  // NOLINT(cppcoreguidelines-owning-memory)
  return *mtx;
}

GpuMemoryBackendSelector & backend_selector()
{
  static auto * selector =
    new GpuMemoryBackendSelector(nullptr);  // NOLINT(cppcoreguidelines-owning-memory)
  return *selector;
}

// agnocast_gpu is loaded, not linked. A node that reaches GPU memory only
// through agnocastlib's API references none of its symbols, so --as-needed
// would drop the DT_NEEDED entry and its registering constructor would never
// run. RTLD_NODELETE because regions dispatch into the library from their
// destructors, which can run after anything that might unload it.
void ensure_backend_loaded()
{
  static const bool loaded = [] {
    if (dlopen("libagnocast_gpu.so", RTLD_NOW | RTLD_GLOBAL | RTLD_NODELETE) != nullptr) {
      return true;
    }
    const char * error = dlerror();
    RCLCPP_ERROR(
      logger, "Agnocast: GPU memory requested but libagnocast_gpu.so is unavailable: %s",
      error != nullptr ? error : "unknown error");
    return false;
  }();
  (void)loaded;
}

}  // namespace

void UniqueFd::reset()
{
  if (fd_ >= 0) {
    if (close(fd_) != 0) {
      // The descriptor was already closed elsewhere, which is a double-ownership
      // bug worth surfacing.
      RCLCPP_WARN(logger, "close() failed for a GPU region descriptor: %s", strerror(errno));
    }
    fd_ = -1;
  }
}

void register_gpu_memory_backend_selector(GpuMemoryBackendSelector selector)
{
  const std::lock_guard<std::mutex> lock(backend_mutex());
  backend_selector() = selector;
}

GpuMemoryBackend * get_gpu_memory_backend()
{
  // Outside the lock: loading runs the package's constructor, which registers
  // the selector.
  ensure_backend_loaded();

  const std::lock_guard<std::mutex> lock(backend_mutex());

  // Resolved once. Support is a property of the machine, so a negative answer
  // will not change within the process, and probing it costs driver calls.
  static GpuMemoryBackend * resolved = nullptr;
  static bool resolution_attempted = false;
  if (!resolution_attempted) {
    resolution_attempted = true;
    const GpuMemoryBackendSelector selector = backend_selector();
    resolved = (selector != nullptr) ? selector() : nullptr;
    if (resolved == nullptr) {
      RCLCPP_ERROR(logger, "Agnocast: no GPU memory backend is supported on this machine");
    }
  }
  return resolved;
}

}  // namespace agnocast::internal
