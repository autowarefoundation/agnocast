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

struct BackendRegistry
{
  std::mutex mutex;
  GpuMemoryBackendSelector selector = nullptr;
  GpuMemoryBackend * resolved = nullptr;
  // Support is a property of the machine, so a negative answer will not change
  // within the process and probing it costs driver calls.
  bool resolution_attempted = false;
};

// Leaked deliberately: a user's static-duration object holding a GPU publisher
// is destroyed in an order this library does not control, so teardown can reach
// this after namespace-scope statics are gone.
BackendRegistry & backend_registry()
{
  static auto * registry = new BackendRegistry();  // NOLINT(cppcoreguidelines-owning-memory)
  return *registry;
}

// Loaded, not linked: a node that reaches GPU memory only through agnocastlib's
// API references none of agnocast_gpu's symbols, so --as-needed would drop the
// DT_NEEDED entry and its registering constructor would never run. RTLD_NODELETE
// because regions dispatch into the library from their destructors.
bool backend_library_loaded()
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
  return loaded;
}

}  // namespace

void UniqueFd::reset() noexcept
try {
  if (fd_ >= 0) {
    if (close(fd_) != 0) {
      // The descriptor was already closed elsewhere, which is a double-ownership
      // bug worth surfacing.
      RCLCPP_WARN(logger, "close() failed for a GPU region descriptor: %s", strerror(errno));
    }
    fd_ = -1;
  }
} catch (...) {
  // Reached from the destructor, where logging is the only thing that can throw
  // and terminating over it would be worse than losing the message.
  fd_ = -1;
}

void register_gpu_memory_backend_selector(GpuMemoryBackendSelector selector)
{
  BackendRegistry & registry = backend_registry();
  const std::lock_guard<std::mutex> lock(registry.mutex);
  registry.selector = selector;
}

GpuMemoryBackend * get_gpu_memory_backend()
{
  // Outside the lock: loading runs the package's constructor, which registers
  // the selector.
  if (!backend_library_loaded()) return nullptr;

  BackendRegistry & registry = backend_registry();
  const std::lock_guard<std::mutex> lock(registry.mutex);
  if (!registry.resolution_attempted) {
    registry.resolution_attempted = true;
    registry.resolved = (registry.selector != nullptr) ? registry.selector() : nullptr;
    if (registry.resolved == nullptr) {
      RCLCPP_ERROR(logger, "Agnocast: no GPU memory backend is supported on this machine");
    }
  }
  return registry.resolved;
}

}  // namespace agnocast::internal
