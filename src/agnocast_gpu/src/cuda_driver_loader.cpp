#include "cuda_driver_loader.hpp"

#include "agnocast/agnocast_utils.hpp"

#include <dlfcn.h>

namespace agnocast::gpu
{

namespace
{

// The versioned name only. The unversioned symlink is the toolkit's stub
// library, which resolves every symbol with no driver behind it.
constexpr const char * kDriverSoname = "libcuda.so.1";

// Several driver entry points are macros onto versioned symbols
// (cuda.h: cuCtxPushCurrent -> cuCtxPushCurrent_v2). The member declarations are
// rewritten by those macros but a hand-written string literal is not, and
// libcuda still exports the superseded entry point, so a mismatch resolves
// silently to the wrong function. Stringifying after expansion keeps the name
// used for dlsym identical to the one that was declared.
#define AGNOCAST_STRINGIFY_EXPANDED(x) #x
#define AGNOCAST_SYMBOL_NAME(x) AGNOCAST_STRINGIFY_EXPANDED(x)
#define AGNOCAST_LOAD(field) load(field, AGNOCAST_SYMBOL_NAME(field))

}  // namespace

template <typename FnT>
bool CudaDriverLoader::load(FnT & slot, const char * name)
{
  dlerror();  // POSIX requires clearing before dlsym, since NULL can be a valid result.
  void * sym = dlsym(handle_, name);
  const char * err = dlerror();
  if (err != nullptr || sym == nullptr) {
    RCLCPP_ERROR(
      logger, "Agnocast GPU: symbol '%s' is missing from %s: %s", name, kDriverSoname,
      err != nullptr ? err : "not found");
    return false;
  }
  slot = reinterpret_cast<FnT>(sym);
  return true;
}

bool CudaDriverLoader::load_all()
{
  bool ok = true;
  ok = AGNOCAST_LOAD(cuInit) && ok;
  ok = AGNOCAST_LOAD(cuDeviceGet) && ok;
  ok = AGNOCAST_LOAD(cuDeviceGetAttribute) && ok;
  ok = AGNOCAST_LOAD(cuDeviceGetUuid_v2) && ok;
  ok = AGNOCAST_LOAD(cuDevicePrimaryCtxRetain) && ok;

  ok = AGNOCAST_LOAD(cuCtxPushCurrent) && ok;
  ok = AGNOCAST_LOAD(cuCtxPopCurrent) && ok;
  ok = AGNOCAST_LOAD(cuCtxGetCurrent) && ok;
  ok = AGNOCAST_LOAD(cuCtxGetDevice) && ok;
  ok = AGNOCAST_LOAD(cuCtxSynchronize) && ok;

  ok = AGNOCAST_LOAD(cuMemGetAllocationGranularity) && ok;
  ok = AGNOCAST_LOAD(cuMemCreate) && ok;
  ok = AGNOCAST_LOAD(cuMemRelease) && ok;
  ok = AGNOCAST_LOAD(cuMemExportToShareableHandle) && ok;
  ok = AGNOCAST_LOAD(cuMemImportFromShareableHandle) && ok;
  ok = AGNOCAST_LOAD(cuMemAddressReserve) && ok;
  ok = AGNOCAST_LOAD(cuMemAddressFree) && ok;
  ok = AGNOCAST_LOAD(cuMemMap) && ok;
  ok = AGNOCAST_LOAD(cuMemUnmap) && ok;
  ok = AGNOCAST_LOAD(cuMemSetAccess) && ok;

  ok = AGNOCAST_LOAD(cuGetErrorName) && ok;
  ok = AGNOCAST_LOAD(cuGetErrorString) && ok;

  return ok;
}

const CudaDriverLoader * CudaDriverLoader::instance()
{
  // Failure is cached too: a machine without a driver will not acquire one
  // mid-process. Leaked, so this stays callable from teardown paths that run
  // after static destructors.
  static const CudaDriverLoader * cached = []() -> const CudaDriverLoader * {
    auto * loader = new CudaDriverLoader();  // NOLINT(cppcoreguidelines-owning-memory)
    loader->handle_ = dlopen(kDriverSoname, RTLD_NOW | RTLD_LOCAL);
    if (loader->handle_ == nullptr) {
      const char * err = dlerror();
      RCLCPP_ERROR(
        logger,
        "Agnocast GPU: cannot load %s (%s). GPU messages are unavailable in this process; "
        "install the NVIDIA driver if this process is expected to use them.",
        kDriverSoname, err != nullptr ? err : "unknown error");
      return nullptr;
    }
    // Never dlclose()d: the driver keeps process-global state, and unloading it
    // while other CUDA users remain in the process is not safe.
    if (!loader->load_all()) return nullptr;
    return loader;
  }();

  return cached;
}

std::string CudaDriverLoader::describe(CUresult result) const
{
  const char * name = nullptr;
  const char * text = nullptr;
  if (cuGetErrorName != nullptr) {
    cuGetErrorName(result, &name);
  }
  if (cuGetErrorString != nullptr) {
    cuGetErrorString(result, &text);
  }
  if (name != nullptr && text != nullptr) {
    return std::string(name) + ": " + text;
  }
  if (name != nullptr) {
    return name;
  }
  return "CUresult " + std::to_string(static_cast<int>(result));
}

}  // namespace agnocast::gpu
