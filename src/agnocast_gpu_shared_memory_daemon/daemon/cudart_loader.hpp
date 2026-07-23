// Runtime loader for the CUDA Runtime API (libcudart.so) via dlopen/dlsym.
//
// Like agnocast_cuda's loader, this exists so the daemon has ZERO build-time
// dependency on the CUDA toolkit: it can be built as a pre-built .deb on a build
// farm with no CUDA, and loads libcudart.so at runtime on the target machine.
// The small set of CUDA types/constants we need is replicated here against
// NVIDIA's stable C ABI (unchanged across CUDA 10/11/12).
//
// The loader is lazy: dlopen happens on first use of instance(), which is only
// reached when a CudaIpcSlotBackend actually touches the GPU. Unit tests that use
// a mock backend never construct it, so they run on machines without CUDA.
#pragma once

#include <dlfcn.h>

#include <cstdio>
#include <cstdlib>

namespace agnocast::gpu_shared_memory_daemon
{

// ---- ABI-compatible CUDA Runtime API types/constants (no <cuda_runtime.h>) ----
using cudaError_t = int;
constexpr cudaError_t cudaSuccess = 0;

// Opaque event handle. cudaEvent_t is a pointer to an opaque driver object.
using cudaEvent_t = void *;

struct cudaIpcMemHandle_t
{
  char reserved[64];  // CUDA_IPC_HANDLE_SIZE = 64
};

struct cudaIpcEventHandle_t
{
  char reserved[64];  // CUDA_IPC_HANDLE_SIZE = 64
};

struct cudaUUID_t
{
  char bytes[16];
};

// Partial, ABI-stable view of cudaDeviceProp used only to read the device UUID.
//
// We obtain the GPU UUID via cudaGetDeviceProperties rather than cudaDeviceGetUuid:
// the latter is NOT exported by every libcudart (e.g. CUDA 12.x runtimes omit it),
// whereas cudaGetDeviceProperties is present across CUDA 10/11/12. cudaDeviceProp's
// first two members have been `char name[256]` followed by `cudaUUID_t uuid` since
// CUDA 10, so `uuid` is at byte offset 256 in every version. We do not replicate the
// full (large, version-varying) struct; instead we over-size the trailing buffer far
// beyond any real cudaDeviceProp (~1 KiB) so the runtime's full write stays in bounds
// and only read the two leading fields.
struct cudaDevicePropUuidView
{
  char name[256];
  cudaUUID_t uuid;
  char reserved_for_full_prop[4096];
};

// cudaEventCreateWithFlags flags. Interprocess events must disable timing.
constexpr unsigned int cudaEventDisableTiming = 0x02;
constexpr unsigned int cudaEventInterprocess = 0x04;

// ---- Function pointer types ----
using cudaSetDevice_t = cudaError_t (*)(int);
using cudaGetDevice_t = cudaError_t (*)(int *);
using cudaGetDeviceProperties_t = cudaError_t (*)(cudaDevicePropUuidView *, int);
using cudaMalloc_t = cudaError_t (*)(void **, size_t);
using cudaFree_t = cudaError_t (*)(void *);
using cudaIpcGetMemHandle_t = cudaError_t (*)(cudaIpcMemHandle_t *, void *);
using cudaEventCreateWithFlags_t = cudaError_t (*)(cudaEvent_t *, unsigned int);
using cudaEventDestroy_t = cudaError_t (*)(cudaEvent_t);
using cudaIpcGetEventHandle_t = cudaError_t (*)(cudaIpcEventHandle_t *, cudaEvent_t);
using cudaGetErrorString_t = const char * (*)(cudaError_t);

// Lazy-loading singleton for the CUDA runtime. See agnocast_cuda/src/cudart_loader.hpp
// for the full rationale (thread safety via magic statics, no dlclose, fallback chain).
class CudartLoader
{
public:
  static CudartLoader & instance()
  {
    static CudartLoader loader;
    return loader;
  }

  cudaSetDevice_t cudaSetDevice;
  cudaGetDevice_t cudaGetDevice;
  cudaGetDeviceProperties_t cudaGetDeviceProperties;
  cudaMalloc_t cudaMalloc;
  cudaFree_t cudaFree;
  cudaIpcGetMemHandle_t cudaIpcGetMemHandle;
  cudaEventCreateWithFlags_t cudaEventCreateWithFlags;
  cudaEventDestroy_t cudaEventDestroy;
  cudaIpcGetEventHandle_t cudaIpcGetEventHandle;
  cudaGetErrorString_t cudaGetErrorString;

private:
  void * handle_ = nullptr;

  CudartLoader()
  {
    dlerror();  // Clear any stale error before the dlopen loop.
    const char * names[] = {"libcudart.so", "libcudart.so.12", "libcudart.so.11.0"};
    for (const char * name : names) {
      handle_ = dlopen(name, RTLD_NOW | RTLD_LOCAL);
      if (handle_ != nullptr) {
        break;
      }
    }
    if (handle_ == nullptr) {
      std::fprintf(
        stderr,
        "[agnocast_gpu_shared_memory_daemon] FATAL: Could not load libcudart.so.\n"
        "  The daemon manages GPU memory and requires the NVIDIA CUDA runtime to be\n"
        "  installed. dlopen error: %s\n",
        dlerror());
      std::abort();
    }

    load_symbol(cudaSetDevice, "cudaSetDevice");
    load_symbol(cudaGetDevice, "cudaGetDevice");
    load_symbol(cudaGetDeviceProperties, "cudaGetDeviceProperties");
    load_symbol(cudaMalloc, "cudaMalloc");
    load_symbol(cudaFree, "cudaFree");
    load_symbol(cudaIpcGetMemHandle, "cudaIpcGetMemHandle");
    load_symbol(cudaEventCreateWithFlags, "cudaEventCreateWithFlags");
    load_symbol(cudaEventDestroy, "cudaEventDestroy");
    load_symbol(cudaIpcGetEventHandle, "cudaIpcGetEventHandle");
    load_symbol(cudaGetErrorString, "cudaGetErrorString");
  }

  // Intentionally never dlclose(): the CUDA runtime holds process-global state and
  // closing during static destruction can race with CUDA cleanup. The OS reclaims at exit.
  ~CudartLoader() = default;

  CudartLoader(const CudartLoader &) = delete;
  CudartLoader & operator=(const CudartLoader &) = delete;

  template <typename T>
  void load_symbol(T & func_ptr, const char * name)
  {
    dlerror();
    func_ptr = reinterpret_cast<T>(dlsym(handle_, name));
    const char * err = dlerror();
    if (err != nullptr) {
      std::fprintf(
        stderr,
        "[agnocast_gpu_shared_memory_daemon] FATAL: Could not load symbol '%s' from "
        "libcudart.so: %s\n",
        name, err);
      std::abort();
    }
  }
};

}  // namespace agnocast::gpu_shared_memory_daemon
