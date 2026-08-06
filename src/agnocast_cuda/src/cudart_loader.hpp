// ============================================================================
// Runtime loader for the CUDA Runtime API (libcudart.so) via dlopen/dlsym.
//
// WHY THIS EXISTS
// ---------------
// agnocast_cuda is distributed as a pre-built .deb from the ROS build farm, which has no
// CUDA toolchain. To eliminate the build-time dependency on <cuda_runtime.h> and -lcudart,
// we replicate the small set of CUDA types/constants we need (stable ABI) and load the
// actual CUDA runtime library at runtime via dlopen. This means:
//   - Build time:  pure C++17, no CUDA headers or compiler required.
//   - Run time:    libcudart.so must be present on the target machine (user installs CUDA).
//
// SAFE ON NON-CUDA SYSTEMS
// ------------------------
// Users who do not use CUDA message types are completely unaffected by this library.
// CudartLoader is lazy: dlopen("libcudart.so") only happens on the first call to
// CudartLoader::instance(), which is only reachable through the pool proxy / C ABI,
// which are only exercised when a CUDA message type is actually used. If no CUDA
// message type is ever instantiated, this loader is never constructed and no CUDA symbols
// are ever resolved. The library can be safely installed and loaded on machines without
// a GPU or CUDA runtime.
//
// ABI STABILITY
// -------------
// The types and constants below are part of NVIDIA's stable C ABI for the CUDA Runtime API.
// They have not changed across CUDA 10.x, 11.x, and 12.x. Specifically:
//   - cudaError_t is a C enum (int-sized). cudaSuccess has been 0 since CUDA 1.0.
//   - cudaIpcMemHandle_t is a 64-byte opaque struct (CUDA_IPC_HANDLE_SIZE = 64).
//   - cudaIpcMemLazyEnablePeerAccess is a flag constant (0x01).
//   - cudaDevAttrIntegrated is enum value 18 in cudaDeviceAttr (append-only enum).
// If NVIDIA ever breaks this ABI (extremely unlikely), the static_assert in
// cuda_ipc_backend.cpp will catch size mismatches at compile time.
//
// ADDING NEW CUDA API CALLS
// -------------------------
// When a new backend or feature needs an additional CUDA runtime function:
//   1. Add the function pointer typedef below (e.g., using cudaMalloc_t = ...).
//   2. Add a public member to CudartLoader (e.g., cudaMalloc_t cudaMalloc;).
//   3. Add a load_symbol() call in the constructor.
// Keep this file as the single place that touches dlopen/dlsym for CUDA.
//
// LIBRARY NAME FALLBACK CHAIN
// ---------------------------
// The constructor tries these names in order:
//   1. "libcudart.so"     — unversioned symlink, present when the CUDA dev package is
//                           installed (e.g., cuda-cudart-dev-12-*) or with the runfile
//                           installer.
//   2. "libcudart.so.12"  — SONAME for CUDA 12.x. Present with the runtime-only deb
//                           package (cuda-cudart-12-*) even without the dev package.
//   3. "libcudart.so.11.0"— SONAME for CUDA 11.x (NVIDIA used major.minor for the SONAME
//                           in the 11.x series; all 11.0–11.8 share this SONAME).
// If a future CUDA major version (e.g., 13.x) is released, add its SONAME to the list.
// ============================================================================
#pragma once

#include <dlfcn.h>

#include <cstdio>
#include <cstdlib>

namespace agnocast::cuda
{

// ---------------------------------------------------------------------------
// ABI-compatible CUDA Runtime API type definitions.
// These replace #include <cuda_runtime.h> so that no CUDA headers are needed at build time.
// See "ABI STABILITY" section in the file header for rationale.
// ---------------------------------------------------------------------------
using cudaError_t = int;
constexpr cudaError_t cudaSuccess = 0;

struct cudaIpcMemHandle_t
{
  char reserved[64];  // CUDA_IPC_HANDLE_SIZE = 64
};

constexpr unsigned int cudaIpcMemLazyEnablePeerAccess = 0x01;

using cudaDeviceAttr = int;
constexpr cudaDeviceAttr cudaDevAttrIntegrated = 18;  // driver_types.h: cudaDevAttrIntegrated = 18

// Opaque event handle (a pointer to a driver object) and its 64-byte IPC handle.
using cudaEvent_t = void *;
using cudaStream_t = void *;

// Per-thread default stream handle (CUDART_VERSION stable value). Recording/waiting
// events on this stream avoids serializing with the legacy default stream and other
// streams. User GPU work must also run on the per-thread default stream (compile
// with nvcc --default-stream per-thread).
inline cudaStream_t cuda_stream_per_thread()
{
  return reinterpret_cast<cudaStream_t>(0x2);
}

// cudaStreamWaitEvent flags: 0 == cudaEventWaitDefault.
constexpr unsigned int cudaEventWaitDefault = 0x00;

struct cudaIpcEventHandle_t
{
  char reserved[64];  // CUDA_IPC_HANDLE_SIZE = 64
};

// 16-byte device UUID (matches cudaDeviceProp::uuid; append-only stable ABI).
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

// ---------------------------------------------------------------------------
// Function pointer types matching CUDA Runtime API signatures.
// ---------------------------------------------------------------------------
using cudaGetDevice_t = cudaError_t (*)(int *);
using cudaDeviceGetAttribute_t = cudaError_t (*)(int *, cudaDeviceAttr, int);
using cudaGetDeviceProperties_t = cudaError_t (*)(cudaDevicePropUuidView *, int);
using cudaIpcGetMemHandle_t = cudaError_t (*)(cudaIpcMemHandle_t *, void *);
using cudaIpcOpenMemHandle_t = cudaError_t (*)(void **, cudaIpcMemHandle_t, unsigned int);
using cudaIpcCloseMemHandle_t = cudaError_t (*)(void *);
using cudaIpcOpenEventHandle_t = cudaError_t (*)(cudaEvent_t *, cudaIpcEventHandle_t);
using cudaEventDestroy_t = cudaError_t (*)(cudaEvent_t);
using cudaEventRecord_t = cudaError_t (*)(cudaEvent_t, cudaStream_t);
using cudaStreamWaitEvent_t = cudaError_t (*)(cudaStream_t, cudaEvent_t, unsigned int);
using cudaFree_t = cudaError_t (*)(void *);
using cudaGetErrorString_t = const char * (*)(cudaError_t);

// ---------------------------------------------------------------------------
// CudartLoader — lazy-loading singleton for the CUDA runtime.
//
// Thread safety: the function-local static in instance() is guaranteed to be
// initialized exactly once (C++11 "magic statics"). After construction, all
// function pointers are immutable and safe to read from any thread.
//
// Lifetime: the singleton is never destroyed during normal execution. We
// intentionally do NOT call dlclose() — see destructor comment.
// ---------------------------------------------------------------------------
class CudartLoader
{
public:
  static CudartLoader & instance()
  {
    static CudartLoader loader;
    return loader;
  }

  // Public function pointers — valid immediately after instance() returns.
  // Callers use these like: CudartLoader::instance().cudaFree(ptr)
  cudaGetDevice_t cudaGetDevice;
  cudaDeviceGetAttribute_t cudaDeviceGetAttribute;
  cudaGetDeviceProperties_t cudaGetDeviceProperties;
  cudaIpcGetMemHandle_t cudaIpcGetMemHandle;
  cudaIpcOpenMemHandle_t cudaIpcOpenMemHandle;
  cudaIpcCloseMemHandle_t cudaIpcCloseMemHandle;
  cudaIpcOpenEventHandle_t cudaIpcOpenEventHandle;
  cudaEventDestroy_t cudaEventDestroy;
  cudaEventRecord_t cudaEventRecord;
  cudaStreamWaitEvent_t cudaStreamWaitEvent;
  cudaFree_t cudaFree;
  cudaGetErrorString_t cudaGetErrorString;

private:
  void * handle_ = nullptr;

  CudartLoader()
  {
    // Try library names in order. See "LIBRARY NAME FALLBACK CHAIN" in the file header.
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
        "[agnocast_cuda] FATAL: Could not load libcudart.so.\n"
        "  Your code uses a CUDA message type (e.g., agnocast::cuda::PointCloud2 or\n"
        "  agnocast::cuda::Image), which requires the NVIDIA CUDA runtime to be installed\n"
        "  on this machine. If you do not need GPU-accelerated message passing, use the\n"
        "  standard (non-CUDA) message types instead.\n"
        "  To fix: install the CUDA runtime (e.g., 'sudo apt install cuda-cudart-12-*')\n"
        "  and ensure libcudart.so is on LD_LIBRARY_PATH.\n"
        "  dlopen error: %s\n",
        dlerror());
      std::abort();
    }

    load_symbol(cudaGetDevice, "cudaGetDevice");
    load_symbol(cudaDeviceGetAttribute, "cudaDeviceGetAttribute");
    load_symbol(cudaGetDeviceProperties, "cudaGetDeviceProperties");
    load_symbol(cudaIpcGetMemHandle, "cudaIpcGetMemHandle");
    load_symbol(cudaIpcOpenMemHandle, "cudaIpcOpenMemHandle");
    load_symbol(cudaIpcCloseMemHandle, "cudaIpcCloseMemHandle");
    load_symbol(cudaIpcOpenEventHandle, "cudaIpcOpenEventHandle");
    load_symbol(cudaEventDestroy, "cudaEventDestroy");
    load_symbol(cudaEventRecord, "cudaEventRecord");
    load_symbol(cudaStreamWaitEvent, "cudaStreamWaitEvent");
    load_symbol(cudaFree, "cudaFree");
    load_symbol(cudaGetErrorString, "cudaGetErrorString");
  }

  // Intentionally never close the handle. The CUDA runtime maintains process-global state
  // (device context, allocations, etc.). Calling dlclose() during static destruction can
  // race with other CUDA cleanup and cause segfaults. The OS reclaims everything at exit.
  ~CudartLoader() = default;

  CudartLoader(const CudartLoader &) = delete;
  CudartLoader & operator=(const CudartLoader &) = delete;

  template <typename T>
  void load_symbol(T & func_ptr, const char * name)
  {
    // POSIX requires clearing dlerror() before calling dlsym(), then checking dlerror()
    // after, because dlsym() can legitimately return NULL for some symbols.
    dlerror();
    func_ptr = reinterpret_cast<T>(dlsym(handle_, name));
    const char * err = dlerror();
    if (err != nullptr) {
      std::fprintf(
        stderr, "[agnocast_cuda] FATAL: Could not load symbol '%s' from libcudart.so: %s\n", name,
        err);
      std::abort();
    }
  }
};

}  // namespace agnocast::cuda
