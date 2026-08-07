#pragma once

// Runtime loader for the slice of the CUDA driver API this package uses.
//
// Types come from <cuda.h> at build time, symbols from dlopen at run time. The
// header keeps struct layouts such as CUmemAllocationProp authoritative rather
// than hand-replicated, while linking no driver lets the package build where
// none is installed and load harmlessly where there is no GPU.

#include <cuda.h>

#include <string>

namespace agnocast::gpu
{

class CudaDriverLoader
{
public:
  // Returns nullptr when the driver is unavailable, logging the cause once.
  // Not fatal by itself: a process loaded on a machine with no driver simply has
  // no GPU backend.
  static const CudaDriverLoader * instance();

  CUresult (*cuInit)(unsigned int) = nullptr;
  CUresult (*cuDeviceGet)(CUdevice *, int) = nullptr;
  CUresult (*cuDeviceGetAttribute)(int *, CUdevice_attribute, CUdevice) = nullptr;
  // The _v2 form, because it reports the MIG compute instance's UUID where the
  // original reports the parent GPU's. MIG instances are memory-isolated, so
  // conflating them would let an import be attempted across that boundary.
  CUresult (*cuDeviceGetUuid_v2)(CUuuid *, CUdevice) = nullptr;
  CUresult (*cuDevicePrimaryCtxRetain)(CUcontext *, CUdevice) = nullptr;

  CUresult (*cuCtxPushCurrent)(CUcontext) = nullptr;
  CUresult (*cuCtxPopCurrent)(CUcontext *) = nullptr;
  CUresult (*cuCtxGetCurrent)(CUcontext *) = nullptr;
  CUresult (*cuCtxGetDevice)(CUdevice *) = nullptr;
  CUresult (*cuCtxSynchronize)() = nullptr;

  CUresult (*cuMemGetAllocationGranularity)(
    size_t *, const CUmemAllocationProp *, CUmemAllocationGranularity_flags) = nullptr;
  CUresult (*cuMemCreate)(
    CUmemGenericAllocationHandle *, size_t, const CUmemAllocationProp *,
    unsigned long long) = nullptr;
  CUresult (*cuMemRelease)(CUmemGenericAllocationHandle) = nullptr;
  CUresult (*cuMemExportToShareableHandle)(
    void *, CUmemGenericAllocationHandle, CUmemAllocationHandleType, unsigned long long) = nullptr;
  CUresult (*cuMemImportFromShareableHandle)(
    CUmemGenericAllocationHandle *, void *, CUmemAllocationHandleType) = nullptr;
  CUresult (*cuMemAddressReserve)(CUdeviceptr *, size_t, size_t, CUdeviceptr, unsigned long long) =
    nullptr;
  CUresult (*cuMemAddressFree)(CUdeviceptr, size_t) = nullptr;
  CUresult (*cuMemMap)(
    CUdeviceptr, size_t, size_t, CUmemGenericAllocationHandle, unsigned long long) = nullptr;
  CUresult (*cuMemUnmap)(CUdeviceptr, size_t) = nullptr;
  CUresult (*cuMemSetAccess)(CUdeviceptr, size_t, const CUmemAccessDesc *, size_t) = nullptr;

  CUresult (*cuGetErrorName)(CUresult, const char **) = nullptr;
  CUresult (*cuGetErrorString)(CUresult, const char **) = nullptr;

  // Formats a driver result for a log line, falling back to the numeric code so
  // a diagnostic is never empty.
  [[nodiscard]] std::string describe(CUresult result) const;

private:
  CudaDriverLoader() = default;

  void * handle_ = nullptr;

  template <typename FnT>
  bool load(FnT & slot, const char * name);

  bool load_all();
};

}  // namespace agnocast::gpu
