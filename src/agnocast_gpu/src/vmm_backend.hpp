#pragma once

// GpuMemoryBackend over the CUDA virtual memory management API. Works on both
// discrete GPUs and automotive SoCs, so it is the default wherever the device
// reports support for it.

#include "agnocast/internal/gpu_backend.hpp"

#include <cuda.h>

#include <array>
#include <mutex>
#include <optional>

namespace agnocast::gpu
{

class CudaDriverLoader;

class VmmBackend : public agnocast::internal::GpuMemoryBackend
{
public:
  [[nodiscard]] bool is_supported() const noexcept override;

  [[nodiscard]] agnocast::internal::MappedGpuRegion create_region(
    uint32_t slot_size, uint32_t slot_count) override;
  [[nodiscard]] std::optional<agnocast::internal::GpuRegionExport> export_region(
    const agnocast::internal::MappedGpuRegion & region) override;
  [[nodiscard]] agnocast::internal::MappedGpuRegion import_region(
    const agnocast::internal::GpuRegionExport & exported) override;

private:
  void release_region(
    void * base, const agnocast::internal::GpuRegionGeometry & geometry,
    uint64_t backend_token) noexcept override;

  // Binds the context for one operation. Push/pop rather than set: these run on
  // whichever executor thread delivered the message, and must not disturb a
  // binding the user's own code established on it.
  class ScopedContext
  {
  public:
    ScopedContext(const CudaDriverLoader * cuda, CUcontext ctx);
    ~ScopedContext();

    ScopedContext(const ScopedContext &) = delete;
    ScopedContext & operator=(const ScopedContext &) = delete;

    [[nodiscard]] bool ok() const { return pushed_; }

  private:
    const CudaDriverLoader * cuda_;
    bool pushed_ = false;
  };

  // Resolves the driver, device and context once. The helpers below assume it
  // has succeeded, so every entry point that allocates or maps calls it first.
  [[nodiscard]] bool ensure_context() const;

  // Split out from ensure_context because answering "can this machine share GPU
  // memory at all?" needs no context, and creating one costs over a hundred
  // megabytes that an unsupported process would then hold for its lifetime.
  [[nodiscard]] bool ensure_device() const;

  [[nodiscard]] bool has_attribute(CUdevice_attribute attr, int number, const char * name) const;
  [[nodiscard]] size_t query_granularity() const;
  // `access_flags` is what separates the exporter's mapping from an importer's:
  // the publisher writes its payload, a subscriber only reads it.
  [[nodiscard]] bool map_and_grant(
    CUmemGenericAllocationHandle handle, size_t size, size_t granularity,
    CUmemAccess_flags access_flags, void ** out_base) const;

  mutable std::mutex mtx_;
  mutable bool device_ready_ = false;
  mutable bool context_ready_ = false;
  mutable CUcontext context_ = nullptr;
  mutable CUdevice device_ = 0;
  mutable std::array<uint8_t, 16> device_uuid_{};
};

}  // namespace agnocast::gpu
