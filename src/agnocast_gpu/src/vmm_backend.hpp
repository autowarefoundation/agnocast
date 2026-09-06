#pragma once

// GpuMemoryBackend over the CUDA virtual memory management API. Works on both
// discrete GPUs and automotive SoCs, so it is the default wherever the device
// reports support for it.

#include "agnocast/internal/gpu_backend.hpp"

#include <cuda.h>

#include <mutex>
#include <optional>

namespace agnocast::gpu
{

class CudaDriverLoader;

class VmmBackend : public agnocast::internal::GpuMemoryBackend
{
public:
  [[nodiscard]] agnocast::internal::GpuMemoryBackendType type() const noexcept override
  {
    return agnocast::internal::GpuMemoryBackendType::Vmm;
  }

  [[nodiscard]] bool is_supported() const noexcept override;

  [[nodiscard]] agnocast::internal::MappedGpuRegion create_region(
    uint32_t slot_size, uint32_t slot_count) override;
  [[nodiscard]] std::optional<agnocast::internal::GpuRegionExport> export_for(
    const agnocast::internal::MappedGpuRegion & region, topic_local_id_t subscriber_id) override;
  [[nodiscard]] agnocast::internal::MappedGpuRegion import_region(
    const agnocast::internal::GpuRegionExport & exported) override;

private:
  void release_region(
    void * base, const agnocast::internal::GpuRegionGeometry & geometry,
    uint64_t backend_token) noexcept override;

  // Binds the context for one operation. Push/pop rather than set, because a
  // context binding belongs to the calling thread and these operations run on
  // whichever executor thread delivered the message, which must not disturb a
  // binding the user's own code established.
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
  // has succeeded, so every entry point calls it first.
  [[nodiscard]] bool ensure_context() const;

  [[nodiscard]] bool has_attribute(CUdevice_attribute attr, int number, const char * name) const;
  [[nodiscard]] size_t query_granularity() const;
  [[nodiscard]] bool map_and_grant(
    CUmemGenericAllocationHandle handle, size_t size, size_t granularity, void ** out_base) const;

  mutable std::mutex mtx_;
  mutable bool context_ready_ = false;
  mutable CUcontext context_ = nullptr;
  mutable CUdevice device_ = 0;
  mutable std::array<uint8_t, 16> device_uuid_{};
};

}  // namespace agnocast::gpu
