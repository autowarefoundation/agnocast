#pragma once

// The interface agnocast_gpu implements. Nothing outside that package and the
// region registry should need this header.

#include "agnocast/agnocast_ioctl.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

namespace agnocast::internal
{

// How a region's memory was allocated and made importable by another process.
// These values cross the userspace-kernel ABI, so never renumber or reuse one.
// docs/gpu_ipc.md covers which mechanisms qualify and why CUDA IPC does not.
enum class GpuMemoryBackendType : uint32_t {
  Unknown = 0,
  // Shared as a POSIX file descriptor. Discrete GPU and SoC.
  Vmm = 1,
  // Shared as an endpoint-bound descriptor. Reserved, not implemented: an
  // NvSciBuf export is reconciled against the destination endpoint, so the bytes
  // one subscriber receives mean nothing to another. Serving it needs an export
  // produced per request, where the kmod stores a single export at registration
  // and hands the same bytes to every importer.
  NvSciBuf = 2,
};

// The descriptor references the whole allocation, not one slot, so leaking one
// retains an entire region until process exit.
class UniqueFd
{
public:
  UniqueFd() = default;
  explicit UniqueFd(int fd) : fd_(fd) {}
  ~UniqueFd() { reset(); }

  UniqueFd(const UniqueFd &) = delete;
  UniqueFd & operator=(const UniqueFd &) = delete;

  UniqueFd(UniqueFd && other) noexcept : fd_(std::exchange(other.fd_, -1)) {}
  UniqueFd & operator=(UniqueFd && other) noexcept
  {
    if (this != &other) {
      reset();
      fd_ = std::exchange(other.fd_, -1);
    }
    return *this;
  }

  [[nodiscard]] int get() const noexcept { return fd_; }
  [[nodiscard]] bool valid() const noexcept { return fd_ >= 0; }

  // Used where the kmod takes over holding the region's liveness reference.
  [[nodiscard]] int release() noexcept { return std::exchange(fd_, -1); }

  // noexcept because the destructor is: it logs, and logging can throw.
  void reset() noexcept;

private:
  int fd_ = -1;
};

// The specification of a region: how large it is, how it is divided into slots,
// and which device it lives on. Description only; MappedGpuRegion below is what
// holds one.
struct GpuRegionGeometry
{
  uint32_t slot_size = 0;
  uint32_t slot_count = 0;
  // The exporter's rounded allocation size, carried rather than recomputed:
  // allocation granularity is a property of the importing device, and a larger
  // one would make the mapping exceed the allocation.
  uint64_t mapped_size = 0;
  // A UUID rather than an ordinal, which would be process-relative. On a
  // MIG-partitioned GPU this is the compute instance's.
  std::array<uint8_t, 16> device_uuid{};
};

// Checked on every import, because a geometry arrives from another process.
// Slot addressing bounds an index against slot_count alone, so addresses stay
// inside the mapping only if the slots are known to fit there.
[[nodiscard]] inline bool is_consistent(const GpuRegionGeometry & geometry) noexcept
{
  return geometry.slot_size != 0 && geometry.slot_count != 0 &&
         static_cast<uint64_t>(geometry.slot_size) * geometry.slot_count <= geometry.mapped_size;
}

struct VmmExportHandle
{
  UniqueFd fd;
};

struct NvSciBufExportHandle
{
  std::vector<uint8_t> descriptor;
};

using GpuRegionExportHandle = std::variant<std::monostate, VmmExportHandle, NvSciBufExportHandle>;

// What crosses the process boundary: everything a peer needs to map the region.
struct GpuRegionExport
{
  GpuMemoryBackendType backend = GpuMemoryBackendType::Unknown;
  GpuRegionGeometry geometry;
  GpuRegionExportHandle handle;
};

class GpuMemoryBackend;

// The owner of a region's mapping in this process, released on destruction: the
// publisher gets one from create_region, each subscriber one from
// import_region. Move-only, because a mapping cannot be released twice.
class MappedGpuRegion
{
public:
  MappedGpuRegion() = default;
  MappedGpuRegion(
    GpuMemoryBackend & backend, void * base, const GpuRegionGeometry & geometry,
    uint64_t backend_token)
  : backend_(&backend), base_(base), geometry_(geometry), backend_token_(backend_token)
  {
  }
  ~MappedGpuRegion() { reset(); }

  MappedGpuRegion(const MappedGpuRegion &) = delete;
  MappedGpuRegion & operator=(const MappedGpuRegion &) = delete;

  MappedGpuRegion(MappedGpuRegion && other) noexcept { swap(other); }
  MappedGpuRegion & operator=(MappedGpuRegion && other) noexcept
  {
    if (this != &other) {
      reset();
      swap(other);
    }
    return *this;
  }

  [[nodiscard]] bool valid() const noexcept { return base_ != nullptr; }
  [[nodiscard]] const GpuRegionGeometry & geometry() const noexcept { return geometry_; }
  [[nodiscard]] uint64_t backend_token() const noexcept { return backend_token_; }

  // The index arrives from another process, so it is checked here rather than
  // trusted: out of range, or a payload larger than a slot, yields nullptr.
  [[nodiscard]] void * slot_address(uint32_t slot_index, uint64_t bytes) const noexcept
  {
    if (base_ == nullptr || slot_index >= geometry_.slot_count || bytes > geometry_.slot_size) {
      return nullptr;
    }
    return static_cast<uint8_t *>(base_) + static_cast<uint64_t>(slot_index) * geometry_.slot_size;
  }

  void reset() noexcept;

private:
  void swap(MappedGpuRegion & other) noexcept
  {
    std::swap(backend_, other.backend_);
    std::swap(base_, other.base_);
    std::swap(geometry_, other.geometry_);
    std::swap(backend_token_, other.backend_token_);
  }

  GpuMemoryBackend * backend_ = nullptr;
  void * base_ = nullptr;
  GpuRegionGeometry geometry_;
  uint64_t backend_token_ = 0;
};

// Every operation here is cold path: a publisher creates a region once, and a
// subscriber imports it on first receipt.
class GpuMemoryBackend
{
public:
  virtual ~GpuMemoryBackend() = default;

  [[nodiscard]] virtual GpuMemoryBackendType type() const noexcept = 0;
  [[nodiscard]] virtual bool is_supported() const noexcept = 0;

  // The backend owns all allocation-size rounding and records the result in the
  // returned region's geometry. Failure yields an invalid region.
  [[nodiscard]] virtual MappedGpuRegion create_region(uint32_t slot_size, uint32_t slot_count) = 0;

  // Produces everything a peer needs to map the region. `subscriber_id` names
  // the peer it is meant for, for a mechanism whose descriptor is bound to its
  // destination; one whose descriptor is not may ignore it and serve every
  // subscriber the same export.
  [[nodiscard]] virtual std::optional<GpuRegionExport> export_for(
    const MappedGpuRegion & region, topic_local_id_t subscriber_id) = 0;

  [[nodiscard]] virtual MappedGpuRegion import_region(const GpuRegionExport & exported) = 0;

private:
  friend class MappedGpuRegion;

  // The caller must ensure no device work can still reach the region: unmapping
  // memory referenced by asynchronous work faults at an arbitrary later point.
  virtual void release_region(
    void * base, const GpuRegionGeometry & geometry, uint64_t backend_token) noexcept = 0;
};

inline void MappedGpuRegion::reset() noexcept
{
  if (backend_ != nullptr && base_ != nullptr) {
    backend_->release_region(base_, geometry_, backend_token_);
  }
  backend_ = nullptr;
  base_ = nullptr;
  geometry_ = GpuRegionGeometry{};
  backend_token_ = 0;
}

using GpuMemoryBackendSelector = GpuMemoryBackend * (*)();

// Called from agnocast_gpu's library constructor. Selection itself is deferred
// to first use: probing support requires the GPU driver, and a process that
// never touches GPU messages should not load it.
void register_gpu_memory_backend_selector(GpuMemoryBackendSelector selector);

// Loads agnocast_gpu on first call. nullptr is not by itself an error.
[[nodiscard]] GpuMemoryBackend * get_gpu_memory_backend();

}  // namespace agnocast::internal
