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
// That is the only axis this type represents. Cross-process GPU synchronization
// must stay independent of it: NvSciBuf memory may be paired with CUDA events,
// NvSciSync, or nothing at all.
//
// These values cross the userspace-kernel ABI so an importer can reject a
// mechanism it cannot handle. The kmod stores them without interpretation and
// takes no part in deciding what a machine supports. Never renumber or reuse.
enum class GpuMemoryBackendType : uint32_t {
  Unknown = 0,
  // Shared as a POSIX file descriptor. Discrete GPU and SoC.
  Vmm = 1,
  // Shared as an endpoint-bound descriptor. Its attribute reconciliation lets a
  // single buffer be addressed by CUDA, NvMedia, DLA or Vulkan, which is why it
  // exists alongside Vmm on SoCs that support both.
  NvSciBuf = 2,
};

// CUDA IPC is deliberately absent. Both mechanisms above give the allocation a
// lifetime that outlives its creator -- a VMM allocation is held by a file
// descriptor, an NvSciBuf object by its own reference count -- so a publisher
// can crash without invalidating the mappings its subscribers are reading. A
// CUDA IPC handle is an opaque token with no backing kernel object, so nothing
// can hold a reference on the allocation's behalf; the memory is freed when the
// exporter dies and subscribers are left with dangling device pointers.
// Restoring that guarantee would take a dedicated process owning every
// allocation: another component to supervise, and a new single point of failure.

// The descriptor references the whole allocation, not one slot, so leaking it
// retains the entire region until process exit.
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

  void reset();

private:
  int fd_ = -1;
};

// The shape of a region, and the only place its invariants are stated. Both
// sides of the process boundary check against these.
struct GpuRegionGeometry
{
  uint32_t slot_size = 0;
  uint32_t slot_count = 0;
  // The exporter's rounded allocation size, carried rather than recomputed:
  // allocation granularity is a property of the importing device, and a larger
  // one would make the mapping exceed the allocation.
  uint64_t mapped_size = 0;
  // A UUID, not an ordinal. Ordinals are process-relative, so two processes
  // with different CUDA_VISIBLE_DEVICES both see an ordinal 0 that is a
  // different physical device. On a MIG-partitioned GPU this is the compute
  // instance's UUID, since instances cannot import each other's memory.
  std::array<uint8_t, 16> device_uuid{};

  [[nodiscard]] bool is_consistent() const noexcept
  {
    return slot_size != 0 && slot_count != 0 &&
           static_cast<uint64_t>(slot_size) * slot_count <= mapped_size;
  }

  [[nodiscard]] bool contains(uint32_t slot_index, uint64_t bytes) const noexcept
  {
    return slot_index < slot_count && bytes <= slot_size;
  }

  [[nodiscard]] uint64_t offset_of(uint32_t slot_index) const noexcept
  {
    return static_cast<uint64_t>(slot_index) * slot_size;
  }
};

struct VmmExportHandle
{
  UniqueFd fd;
};

struct NvSciBufExportHandle
{
  std::vector<uint8_t> descriptor;
};

// Naming each alternative keeps the meaning of the payload explicit at the type
// level rather than leaving a bare descriptor or byte buffer to convention.
using GpuRegionExportHandle = std::variant<std::monostate, VmmExportHandle, NvSciBufExportHandle>;

// What crosses the process boundary: everything a peer needs to map the region.
struct GpuRegionExport
{
  GpuMemoryBackendType backend = GpuMemoryBackendType::Unknown;
  GpuRegionGeometry geometry;
  GpuRegionExportHandle handle;
};

class GpuMemoryBackend;

// What stays in this process: the mapping, and its owner. Move-only, because a
// mapping cannot be released twice.
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

  // Messages address memory by slot index, never by device address: device
  // virtual addresses are process-local, VMM imports may land elsewhere, and the
  // CUDA NvSciBuf import path offers no way to choose one. The index arrives
  // from another process, so it is checked here rather than trusted.
  [[nodiscard]] void * slot_address(uint32_t slot_index, uint64_t bytes) const noexcept
  {
    if (base_ == nullptr || !geometry_.contains(slot_index, bytes)) return nullptr;
    return static_cast<uint8_t *>(base_) + geometry_.offset_of(slot_index);
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

  // Per-subscriber because NvSciBuf export descriptors are bound to a
  // destination endpoint. Backends without that restriction ignore the id.
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
