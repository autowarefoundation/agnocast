#pragma once

#include "agnocast/agnocast_ioctl.hpp"

#include <array>
#include <cassert>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace agnocast::internal
{

// How a region's memory was allocated and made importable by another process.
// This is the only axis represented by this type. Cross-process GPU
// synchronization is deliberately absent from the current design and is not
// part of a region's memory-backend identity.
//
// If cross-process GPU synchronization is introduced later, it must remain an
// independent axis. For example, NvSciBuf memory may be used with CUDA events,
// NvSciSync, or no explicit synchronization. Synchronization mechanisms must
// therefore be represented separately rather than added as values here.
//
// These numeric values are part of the userspace-kernel ABI: they are carried
// through the kmod so that a subscriber can reject a region whose mechanism it
// cannot import. The kmod stores them without interpretation and takes no part
// in deciding what a machine supports; that is determined locally by the
// agnocast_gpu package. Existing values must never be renumbered or reused.
enum class GpuMemoryBackendType : uint32_t {
  Unknown = 0,
  // CUDA VMM allocation, shared as a POSIX file descriptor. Discrete GPU and SoC.
  Vmm = 1,
  // NvSciBuf allocation, shared as an endpoint-bound descriptor. Its attribute
  // reconciliation lets a single buffer be addressed by CUDA, NvMedia, DLA or
  // Vulkan, which is why it exists alongside Vmm on SoCs that support both.
  NvSciBuf = 2,
};

// CUDA IPC is deliberately not a member. Both mechanisms above give the
// allocation a lifetime that outlives its creator: a VMM allocation is kept
// alive by a file descriptor, which is a kernel object any process can hold, and
// an NvSciBuf object carries its own reference count. A publisher can therefore
// crash without invalidating the mappings its subscribers are still reading.
//
// A CUDA IPC handle is an opaque token with no backing kernel object, so nothing
// can hold a reference on the allocation's behalf. Ownership stays with the
// exporting process, and when it dies the memory is freed while subscribers
// still have it mapped, leaving them with dangling device pointers. Restoring
// that guarantee would take a dedicated process owning every allocation, adding
// a component to supervise and a new single point of failure.

// Owns the exported POSIX file descriptor.
//
// The descriptor holds a reference to the entire CUDA VMM allocation, not to an
// individual slot. The allocation cannot be reclaimed until all mappings are
// unmapped and all CUDA and shareable-handle references are released. Leaking
// this descriptor can therefore retain the entire region until process exit.
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

  // Hands ownership to the caller, which must close it. Used where the kmod
  // takes over holding the region's liveness reference.
  [[nodiscard]] int release() noexcept { return std::exchange(fd_, -1); }

  void reset();

private:
  int fd_ = -1;
};

struct VmmExportHandle
{
  UniqueFd fd;
};

struct NvSciBufExportHandle
{
  std::vector<uint8_t> descriptor;
};

// The exported handle, in the form the producing mechanism defines. Naming each
// alternative keeps the meaning of the payload explicit at the type level rather
// than leaving a bare descriptor or byte buffer to be interpreted by convention.
using GpuRegionExportHandle = std::variant<std::monostate, VmmExportHandle, NvSciBufExportHandle>;

// What a peer needs in order to map a region.
struct GpuRegionDescriptor
{
  GpuMemoryBackendType backend = GpuMemoryBackendType::Unknown;
  uint32_t slot_size = 0;
  uint32_t slot_count = 0;
  // The exporter's rounded allocation size. Carried rather than recomputed
  // because allocation granularity is a property of the importing device, and a
  // larger one would make the mapping exceed the allocation.
  uint64_t mapped_size = 0;
  // Identifies the GPU by UUID, not by ordinal: ordinals are process-relative,
  // so two processes with different CUDA_VISIBLE_DEVICES both see an ordinal 0
  // that is a different physical device. On a MIG-partitioned GPU this is the
  // compute instance's UUID, since instances are memory-isolated from one
  // another and cannot import each other's regions.
  std::array<uint8_t, 16> device_uuid{};
  GpuRegionExportHandle handle;
};

class GpuMemoryBackend;

// Everything a backend needs to describe a mapping it has just established.
// Exists only to be handed to GpuRegion, which becomes its owner.
struct GpuRegionMapping
{
  void * base = nullptr;
  uint64_t mapped_size = 0;
  uint32_t slot_size = 0;
  uint32_t slot_count = 0;
  std::array<uint8_t, 16> device_uuid{};
  // Opaque to everything but the backend that produced it. Process-local, and
  // never serialized.
  uint64_t backend_token = 0;
};

// A process-local mapping of a publisher-owned GPU region.
//
// Owns the mapping: releasing it is this object's responsibility, not the
// caller's. Move-only, because a mapping cannot be released twice.
class GpuRegion
{
public:
  GpuRegion() = default;
  ~GpuRegion() { reset(); }

  GpuRegion(const GpuRegion &) = delete;
  GpuRegion & operator=(const GpuRegion &) = delete;

  GpuRegion(GpuRegion && other) noexcept { swap(other); }
  GpuRegion & operator=(GpuRegion && other) noexcept
  {
    if (this != &other) {
      reset();
      swap(other);
    }
    return *this;
  }

  [[nodiscard]] bool valid() const noexcept { return mapping_.base != nullptr; }
  [[nodiscard]] uint32_t slot_size() const noexcept { return mapping_.slot_size; }
  [[nodiscard]] uint32_t slot_count() const noexcept { return mapping_.slot_count; }
  [[nodiscard]] uint64_t mapped_size() const noexcept { return mapping_.mapped_size; }
  [[nodiscard]] const std::array<uint8_t, 16> & device_uuid() const noexcept
  {
    return mapping_.device_uuid;
  }

  // Messages identify memory by slot index, never by device address. Device
  // virtual addresses are process-local and are not part of the IPC contract:
  // CUDA VMM imports may use different mapping addresses, and the CUDA NvSciBuf
  // import path provides no way to select one.
  //
  // A slot index originates in another process, so it must be validated where it
  // is received rather than here; this assertion only documents that the caller
  // has already done so.
  [[nodiscard]] void * slot_address(uint32_t slot_index) const noexcept
  {
    assert(slot_index < mapping_.slot_count);
    return static_cast<uint8_t *>(mapping_.base) +
           static_cast<uint64_t>(slot_index) * mapping_.slot_size;
  }

  void reset() noexcept;

private:
  friend class GpuMemoryBackend;

  void swap(GpuRegion & other) noexcept
  {
    std::swap(backend_, other.backend_);
    std::swap(mapping_, other.mapping_);
  }

  GpuMemoryBackend * backend_ = nullptr;
  GpuRegionMapping mapping_;
};

// Implemented by the agnocast_gpu package.
//
// Region lifecycle operations are cold path: a publisher creates one region,
// each subscriber imports it on first receipt, and an export descriptor is
// created once for each publisher-subscriber pair.
class GpuMemoryBackend
{
public:
  virtual ~GpuMemoryBackend() = default;

  [[nodiscard]] virtual GpuMemoryBackendType type() const noexcept = 0;
  [[nodiscard]] virtual bool is_supported() const noexcept = 0;

  // Creates one backend allocation containing slot_count fixed-size slots.
  // The backend owns all allocation-size rounding and records the exact mapped
  // size in the returned region.
  [[nodiscard]] virtual bool create_region(
    uint32_t slot_size, uint32_t slot_count, GpuRegion & out_region) = 0;

  // Exports a descriptor for one destination subscriber. This operation is
  // per-subscriber because NvSciBuf export descriptors are bound to a
  // destination endpoint. Backends without that restriction may return
  // equivalent descriptors for every subscriber.
  [[nodiscard]] virtual bool export_for(
    const GpuRegion & region, topic_local_id_t subscriber_id,
    GpuRegionDescriptor & out_descriptor) = 0;

  [[nodiscard]] virtual bool import_region(
    const GpuRegionDescriptor & descriptor, GpuRegion & out_region) = 0;

protected:
  // Transfers a mapping the backend has just established to `region`, which
  // becomes responsible for releasing it.
  void adopt(GpuRegion & region, const GpuRegionMapping & mapping) noexcept
  {
    region.reset();
    region.backend_ = this;
    region.mapping_ = mapping;
  }

  [[nodiscard]] static const GpuRegionMapping & mapping_of(const GpuRegion & region) noexcept
  {
    return region.mapping_;
  }

private:
  friend class GpuRegion;

  // Invoked only by GpuRegion's destructor. The caller must ensure that no
  // device work can access the region before it is unmapped; unmapping memory
  // still referenced by asynchronous device work may cause a later device fault.
  virtual void release_region(const GpuRegionMapping & mapping) noexcept = 0;
};

inline void GpuRegion::reset() noexcept
{
  if (backend_ != nullptr && mapping_.base != nullptr) {
    backend_->release_region(mapping_);
  }
  backend_ = nullptr;
  mapping_ = GpuRegionMapping{};
}

// Chooses the backend to use on this machine, or nullptr if none of the
// mechanisms the package was built with is supported here.
using GpuMemoryBackendSelector = GpuMemoryBackend * (*)();

// Registered when agnocast_gpu is loaded. The selector is not run until the
// first request, because probing support requires the GPU driver and a process
// that never publishes GPU messages should not load it.
void register_gpu_memory_backend_selector(GpuMemoryBackendSelector selector);

// Loads the backend package on first call. Returns nullptr when it is
// unavailable, which is not by itself an error: a process that publishes no GPU
// messages never needs one.
[[nodiscard]] GpuMemoryBackend * get_gpu_memory_backend();

// Every region this process has mapped, keyed by the id the kernel module
// assigned it. A message identifies its memory by that id and a slot index, so
// resolving it needs no knowledge of which topic or publisher produced it.
// Regions live until the process exits: they are mapped once and shared by every
// message that uses them.
void register_mapped_region(uint32_t region_id, GpuRegion && region);
[[nodiscard]] const GpuRegion * find_mapped_region(uint32_t region_id);

// Hands out the slots of one region. Publisher-local: a slot is free exactly
// when the message occupying it has been destroyed, which the kernel module
// already decides, so nothing here duplicates that bookkeeping.
class GpuSlotPool
{
public:
  GpuSlotPool(uint32_t region_id, uint32_t slot_count);

  [[nodiscard]] uint32_t region_id() const noexcept { return region_id_; }

  // Fails rather than waits when every slot is in flight. Blocking here would
  // stall the publishing thread on subscribers it does not control.
  [[nodiscard]] bool acquire(uint32_t & out_slot_index);
  void release(uint32_t slot_index);

  [[nodiscard]] size_t available() const;

private:
  uint32_t region_id_;
  mutable std::mutex mutex_;
  std::vector<uint32_t> free_slots_;
};

// Returns a slot to the pool that owns it. A no-op in a process that does not
// own that region.
void release_gpu_slot(uint32_t region_id, uint32_t slot_index);

// Marks a message whose payload lives in GPU device memory. The publisher uses
// it to decide that borrowing must also reserve a slot.
struct gpu_message_tag
{
};

template <typename T>
inline constexpr bool is_gpu_message_v = std::is_base_of_v<gpu_message_tag, std::remove_const_t<T>>;

// A device buffer as it appears inside a message. Stored in host shared memory,
// so it holds only values that mean the same thing in every process; the device
// address is resolved per process through the region table.
template <typename T>
class gpu_array
{
public:
  gpu_array() = default;
  gpu_array(uint32_t region_id, uint32_t slot_index, uint64_t count, topic_local_id_t publisher_id)
  : region_id_(region_id), slot_index_(slot_index), count_(count), publisher_id_(publisher_id)
  {
  }

  // The slot returns when the message is destroyed, which is the rule the host
  // payload already follows: publish() hands released messages back for
  // deletion, and a borrow dropped without publishing deletes its message too,
  // so both paths reclaim here without either knowing about slots.
  //
  // Releasing is a no-op in a process that does not own the region, so a
  // subscriber reading this handle in shared memory frees nothing.
  ~gpu_array() { release(); }

  // Move-only: two handles to one slot would release it twice.
  gpu_array(const gpu_array &) = delete;
  gpu_array & operator=(const gpu_array &) = delete;

  gpu_array(gpu_array && other) noexcept { swap(other); }
  gpu_array & operator=(gpu_array && other) noexcept
  {
    if (this != &other) {
      release();
      swap(other);
    }
    return *this;
  }

  [[nodiscard]] T * get() const
  {
    const GpuRegion * region = find_mapped_region(region_id_);
    if (region == nullptr) return nullptr;
    return static_cast<T *>(region->slot_address(slot_index_));
  }

  [[nodiscard]] uint64_t size() const noexcept { return count_; }
  [[nodiscard]] uint32_t region_id() const noexcept { return region_id_; }
  [[nodiscard]] uint32_t slot_index() const noexcept { return slot_index_; }
  [[nodiscard]] bool valid() const noexcept { return region_id_ != 0; }

  // Scaffolding. The kernel module keys a region on its publisher, so a
  // subscriber needs the publisher to map one for the first time, and nothing
  // yet hands it the publisher of the message it is holding. Once the receive
  // path supplies that, region_id alone identifies the region and this goes away.
  [[nodiscard]] topic_local_id_t publisher_id() const noexcept { return publisher_id_; }

private:
  void release() noexcept
  {
    if (region_id_ != 0) {
      release_gpu_slot(region_id_, slot_index_);
      region_id_ = 0;
    }
  }

  void swap(gpu_array & other) noexcept
  {
    std::swap(region_id_, other.region_id_);
    std::swap(slot_index_, other.slot_index_);
    std::swap(count_, other.count_);
    std::swap(publisher_id_, other.publisher_id_);
  }

  // Zero is never a valid region id, so a default-constructed handle resolves to
  // nothing rather than to region 0.
  uint32_t region_id_ = 0;
  uint32_t slot_index_ = 0;
  uint64_t count_ = 0;
  topic_local_id_t publisher_id_ = -1;
};

// Publisher side. Allocates the region, hands its liveness reference to the
// kernel module, registers it in this process's table, and returns a pool over
// its slots. Returns nullptr on failure.
// Lets a slot be returned knowing only what a message carries. The publisher's
// pool is registered here when it is created.
void register_slot_pool(uint32_t region_id, GpuSlotPool * pool);

[[nodiscard]] std::unique_ptr<GpuSlotPool> create_gpu_slot_pool(
  const std::string & topic_name, topic_local_id_t publisher_id, uint32_t slot_size,
  uint32_t slot_count);

// Publisher side. Allocates the region and hands its liveness reference to the
// kernel module, which holds it so the memory outlives this process.
[[nodiscard]] bool create_and_register_gpu_region(
  const std::string & topic_name, topic_local_id_t publisher_id, uint32_t slot_size,
  uint32_t slot_count, GpuRegion & out_region, uint32_t & out_region_id);

// Subscriber side. Retrieves a publisher's region from the kernel module, which
// installs a descriptor for it in this process, and maps it.
[[nodiscard]] bool import_gpu_region(
  const std::string & topic_name, topic_local_id_t publisher_id, topic_local_id_t subscriber_id,
  uint32_t wanted_region_id, GpuRegion & out_region, uint32_t & out_region_id);

// Subscriber side. Imports a publisher's region if this process has not already
// mapped it, and registers it so messages can resolve against it. Idempotent.
// `wanted_region_id` is the region a message names; 0 asks for any. Returns
// immediately when that region is already mapped.
[[nodiscard]] bool ensure_gpu_region_mapped(
  const std::string & topic_name, topic_local_id_t publisher_id, topic_local_id_t subscriber_id,
  uint32_t wanted_region_id);

}  // namespace agnocast::internal
