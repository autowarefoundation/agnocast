#pragma once

// What a message carries, and how a process turns it back into a device
// address. Deliberately free of the backend interface so that including
// agnocast.hpp does not drag the GPU SPI into every translation unit.
// docs/gpu_memory.md explains the addressing scheme.

#include "agnocast/agnocast_ioctl.hpp"

#include <cstdint>
#include <string_view>
#include <type_traits>
#include <utility>

namespace agnocast::internal
{

// Names a region to the kmod. `subscriber_id` is the receiving endpoint, which
// the kmod checks against the calling process before handing out a descriptor.
// `region_id` is the id read out of the message being resolved, or 0 for "any",
// which is what a caller that has not yet seen a message asks for.
struct GpuRegionRef
{
  std::string_view topic_name;
  topic_local_id_t publisher_id = -1;
  topic_local_id_t subscriber_id = -1;
  uint32_t region_id = 0;
};

// Every region this process has mapped, keyed by the id the kmod assigned it.
//
// A mapping lives until the process exits, or until the publisher owning the
// region destroys it -- which it may do only once nothing refers to the region.
// Addresses are never handed out past the lock for that reason: `resolve` does
// the lookup and the bounds check together, so a region cannot be unmapped
// between them.
class GpuRegionRegistry
{
public:
  static GpuRegionRegistry & instance();

  [[nodiscard]] bool is_mapped(uint32_t region_id) const;

  // nullptr when the region is not mapped here, or the slot does not hold
  // `bytes`.
  [[nodiscard]] void * resolve(uint32_t region_id, uint32_t slot_index, uint64_t bytes) const;

  // Subscriber side. Idempotent: returns immediately when the region the
  // message refers to is already mapped.
  [[nodiscard]] bool ensure_mapped(const GpuRegionRef & ref);

  // Publisher side. Allocates the region and hands its liveness reference to the
  // kmod, which holds it so the memory outlives this process. Returns the id the
  // kmod assigned, or 0 on failure.
  [[nodiscard]] uint32_t create(
    std::string_view topic_name, topic_local_id_t publisher_id, uint32_t slot_size,
    uint32_t slot_count);

  // Publisher side. Releases the mapping and the kmod's reference. The caller
  // must know that no message refers to the region: a subscriber that already
  // imported it keeps its own reference and reads on, but one that has not will
  // no longer be able to.
  void destroy(std::string_view topic_name, topic_local_id_t publisher_id, uint32_t region_id);

private:
  GpuRegionRegistry() = default;
};

// nullptr when the region is not mapped in this process, or when the slot does
// not hold `bytes`.
[[nodiscard]] void * resolve_gpu_slot(
  uint32_t region_id, uint32_t slot_index, uint64_t bytes) noexcept;

// Returns a slot to the pool that owns it. A no-op in a process that does not
// own the region, so a subscriber dropping a handle frees nothing.
void release_gpu_slot(uint32_t region_id, uint32_t slot_index) noexcept;

// Marks a message whose payload lives in GPU device memory, so the publisher
// knows that borrowing must also reserve a slot.
struct gpu_message_tag
{
};

template <typename T>
inline constexpr bool is_gpu_message_v = std::is_base_of_v<gpu_message_tag, std::remove_const_t<T>>;

// A device buffer as it appears inside a message: the publisher fills one in
// when it borrows, and a subscriber resolves it back to an address of its own.
// It lives in host shared memory, so it holds only values that mean the same
// thing in every process -- never a device address.
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
  // deletion, and a borrow dropped without publishing deletes its message too.
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

  // Const-qualified on the message, const on the way out. A subscriber holds an
  // ipc_shared_ptr<const MessageT>, so this is the overload it reaches, and its
  // mapping of the region is read-only: a device pointer that let it write would
  // fault rather than corrupt, but the type is what says so at the call site.
  [[nodiscard]] const T * get() const noexcept
  {
    return static_cast<const T *>(resolve_gpu_slot(region_id_, slot_index_, count_ * sizeof(T)));
  }

  // The publisher's own message is non-const, and its mapping is writable.
  [[nodiscard]] T * get() noexcept
  {
    return static_cast<T *>(resolve_gpu_slot(region_id_, slot_index_, count_ * sizeof(T)));
  }

  [[nodiscard]] uint64_t size() const noexcept { return count_; }
  [[nodiscard]] uint32_t region_id() const noexcept { return region_id_; }
  [[nodiscard]] uint32_t slot_index() const noexcept { return slot_index_; }
  [[nodiscard]] bool valid() const noexcept { return region_id_ != 0; }

  // Scaffolding. The kmod keys a region on its publisher, and the receive path
  // does not yet hand a subscriber the publisher of the message it holds. Once
  // it does, the region id alone suffices and this goes away.
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

}  // namespace agnocast::internal
